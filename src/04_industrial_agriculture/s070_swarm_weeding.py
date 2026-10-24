"""
S070 Swarm Weeding Task
=======================
Three drones sweep a 40 x 40 m field in three 13.3 m sub-strips (Phase 1), detect weeds
with a 1.5 m-radius sensor (P_detect = 0.85 per pass encounter), then share out the
detected weeds and hover 2 s over each one under PID control (Phase 2).

Compared over a 200-trial Monte Carlo (same weeds, detections and hover noise for every
strategy):
  Greedy           card baseline: repeatedly give the closest (drone, weed) pair to that
                   drone and move the drone's notional position to the weed
  Hungarian + NN   card's capacitated Hungarian assignment on end-of-sweep distances,
                   each drone then visits its weeds in nearest-neighbour order
  Hungarian (card) same assignment, weeds visited in the order the solver returns them
and two hover controllers: the card's PID (Kp, Ki, Kd) = (2, 0.1, 0.5) and the same loop
without the derivative term.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s070_swarm_weeding.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.optimize import linear_sum_assignment

# ── Parameters (card) ─────────────────────────────────────────────────────────
FIELD       = 40.0
N_DRONES    = 3
N_WEEDS     = 30
R_SENSE     = 1.5
SWEEP_MAX   = 2 * R_SENSE              # 3.0 m maximum pass spacing
P_DETECT    = 0.85
V           = 1.0                      # m/s
SIGMA_GPS   = 0.10
T_TREAT     = 2.0
EPS_POS     = 0.15                     # m RMS treatment accuracy
KP, KI, KD  = 2.0, 0.1, 0.5
DT          = 0.05
SEED        = 42
START_SD    = 0.3                      # m arrival offset std per axis (card code)
STRIP_W     = FIELD / N_DRONES

# ── Parameters (added) ───────────────────────────────────────────────────────
N_TRIALS    = 200
ANIM_STEP   = 2.0                      # s per GIF frame
STRATS      = ['greedy', 'hung_nn', 'hung_card']
LABELS      = {'greedy': 'Greedy nearest-drone', 'hung_nn': 'Hungarian + NN order',
               'hung_card': 'Hungarian, solver order (card)'}
COLORS      = {'greedy': '#7F7F7F', 'hung_nn': '#1F77B4', 'hung_card': '#9467BD'}
DRONE_COLS  = ['tab:red', 'tab:orange', 'tab:purple']

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'outputs',
                          '04_industrial_agriculture', 's070_swarm_weeding')


# ── Phase 1 ──────────────────────────────────────────────────────────────────

def pass_positions(i, layout='formula'):
    """Cross-row (y) positions of the passes of drone i."""
    y0, y1 = i * STRIP_W, (i + 1) * STRIP_W
    if layout == 'formula':                 # n = ceil(L_strip / 2 r_s), evenly spaced
        n = int(np.ceil(STRIP_W / SWEEP_MAX))
        sp = STRIP_W / n
        return y0 + sp / 2 + sp * np.arange(n)
    return np.arange(y0 + SWEEP_MAX / 2, y1, SWEEP_MAX)   # card code: arange, 3 m


def sweep_path(i, layout='formula'):
    pts = []
    for k, y in enumerate(pass_positions(i, layout)):
        xs = (0.0, FIELD) if k % 2 == 0 else (FIELD, 0.0)
        pts += [[xs[0], y], [xs[1], y]]
    return np.array(pts)


def path_length(p):
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def detect(weeds, rng, layout='formula'):
    """One Bernoulli(P_DETECT) draw per (pass, weed) encounter. An encounter happens
    when the GPS-noisy lateral distance of the pass to the weed is <= r_s."""
    det = np.zeros(len(weeds), dtype=bool)
    n_enc = np.zeros(len(weeds), dtype=int)
    for i in range(N_DRONES):
        for y in pass_positions(i, layout):
            lat = np.abs(weeds[:, 1] - (y + SIGMA_GPS * rng.standard_normal(len(weeds))))
            enc = lat <= R_SENSE
            hit = enc & (rng.random(len(weeds)) < P_DETECT)
            n_enc += enc
            det |= hit
    return det, n_enc


def per_step_detection_prob():
    """Card code variant: a Bernoulli draw every 0.05 s while the weed is in range.
    Mean detection probability over lateral offsets uniform in [0, r_s)."""
    y = np.linspace(0, R_SENSE, 1000, endpoint=False)
    n_steps = np.floor(2 * np.sqrt(R_SENSE ** 2 - y ** 2) / (V * DT))
    return float(np.mean(1 - (1 - P_DETECT) ** n_steps))


# ── Phase 2 assignment ───────────────────────────────────────────────────────

def greedy_assign(ends, w):
    assign = [[] for _ in range(N_DRONES)]
    cur = ends.copy()
    left = list(range(len(w)))
    while left:
        d = np.linalg.norm(cur[:, None, :] - w[left][None, :, :], axis=2)
        i, jj = np.unravel_index(np.argmin(d), d.shape)
        j = left[jj]
        assign[i].append(j)
        cur[i] = w[j]
        left.remove(j)
    return assign


def hungarian_assign(ends, w):
    """Card: replicate each drone row K = ceil(M/N) times (capacity), solve LSAP."""
    M = len(w)
    if M == 0:
        return [[] for _ in range(N_DRONES)]
    K = int(np.ceil(M / N_DRONES))
    C = np.linalg.norm(ends[:, None, :] - w[None, :, :], axis=2)
    rows, cols = linear_sum_assignment(np.tile(C, (K, 1)))
    assign = [[] for _ in range(N_DRONES)]
    for r, c in zip(rows, cols):
        assign[r % N_DRONES].append(int(c))
    return assign


def nn_order(start, w, idx):
    order, cur, left = [], start.copy(), list(idx)
    while left:
        d = [np.linalg.norm(w[j] - cur) for j in left]
        j = left[int(np.argmin(d))]
        order.append(j)
        cur = w[j]
        left.remove(j)
    return order


def route_cost(ends, w, assign):
    dist = np.zeros(N_DRONES)
    for i, lst in enumerate(assign):
        cur = ends[i]
        for j in lst:
            dist[i] += np.linalg.norm(w[j] - cur)
            cur = w[j]
    times = dist / V + np.array([len(a) for a in assign]) * T_TREAT
    return dist, times


# ── Hover ────────────────────────────────────────────────────────────────────

def pid_hover(z0, zn, kd):
    """Vectorised card PID (velocity command, pos += u dt) for many hovers.
    z0: (H,2) N(0,1) start offsets; zn: (H,S,2) N(0,1) GPS noise. Returns RMS and traj."""
    pos = START_SD * z0
    integ = np.zeros_like(pos)
    prev = -pos
    traj = [pos.copy()]
    for k in range(zn.shape[1]):
        e = -(pos + SIGMA_GPS * zn[:, k])
        integ += e * DT
        u = KP * e + KI * integ + kd * (e - prev) / DT
        pos = pos + u * DT
        prev = e
        traj.append(pos.copy())
    traj = np.stack(traj, axis=1)
    rms = np.sqrt(np.mean(np.sum(traj ** 2, axis=2), axis=1))
    return rms, traj


# ── Simulation ───────────────────────────────────────────────────────────────

def run_trial(seed):
    rng = np.random.default_rng(seed)
    weeds = rng.uniform(0, FIELD, size=(N_WEEDS, 2))
    det, n_enc = detect(weeds, rng)
    det_card, _ = detect(weeds, np.random.default_rng(seed + 10_000), layout='card')
    ends = np.array([sweep_path(i)[-1] for i in range(N_DRONES)])
    idx = np.where(det)[0]
    w = weeds[idx]
    out = {'weeds': weeds, 'det': det, 'n_enc': n_enc, 'det_card': det_card, 'ends': ends,
           'idx': idx}
    hung = hungarian_assign(ends, w)
    plans = {'greedy': greedy_assign(ends, w), 'hung_card': hung,
             'hung_nn': [nn_order(ends[i], w, hung[i]) for i in range(N_DRONES)]}
    K = int(np.ceil(len(w) / N_DRONES))
    for s, a in plans.items():
        assert sorted(j for lst in a for j in lst) == list(range(len(w))), 'weed not covered once'
        if s != 'greedy':
            assert max(len(lst) for lst in a) <= K, 'Hungarian capacity violated'
        dist, times = route_cost(ends, w, a)
        out[s] = {'assign': a, 'dist': dist, 'times': times}
    # hover noise per detected weed (shared by every strategy and controller)
    n_steps = int(round(T_TREAT / DT))
    z0 = rng.standard_normal((len(idx), 2))
    zn = rng.standard_normal((len(idx), n_steps, 2))
    out['rms_pid'], out['traj_pid'] = pid_hover(z0, zn, KD)
    out['rms_pi'], _ = pid_hover(z0, zn, 0.0)
    return out


def run_simulation():
    trials = [run_trial(SEED + t) for t in range(N_TRIALS)]
    t_phase1 = max(path_length(sweep_path(i)) for i in range(N_DRONES)) / V
    return trials, t_phase1


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_field_overview(tr, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    w = tr['weeds'][tr['idx']]
    for ax, s in zip(axes, ['hung_nn', 'greedy']):
        for i in range(N_DRONES):
            ax.axhspan(i * STRIP_W, (i + 1) * STRIP_W, color=DRONE_COLS[i], alpha=0.06)
            p = sweep_path(i)
            ax.plot(p[:, 0], p[:, 1], color=DRONE_COLS[i], lw=0.6, alpha=0.6)
            cur = tr['ends'][i]
            for j in tr[s]['assign'][i]:
                ax.annotate('', xy=w[j], xytext=cur,
                            arrowprops=dict(arrowstyle='->', color=DRONE_COLS[i], lw=1.5))
                cur = w[j]
            ax.plot(*tr['ends'][i], '^', color=DRONE_COLS[i], ms=12, mec='k')
        ax.scatter(*tr['weeds'][~tr['det']].T, color='grey', s=40, label='missed weed', zorder=4)
        ax.scatter(*w.T, color='gold', edgecolor='k', s=60, label='detected weed', zorder=5)
        d = tr[s]['dist']
        ax.set_title(f"{LABELS[s]}: Phase-2 distance {d.sum():.1f} m, makespan "
                     f"{tr[s]['times'].max():.1f} s\nweeds per drone "
                     f"{[len(a) for a in tr[s]['assign']]}", fontsize=10)
        ax.set_xlim(-1, FIELD + 1)
        ax.set_ylim(-1, FIELD + 1)
        ax.set_aspect('equal')
        ax.set_xlabel('x (m, along rows)')
        ax.set_ylabel('y (m)')
        ax.legend(loc='upper right', fontsize=8)
    fig.suptitle(f'S070 Trial 0: {tr["det"].sum()}/{N_WEEDS} weeds detected; thin lines = '
                 'Phase-1 sweeps, arrows = Phase-2 routes, triangles = end of sweep', y=1.03)
    plt.tight_layout()
    path = os.path.join(out_dir, 'field_overview.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics(trials, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    rp = np.concatenate([t['rms_pid'] for t in trials])
    ri = np.concatenate([t['rms_pi'] for t in trials])
    bins = np.linspace(0, max(rp.max(), ri.max()), 40)
    axes[0].hist(rp, bins=bins, alpha=0.7, color='tab:red', label=f'card PID, mean {rp.mean():.3f} m')
    axes[0].hist(ri, bins=bins, alpha=0.7, color='tab:blue', label=f'PI (Kd = 0), mean {ri.mean():.3f} m')
    axes[0].axvline(EPS_POS, color='k', ls='--', label='0.15 m threshold')
    axes[0].set_xlabel('Hover RMS error over the 2 s dwell (m)')
    axes[0].set_ylabel('Hovers')
    axes[0].set_title(f'Hover accuracy ({len(rp)} hovers)')
    axes[0].legend(fontsize=8)
    for k, (key, lab) in enumerate([('dist', 'Phase-2 total distance (m)'),
                                    ('times', 'Phase-2 makespan (s)')]):
        vals = [[t[s][key].sum() if key == 'dist' else t[s][key].max() for t in trials]
                for s in STRATS]
        m = [np.mean(v) for v in vals]
        sd = [np.std(v) for v in vals]
        x = np.arange(len(STRATS)) + 0.25 * (k - 0.5)
        ax = axes[1] if k == 0 else axes[1].twinx()
        bars = ax.bar(x, m, width=0.25, yerr=sd, capsize=4,
                      color=[COLORS[s] for s in STRATS], alpha=1.0 if k == 0 else 0.45,
                      edgecolor='k', hatch='' if k == 0 else '//')
        for b, v in zip(bars, m):
            ax.text(b.get_x() + b.get_width() / 2, v * 1.02, f'{v:.1f}', ha='center', fontsize=8)
        ax.set_ylabel(lab)
        ax.set_ylim(0, max(m) * 1.4)
    axes[1].set_xticks(np.arange(len(STRATS)))
    axes[1].set_xticklabels([LABELS[s] for s in STRATS], fontsize=8)
    axes[1].set_title(f'Phase 2, mean ± std over {N_TRIALS} trials\n(solid = distance, '
                      'hatched = makespan, right axis)')
    det = np.array([t['det'].sum() for t in trials])
    ok = np.array([(t['rms_pid'] <= EPS_POS).sum() for t in trials])
    tot = N_WEEDS * len(trials)
    sizes = [tot - det.sum(), det.sum() - ok.sum(), ok.sum()]
    axes[2].pie(sizes, labels=['missed\n(not detected)', 'detected,\nhover RMS > 0.15 m',
                               'treated'],
                colors=['#bdbdbd', '#fdae6b', '#31a354'], autopct='%1.1f%%')
    axes[2].set_title('Weed outcome with the card PID (all trials)')
    fig.suptitle('S070 Metrics')
    plt.tight_layout()
    path = os.path.join(out_dir, 'metrics.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_mc_comparison(trials, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))
    dg = np.array([t['greedy']['dist'].sum() for t in trials])
    for s, mk in [('hung_nn', 'o'), ('hung_card', 's')]:
        dh = np.array([t[s]['dist'].sum() for t in trials])
        axes[0].scatter(dh, dg, s=14, marker=mk, color=COLORS[s], alpha=0.7, label=LABELS[s])
    lim = [0, max(axes[0].get_xlim()[1], axes[0].get_ylim()[1])]
    axes[0].plot(lim, lim, 'k--', lw=1, label='equal distance')
    axes[0].set_xlabel('Hungarian Phase-2 distance (m)')
    axes[0].set_ylabel('Greedy Phase-2 distance (m)')
    axes[0].set_title('Per trial (points below the diagonal: greedy shorter)')
    axes[0].legend(fontsize=8)
    for s in STRATS:
        r = np.array([t[s]['dist'].sum() for t in trials]) / dg
        axes[1].hist(r, bins=30, alpha=0.6, color=COLORS[s], label=LABELS[s])
    axes[1].axvline(1, color='k', ls='--')
    axes[1].set_xlabel('Phase-2 distance relative to greedy')
    axes[1].set_ylabel('Trials')
    axes[1].set_title('Distance ratio to greedy (card expects greedy / Hungarian >= 1)')
    axes[1].legend(fontsize=8)
    fig.suptitle(f'S070 Assignment comparison over {N_TRIALS} trials (same weeds and detections)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'mc_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def drone_timeline(tr, s, t_phase1):
    """Per-drone list of (t, x, y) knots: sweep, wait, Phase-2 legs and hovers."""
    w = tr['weeds'][tr['idx']]
    lines = []
    for i in range(N_DRONES):
        p = sweep_path(i)
        t = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1)) / V])
        knots = [(tt, *pp) for tt, pp in zip(t, p)]
        cur_t, cur = t_phase1, p[-1]
        knots.append((cur_t, *cur))
        for j in tr[s]['assign'][i]:
            cur_t += np.linalg.norm(w[j] - cur) / V
            knots.append((cur_t, *w[j]))
            cur_t += T_TREAT
            knots.append((cur_t, *w[j]))
            cur = w[j]
        lines.append(np.array(knots))
    return lines


def save_animation(tr, t_phase1, out_dir, s='hung_nn'):
    lines = drone_timeline(tr, s, t_phase1)
    t_end = max(l[-1, 0] for l in lines)
    n_frames = int(np.ceil(t_end / ANIM_STEP)) + 1
    times = np.minimum(np.arange(n_frames) * ANIM_STEP, t_end)
    fig, ax = plt.subplots(figsize=(7, 7))
    for i in range(N_DRONES):
        ax.axhspan(i * STRIP_W, (i + 1) * STRIP_W, color=DRONE_COLS[i], alpha=0.06)
    ax.scatter(*tr['weeds'].T, color='grey', s=30, zorder=3)
    w = tr['weeds'][tr['idx']]
    # time each weed is detected: when its detecting drone passes closest (approximate,
    # first pass of the owning drone over the weed's lateral band)
    gold = ax.scatter([], [], color='gold', edgecolor='k', s=60, zorder=4)
    treated = ax.scatter([], [], color='green', edgecolor='k', s=60, zorder=5)
    trails = [ax.plot([], [], color=DRONE_COLS[i], lw=0.8)[0] for i in range(N_DRONES)]
    dots = [ax.plot([], [], 'o', color=DRONE_COLS[i], mec='k', ms=9, zorder=6)[0]
            for i in range(N_DRONES)]
    ax.set_xlim(-1, FIELD + 1)
    ax.set_ylim(-1, FIELD + 1)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    t_det = np.full(len(w), t_phase1)
    for i in range(N_DRONES):
        p = sweep_path(i)
        tt = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1)) / V])
        for k in range(0, len(p), 2):
            in_band = (np.abs(w[:, 1] - p[k, 1]) <= R_SENSE) & (t_det == t_phase1)
            t_pass = tt[k] + np.abs(w[:, 0] - p[k, 0]) / V
            t_det = np.where(in_band, np.minimum(t_det, t_pass), t_det)
    t_treat = {}
    for i, l in enumerate(lines):
        k = len(sweep_path(i)) + 1
        for j in tr[s]['assign'][i]:
            t_treat[j] = l[k + 1, 0]
            k += 2

    def update(fi):
        t = times[fi]
        for i, l in enumerate(lines):
            x = np.interp(t, l[:, 0], l[:, 1])
            y = np.interp(t, l[:, 0], l[:, 2])
            m = l[:, 0] <= t
            trails[i].set_data(np.append(l[m, 1], x), np.append(l[m, 2], y))
            dots[i].set_data([x], [y])
        g = w[t_det <= t]
        gold.set_offsets(g if len(g) else np.empty((0, 2)))
        dn = [j for j, tt in t_treat.items() if tt <= t]
        treated.set_offsets(w[dn] if dn else np.empty((0, 2)))
        phase = 'Phase 1 sweep' if t < t_phase1 else 'Phase 2 treatment'
        ax.set_title(f'S070 {phase} ({LABELS[s]}), t = {t:.0f} s\n'
                     f'detected {int(np.sum(t_det <= t))}, treatment hovers done {len(dn)} (green)', fontsize=10)
        return dots

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12, dpi=70)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames, t_end = {t_end:.1f} s)')


# ── Main ─────────────────────────────────────────────────────────────────────

def _ms(v, d=2):
    v = np.asarray(v, dtype=float)
    return f'{v.mean():.{d}f} ± {v.std():.{d}f}'


if __name__ == '__main__':
    trials, t_phase1 = run_simulation()
    print('=== S070 Swarm Weeding ===')
    for lay in ['formula', 'card']:
        ys = pass_positions(0, lay)
        gap = STRIP_W - (ys[-1] + R_SENSE)
        print(f'Pass layout [{lay}]: {len(ys)} passes per strip at '
              f'{", ".join(f"{y:.2f}" for y in ys)} m; uncovered band at the strip edge '
              f'{max(gap, 0):.3f} m ({100 * max(gap, 0) / STRIP_W:.1f} % of the strip)')
    print(f'Phase 1 path per drone {path_length(sweep_path(0)):.2f} m, time {t_phase1:.2f} s')
    det = np.array([t['det'].sum() for t in trials])
    detc = np.array([t['det_card'].sum() for t in trials])
    enc2 = np.mean(np.concatenate([t['n_enc'] >= 2 for t in trials]))
    enc0 = np.mean(np.concatenate([t['n_enc'] == 0 for t in trials]))
    print(f'Detected weeds (per-pass Bernoulli, formula layout): {_ms(det)} / {N_WEEDS} '
          f'({100 * det.mean() / N_WEEDS:.2f} %); weeds with >= 2 encounters {100 * enc2:.2f} %, '
          f'with none {100 * enc0:.2f} %')
    print(f'Detected weeds with the card code pass layout (3 m arange): {_ms(detc)} / {N_WEEDS} '
          f'({100 * detc.mean() / N_WEEDS:.2f} %)')
    print(f'Card code per-step Bernoulli (every {DT} s in range): mean detection probability '
          f'{per_step_detection_prob():.4f}')
    rp = np.concatenate([t['rms_pid'] for t in trials])
    ri = np.concatenate([t['rms_pi'] for t in trials])
    tail = np.concatenate([np.sqrt(np.mean(np.sum(t['traj_pid'][:, 21:] ** 2, axis=2), axis=1))
                           for t in trials])
    print(f'Hover RMS over 2 s ({len(rp)} hovers): card PID {rp.mean():.4f} m '
          f'(<= 0.15 m: {100 * np.mean(rp <= EPS_POS):.2f} %), PI Kd=0 {ri.mean():.4f} m '
          f'(<= 0.15 m: {100 * np.mean(ri <= EPS_POS):.2f} %); card PID over the last 1 s '
          f'{tail.mean():.4f} m; card claim 0.06 m')
    s_card = 0.06
    print(f'Card P_success arithmetic: 1 - exp(-eps^2 / 2 sigma^2) with sigma = {s_card} m = '
          f'{1 - np.exp(-EPS_POS ** 2 / (2 * s_card ** 2)):.4f} (card states 0.997)')
    ok_pid = np.array([(t['rms_pid'] <= EPS_POS).sum() for t in trials])
    ok_pi = np.array([(t['rms_pi'] <= EPS_POS).sum() for t in trials])
    print(f'Weeds treated per trial: card PID {_ms(ok_pid)}, PI {_ms(ok_pi)} '
          f'(card expects 25.4)')
    dg = np.array([t['greedy']['dist'].sum() for t in trials])
    for s in STRATS:
        d = np.array([t[s]['dist'].sum() for t in trials])
        mk = np.array([t[s]['times'].max() for t in trials])
        mx = np.array([max(len(a) for a in t[s]['assign']) for t in trials])
        print(f'[{s}] {LABELS[s]}: Phase-2 distance {_ms(d, 1)} m, makespan {_ms(mk, 1)} s, '
              f'max weeds per drone {_ms(mx, 2)}, total mission (Phase 1 + makespan) '
              f'{_ms(t_phase1 + mk, 1)} s')
    for s in ['hung_nn', 'hung_card']:
        d = np.array([t[s]['dist'].sum() for t in trials])
        mk = np.array([t[s]['times'].max() for t in trials])
        mkg = np.array([t['greedy']['times'].max() for t in trials])
        rho = dg / d
        print(f'rho = D_greedy / D_{s}: {_ms(rho, 3)}; trials with rho >= 1: '
              f'{int(np.sum(rho >= 1))}/{N_TRIALS}; shorter makespan than greedy: '
              f'{int(np.sum(mk < mkg))}/{N_TRIALS}')
    t0 = trials[0]
    print(f'Trial 0: detected {t0["det"].sum()}/{N_WEEDS}; ' + ', '.join(
        f'{s} distance {t0[s]["dist"].sum():.1f} m, makespan {t0[s]["times"].max():.1f} s'
        for s in STRATS))

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_field_overview(trials[0], out)
    plot_metrics(trials, out)
    plot_mc_comparison(trials, out)
    save_animation(trials[0], t_phase1, out)
