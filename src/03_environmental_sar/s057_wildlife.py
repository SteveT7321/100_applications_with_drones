"""
S057 Wildlife Population Census
================================
A single survey drone flies a lawnmower (boustrophedon) pattern over a 500 x 500 m
reserve that holds G = 20 clustered animal groups. The nadir camera (half-angle 25 deg)
sweeps a strip of width w(h) = 2 h tan(25 deg). Each animal inside a strip is detected
with probability P_d = P_d0 exp(-alpha h) exp(-beta v) (1 - c_g). Three fixed operating
points (high/fast, mid/medium, low/slow) are compared with common random numbers: every
strategy sees the same animal positions and the same detection uniforms. The population
is estimated with the card's strip-transect correction N_raw / P_d(h, v, c_bar).

Added beyond the card: a battery model (30 min endurance, return-to-launch and battery
swap), a 20 % side-overlap sensitivity run that makes double counting possible, and a
200-reserve Monte Carlo that separates estimator bias from sampling noise.

Usage:
    conda activate drones
    python src/03_environmental_sar/s057_wildlife.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D              # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── Parameters (card) ─────────────────────────────────────────────────────────
AREA_SIZE       = 500.0      # m — reserve side length W
N_GROUPS        = 20         # animal groups G
N_ANIMALS_RANGE = (1, 20)    # animals per group, inclusive
SIGMA_GROUP     = 8.0        # m — intra-group spread
THETA_FOV       = 25.0       # deg — camera half-angle
P_D0            = 0.95       # baseline detection probability
ALPHA           = 0.04       # 1/m — altitude decay
BETA            = 0.08       # s/m — speed decay
CAMO_RANGE      = (0.0, 0.6) # camouflage factor range
N_MONTE         = 50         # Monte Carlo trials per strategy (card)
RESERVE_SEED    = 42         # seed of the card's reserve

STRATEGIES = [
    (40.0, 12.0, 'High-alt fast'),
    (20.0,  6.0, 'Mid-alt medium'),
    ( 8.0,  2.0, 'Low-alt slow'),
]
STRAT_COLORS = ['#9467BD', '#FF7F0E', '#1F77B4']

# ── Parameters (added) ────────────────────────────────────────────────────────
E_BATT       = 1800.0        # s — battery endurance (30 min)
T_RESERVE    = 180.0         # s — landing reserve (10 % of endurance)
T_SWAP       = 300.0         # s — land, swap battery, take off
V_TRANSIT    = 12.0          # m/s — return-to-launch / re-entry transit speed
HOME         = np.array([0.0, 0.0])   # launch point (reserve corner)
DS_PATH      = 2.0           # m — path discretisation for the battery walk
P_OVERLAP    = 0.20          # side overlap of the sensitivity run
N_RESERVES   = 200           # random reserves for the bias / noise split
MC_SEED      = 2024          # seed of the per-trial positions and detection uniforms
RES_SEED0    = 10_000        # first seed of the random-reserve Monte Carlo
ANIM_STEP    = 32            # trajectory points per GIF frame

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's057_wildlife',
)


# ── Model ─────────────────────────────────────────────────────────────────────

def strip_width(h):
    return 2.0 * h * np.tan(np.radians(THETA_FOV))


def detection_prob(h, v, camouflage):
    return P_D0 * np.exp(-ALPHA * h) * np.exp(-BETA * v) * (1.0 - camouflage)


def strip_layout(h, overlap=0.0):
    """Strip centres (y) and spacing. overlap = 0 reproduces the card's contiguous strips."""
    w = strip_width(h)
    if overlap == 0.0:
        n = int(np.ceil(AREA_SIZE / w))
        s = w
    else:
        s = w * (1.0 - overlap)
        n = int(np.ceil((AREA_SIZE - w) / s)) + 1
    centres = w / 2.0 + s * np.arange(n)
    return w, s, n, centres


def path_length(h, overlap=0.0):
    w, s, n, _ = strip_layout(h, overlap)
    return n * AREA_SIZE + (n - 1) * s


def mean_multiplicity(h, overlap=0.0):
    """Mean number of strips covering a point of the reserve (strips clipped to [0, W])."""
    w, s, n, c = strip_layout(h, overlap)
    lo = np.clip(c - w / 2, 0, AREA_SIZE)
    hi = np.clip(c + w / 2, 0, AREA_SIZE)
    return float(np.sum(hi - lo) / AREA_SIZE)


def card_overlap_delta(h):
    """The card's overlap fraction delta = max(0, 2w - W/n) / w (verified, not used)."""
    w, _, n, _ = strip_layout(h)
    return max(0.0, 2 * w - AREA_SIZE / n) / w


def generate_reserve(rng):
    centroids = rng.uniform(20, AREA_SIZE - 20, size=(N_GROUPS, 2))
    sizes = rng.integers(N_ANIMALS_RANGE[0], N_ANIMALS_RANGE[1] + 1, size=N_GROUPS)
    camo = rng.uniform(*CAMO_RANGE, size=N_GROUPS)
    return centroids, sizes, camo


def draw_individuals(centroids, sizes, rng):
    """Individual positions (clipped to the reserve), group index, and two detection uniforms."""
    gid = np.repeat(np.arange(len(sizes)), sizes)
    pos = centroids[gid] + rng.normal(0.0, SIGMA_GROUP, size=(len(gid), 2))
    pos = np.clip(pos, 0.0, AREA_SIZE)
    u = rng.random((len(gid), 2))          # u[:,k] = uniform for the k-th strip covering it
    return pos, gid, u


def covering_strips(y, h, overlap=0.0):
    """(n_ind, 2) strip indices covering each animal; -1 where there is no second strip."""
    w, s, n, c = strip_layout(h, overlap)
    if overlap == 0.0:
        # card: contiguous half-open strips [j w, (j+1) w); an animal is in exactly one
        j = np.minimum((y // w).astype(int), n - 1)
        inside = np.zeros((len(y), n), dtype=bool)
        inside[np.arange(len(y)), j] = True
    else:
        inside = np.abs(y[:, None] - c[None, :]) <= w / 2.0      # (n_ind, n_strips)
    cnt = inside.sum(axis=1)
    assert np.all(cnt >= 1), 'animal outside every strip (coverage gap)'
    assert np.all(cnt <= 2), 'more than two strips over one point'
    idx = -np.ones((len(y), 2), dtype=int)
    for i in range(len(y)):
        js = np.flatnonzero(inside[i])
        idx[i, :len(js)] = js
    return idx


def census(h, v, pos, gid, u, camo, overlap=0.0):
    """One census. Returns raw count, unique detections, detected mask, detection strip."""
    pd = detection_prob(h, v, camo[gid])
    strips = covering_strips(pos[:, 1], h, overlap)
    det = (strips >= 0) & (u < pd[:, None])                       # (n_ind, 2)
    raw = int(det.sum())
    detected = det.any(axis=1)
    first_strip = np.where(det[:, 0], strips[:, 0], np.where(det[:, 1], strips[:, 1], -1))
    return raw, int(detected.sum()), detected, first_strip


def fly_mission(h, v, overlap=0.0, record=False):
    """Walk the lawnmower with a battery model; return times, sorties and optional trajectory.

    The drone launches from HOME, flies the strips at survey speed v and, before every
    path step, checks that it can still reach HOME at V_TRANSIT with T_RESERVE left.
    Otherwise it returns, swaps the battery (T_SWAP) and flies back to the resume point.
    """
    w, s, n, c = strip_layout(h, overlap)
    wps = []
    for j, yc in enumerate(c):
        xs = (0.0, AREA_SIZE) if j % 2 == 0 else (AREA_SIZE, 0.0)
        wps.append((xs[0], yc, j))
        wps.append((xs[1], yc, j))
    pts, strip_of = [], []
    for k in range(len(wps) - 1):
        a = np.array(wps[k][:2]); b = np.array(wps[k + 1][:2])
        m = max(1, int(np.ceil(np.linalg.norm(b - a) / DS_PATH)))
        for q in range(m):
            pts.append(a + (b - a) * q / m)
            strip_of.append(wps[k][2] if wps[k][2] == wps[k + 1][2] else -1)
    pts.append(np.array(wps[-1][:2])); strip_of.append(wps[-1][2])
    pts = np.array(pts)

    t = 0.0
    batt = E_BATT
    sorties = 1
    t_survey = 0.0
    t_transit = 0.0
    traj = []   # (t, x, y, mode, strip) mode 0 survey, 1 transit, 2 swap

    def transit(a, b, t, batt):
        d = np.linalg.norm(b - a)
        m = max(1, int(np.ceil(d / (V_TRANSIT * 2.0))))
        for q in range(1, m + 1):
            p = a + (b - a) * q / m
            if record:
                traj.append((t + d / V_TRANSIT * q / m, p[0], p[1], 1, -1))
        return t + d / V_TRANSIT, batt - d / V_TRANSIT, d / V_TRANSIT

    t, batt, dt_ = transit(HOME, pts[0], t, batt)
    t_transit += dt_
    if record:
        traj.insert(0, (0.0, HOME[0], HOME[1], 1, -1))
    for i in range(len(pts) - 1):
        seg = np.linalg.norm(pts[i + 1] - pts[i])
        dt_seg = seg / v
        need = dt_seg + np.linalg.norm(pts[i + 1] - HOME) / V_TRANSIT + T_RESERVE
        if batt < need:
            t, batt, dt_ = transit(pts[i], HOME, t, batt)
            t_transit += dt_
            assert batt >= T_RESERVE - 1e-6, 'landed below the battery reserve'
            t += T_SWAP
            if record:
                traj.append((t, HOME[0], HOME[1], 2, -1))
            batt = E_BATT
            sorties += 1
            t, batt, dt_ = transit(HOME, pts[i], t, batt)
            t_transit += dt_
        t += dt_seg
        t_survey += dt_seg
        batt -= dt_seg
        assert batt >= 0.0, 'flying on an empty battery'
        assert seg / dt_seg <= max(v, V_TRANSIT) + 1e-9, 'speed limit exceeded'
        if record:
            traj.append((t, pts[i + 1][0], pts[i + 1][1], 0, strip_of[i]))
    t, batt, dt_ = transit(pts[-1], HOME, t, batt)
    t_transit += dt_
    assert batt >= T_RESERVE - 1e-6, 'landed below the battery reserve'
    return {
        't_total': t, 't_survey': t_survey, 't_transit': t_transit,
        'sorties': sorties, 'traj': np.array(traj) if record else None,
    }


def summarise_trials(raws, uniques, n_true, pd_mean, mult=1.0):
    raws = np.asarray(raws, float)
    uniques = np.asarray(uniques, float)
    corr = raws / pd_mean
    corr_mult = raws / (pd_mean * mult)
    rel = (corr - n_true) / n_true
    rel_mult = (corr_mult - n_true) / n_true
    dbl = np.where(raws > 0, (raws - uniques) / np.maximum(raws, 1), 0.0)
    return {
        'raw_mean': raws.mean(), 'raw_std': raws.std(),
        'corr': corr, 'corr_mean': corr.mean(), 'corr_std': corr.std(),
        'corr_mult_mean': corr_mult.mean(), 'corr_mult_std': corr_mult.std(),
        'miss_mean': n_true - uniques.mean(),
        'miss_rate': 1.0 - uniques.mean() / n_true,
        'double_rate': dbl.mean(),
        'eps_card': abs(corr.mean() - n_true) / n_true,
        'bias': corr.mean() / n_true - 1.0,
        'bias_mult': corr_mult.mean() / n_true - 1.0,
        'mae': np.mean(np.abs(rel)), 'rmse_rel': np.sqrt(np.mean(rel ** 2)),
        'mae_mult': np.mean(np.abs(rel_mult)),
        'abs_err': np.abs(corr - n_true),
    }


def run_simulation():
    rng_res = np.random.default_rng(RESERVE_SEED)
    centroids, sizes, camo = generate_reserve(rng_res)
    n_true = int(sizes.sum())
    c_bar = float(camo.mean())

    # Common random numbers: the same individuals and uniforms for every strategy.
    rng_mc = np.random.default_rng(MC_SEED)
    trials = [draw_individuals(centroids, sizes, rng_mc) for _ in range(N_MONTE)]

    results = []
    for (h, v, label) in STRATEGIES:
        w, s, n, _ = strip_layout(h)
        L = path_length(h)
        pd_mean = detection_prob(h, v, c_bar)
        raws, uniq, first0 = [], [], None
        for k, (pos, gid, u) in enumerate(trials):
            raw, un, detected, fs = census(h, v, pos, gid, u, camo)
            raws.append(raw); uniq.append(un)
            if k == 0:
                first0 = (detected, fs)
        summ = summarise_trials(raws, uniq, n_true, pd_mean)
        # expected corrected count from the model (no sampling noise)
        e_corr = np.sum(sizes * detection_prob(h, v, camo)) / pd_mean
        # overlap sensitivity: 20 % side overlap, same individuals / uniforms
        mult = mean_multiplicity(h, P_OVERLAP)
        raws_o, uniq_o = [], []
        for (pos, gid, u) in trials:
            raw, un, _, _ = census(h, v, pos, gid, u, camo, overlap=P_OVERLAP)
            raws_o.append(raw); uniq_o.append(un)
        summ_o = summarise_trials(raws_o, uniq_o, n_true, pd_mean, mult)
        mission = fly_mission(h, v)
        mission_o = fly_mission(h, v, overlap=P_OVERLAP)
        results.append({
            'h': h, 'v': v, 'label': label, 'w': w, 'n_passes': n, 'L': L,
            'T_card': L / v, 'pd_mean': pd_mean, 'e_corr': e_corr,
            'delta_card': card_overlap_delta(h), **summ,
            'overlap': {**summ_o, 'mult': mult, 'n_passes': strip_layout(h, P_OVERLAP)[2],
                        'L': path_length(h, P_OVERLAP), 'T_card': path_length(h, P_OVERLAP) / v,
                        'mission': mission_o},
            'mission': mission, 'trial0': first0,
        })

    # per-trial winner (smallest absolute error); ties shared
    errs = np.array([r['abs_err'] for r in results])        # (3, N_MONTE)
    best = errs.min(axis=0)
    wins = [(np.isclose(errs[i], best) & (np.isclose(errs, best).sum(axis=0) == 1)).sum()
            for i in range(len(results))]
    ties = int((np.isclose(errs, best).sum(axis=0) > 1).sum())

    # random-reserve Monte Carlo: one census per reserve, CRN across strategies
    rr = {i: {'rel': []} for i in range(len(STRATEGIES))}
    bias_term = []
    for k in range(N_RESERVES):
        rng = np.random.default_rng(RES_SEED0 + k)
        cen, siz, cam = generate_reserve(rng)
        pos, gid, u = draw_individuals(cen, siz, rng)
        nt = siz.sum()
        bias_term.append(np.sum(siz * (1 - cam)) / ((1 - cam.mean()) * nt) - 1.0)
        for i, (h, v, _) in enumerate(STRATEGIES):
            raw, _, _, _ = census(h, v, pos, gid, u, cam)
            rr[i]['rel'].append(raw / detection_prob(h, v, cam.mean()) / nt - 1.0)
    rr_err = np.array([np.abs(rr[i]['rel']) for i in range(len(STRATEGIES))])
    rr_best = rr_err.min(axis=0)
    rr_wins = [int((np.isclose(rr_err[i], rr_best)
                    & (np.isclose(rr_err, rr_best).sum(axis=0) == 1)).sum())
               for i in range(len(STRATEGIES))]
    rr_ties = int((np.isclose(rr_err, rr_best).sum(axis=0) > 1).sum())
    random_res = {
        'rel': [np.array(rr[i]['rel']) for i in range(len(STRATEGIES))],
        'wins': rr_wins, 'ties': rr_ties, 'bias_term': np.array(bias_term),
    }

    reserve = {'centroids': centroids, 'sizes': sizes, 'camo': camo,
               'n_true': n_true, 'c_bar': c_bar, 'trial0': trials[0]}
    return reserve, results, {'wins': wins, 'ties': ties}, random_res


# ── Plots ─────────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_reserve_map(reserve, results, out_dir):
    cen, siz, cam = reserve['centroids'], reserve['sizes'], reserve['camo']
    pos, gid, _ = reserve['trial0']
    mid = results[1]
    fig, ax = plt.subplots(figsize=(7.5, 7))
    for yc in strip_layout(mid['h'])[3]:
        for yb in (yc - mid['w'] / 2, yc + mid['w'] / 2):
            ax.axhline(yb, color='grey', ls='--', lw=0.5, alpha=0.6)
    ax.scatter(pos[:, 0], pos[:, 1], s=4, c='k', alpha=0.5, label='individuals (trial 0)')
    sc = ax.scatter(cen[:, 0], cen[:, 1], s=30 + 25 * siz, c=cam, cmap='Greens_r',
                    vmin=0, vmax=0.6, edgecolors='k', alpha=0.75, label='group centroid')
    for g in range(len(siz)):
        ax.annotate(str(siz[g]), cen[g], ha='center', va='center', fontsize=7)
    fig.colorbar(sc, ax=ax, label='camouflage factor $c_g$')
    ax.plot(*HOME, 's', color='red', ms=10, label='launch point')
    ax.set_xlim(0, AREA_SIZE); ax.set_ylim(0, AREA_SIZE); ax.set_aspect('equal')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title(f'S057 reserve: {N_GROUPS} groups, $N^*$ = {reserve["n_true"]} animals\n'
                 f'dashed = mid-altitude strip edges (w = {mid["w"]:.1f} m, '
                 f'{mid["n_passes"]} passes)\nmarker size and label = group size', fontsize=10)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.09), ncol=3, fontsize=8)
    _save(fig, out_dir, 'reserve_map.png')


def plot_detection_surface(reserve, results, out_dir):
    hs = np.linspace(5, 50, 181)
    vs = np.linspace(1, 15, 141)
    H, V = np.meshgrid(hs, vs)
    P = detection_prob(H, V, reserve['c_bar'])
    fig, ax = plt.subplots(figsize=(7.5, 5.5))
    im = ax.pcolormesh(H, V, P, cmap='viridis', shading='auto')
    cs = ax.contour(H, V, P, levels=[0.05, 0.1, 0.2, 0.3, 0.4, 0.5], colors='w', linewidths=0.8)
    ax.clabel(cs, fmt='%.2f', fontsize=8)
    for r, col in zip(results, STRAT_COLORS):
        ax.plot(r['h'], r['v'], 'o', color=col, ms=11, mec='w', mew=1.5)
        ax.annotate(f"{r['label']}\n$P_d$={r['pd_mean']:.3f}", (r['h'], r['v']),
                    xytext=(8, 6), textcoords='offset points', color='w', fontsize=8)
    fig.colorbar(im, ax=ax, label=r'$\overline{P}_d(h, v, \bar{c})$')
    ax.set_xlabel('altitude h (m)'); ax.set_ylabel('ground speed v (m/s)')
    ax.set_title(rf'S057 detection probability, mean camouflage $\bar c$ = {reserve["c_bar"]:.3f}')
    _save(fig, out_dir, 'detection_surface.png')


def plot_count_distribution(reserve, results, random_res, out_dir):
    fig, axs = plt.subplots(1, 2, figsize=(12, 5))
    ax = axs[0]
    data = [r['corr'] for r in results]
    bp = ax.boxplot(data, patch_artist=True, widths=0.5)
    for patch, col in zip(bp['boxes'], STRAT_COLORS):
        patch.set_facecolor(col); patch.set_alpha(0.6)
    ax.axhline(reserve['n_true'], color='red', ls='--', lw=1.5, label=f'$N^*$ = {reserve["n_true"]}')
    for i, r in enumerate(results):
        ax.plot(i + 1, r['e_corr'], 'kD', ms=6, label='model expectation' if i == 0 else None)
    ax.set_xticks([1, 2, 3], [r['label'] for r in results])
    ax.set_ylabel(r'corrected count $\hat N_{corrected}$')
    ax.set_title(f'Card reserve, {N_MONTE} CRN trials')
    ax.legend(fontsize=8)
    ax = axs[1]
    data = [100 * x for x in random_res['rel']]
    bp = ax.boxplot(data, patch_artist=True, widths=0.5)
    for patch, col in zip(bp['boxes'], STRAT_COLORS):
        patch.set_facecolor(col); patch.set_alpha(0.6)
    ax.axhline(0, color='red', ls='--', lw=1.5)
    ax.set_xticks([1, 2, 3], [r['label'] for r in results])
    ax.set_ylabel(r'relative error $(\hat N - N^*)/N^*$ (%)')
    ax.set_title(f'{N_RESERVES} random reserves, one census each (CRN)')
    fig.suptitle('S057 population estimates')
    _save(fig, out_dir, 'count_distribution.png')


def plot_miss_vs_time(reserve, results, out_dir):
    fig, ax = plt.subplots(figsize=(7.5, 5.5))
    T = [r['T_card'] for r in results]
    Tt = [r['mission']['t_total'] for r in results]
    M = [r['miss_mean'] for r in results]
    ax.plot(T, M, 'k-', lw=1, alpha=0.6, label='Pareto front (survey time)')
    for r, col, tt in zip(results, STRAT_COLORS, Tt):
        ax.plot(r['T_card'], r['miss_mean'], 'o', color=col, ms=11, mec='k')
        ax.plot(tt, r['miss_mean'], 's', color=col, ms=9, mfc='none', mew=2)
        ax.plot([r['T_card'], tt], [r['miss_mean']] * 2, ':', color=col)
        ax.annotate(f"{r['label']}\n{r['mission']['sorties']} sortie(s)",
                    (r['T_card'], r['miss_mean']), xytext=(6, 8), textcoords='offset points',
                    fontsize=8)
    ax.plot([], [], 'ko', label=r'$T_{mission}$ = L / v (card)')
    ax.plot([], [], 'ks', mfc='none', label='incl. transits + battery swaps')
    ax.set_xscale('log')
    ax.set_xlabel('mission time (s, log scale)')
    ax.set_ylabel(f'mean missed animals (of {reserve["n_true"]})')
    ax.set_title('S057 miss count vs mission time')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, which='both')
    _save(fig, out_dir, 'miss_vs_time.png')


def plot_overlap_sensitivity(results, out_dir):
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.8))
    x = np.arange(len(results))
    ax = axs[0]
    b0 = [100 * r['bias'] for r in results]
    b1 = [100 * r['overlap']['bias'] for r in results]
    b2 = [100 * r['overlap']['bias_mult'] for r in results]
    ax.bar(x - 0.27, b0, 0.27, color='#888888', label='no overlap, card estimator')
    ax.bar(x, b1, 0.27, color='#D62728', label='20 % overlap, card estimator')
    ax.bar(x + 0.27, b2, 0.27, color='#2CA02C', label='20 % overlap, / mean multiplicity')
    ax.axhline(0, color='k', lw=0.8)
    ax.set_xticks(x, [r['label'] for r in results])
    ax.set_ylabel('bias of mean estimate (%)')
    ax.set_title(f'Estimator bias ({N_MONTE} CRN trials)')
    ax.legend(fontsize=8)
    ax = axs[1]
    d1 = [100 * r['overlap']['double_rate'] for r in results]
    m0 = [100 * r['miss_rate'] for r in results]
    m1 = [100 * r['overlap']['miss_rate'] for r in results]
    ax.bar(x - 0.27, m0, 0.27, color='#888888', label='miss rate, no overlap')
    ax.bar(x, m1, 0.27, color='#1F77B4', label='miss rate, 20 % overlap')
    ax.bar(x + 0.27, d1, 0.27, color='#D62728', label='double-count rate, 20 % overlap')
    ax.set_xticks(x, [r['label'] for r in results])
    ax.set_ylabel('%')
    ax.set_title('Miss and double-count rates')
    ax.legend(fontsize=8)
    fig.suptitle('S057 side-overlap sensitivity')
    _save(fig, out_dir, 'overlap_sensitivity.png')


def plot_survey_geometry_3d(results, out_dir):
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')
    x0, x1 = 0.0, 120.0
    for r, col in zip(results, STRAT_COLORS):
        h, w = r['h'], r['w']
        yc = w / 2.0
        ax.plot([x0, x1], [yc, yc], [h, h], color=col, lw=2.5, label=f"{r['label']} (h = {h:.0f} m, w = {w:.1f} m)")
        xd = 60.0
        for sgn in (-1, 1):
            ax.plot([xd, xd], [yc, yc + sgn * w / 2], [h, 0], color=col, lw=0.8)
        ax.add_collection3d(Poly3DCollection(
            [[(x0, 0, 0), (x1, 0, 0), (x1, w, 0), (x0, w, 0)]], facecolor=col, alpha=0.18,
            edgecolor=col))
        ax.scatter([xd], [yc], [h], color=col, s=40, edgecolors='k')
    ax.set_xlim(0, 120); ax.set_ylim(0, 60); ax.set_zlim(0, 45)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('altitude (m)')
    ax.view_init(elev=22, azim=-60)
    ax.set_box_aspect(None, zoom=0.85)
    ax.set_title('S057 camera footprint geometry: first strip of each strategy\n'
                 r'(lines from the drone = $\pm 25°$ field-of-view edges)')
    ax.legend(fontsize=8, loc='upper left')
    _save(fig, out_dir, 'survey_geometry_3d.png')


def save_animation(reserve, results, out_dir):
    """Top-down replay of the mid-altitude census (trial 0)."""
    r = results[1]
    m = fly_mission(r['h'], r['v'], record=True)
    traj = m['traj']
    pos, gid, _ = reserve['trial0']
    detected, first_strip = r['trial0']
    # detection time: first time the drone, surveying strip j, passes the animal's x
    t_det = np.full(len(pos), np.inf)
    for i in np.flatnonzero(detected):
        sel = np.flatnonzero((traj[:, 3] == 0) & (traj[:, 4] == first_strip[i]))
        k = sel[np.argmin(np.abs(traj[sel, 1] - pos[i, 0]))]
        t_det[i] = traj[k, 0]
    n = len(traj)
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    idx = [min(f * ANIM_STEP, n - 1) for f in range(n_frames)]
    assert len(set(idx)) == n_frames

    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.set_xlim(-10, AREA_SIZE + 10); ax.set_ylim(-10, AREA_SIZE + 10); ax.set_aspect('equal')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.plot(*HOME, 's', color='red', ms=9)
    hollow = ax.scatter(pos[:, 0], pos[:, 1], s=14, facecolors='none', edgecolors='green', lw=0.8)
    filled = ax.scatter([], [], s=14, color='green')
    trail, = ax.plot([], [], '-', color='red', lw=0.6, alpha=0.5)
    drone, = ax.plot([], [], 'o', color='red', ms=8, mec='k')
    foot = Rectangle((0, 0), r['w'], r['w'], fc='red', alpha=0.25, ec='red')
    ax.add_patch(foot)
    title = ax.set_title('')

    def update(f):
        k = idx[f]
        t, x, y, mode = traj[k, 0], traj[k, 1], traj[k, 2], int(traj[k, 3])
        trail.set_data(traj[:k + 1, 1], traj[:k + 1, 2])
        drone.set_data([x], [y])
        foot.set_visible(mode == 0)
        foot.set_xy((x - r['w'] / 2, y - r['w'] / 2))
        seen = t_det <= t
        filled.set_offsets(pos[seen] if seen.any() else np.empty((0, 2)))
        hollow.set_offsets(pos[~seen] if (~seen).any() else np.empty((0, 2)))
        state = {0: 'surveying', 1: 'transit', 2: 'battery swap'}[mode]
        title.set_text(f'S057 mid-altitude census (h = {r["h"]:.0f} m, v = {r["v"]:.0f} m/s)\n'
                       f't = {t:6.0f} s  {state}  detected {int(seen.sum())}/{len(pos)}')
        return [trail, drone, foot, filled, hollow, title]

    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=15, dpi=70)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames from {n} trajectory points, step {ANIM_STEP})')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    reserve, results, wins, random_res = run_simulation()
    nt = reserve['n_true']

    print('\n=== S057 Wildlife Population Census — Results ===')
    print(f"  Reserve (seed {RESERVE_SEED}): N* = {nt} animals in {N_GROUPS} groups "
          f"(sizes {reserve['sizes'].min()}-{reserve['sizes'].max()}), "
          f"mean camouflage c_bar = {reserve['c_bar']:.3f}, "
          f"population-weighted camouflage = {np.sum(reserve['sizes'] * reserve['camo']) / nt:.3f}")
    print(f"  Expected bias of N_raw / P_d(c_bar) on this reserve (camouflage weighting): "
          f"{100 * (results[0]['e_corr'] / nt - 1):+.1f}% (same for every strategy)")
    print(f"  Battery: endurance {E_BATT:.0f} s, reserve {T_RESERVE:.0f} s, "
          f"swap {T_SWAP:.0f} s, transit {V_TRANSIT:.0f} m/s")
    for r in results:
        print(f"  [{r['label']}]  h = {r['h']:.0f} m, v = {r['v']:.0f} m/s")
        print(f"    strip width {r['w']:.2f} m, passes {r['n_passes']}, path {r['L']:.0f} m, "
              f"T_mission (card) {r['T_card']:.0f} s = {r['T_card'] / 60:.1f} min")
        print(f"    with battery: {r['mission']['sorties']} sortie(s), total "
              f"{r['mission']['t_total']:.0f} s = {r['mission']['t_total'] / 3600:.2f} h "
              f"(survey {r['mission']['t_survey']:.0f} s, transit {r['mission']['t_transit']:.0f} s)")
        print(f"    mean P_d {r['pd_mean']:.4f}; card overlap delta = {r['delta_card']:.3f}")
        print(f"    raw count {r['raw_mean']:.1f} +/- {r['raw_std']:.1f}; corrected "
              f"{r['corr_mean']:.1f} +/- {r['corr_std']:.1f} (model expectation {r['e_corr']:.1f}), "
              f"range {r['corr'].min():.1f}-{r['corr'].max():.1f}, scale factor 1/P_d = "
              f"{1 / r['pd_mean']:.1f}")
        print(f"    missed {r['miss_mean']:.1f} animals (miss rate {100 * r['miss_rate']:.1f}%), "
              f"double-count rate {100 * r['double_rate']:.1f}%")
        print(f"    card error eps = |mean - N*|/N* = {100 * r['eps_card']:.1f}%; per-survey "
              f"mean |error| {100 * r['mae']:.1f}%, RMS error {100 * r['rmse_rel']:.1f}%")
    print(f"  Per-trial wins (smallest |N_hat - N*|, {N_MONTE} trials): " +
          ', '.join(f"{r['label']} {w}" for r, w in zip(results, wins['wins'])) +
          f", ties {wins['ties']}")
    print(f'\n  --- {int(P_OVERLAP * 100)} % side-overlap sensitivity (same individuals / uniforms) ---')
    for r in results:
        o = r['overlap']
        print(f"  [{r['label']}] passes {o['n_passes']}, path {o['L']:.0f} m, T_mission "
              f"{o['T_card']:.0f} s, {o['mission']['sorties']} sortie(s), total "
              f"{o['mission']['t_total']:.0f} s; mean multiplicity {o['mult']:.3f}")
        print(f"    double-count rate {100 * o['double_rate']:.1f}%, miss rate "
              f"{100 * o['miss_rate']:.1f}%; bias card estimator {100 * o['bias']:+.1f}%, "
              f"multiplicity-corrected {100 * o['bias_mult']:+.1f}%")
    print(f'\n  --- {N_RESERVES} random reserves, one census each (CRN) ---')
    bt = random_res['bias_term']
    print(f"  camouflage-weighting bias term: mean {100 * bt.mean():+.1f}%, "
          f"std {100 * bt.std():.1f}%, range {100 * bt.min():+.1f}% to {100 * bt.max():+.1f}%")
    for r, rel, w in zip(results, random_res['rel'], random_res['wins']):
        print(f"  [{r['label']}] relative error mean {100 * rel.mean():+.1f}% "
              f"+/- {100 * rel.std():.1f}%, mean |error| {100 * np.mean(np.abs(rel)):.1f}%, "
              f"wins {w}")
    print(f"  ties {random_res['ties']}")
    print()

    plot_reserve_map(reserve, results, out)
    plot_detection_surface(reserve, results, out)
    plot_count_distribution(reserve, results, random_res, out)
    plot_miss_vs_time(reserve, results, out)
    plot_overlap_sensitivity(results, out)
    plot_survey_geometry_3d(results, out)
    save_animation(reserve, results, out)
