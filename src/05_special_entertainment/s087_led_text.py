"""
S087 Aerial Light Show Text Display
===================================
A drone swarm spells "DRONE" in a 6 x 5 bitmap font (2 m pixel pitch) at z = 50 m, then
morphs to "SHOW". Drones are matched to lit pixels with the Hungarian algorithm and fly
with a PID acceleration command (|a| <= 3 m/s^2, |v| <= 5 m/s). During each transition a
priority rule lets the drone with the longer remaining path go first: a drone within
r_sep = 1.5 m of a higher-priority drone is limited to 50 % speed.

The card's font gives 74 lit pixels for "DRONE" and 58 for "SHOW" (not 30 each), so the
fleet has 74 drones; for "SHOW" the 16 spare drones switch their LEDs off and park in a
row 5 m below the display.

Configurations (same staging layout for every configuration in each seed):
  (1) Hungarian + priority yield     (card)
  (2) Hungarian, no avoidance        (added)
  (3) Nearest-neighbour greedy + priority yield (card's cost baseline, here also flown)

Usage:
    conda activate drones
    python src/05_special_entertainment/s087_led_text.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.optimize import linear_sum_assignment

# ── Parameters (card) ─────────────────────────────────────────────────────────
Z_SHOW = 50.0
D_PX = 2.0
FONT_H, FONT_W = 6, 5
GAP_COLS = 2
V_MAX = 5.0
A_MAX = 3.0
R_SEP = 1.5
YIELD_FACTOR = 0.5
EPS_POS = 0.5
DT = 0.05
KP, KI, KD = 2.0, 0.05, 0.8
V_ACCEPT = 0.95
COLLISION_RADIUS = 0.5    # domain constant

FONT = {
    'D': [[1,1,1,0,0],[1,0,0,1,0],[1,0,0,1,0],[1,0,0,1,0],[1,0,0,1,0],[1,1,1,0,0]],
    'R': [[1,1,1,0,0],[1,0,0,1,0],[1,1,1,0,0],[1,0,1,0,0],[1,0,0,1,0],[1,0,0,0,1]],
    'O': [[0,1,1,1,0],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1],[0,1,1,1,0]],
    'N': [[1,0,0,0,1],[1,1,0,0,1],[1,0,1,0,1],[1,0,0,1,1],[1,0,0,0,1],[1,0,0,0,1]],
    'E': [[1,1,1,1,1],[1,0,0,0,0],[1,1,1,1,0],[1,0,0,0,0],[1,0,0,0,0],[1,1,1,1,1]],
    'S': [[0,1,1,1,1],[1,0,0,0,0],[0,1,1,0,0],[0,0,0,1,0],[0,0,0,0,1],[1,1,1,1,0]],
    'H': [[1,0,0,0,1],[1,0,0,0,1],[1,1,1,1,1],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1]],
    'W': [[1,0,0,0,1],[1,0,0,0,1],[1,0,1,0,1],[1,0,1,0,1],[1,1,0,1,1],[1,0,0,0,1]],
}

# ── Parameters (added) ────────────────────────────────────────────────────────
T_PHASE = 40.0            # s simulated per transition
PARK_Z = Z_SHOW - 5.0     # m — row of dark spare drones during "SHOW"
PARK_Y = 5.0              # m
STAGE_LO = np.array([0.0, -2.0, 0.5])
STAGE_HI = np.array([64.0, 12.0, 5.0])
STAGE_MIN_SEP = 1.5       # m — launch pads at least r_sep apart
N_MC = 20
SHOW_SEED = 0
ANIM_STEP = 4
ANIM_TAIL = 2.0

N_PHASE = int(round(T_PHASE / DT))
CONFIGS = ['hung', 'hung_free', 'greedy']
LABELS = {'hung': 'Hungarian + priority yield', 'hung_free': 'Hungarian, no avoidance',
          'greedy': 'NN greedy + priority yield'}
COLORS = {'hung': 'tab:red', 'hung_free': 'tab:grey', 'greedy': 'tab:orange'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '05_special_entertainment', 's087_led_text')


def word_to_positions(word):
    pts = []
    x_off = 0
    for ch in word:
        bm = np.array(FONT[ch])
        for i in range(FONT_H):
            for j in range(FONT_W):
                if bm[i, j]:
                    pts.append([(x_off + j) * D_PX, (FONT_H - 1 - i) * D_PX, Z_SHOW])
        x_off += FONT_W + GAP_COLS
    return np.array(pts)


def cost_matrix(a, b):
    return np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)


def hungarian(starts, targets):
    r, c = linear_sum_assignment(cost_matrix(starts, targets))
    out = np.empty(len(starts), dtype=int)
    out[r] = c
    return out


def nn_greedy(starts, targets):
    """Naive nearest-neighbour: drones in index order take the nearest free target."""
    C = cost_matrix(starts, targets)
    out = np.empty(len(starts), dtype=int)
    for k in range(len(starts)):
        m = int(np.argmin(C[k]))
        out[k] = m
        C[:, m] = np.inf
    return out


def staging(seed, n):
    rng = np.random.default_rng(seed)
    pts = []
    while len(pts) < n:
        p = rng.uniform(STAGE_LO, STAGE_HI)
        if all(np.linalg.norm(p - q) >= STAGE_MIN_SEP for q in pts):
            pts.append(p)
    return np.array(pts)


def fly(pos0, vel0, goals, lit, avoid, log=False):
    """Fly every drone to its goal for T_PHASE. lit: bool mask of drones that show a pixel."""
    n = len(pos0)
    pos, vel = pos0.copy(), vel0.copy()
    integ = np.zeros_like(pos)
    iu = np.triu_indices(n, 1)
    ever_close = np.zeros(len(iu[0]), dtype=bool)
    min_sep = np.inf
    fid = np.zeros(N_PHASE + 1)
    fid[0] = np.mean(np.linalg.norm(pos[lit] - goals[lit], axis=1) <= EPS_POS)
    t_all = None
    yield_steps = 0
    bind_steps = 0
    a_peak = 0.0
    pair_min = np.full(len(iu[0]), np.inf)
    pair_tmin = np.zeros(len(iu[0]))
    hist = np.zeros((N_PHASE + 1, n, 3)) if log else None
    ylog = np.zeros((N_PHASE + 1, n), dtype=bool) if log else None
    if log:
        hist[0] = pos
    for k in range(N_PHASE):
        err = goals - pos
        rem = np.linalg.norm(err, axis=1)
        d = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
        dp = d[iu]
        min_sep = min(min_sep, float(dp.min()))
        ever_close |= dp < COLLISION_RADIUS
        closer = dp < pair_min
        pair_min = np.where(closer, dp, pair_min)
        pair_tmin = np.where(closer, k * DT, pair_tmin)
        vcap = np.full(n, V_MAX)
        if avoid:
            rank = np.empty(n, dtype=int)
            rank[np.argsort(-rem, kind='stable')] = np.arange(n)
            higher = rank[None, :] < rank[:, None]          # j has priority over k
            yld = np.any((d < R_SEP) & higher, axis=1)
            vcap[yld] *= YIELD_FACTOR
            yield_steps += int(yld.sum())
            if log:
                ylog[k] = yld
        integ_new = integ + err * DT
        a = KP * err + KI * integ_new - KD * vel
        an = np.linalg.norm(a, axis=1)
        sat = an > A_MAX
        integ = np.where(sat[:, None], integ, integ_new)        # conditional integration
        a = KP * err + KI * integ - KD * vel
        an = np.linalg.norm(a, axis=1, keepdims=True)
        a = np.where(an > A_MAX, a / np.maximum(an, 1e-12) * A_MAX, a)
        v_tgt = vel + a * DT
        vn = np.linalg.norm(v_tgt, axis=1, keepdims=True)
        over = vn[:, 0] > vcap
        v_tgt = np.where(over[:, None], v_tgt / np.maximum(vn, 1e-12) * vcap[:, None], v_tgt)
        if avoid:
            bind_steps += int(np.sum(over & yld & (vn[:, 0] > YIELD_FACTOR * V_MAX)))
        # the speed cap is reached by braking at no more than A_MAX (no instant velocity jump)
        dv = v_tgt - vel
        dvn = np.linalg.norm(dv, axis=1, keepdims=True)
        dv = np.where(dvn > A_MAX * DT, dv / np.maximum(dvn, 1e-12) * A_MAX * DT, dv)
        vel_new = vel + dv
        a_real = np.linalg.norm(vel_new - vel, axis=1) / DT
        assert np.all(a_real <= A_MAX + 1e-9)                    # realised acceleration
        assert np.all(np.linalg.norm(vel_new, axis=1) <= V_MAX + 1e-9)
        a_peak = max(a_peak, float(a_real.max()))
        vel = vel_new
        pos = pos + vel * DT
        assert np.all(pos[:, 2] > 0.0)
        e = np.linalg.norm(pos - goals, axis=1)
        fid[k + 1] = np.mean(e[lit] <= EPS_POS)
        if t_all is None and e.max() <= EPS_POS:
            t_all = (k + 1) * DT
        if log:
            hist[k + 1] = pos
    above = np.where(fid >= V_ACCEPT)[0]
    res = {'pos': pos, 'vel': vel, 'fid': fid, 't_all': t_all,
           't_v95': float(above[0] * DT) if len(above) else None,
           'min_sep': min_sep, 'pairs_close': int(ever_close.sum()),
           'yield_frac': yield_steps / (N_PHASE * n),
           'yield_steps': yield_steps, 'bind_steps': bind_steps, 'a_peak': a_peak,
           'pair_min': pair_min, 'pair_tmin': pair_tmin, 'pairs': iu,
           'final_max': float(np.linalg.norm(pos - goals, axis=1).max())}
    if log:
        res['hist'] = hist
        res['yield'] = ylog
    return res


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _segments_cross(p1, p2, q1, q2, tol=1e-6):
    """Proper intersection of two 2D segments (collinear overlaps excluded)."""
    d1, d2 = p2 - p1, q2 - q1
    den = _cross2(d1, d2)
    if abs(den) < tol:
        return False
    t = _cross2(q1 - p1, d2) / den
    u = _cross2(q1 - p1, d1) / den
    return 0.0 < t < 1.0 and 0.0 < u < 1.0


def _point_seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def overshoot(hist, start, goal, still=0.1):
    """Distance (m) each drone has passed beyond its goal along its travel direction, per step."""
    d = goal - start
    n = np.linalg.norm(d, axis=1, keepdims=True)
    u = np.where(n > still, d / np.maximum(n, 1e-12), 0.0)
    return np.einsum('tnd,nd->tn', hist - goal[None], u)


def classify_close_pairs(r, still=0.1):
    """Classify the morph's close pairs (< COLLISION_RADIUS) by what the two drones are doing,
    evaluated at the moment of closest approach (first matching class wins)."""
    ph = r['ph2']
    start, goal, lit = r['ph1']['pos'], r['B'][r['sB']], r['lit_B']
    disp = np.linalg.norm(goal - start, axis=1)
    moving = disp >= still
    ov = overshoot(ph['hist'], start, goal, still)
    iu = ph['pairs']
    cats = {'spare drone involved': [],
            'a drone has overshot its pixel by > 0.5 m': [],
            'moving drone passes a stationary lit drone': [],
            'follower on the same row': [],
            "path runs within 0.5 m of the other's pixel": [],
            'straight paths cross': [], 'other': []}
    for p in np.where(ph['pair_min'] < COLLISION_RADIUS)[0]:
        i, j = iu[0][p], iu[1][p]
        k = int(round(ph['pair_tmin'][p] / DT))
        rec = (int(i), int(j), float(ph['pair_min'][p]), float(ph['pair_tmin'][p]))
        di = (goal[i] - start[i])[:2]
        dj = (goal[j] - start[j])[:2]
        if not (lit[i] and lit[j]):
            cats['spare drone involved'].append(rec)
        elif max(ov[k, i], ov[k, j]) > COLLISION_RADIUS:
            cats['a drone has overshot its pixel by > 0.5 m'].append(rec)
        elif not (moving[i] and moving[j]):
            cats['moving drone passes a stationary lit drone'].append(rec)
        elif (abs(_cross2(di / np.linalg.norm(di), dj / np.linalg.norm(dj))) < 1e-3
              and abs(_cross2(di / np.linalg.norm(di), (start[j] - start[i])[:2])) < still):
            cats['follower on the same row'].append(rec)
        elif min(_point_seg_dist(goal[j], start[i], goal[i]), _point_seg_dist(goal[i], start[j], goal[j]),
                 _point_seg_dist(start[j], start[i], goal[i]),
                 _point_seg_dist(start[i], start[j], goal[j])) < COLLISION_RADIUS:
            cats["path runs within 0.5 m of the other's pixel"].append(rec)
        elif _segments_cross(start[i, :2], goal[i, :2], start[j, :2], goal[j, :2]):
            cats['straight paths cross'].append(rec)
        else:
            cats['other'].append(rec)
    lm = np.where(lit & moving)[0]
    n_cross = sum(_segments_cross(start[a, :2], goal[a, :2], start[b, :2], goal[b, :2])
                  for ia, a in enumerate(lm) for b in lm[ia + 1:])
    mov = np.where(moving)[0]
    ov_max = ov[:, mov].max(axis=0)
    ov_stats = (float(ov_max.max()), float(ov_max.mean()), int(np.sum(ov_max > COLLISION_RADIUS)),
                len(mov), int(mov[np.argmax(ov_max)]))
    return (cats, int(n_cross), int(np.sum(lit & ~moving)), int(np.sum(lit)),
            int(np.sum(lit & moving)), ov_stats)


def run_mission(seed, config, words, log=False):
    A, B_lit = words
    n = len(A)
    start = staging(seed, n)
    park = np.column_stack([np.arange(n - len(B_lit)) * D_PX, np.full(n - len(B_lit), PARK_Y),
                            np.full(n - len(B_lit), PARK_Z)])
    B = np.vstack([B_lit, park])
    lit_B = np.arange(n) < len(B_lit)
    assign = nn_greedy if config == 'greedy' else hungarian
    avoid = config != 'hung_free'

    sA = assign(start, A)
    costA = float(np.linalg.norm(start - A[sA], axis=1).sum())
    ph1 = fly(start, np.zeros_like(start), A[sA], np.ones(n, dtype=bool), avoid, log)
    sB = assign(ph1['pos'], B)
    costB = float(np.linalg.norm(ph1['pos'] - B[sB], axis=1).sum())
    ph2 = fly(ph1['pos'], ph1['vel'], B[sB], lit_B[sB], avoid, log)
    alt = {'costA_h': float(np.linalg.norm(start - A[hungarian(start, A)], axis=1).sum()),
           'costA_g': float(np.linalg.norm(start - A[nn_greedy(start, A)], axis=1).sum()),
           'costB_h': float(np.linalg.norm(ph1['pos'] - B[hungarian(ph1['pos'], B)], axis=1).sum()),
           'costB_g': float(np.linalg.norm(ph1['pos'] - B[nn_greedy(ph1['pos'], B)], axis=1).sum())}
    return {'start': start, 'A': A, 'B': B, 'sA': sA, 'sB': sB, 'lit_B': lit_B[sB],
            'costA': costA, 'costB': costB, 'ph1': ph1, 'ph2': ph2, **alt}


def run_simulation():
    A = word_to_positions('DRONE')
    B = word_to_positions('SHOW')
    show = {c: run_mission(SHOW_SEED, c, (A, B), log=True) for c in CONFIGS}
    keys = ['costA', 'costB', 'A_t95', 'A_tall', 'A_minsep', 'A_pairs', 'B_t95', 'B_tall',
            'B_minsep', 'B_pairs', 'B_yield']
    mc = {c: {k: [] for k in keys} for c in CONFIGS}
    for seed in range(N_MC):
        for c in CONFIGS:
            r = run_mission(seed, c, (A, B))
            m = mc[c]
            m['costA'].append(r['costA'])
            m['costB'].append(r['costB'])
            for ph, tag in [('ph1', 'A'), ('ph2', 'B')]:
                p = r[ph]
                m[f'{tag}_t95'].append(np.nan if p['t_v95'] is None else p['t_v95'])
                m[f'{tag}_tall'].append(np.nan if p['t_all'] is None else p['t_all'])
                m[f'{tag}_minsep'].append(p['min_sep'])
                m[f'{tag}_pairs'].append(p['pairs_close'])
            m['B_yield'].append(r['ph2']['yield_frac'])
    mc = {c: {k: np.array(v) for k, v in d.items()} for c, d in mc.items()}
    return A, B, show, mc


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_formations(show, out_dir):
    r = show['hung']
    fig, axes = plt.subplots(2, 1, figsize=(14, 7.5))
    for ax, (ph, tgt, word, lit) in zip(axes, [
            ('ph1', r['A'][r['sA']], 'DRONE', np.ones(len(r['sA']), dtype=bool)),
            ('ph2', r['B'][r['sB']], 'SHOW', r['lit_B'])]):
        p = r[ph]['pos']
        ax.scatter(tgt[lit, 0], tgt[lit, 1], s=220, color='grey', alpha=0.35,
                   label=f'Lit pixel (target, z = {Z_SHOW:.0f} m)')
        ax.scatter(p[lit, 0], p[lit, 1], s=22, color='red', label='Drone, LED on')
        if (~lit).any():
            ax.scatter(p[~lit, 0], p[~lit, 1], s=30, color='k', marker='x',
                       label=f'Spare drone, LED off (z = {PARK_Z:.0f} m)')
        err = np.linalg.norm(p - tgt, axis=1)
        ax.set_title(f'"{word}" at the end of its {T_PHASE:.0f} s phase — max error '
                     f'{100 * err.max():.1f} cm (top-down view)')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_xlim(-3, 67)
        ax.set_ylim(-3, 14)
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='upper right', ncol=3)
    fig.suptitle(f'S087 Formations, Hungarian + priority yield (seed {SHOW_SEED})')
    plt.tight_layout()
    path = os.path.join(out_dir, 'formations.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_assignment_cost(show, mc, out_dir):
    r = show['hung']
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 4.8))
    x = np.arange(2)
    hv = [r['costA_h'], r['costB_h']]
    gv = [r['costA_g'], r['costB_g']]
    ax.bar(x - 0.18, hv, 0.36, color='tab:red', label='Hungarian')
    ax.bar(x + 0.18, gv, 0.36, color='tab:orange', label='NN greedy')
    for i in range(2):
        ax.text(x[i] + 0.18, gv[i], f'+{100 * (gv[i] / hv[i] - 1):.1f} %', ha='center',
                va='bottom', fontsize=9)
    ax.set_xticks(x, ['ground → "DRONE"', '"DRONE" → "SHOW"'])
    ax.set_ylabel('Total assignment distance (m)')
    ax.set_title(f'Assignment cost, seed {SHOW_SEED} (label = Δ_cost of greedy)')
    ax.legend()
    ax.grid(alpha=0.3, axis='y')
    dA = 100 * (mc['greedy']['costA'] / mc['hung']['costA'] - 1)
    dB = 100 * (mc['greedy']['costB'] / mc['hung']['costB'] - 1)
    ax2.boxplot([dA, dB], showmeans=True)
    ax2.set_xticks([1, 2], ['ground → "DRONE"', '"DRONE" → "SHOW"'])
    ax2.set_ylabel('Δ_cost of NN greedy over Hungarian (%)')
    ax2.set_title(f'Δ_cost over {N_MC} staging layouts')
    ax2.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'assignment_cost.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_fidelity(show, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8), sharey=True)
    t = np.arange(N_PHASE + 1) * DT
    for ax, ph, title in [(axes[0], 'ph1', 'Ascent: ground → "DRONE"'),
                          (axes[1], 'ph2', 'Morph: "DRONE" → "SHOW"')]:
        for c in CONFIGS:
            p = show[c][ph]
            lab = LABELS[c] + (f" (V ≥ 0.95 at {p['t_v95']:.2f} s)" if p['t_v95'] is not None else '')
            ax.plot(t, p['fid'], color=COLORS[c], lw=1.4, label=lab)
        ax.axhline(V_ACCEPT, color='k', ls='--', lw=1, label='V = 0.95')
        ax.set_xlabel('Time in phase (s)')
        ax.set_title(title)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='lower right')
    axes[0].set_ylabel('Visual fidelity V')
    fig.suptitle(f'S087 Visual fidelity (share of lit pixels with a drone within {EPS_POS} m), '
                 f'seed {SHOW_SEED}')
    plt.tight_layout()
    path = os.path.join(out_dir, 'fidelity.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.6))
    items = [('B_t95', 'Morph: time to V ≥ 0.95 (s)'), ('B_tall', 'Morph: time all within 0.5 m (s)'),
             ('B_minsep', 'Morph: min separation (m)'), ('A_minsep', 'Ascent: min separation (m)')]
    for ax, (key, lab) in zip(axes, items):
        bp = ax.boxplot([mc[c][key][~np.isnan(mc[c][key])] for c in CONFIGS], showmeans=True,
                        patch_artist=True)
        for p, c in zip(bp['boxes'], CONFIGS):
            p.set_facecolor(COLORS[c])
            p.set_alpha(0.4)
        ax.set_xticks([1, 2, 3], ['Hungarian\n+ yield', 'Hungarian\nfree', 'NN greedy\n+ yield'])
        ax.set_ylabel(lab)
        ax.grid(alpha=0.3)
        if 'minsep' in key:
            ax.axhline(COLLISION_RADIUS, color='red', ls='--', lw=1)
    fig.suptitle(f'S087 Monte Carlo over {N_MC} staging layouts (same layout for every '
                 'configuration; triangle = mean; red = 0.5 m collision radius)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(show, out_dir):
    r = show['hung']
    p2 = r['ph2']
    hist, ylog = p2['hist'], p2['yield']
    t_end = T_PHASE if p2['t_all'] is None else min(T_PHASE, p2['t_all'] + ANIM_TAIL)
    n_used = int(round(t_end / DT)) + 1
    n_frames = int(np.ceil((n_used - 1) / ANIM_STEP)) + 1
    idx = [min(i * ANIM_STEP, n_used - 1) for i in range(n_frames)]
    lit = r['lit_B']
    A_t, B_t = r['A'][r['sA']], r['B'][r['sB']]
    fig = plt.figure(figsize=(11, 5.5))
    ax = fig.add_subplot(111, projection='3d')
    fig.subplots_adjust(left=0, right=1, bottom=0, top=0.92)
    ax.scatter(B_t[lit, 0], B_t[lit, 1], B_t[lit, 2], s=60, color='grey', alpha=0.25)
    sc = ax.scatter(hist[0, :, 0], hist[0, :, 1], hist[0, :, 2], s=14, depthshade=False)
    trails = [ax.plot([], [], [], color='red', lw=0.6, alpha=0.4)[0] for _ in range(len(lit))]
    txt = ax.text2D(0.02, 0.92, '', transform=ax.transAxes, fontsize=9)
    ax.set_xlim(-2, 66)
    ax.set_ylim(-4, 14)
    ax.set_zlim(PARK_Z - 2, Z_SHOW + 2)
    ax.set_zticks([PARK_Z, Z_SHOW])
    ax.set_box_aspect((3.4, 1.0, 0.6), zoom=1.4)
    ax.view_init(elev=35, azim=-75)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_title('S087 Morph "DRONE" → "SHOW" (red = LED on, pink = yielding, black = spare)')

    def update(fi):
        k = idx[fi]
        p = hist[k]
        cols = np.array(['red'] * len(lit), dtype=object)
        cols[ylog[k]] = 'pink'
        cols[~lit] = 'black'
        sc._offsets3d = (p[:, 0], p[:, 1], p[:, 2])
        sc.set_color(list(cols))
        k0 = max(0, k - 20)
        for j, tr in enumerate(trails):
            tr.set_data(hist[k0:k + 1, j, 0], hist[k0:k + 1, j, 1])
            tr.set_3d_properties(hist[k0:k + 1, j, 2])
        txt.set_text(f't = {k * DT:5.2f} s   V = {p2["fid"][k]:.2f}   yielding: {int(ylog[k].sum())}')
        return [sc, txt] + trails

    ani = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=72)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    A, B, show, mc = run_simulation()
    print('\n=== S087 Aerial Light Show Text Display — Results ===')
    for w in ['DRONE', 'SHOW']:
        counts = [int(np.sum(FONT[ch])) for ch in w]
        print(f'  "{w}": lit pixels per glyph {counts}, total {sum(counts)} (card claims 30)')
    print(f'  Fleet N = {len(A)}; "SHOW" leaves {len(A) - len(B)} spare drones parked dark at '
          f'z = {PARK_Z:.0f} m; word widths {A[:, 0].max():.0f} m / {B[:, 0].max():.0f} m')
    print(f'  Showcase seed {SHOW_SEED}:')
    print(f"  {'Configuration':<30}{'phase':>8}{'cost (m)':>10}{'t V>=0.95 (s)':>15}"
          f"{'t all<0.5m (s)':>16}{'min sep (m)':>13}{'pairs<0.5m':>12}{'yield %':>9}{'final max (cm)':>16}")
    for c in CONFIGS:
        r = show[c]
        for ph, tag, cost in [('ph1', 'ascent', r['costA']), ('ph2', 'morph', r['costB'])]:
            p = r[ph]
            t95 = f"{p['t_v95']:.2f}" if p['t_v95'] is not None else 'never'
            ta = f"{p['t_all']:.2f}" if p['t_all'] is not None else 'never'
            print(f"  {LABELS[c]:<30}{tag:>8}{cost:>10.1f}{t95:>15}{ta:>16}{p['min_sep']:>13.3f}"
                  f"{p['pairs_close']:>12}{100 * p['yield_frac']:>9.2f}{100 * p['final_max']:>16.2f}")
    r = show['hung']
    print(f"  Delta_cost (NN greedy vs Hungarian), seed {SHOW_SEED}: ascent "
          f"{100 * (r['costA_g'] / r['costA_h'] - 1):.2f}%, morph {100 * (r['costB_g'] / r['costB_h'] - 1):.2f}%")
    for tag, key in [('ascent', 'costA'), ('morph', 'costB')]:
        d = 100 * (mc['greedy'][key] / mc['hung'][key] - 1)
        print(f'  Delta_cost over {N_MC} layouts, {tag}: mean {d.mean():.2f}%, '
              f'min {d.min():.2f}%, max {d.max():.2f}%')
    print(f'  Monte Carlo over {N_MC} staging layouts (mean ± std):')
    print(f"  {'Configuration':<30}{'ascent t95':>14}{'ascent t_all':>15}{'ascent minsep':>16}"
          f"{'asc pairs':>14}{'morph t95':>14}{'morph t_all':>15}{'morph minsep':>16}{'morph pairs':>15}")
    for c in CONFIGS:
        m = mc[c]

        def ms(x, f='.2f'):
            x = x[~np.isnan(x)]
            return f'{x.mean():{f}} ± {x.std():{f}}' if len(x) else 'never'
        print(f"  {LABELS[c]:<30}{ms(m['A_t95']):>14}{ms(m['A_tall']):>15}{ms(m['A_minsep'], '.3f'):>16}"
              f"{ms(m['A_pairs'].astype(float)):>14}{ms(m['B_t95']):>14}{ms(m['B_tall']):>15}"
              f"{ms(m['B_minsep'], '.3f'):>16}{ms(m['B_pairs'].astype(float)):>15}")
    for c in CONFIGS:
        m = mc[c]
        print(f"  {LABELS[c]}: converged (all within 0.5 m) ascent {int(np.sum(~np.isnan(m['A_tall'])))}/{N_MC}, "
              f"morph {int(np.sum(~np.isnan(m['B_tall'])))}/{N_MC}; layouts with a pair < 0.5 m: "
              f"ascent {int(np.sum(m['A_pairs'] > 0))}, morph {int(np.sum(m['B_pairs'] > 0))}; "
              f"morph yield share {100 * m['B_yield'].mean():.2f}%")
    h, f, g = mc['hung'], mc['hung_free'], mc['greedy']
    print(f"  Yield vs no avoidance (Hungarian): larger morph min sep in "
          f"{int(np.sum(h['B_minsep'] > f['B_minsep']))}/{N_MC} (ties {int(np.sum(h['B_minsep'] == f['B_minsep']))}), "
          f"faster morph t_all in {int(np.sum(h['B_tall'] < f['B_tall']))}/{N_MC} "
          f"(ties {int(np.sum(h['B_tall'] == f['B_tall']))})")
    print(f"  Hungarian vs NN greedy (both yield): faster morph t95 in "
          f"{int(np.sum(h['B_t95'] < g['B_t95']))}/{N_MC} (ties {int(np.sum(h['B_t95'] == g['B_t95']))}), "
          f"faster ascent t95 in {int(np.sum(h['A_t95'] < g['A_t95']))}/{N_MC}, "
          f"larger morph min sep in {int(np.sum(h['B_minsep'] > g['B_minsep']))}/{N_MC}")
    print(f"  Hungarian morph spread over layouts: t_all std {h['B_tall'].std():.3f} s, min sep std "
          f"{h['B_minsep'].std():.4f} m, distinct t_all values {len(np.unique(h['B_tall']))}")
    print('  Yield rule activity in the showcase (drone-steps yielding / steps where the 2.5 m/s '
          'cap actually limited the speed):')
    for c in ['hung', 'greedy']:
        for ph, tag in [('ph1', 'ascent'), ('ph2', 'morph')]:
            p = show[c][ph]
            print(f"    {LABELS[c]:<28} {tag:<7} {p['yield_steps']:>5} / {p['bind_steps']:>4}")
    print('  Peak realised acceleration (showcase): ' + ', '.join(
        f"{LABELS[c]} {max(show[c]['ph1']['a_peak'], show[c]['ph2']['a_peak']):.3f}"
        for c in CONFIGS) + f' m/s^2 (limit {A_MAX})')
    for c in ['hung', 'hung_free']:
        cats, n_cross, n_still, n_lit, n_lit_mov, ovs = classify_close_pairs(show[c])
        print(f'  Morph close pairs (< {COLLISION_RADIUS} m), {LABELS[c]}, seed {SHOW_SEED}: '
              f'{n_lit} lit drones, {n_still} of them stay on their pixel (move < 0.1 m), '
              f'{n_lit_mov} move; proper crossings among their straight paths: {n_cross}')
        print(f'    overshoot past the own pixel (moving drones): max {ovs[0]:.2f} m (drone {ovs[4]}), '
              f'mean of per-drone max {ovs[1]:.2f} m, {ovs[2]}/{ovs[3]} drones overshoot by > 0.5 m '
              f'(braking from {V_MAX:.0f} m/s at {A_MAX:.0f} m/s^2 needs {V_MAX ** 2 / (2 * A_MAX):.2f} m)')
        for name, recs in cats.items():
            ex = ''
            if recs:
                i, j, dmin, tmin = min(recs, key=lambda x: x[2])
                st, gl = show[c]['ph1']['pos'], show[c]['B'][show[c]['sB']]
                ex = (f'; closest: drones {i} ({st[i, 0]:.0f},{st[i, 1]:.0f})->({gl[i, 0]:.0f},{gl[i, 1]:.0f},'
                      f'{gl[i, 2]:.0f}) and {j} ({st[j, 0]:.0f},{st[j, 1]:.0f})->({gl[j, 0]:.0f},'
                      f'{gl[j, 1]:.0f},{gl[j, 2]:.0f}), {dmin:.3f} m at t = {tmin:.2f} s')
            print(f'    {name:<44} {len(recs):>3}{ex}')

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_formations(show, out)
    plot_assignment_cost(show, mc, out)
    plot_fidelity(show, out)
    plot_monte_carlo(mc, out)
    save_animation(show, out)
