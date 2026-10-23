"""
S010 Asymmetric Speed — Bounded Arena Trapping
================================================
Evader is faster than pursuer (v_E = 4.5 > v_P = 3.0 m/s).  In open space,
capture is impossible.  A bounded square arena lets the pursuer exploit
walls and corners.

Pursuer strategies:
  1. direct      — pure pursuit
  2. wall_herd   — blend: α·(unit vector to evader) + (1-α)·(outward normal of
                   the wall nearest the evader), normalised to v_P
  3. corner_trap — aim at the corner closest to the evader

Evader strategies:
  - flee  (card) — straight away from the pursuer; at a wall the outward part
                   of the velocity is lost (it presses into the wall)
  - aware        — flee direction plus repulsion from walls closer than
                   D_SAFE (gain K_REPEL); at a wall it slides along the wall at
                   full speed (in a corner it turns along a free wall)

Arena rule (both players): a velocity component pointing out of the arena at
a wall is removed; the 'aware' evader renormalises what is left.

Also produces:
  - Apollonius (equal-time) circle snapshots
  - Capture time vs blend factor α for both evaders

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s010_asymmetric_speed.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# ── Parameters ───────────────────────────────────────────────
ARENA      = 5.0    # ±ARENA m square (10×10 m floor)
V_PURSUER  = 3.0    # m/s
V_EVADER   = 4.5    # m/s
ALPHA      = 0.5    # wall-herd blend
CAPTURE_R  = 0.15   # m
DT         = 1 / 48
MAX_TIME   = 60.0   # s
ALPHAS     = np.round(np.linspace(0.0, 1.0, 21), 2)
SNAP_TIMES = [0.0, 5.0, 10.0, 20.0]   # s — Apollonius snapshots (card)
D_SAFE     = 1.5    # m — wall-aware evader: wall repulsion range
K_REPEL    = 2.0    # wall-aware evader: repulsion gain

INIT_PURSUER = np.array([0.0, 0.0])
INIT_EVADER  = np.array([2.0, 1.5])

CORNERS = np.array([[-ARENA, -ARENA], [ARENA, -ARENA],
                    [-ARENA, ARENA], [ARENA, ARENA]])
WALL_NORMALS = np.array([[-1.0, 0.0], [1.0, 0.0], [0.0, -1.0], [0.0, 1.0]])

STRATEGIES = ['direct', 'wall_herd', 'corner_trap']
EVADERS = ['flee', 'aware']
LABELS = {'direct': 'Direct pursuit', 'wall_herd': f'Wall herding (α={ALPHA})',
          'corner_trap': 'Corner trap'}
EV_LABELS = {'flee': 'Evader: flee (card)', 'aware': 'Evader: wall-aware'}
COLORS = {'direct': 'darkorange', 'wall_herd': 'steelblue', 'corner_trap': 'mediumseagreen'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's010_asymmetric_speed',
)


# ── Helpers ──────────────────────────────────────────────────

def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.zeros_like(v)


def remove_outward(pos, d):
    """Drop velocity components that point out of the arena at a wall."""
    d = d.copy()
    for k in range(2):
        if pos[k] >= ARENA - 1e-9 and d[k] > 0:
            d[k] = 0.0
        if pos[k] <= -ARENA + 1e-9 and d[k] < 0:
            d[k] = 0.0
    return d


# ── Pursuer strategies (return unit direction) ───────────────

def dir_direct(p, e, alpha=ALPHA):
    return unit(e - p)


def nearest_wall_normal(e):
    d = [e[0] + ARENA, ARENA - e[0], e[1] + ARENA, ARENA - e[1]]
    return WALL_NORMALS[int(np.argmin(d))]


def dir_wall_herd(p, e, alpha=ALPHA):
    """α·(toward evader) + (1-α)·(outward normal of the evader's nearest wall).
    The wall term is a direction, not a point, so it never cancels the
    pursuit term (the old point-based version produced a zero command)."""
    return unit(alpha * unit(e - p) + (1 - alpha) * nearest_wall_normal(e))


def dir_corner_trap(p, e, alpha=ALPHA):
    corner = CORNERS[np.argmin(np.linalg.norm(CORNERS - e, axis=1))]
    return unit(corner - p)


PURSUERS = {'direct': dir_direct, 'wall_herd': dir_wall_herd,
            'corner_trap': dir_corner_trap}


# ── Evader strategies ────────────────────────────────────────

def wall_repulsion(e):
    """Sum of inward wall normals, weight (D_SAFE - δ)/D_SAFE for walls closer
    than D_SAFE (δ = distance to the wall)."""
    dists = np.array([e[0] + ARENA, ARENA - e[0], e[1] + ARENA, ARENA - e[1]])
    w = np.clip((D_SAFE - dists) / D_SAFE, 0.0, 1.0)
    return -(w[:, None] * WALL_NORMALS).sum(axis=0)


def evader_velocity(p, e, kind):
    d = unit(e - p)
    if np.linalg.norm(d) < 1e-12:
        d = np.array([1.0, 0.0])
    if kind == 'aware':
        d = unit(d + K_REPEL * wall_repulsion(e))
        if np.linalg.norm(d) < 1e-12:
            d = unit(e - p)
    v = remove_outward(e, d)
    if kind == 'aware':
        if np.linalg.norm(v) < 1e-9:                  # pushing straight into wall/corner
            v = remove_outward(e, np.array([-d[1], d[0]]))
        v = unit(v)                                   # slide along walls at full speed
    return V_EVADER * v


# ── Apollonius circle ─────────────────────────────────────────

def apollonius_circle(p, e, v_p=V_PURSUER, v_e=V_EVADER):
    """
    Equal-time circle {X : |X-P|/v_p = |X-E|/v_e}, i.e. |X-P| = r|X-E| with
    r = v_p/v_e.  For r < 1 it encloses the pursuer: points inside are
    reached by the pursuer first, all points outside by the evader first.
    Radius r·d/(1-r²); centre on the line E→P, at distance d/(1-r²) from E
    (beyond P).
    """
    r = v_p / v_e
    d = np.linalg.norm(e - p)
    if abs(1 - r ** 2) < 1e-9 or d < 1e-9:
        return None, None
    centre = e + (p - e) / d * d / (1 - r ** 2)
    return centre, r * d / (1 - r ** 2)


def check_apollonius():
    """Equal-time check on four points of the circle at t = 0."""
    c, R = apollonius_circle(INIT_PURSUER, INIT_EVADER)
    worst = 0.0
    for th in np.linspace(0, 2 * np.pi, 4, endpoint=False):
        x = c + R * np.array([np.cos(th), np.sin(th)])
        tp = np.linalg.norm(x - INIT_PURSUER) / V_PURSUER
        te = np.linalg.norm(x - INIT_EVADER) / V_EVADER
        worst = max(worst, abs(tp - te))
    inside_p = np.linalg.norm(INIT_PURSUER - c) < R
    inside_e = np.linalg.norm(INIT_EVADER - c) < R
    return c, R, worst, inside_p, inside_e


# ── Simulation ────────────────────────────────────────────────

def run_simulation(strategy, evader='flee', alpha=ALPHA):
    p, e = INIT_PURSUER.astype(float).copy(), INIT_EVADER.astype(float).copy()
    p_list, e_list = [p.copy()], [e.copy()]
    captured, cap_time = False, None
    for k in range(int(round(MAX_TIME / DT))):
        vp = V_PURSUER * remove_outward(p, PURSUERS[strategy](p, e, alpha))
        ve = evader_velocity(p, e, evader)
        p = np.clip(p + vp * DT, -ARENA, ARENA)
        e = np.clip(e + ve * DT, -ARENA, ARENA)
        p_list.append(p.copy()); e_list.append(e.copy())
        if np.linalg.norm(p - e) < CAPTURE_R:
            captured, cap_time = True, (k + 1) * DT
            break
    P, E = np.array(p_list), np.array(e_list)
    return dict(p=P, e=E, t=np.arange(len(P)) * DT, d=np.linalg.norm(P - E, axis=1),
                captured=captured, cap_time=cap_time)


def describe(r):
    """Short text summary of how a run ended."""
    if r['captured']:
        return f'captured @ {r["cap_time"]:.3f} s at ({r["e"][-1, 0]:.2f}, {r["e"][-1, 1]:.2f})'
    last = r['d'][r['t'] >= MAX_TIME - 10]
    n10 = int(round(10 / DT))
    plen = np.sum(np.linalg.norm(np.diff(r['p'][-n10 - 1:], axis=0), axis=1))
    elen = np.sum(np.linalg.norm(np.diff(r['e'][-n10 - 1:], axis=0), axis=1))
    return (f'timeout {MAX_TIME:.0f} s | last 10 s: distance {last.min():.2f}–{last.max():.2f} m, '
            f'path length pursuer {plen:.1f} m, evader {elen:.1f} m | final P '
            f'({r["p"][-1, 0]:.2f}, {r["p"][-1, 1]:.2f}), E ({r["e"][-1, 0]:.2f}, '
            f'{r["e"][-1, 1]:.2f})')


# ── Plots ────────────────────────────────────────────────────

def _draw_arena(ax):
    ax.add_patch(plt.Rectangle((-ARENA, -ARENA), 2 * ARENA, 2 * ARENA, fill=False,
                               edgecolor='grey', linestyle='--', linewidth=1.2))
    ax.scatter(CORNERS[:, 0], CORNERS[:, 1], color='grey', s=40, marker='s', zorder=3)


def plot_trajectories_2d(results, out_dir):
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    for row, ev in enumerate(EVADERS):
        for col, s in enumerate(STRATEGIES):
            ax = axes[row, col]
            r = results[(s, ev)]
            _draw_arena(ax)
            ax.plot(r['p'][:, 0], r['p'][:, 1], color='red', linewidth=1.4, label='Pursuer')
            ax.plot(r['e'][:, 0], r['e'][:, 1], color='royalblue', linewidth=1.2,
                    alpha=0.7, label='Evader')
            ax.scatter(*r['p'][0], color='red', s=60, zorder=5)
            ax.scatter(*r['e'][0], color='blue', s=60, zorder=5)
            if r['captured']:
                ax.scatter(*r['p'][-1], color='black', s=100, marker='X', zorder=6)
                st = f'captured {r["cap_time"]:.2f} s'
            else:
                ax.scatter(*r['p'][-1], color='red', s=60, marker='^', zorder=6)
                ax.scatter(*r['e'][-1], color='blue', s=60, marker='s', zorder=6)
                st = f'no capture in {MAX_TIME:.0f} s'
            ax.set_xlim(-ARENA - 1, ARENA + 1); ax.set_ylim(-ARENA - 1, ARENA + 1)
            ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
            ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
            ax.set_title(f'{LABELS[s]} vs {EV_LABELS[ev]}\n{st}', fontsize=9)
            ax.legend(fontsize=7, loc='lower left')
    fig.suptitle(f'S010 Asymmetric Speed — Top-Down Trajectories '
                 f'(v_P={V_PURSUER}, v_E={V_EVADER} m/s)', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_2d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_apollonius_snapshots(r, title, out_dir):
    idxs = [min(int(round(t / DT)), len(r['p']) - 1) for t in SNAP_TIMES]
    fig, axes = plt.subplots(1, len(SNAP_TIMES), figsize=(17, 4.5))
    for ax, si, st in zip(axes, idxs, SNAP_TIMES):
        _draw_arena(ax)
        ax.plot(r['p'][:si + 1, 0], r['p'][:si + 1, 1], color='red', linewidth=1.0, alpha=0.4)
        ax.plot(r['e'][:si + 1, 0], r['e'][:si + 1, 1], color='royalblue', linewidth=1.0,
                alpha=0.4)
        ax.scatter(*r['p'][si], color='red', s=70, zorder=5, label='Pursuer')
        ax.scatter(*r['e'][si], color='blue', s=70, zorder=5, label='Evader')
        c, R = apollonius_circle(r['p'][si], r['e'][si])
        if c is not None:
            ax.add_patch(Circle(c, R, fill=True, facecolor='red', alpha=0.10,
                                edgecolor='red', linewidth=1.5,
                                label=f'Pursuer-first region (R={R:.2f} m)'))
        ax.set_xlim(-ARENA - 2, ARENA + 2); ax.set_ylim(-ARENA - 2, ARENA + 2)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_title(f't = {r["t"][si]:.1f} s, d = {r["d"][si]:.2f} m', fontsize=9)
        ax.legend(fontsize=7, loc='upper left')
    fig.suptitle(f'S010 — Apollonius (equal-time) circle snapshots: {title}', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'apollonius_snapshots.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_alpha_sensitivity(sweep, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    for ev, mk, col in zip(EVADERS, ['o', 's'], ['steelblue', 'purple']):
        caps = np.array([c if c is not None else np.nan for c in sweep[ev]])
        ok = ~np.isnan(caps)
        ax.plot(ALPHAS[ok], caps[ok], marker=mk, color=col, linewidth=1.5,
                label=f'{EV_LABELS[ev]}: captured')
        if (~ok).any():
            ax.scatter(ALPHAS[~ok], np.full((~ok).sum(), MAX_TIME), marker='X', s=70,
                       color=col, label=f'{EV_LABELS[ev]}: no capture in {MAX_TIME:.0f} s')
    ax.axvline(ALPHA, color='grey', linestyle='--', label=f'Default α={ALPHA}')
    ax.set_yscale('log')
    ax.set_xlabel('Blend factor α  (0 = wall normal only, 1 = pure pursuit)')
    ax.set_ylabel('Capture time (s, log)')
    ax.set_title('S010 — Wall-herding capture time vs blend factor α', fontsize=10)
    ax.legend(fontsize=8); ax.grid(True, which='both', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'alpha_sensitivity.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_bar(results, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    w = 0.38
    x = np.arange(len(STRATEGIES))
    for j, (ev, hatch) in enumerate(zip(EVADERS, ['', '//'])):
        for i, s in enumerate(STRATEGIES):
            r = results[(s, ev)]
            h = r['cap_time'] if r['captured'] else MAX_TIME
            ax.bar(x[i] + (j - 0.5) * w, h, width=w, color=COLORS[s] if r['captured'] else 'tomato',
                   edgecolor='black', hatch=hatch, label=EV_LABELS[ev] if i == 0 else None)
            ax.text(x[i] + (j - 0.5) * w, h * 1.08,
                    f'{h:.2f} s' if r['captured'] else 'no capture', ha='center', fontsize=8)
    ax.set_yscale('log')
    ax.set_xticks(x); ax.set_xticklabels([LABELS[s] for s in STRATEGIES])
    ax.set_ylabel('Capture time (s, log)  [red = no capture in 60 s]')
    ax.set_title('S010 — Capture time by strategy (plain = flee evader, hatched = wall-aware evader)',
                 fontsize=10)
    ax.set_ylim(1, MAX_TIME * 2.5)
    ax.grid(axis='y', which='both', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_bar.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    import matplotlib.animation as animation
    keys = [(s, 'aware') for s in STRATEGIES]
    T_ANIM = 20.0
    n_max = int(round(T_ANIM / DT)) + 1
    step = int(np.ceil(n_max / 140))
    n_frames = (n_max - 1) // step + 2

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.3))
    arts = []
    for ax, key in zip(axes, keys):
        _draw_arena(ax)
        ax.set_xlim(-ARENA - 1, ARENA + 1); ax.set_ylim(-ARENA - 1, ARENA + 1)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        lp, = ax.plot([], [], color='red', linewidth=1.2, alpha=0.8)
        le, = ax.plot([], [], color='royalblue', linewidth=1.2, alpha=0.8)
        dp, = ax.plot([], [], 'r^', markersize=8)
        de, = ax.plot([], [], 'bs', markersize=8)
        ti = ax.set_title(LABELS[key[0]], fontsize=9)
        arts.append((key, lp, le, dp, de, ti))

    def update(i):
        out = []
        for key, lp, le, dp, de, ti in arts:
            r = results[key]
            n = len(r['p'])
            si = min(i * step, n - 1, n_max - 1)
            lo = max(0, si - int(round(3.0 / DT)))          # 3 s trail
            lp.set_data(r['p'][lo:si + 1, 0], r['p'][lo:si + 1, 1])
            le.set_data(r['e'][lo:si + 1, 0], r['e'][lo:si + 1, 1])
            dp.set_data([r['p'][si, 0]], [r['p'][si, 1]])
            de.set_data([r['e'][si, 0]], [r['e'][si, 1]])
            if r['captured'] and si == n - 1:
                ti.set_text(f'{LABELS[key[0]]}\ncaptured {r["cap_time"]:.2f} s')
            else:
                ti.set_text(f'{LABELS[key[0]]}\nt={r["t"][si]:.1f} s  dist={r["d"][si]:.2f} m')
            out += [lp, le, dp, de, ti]
        return out

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    fig.suptitle(f'S010 — wall-aware evader, first {T_ANIM:.0f} s (3 s trails; '
                 f'v_P={V_PURSUER}, v_E={V_EVADER} m/s)', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=80)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames, every {step} steps)')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    r_ratio = V_PURSUER / V_EVADER
    print(f'Speed ratio r = {r_ratio:.3f}  (< 1 → open-space capture impossible)')
    c, R, worst, in_p, in_e = check_apollonius()
    print(f'Apollonius circle at t=0: centre ({c[0]:.3f}, {c[1]:.3f}), R = {R:.3f} m | '
          f'max |t_P - t_E| on the circle = {worst:.2e} s | contains pursuer: {in_p}, '
          f'contains evader: {in_e}\n')

    results = {}
    for ev in EVADERS:
        for s in STRATEGIES:
            r = run_simulation(s, ev)
            results[(s, ev)] = r
            print(f'[{s:<11} vs {ev:<5}] {describe(r)}')

    sweep = {ev: [] for ev in EVADERS}
    print('\nWall-herding α sweep (capture time, s; "-" = no capture in 60 s):')
    for ev in EVADERS:
        for a in ALPHAS:
            r = run_simulation('wall_herd', ev, alpha=a)
            sweep[ev].append(r['cap_time'] if r['captured'] else None)
        row = ' '.join(f'{a:.2f}:{c:.2f}' if c is not None else f'{a:.2f}:-'
                       for a, c in zip(ALPHAS, sweep[ev]))
        caps = [a for a, c in zip(ALPHAS, sweep[ev]) if c is not None]
        print(f'  {ev:<5}: {row}')
        print(f'         captured for {len(caps)}/{len(ALPHAS)} α values'
              + (f'; α in {{{", ".join(f"{a:.2f}" for a in caps)}}}' if caps else ''))

    snap_key = ('wall_herd', 'aware')
    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_2d(results, out_dir)
    plot_apollonius_snapshots(results[snap_key],
                              f'{LABELS[snap_key[0]]} vs {EV_LABELS[snap_key[1]]}', out_dir)
    plot_alpha_sensitivity(sweep, out_dir)
    plot_capture_bar(results, out_dir)
    save_animation(results, out_dir)
