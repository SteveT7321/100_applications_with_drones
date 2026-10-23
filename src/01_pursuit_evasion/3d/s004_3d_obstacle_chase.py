"""
S004 3D Upgrade — Obstacle-Course Chase
=========================================
Pursuer (3D APF) chases an evader through 4 spheres + 1 vertical cylinder
inside a walled arena x in [-5, 5], y in [-3, 3], z in [0.3, 10] m (hard walls
for both agents, as in the fixed 2D S004), so the evader stays in the course.

Pursuer modes (2 x 2):
  apf_fixed_z     — APF, everything held at z = 2 m (2D baseline)
  apf_3d          — APF with full 3D obstacle normals
  detour_fixed_z  — APF + detour waypoint; only horizontal (left/right) detours
  detour_3d       — APF + detour waypoint; horizontal, over or under detours
                    (the card's over-fly decision: the shortest P -> w -> E wins)

Evader tactics:
  straight        — straight escape + APF (as in 2D)
  shield          — card's "altitude shield": hide low behind the tall cylinder,
                    on the side opposite the pursuer

The nominal case uses the card's start positions. A Monte Carlo over 40 random
evader start points (same draws for every mode) checks that the ranking does
not depend on one geometry.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s004_3d_obstacle_chase.py
"""
import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

# ── Scenario parameters ─────────────────────────────────────
PURSUER_SPEED = 5.0
EVADER_SPEED  = 3.5
DT            = 1 / 48
MAX_TIME      = 30.0
CAPTURE_R     = 0.15
COLLISION_R   = 0.05

INIT_PURSUER = np.array([-4.0, 0.0, 2.0])
INIT_EVADER  = np.array([ 4.0, 0.0, 2.0])
FIXED_Z      = 2.0

Z_MIN = 0.3
Z_MAX = 10.0
ARENA_LO = np.array([-5.0, -3.0, Z_MIN])
ARENA_HI = np.array([ 5.0,  3.0, Z_MAX])

# APF parameters
K_REP = 4.0
RHO0  = 1.5

# Detour waypoint: clearance from the obstacle surface
DETOUR_MARGIN = 0.8
WAYPOINT_MIN_CLEAR = 0.5  # a waypoint must be this far from every obstacle surface
RHO0_SAFETY   = 0.4     # repulsion range for the detour pursuers (m)
DETOUR_HYST   = 0.5     # keep the previous detour side unless another is this much shorter (m)

# Altitude-shield evader
SHIELD_STANDOFF = 1.0     # distance of the hiding point from the cylinder surface (m)
SHIELD_Z        = 1.0     # hide low (3D); fixed-z world uses z = 2
SHIELD_Z_SWEEP  = [1.0, 2.0, 3.0, 4.0]

N_MC    = 40
STALL_SPEED = 0.5  # pursuer 'stalled' below this speed (m/s)
Z_PLOT  = 8.0     # z-axis limit used in the figures
MC_SEED = 4

# ── Obstacle layout ─────────────────────────────────────────
SPHERES = [
    {'c': np.array([-2.0,  0.0, 2.0]), 'r': 0.60},
    {'c': np.array([-0.5,  0.5, 3.5]), 'r': 0.55},   # elevated
    {'c': np.array([ 1.0, -0.4, 1.5]), 'r': 0.50},   # low
    {'c': np.array([ 2.5,  0.3, 2.5]), 'r': 0.45},
]
CYLINDER = {'xy': np.array([0.5, -1.0]), 'r': 0.4, 'z_bot': 0.0, 'z_top': 5.0}

OBST_NAMES = ['sphere1', 'sphere2', 'sphere3', 'sphere4', 'cylinder']

MODES = ['apf_fixed_z', 'apf_3d', 'detour_fixed_z', 'detour_3d']
MODE_LABELS = {
    'apf_fixed_z':    'APF, fixed z (2D baseline)',
    'apf_3d':         'APF, full 3D',
    'detour_fixed_z': 'APF + detour, fixed z',
    'detour_3d':      'APF + detour, 3D (over-fly allowed)',
}
TACTICS = ['straight', 'shield']

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's004_3d_obstacle_chase',
))


# ── APF helpers ──────────────────────────────────────────────

def sphere_repulsion(pos, rho0=RHO0):
    v_rep = np.zeros(3)
    for sp in SPHERES:
        diff = pos - sp['c']
        dist = np.linalg.norm(diff) + 1e-8
        rho  = dist - sp['r']
        if 0 < rho < rho0:
            v_rep += K_REP * (1.0 / rho - 1.0 / rho0) / rho**2 * diff / dist
    return v_rep


def cylinder_repulsion(pos, rho0=RHO0):
    """Horizontal repulsion, active only while z_bot <= z <= z_top."""
    cyl = CYLINDER
    if not (cyl['z_bot'] <= pos[2] <= cyl['z_top']):
        return np.zeros(3)
    d = pos[:2] - cyl['xy']
    dist = np.linalg.norm(d) + 1e-8
    rho = dist - cyl['r']
    if 0 < rho < rho0:
        return K_REP * (1.0 / rho - 1.0 / rho0) / rho**2 * np.array([d[0], d[1], 0.0]) / dist
    return np.zeros(3)


def total_repulsion(pos, rho0=RHO0):
    return sphere_repulsion(pos, rho0) + cylinder_repulsion(pos, rho0)


def inside_any_obstacle(pos):
    for sp in SPHERES:
        if np.linalg.norm(pos - sp['c']) < sp['r'] + COLLISION_R:
            return True
    cyl = CYLINDER
    return bool(np.linalg.norm(pos[:2] - cyl['xy']) < cyl['r'] + COLLISION_R
                and cyl['z_bot'] <= pos[2] <= cyl['z_top'])


# ── Detour waypoint (over-fly decision) ──────────────────────

def _side_offset(u, d, R):
    """Distance s along unit u (perpendicular to the path) from the path point
    nearest the obstacle centre so that the waypoint is R from the centre;
    d = centre - nearest point (perpendicular to the path)."""
    ud = float(np.dot(u, d))
    return ud + np.sqrt(max(ud**2 + R**2 - float(np.dot(d, d)), 0.0))


def detour_waypoint(p, e, allow_vertical, prev=None):
    """If the straight segment p -> e passes within (radius + DETOUR_MARGIN) of
    an obstacle, return (waypoint, kind, obstacle index) for the shortest
    detour p -> w -> e; kind is 'left'/'right', 'over' or 'under'. In the
    fixed-z world only left/right are allowed. prev = (kind, index) of the last
    step's choice: it is kept unless another option is shorter by more than
    DETOUR_HYST (stops left/right chattering). Returns None if the path is clear."""
    seg = e - p
    L = np.linalg.norm(seg)
    if L < 1e-6:
        return None
    l_hat = seg / L
    h = np.cross(np.array([0.0, 0.0, 1.0]), l_hat)
    h_hat = h / (np.linalg.norm(h) + 1e-12)                  # horizontal, perpendicular
    v = np.array([0.0, 0.0, 1.0]) - l_hat[2] * l_hat
    v_hat = v / (np.linalg.norm(v) + 1e-12)                  # vertical-ish, perpendicular

    blockers = []
    for i, sp in enumerate(SPHERES):
        s = np.clip(np.dot(sp['c'] - p, l_hat), 0.0, L)
        q = p + s * l_hat
        R = sp['r'] + DETOUR_MARGIN
        if np.linalg.norm(sp['c'] - q) < R and 0.0 < s < L:
            blockers.append((s, i, q, sp['c'] - q, R))
    cyl = CYLINDER
    seg_xy = seg[:2]
    Lxy = np.linalg.norm(seg_xy)
    if Lxy > 1e-6:
        lxy = seg_xy / Lxy
        sxy = np.clip(np.dot(cyl['xy'] - p[:2], lxy), 0.0, Lxy)
        q = p + (sxy / Lxy) * seg
        R = cyl['r'] + DETOUR_MARGIN
        dxy = cyl['xy'] - q[:2]
        if (np.linalg.norm(dxy) < R and q[2] <= cyl['z_top'] + DETOUR_MARGIN
                and 0.0 < sxy < Lxy):
            blockers.append((sxy / Lxy * L, len(SPHERES), q,
                             np.array([dxy[0], dxy[1], 0.0]), R))
    if not blockers:
        return None
    _, idx, q, d, R = min(blockers, key=lambda b: b[0])     # first obstacle on the path

    cands = []
    for kind, u in (('left', h_hat), ('right', -h_hat)):
        cands.append((kind, q + _side_offset(u, d, R) * u))
    if allow_vertical:
        if idx < len(SPHERES):
            for kind, u in (('over', v_hat), ('under', -v_hat)):
                w = q + _side_offset(u, d, R) * u
                if w[2] >= Z_MIN + 0.2:
                    cands.append((kind, w))
        else:
            w = q.copy()
            w[2] = cyl['z_top'] + DETOUR_MARGIN
            cands.append(('over', w))
    # A waypoint must lie inside the arena and keep clear of every other obstacle;
    # among those, the shortest p -> w -> e wins. If none is feasible, take the
    # one with the most clearance.
    feas = [c for c in cands
            if np.all(c[1] >= ARENA_LO + 0.2) and np.all(c[1] <= ARENA_HI - 0.2)
            and clearance(c[1]) >= WAYPOINT_MIN_CLEAR]
    if feas:
        plen = lambda c: np.linalg.norm(c[1] - p) + np.linalg.norm(e - c[1])  # noqa: E731
        best = min(feas, key=plen)
        if prev is not None and prev[1] == idx:
            keep = [c for c in feas if c[0] == prev[0]]
            if keep and plen(keep[0]) <= plen(best) + DETOUR_HYST:
                best = keep[0]
    else:
        best = max(cands, key=lambda c: clearance(np.clip(c[1], ARENA_LO, ARENA_HI)))
    return best[1], best[0], idx


def clearance(pos):
    """Distance from pos to the nearest obstacle surface."""
    d = min(np.linalg.norm(pos - sp['c']) - sp['r'] for sp in SPHERES)
    cyl = CYLINDER
    dxy = np.linalg.norm(pos[:2] - cyl['xy']) - cyl['r']
    if pos[2] <= cyl['z_top']:
        dc = dxy
    else:
        dc = np.hypot(max(dxy, 0.0), pos[2] - cyl['z_top'])
    return min(d, dc)


# ── Simulation ───────────────────────────────────────────────

def bounded_step(pos, v_cmd, max_speed, fixed_z):
    if fixed_z:
        v_cmd = np.array([v_cmd[0], v_cmd[1], 0.0])
    speed = np.linalg.norm(v_cmd)
    if speed > max_speed:
        v_cmd = v_cmd / speed * max_speed
    return np.clip(pos + v_cmd * DT, ARENA_LO, ARENA_HI)


def shield_point(p_pursuer, fixed_z):
    """Point behind the cylinder, on the side opposite the pursuer."""
    away = CYLINDER['xy'] - p_pursuer[:2]
    away = away / (np.linalg.norm(away) + 1e-8)
    xy = CYLINDER['xy'] + (CYLINDER['r'] + SHIELD_STANDOFF) * away
    return np.array([xy[0], xy[1], FIXED_Z if fixed_z else SHIELD_Z])


def run_simulation(mode='detour_3d', tactic='straight', init_evader=INIT_EVADER,
                   obstacles_on=True):
    fixed_z = mode.endswith('fixed_z')
    use_detour = mode.startswith('detour')
    p = INIT_PURSUER.copy()
    e = np.array(init_evader, dtype=float)
    if fixed_z:
        p[2] = e[2] = FIXED_Z
    p_traj, e_traj = [p.copy()], [e.copy()]
    detour_kinds = []
    detour_obst = []
    prev_choice = None
    outcome, t_end = 'timeout', MAX_TIME
    rep = total_repulsion if obstacles_on else (lambda x, rho0=RHO0: np.zeros(3))
    # detour pursuers already keep DETOUR_MARGIN clearance through the waypoint;
    # they keep only a short-range safety repulsion
    rho0_p = RHO0_SAFETY if use_detour else RHO0

    for step in range(int(MAX_TIME / DT)):
        t = step * DT
        if obstacles_on and inside_any_obstacle(p):
            outcome, t_end = 'pursuer_crash', t
            break
        if obstacles_on and inside_any_obstacle(e):
            outcome, t_end = 'evader_crash', t
            break
        if np.linalg.norm(p - e) < CAPTURE_R:
            outcome, t_end = 'captured', t
            break

        # Pursuer
        target, kind, idx = e, None, None
        if use_detour and obstacles_on:
            wp = detour_waypoint(p, e, allow_vertical=not fixed_z, prev=prev_choice)
            if wp is not None:
                target, kind, idx = wp
                prev_choice = (kind, idx)
            else:
                prev_choice = None
        detour_kinds.append(kind)
        detour_obst.append(idx)
        r_pt = target - p
        v_p = PURSUER_SPEED * r_pt / (np.linalg.norm(r_pt) + 1e-8) + rep(p, rho0_p)

        # Evader
        if tactic == 'straight':
            r_ep = e - p
            v_e = EVADER_SPEED * r_ep / (np.linalg.norm(r_ep) + 1e-8)
        else:
            r_s = shield_point(p, fixed_z) - e
            dist_s = np.linalg.norm(r_s)
            v_e = r_s / DT if dist_s < EVADER_SPEED * DT else EVADER_SPEED * r_s / dist_s
        v_e = v_e + rep(e)

        p = bounded_step(p, v_p, PURSUER_SPEED, fixed_z)
        e = bounded_step(e, v_e, EVADER_SPEED, fixed_z)
        p_traj.append(p.copy())
        e_traj.append(e.copy())

    return dict(p=np.array(p_traj), e=np.array(e_traj), outcome=outcome, t=t_end,
                kinds=detour_kinds, obst=detour_obst, mode=mode, tactic=tactic)


def path_length(traj):
    return float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1)))


def summarize(r):
    kinds = [k for k in r['kinds'] if k is not None]
    n_vert = sum(k in ('over', 'under') for k in kinds)
    speed = np.linalg.norm(np.diff(r['p'], axis=0), axis=1) / DT
    return dict(outcome=r['outcome'], t=r['t'], Lp=path_length(r['p']),
                Le=path_length(r['e']), n_detour=len(kinds), n_vert=n_vert,
                stall=float(np.sum(speed < STALL_SPEED) * DT),
                zp=(r['p'][:, 2].min(), r['p'][:, 2].max()),
                ze=(r['e'][:, 2].min(), r['e'][:, 2].max()))


def sample_starts(n, seed):
    rng = np.random.default_rng(seed)
    starts = []
    while len(starts) < n:
        s = np.array([rng.uniform(1.5, 4.5), rng.uniform(-2.5, 2.5), FIXED_Z])
        if not inside_any_obstacle(s) and min(
                np.linalg.norm(s - sp['c']) - sp['r'] for sp in SPHERES) > 0.3 \
                and np.linalg.norm(s[:2] - CYLINDER['xy']) - CYLINDER['r'] > 0.3:
            starts.append(s)
    return starts


# ── Drawing helpers ──────────────────────────────────────────

def draw_obstacles(ax, alpha=0.20):
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 12)
    for sp in SPHERES:
        cx, cy, cz, r = *sp['c'], sp['r']
        ax.plot_surface(cx + r * np.outer(np.cos(u), np.sin(v)),
                        cy + r * np.outer(np.sin(u), np.sin(v)),
                        cz + r * np.outer(np.ones_like(u), np.cos(v)),
                        color='gray', alpha=alpha, linewidth=0)
    th = np.linspace(0, 2 * np.pi, 30)
    tg, zg = np.meshgrid(th, [CYLINDER['z_bot'], CYLINDER['z_top']])
    ax.plot_surface(CYLINDER['xy'][0] + CYLINDER['r'] * np.cos(tg),
                    CYLINDER['xy'][1] + CYLINDER['r'] * np.sin(tg), zg,
                    color='dimgray', alpha=alpha, linewidth=0)


def _status(r):
    return {'captured': f'Captured {r["t"]:.2f} s',
            'pursuer_crash': f'P crash {r["t"]:.2f} s',
            'evader_crash': f'E crash {r["t"]:.2f} s',
            'timeout': 'Timeout'}[r['outcome']]


def _setup_ax(ax):
    ax.set_xlim(ARENA_LO[0], ARENA_HI[0])
    ax.set_ylim(ARENA_LO[1], ARENA_HI[1])
    ax.set_zlim(0, Z_PLOT)
    ax.set_xlabel('X (m)', fontsize=8)
    ax.set_ylabel('Y (m)', fontsize=8)
    ax.set_zlabel('Z (m)', fontsize=8)


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories(nominal, tactic, fname):
    fig = plt.figure(figsize=(16, 12))
    for i, mode in enumerate(MODES):
        r = nominal[(mode, tactic)]
        ax = fig.add_subplot(2, 2, i + 1, projection='3d')
        draw_obstacles(ax)
        ax.plot(*r['p'].T, color='red', linewidth=1.8, label='Pursuer')
        ax.plot(*r['e'].T, color='blue', linewidth=1.8, label='Evader')
        ax.scatter(*r['p'][0], color='red', s=40)
        ax.scatter(*r['e'][0], color='blue', s=40)
        vert = [k for k, kk in enumerate(r['kinds']) if kk in ('over', 'under')]
        if vert:
            ax.scatter(*r['p'][vert].T, color='orange', s=8, label='over/under detour active')
        ax.scatter(*r['p'][-1], color='black', s=100, marker='X')
        _setup_ax(ax)
        ax.set_title(f'{MODE_LABELS[mode]}\n{_status(r)}, pursuer path {path_length(r["p"]):.1f} m',
                     fontsize=10)
        ax.legend(loc='upper left', fontsize=7)
    fig.suptitle(f'S004 3D — {tactic} evader: trajectories (axes = arena, z shown 0-{Z_PLOT:.0f} m)',
                 fontsize=13)
    path = os.path.join(OUTPUT_DIR, fname)
    plt.savefig(path, dpi=100, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_profile(nominal):
    fig, axes = plt.subplots(2, 2, figsize=(13, 7), sharey=True)
    for row, tactic in enumerate(TACTICS):
        for col, mode in enumerate(['apf_3d', 'detour_3d']):
            r = nominal[(mode, tactic)]
            ax = axes[row, col]
            t = np.arange(len(r['p'])) * DT
            ax.plot(t, r['p'][:, 2], color='red', label='Pursuer z')
            ax.plot(t, r['e'][:, 2], color='blue', label='Evader z')
            for sp in SPHERES:
                ax.axhspan(sp['c'][2] - sp['r'], sp['c'][2] + sp['r'], color='gray', alpha=0.08)
            ax.axhline(CYLINDER['z_top'], color='dimgray', linestyle='--', linewidth=0.8,
                       label='cylinder top')
            ax.set_title(f'{MODE_LABELS[mode]} — {tactic} evader ({_status(r)})', fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.set_ylim(0, Z_PLOT)
            if col == 0:
                ax.set_ylabel('Altitude z (m)')
            if row == 1:
                ax.set_xlabel('Time (s)')
            ax.legend(fontsize=7, loc='upper right')
    fig.suptitle('S004 3D — Altitude profiles of the 3D modes (gray bands: sphere altitude ranges)',
                 fontsize=12)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'altitude_profile.png')
    plt.savefig(path, dpi=110, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_path_length_comparison(nominal):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    x = np.arange(len(MODES))
    for ax, tactic in zip(axes, TACTICS):
        lp = [path_length(nominal[(m, tactic)]['p']) for m in MODES]
        le = [path_length(nominal[(m, tactic)]['e']) for m in MODES]
        bp = ax.bar(x - 0.2, lp, 0.4, color='red', alpha=0.75, edgecolor='black', label='Pursuer')
        be = ax.bar(x + 0.2, le, 0.4, color='blue', alpha=0.75, edgecolor='black', label='Evader')
        for bars, vals in ((bp, lp), (be, le)):
            for b, v in zip(bars, vals):
                ax.text(b.get_x() + b.get_width() / 2, v + 0.3, f'{v:.1f}', ha='center',
                        fontsize=8)
        ax.set_xticks(x)
        ax.set_xticklabels([MODE_LABELS[m].replace(', ', ',\n', 1) + '\n' +
                            _status(nominal[(m, tactic)]) for m in MODES], fontsize=8)
        ax.set_ylabel('Path length (m)')
        ax.set_ylim(0, max(lp + le) * 1.15)
        ax.set_title(f'{tactic} evader (card start positions)')
        ax.legend(loc='upper left')
        ax.grid(True, axis='y', alpha=0.3)
    fig.suptitle('S004 3D — Path length and outcome by pursuer mode', fontsize=12)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'path_length_comparison.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_monte_carlo(mc):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), sharey=True)
    for ax, tactic in zip(axes, TACTICS):
        data, labels = [], []
        for m in MODES:
            rs = mc[(m, tactic)]
            data.append([s['t'] if s['outcome'] == 'captured' else MAX_TIME for s in rs])
            ncap = sum(s['outcome'] == 'captured' for s in rs)
            labels.append(MODE_LABELS[m].replace(', ', ',\n', 1) + f'\n{ncap}/{len(rs)} captured')
        ax.boxplot(data, showmeans=True)
        ax.set_xticks(range(1, len(MODES) + 1))
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_title(f'{tactic} evader — {N_MC} random evader starts (same for every mode)')
        ax.set_ylabel('Capture time (s)  (non-captures plotted at 30 s)')
        ax.grid(True, axis='y', alpha=0.3)
    fig.suptitle('S004 3D — Monte Carlo over evader start positions', fontsize=12)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'monte_carlo.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(nominal):
    """Side by side: horizontal-only detour vs 3D detour, straight evader."""
    runs = [nominal[('detour_fixed_z', 'straight')], nominal[('detour_3d', 'straight')]]
    step_skip = 3
    n_steps = max(len(r['p']) for r in runs)
    frames = list(range(0, n_steps, step_skip))
    if frames[-1] != n_steps - 1:
        frames.append(n_steps - 1)

    fig = plt.figure(figsize=(13, 5.5))
    arts = []
    for i, r in enumerate(runs):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        draw_obstacles(ax)
        _setup_ax(ax)
        ax.set_title(f'{MODE_LABELS[r["mode"]]}\n{_status(r)}', fontsize=10)
        pt, = ax.plot([], [], [], color='red', linewidth=1.3)
        et, = ax.plot([], [], [], color='blue', linewidth=1.3)
        pd, = ax.plot([], [], [], 'r^', markersize=7)
        ed, = ax.plot([], [], [], 'bs', markersize=7)
        arts.append((r, pt, et, pd, ed))
    title = fig.suptitle('', fontsize=11)

    def update(i):
        k = frames[i]
        for r, pt, et, pd, ed in arts:
            j = min(k, len(r['p']) - 1)
            P, E = r['p'][:j + 1], r['e'][:j + 1]
            pt.set_data(P[:, 0], P[:, 1]); pt.set_3d_properties(P[:, 2])
            et.set_data(E[:, 0], E[:, 1]); et.set_3d_properties(E[:, 2])
            pd.set_data([P[-1, 0]], [P[-1, 1]]); pd.set_3d_properties([P[-1, 2]])
            ed.set_data([E[-1, 0]], [E[-1, 1]]); ed.set_3d_properties([E[-1, 2]])
        title.set_text(f'S004 3D — straight evader, pursuer red / evader blue   t = {k * DT:.2f} s')
        return []

    ani = FuncAnimation(fig, update, frames=len(frames), interval=50, blit=False)
    path = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=20), dpi=80)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print('Nominal case (card starts: P (-4, 0, 2), E (4, 0, 2)):')
    nominal = {}
    for tactic in TACTICS:
        for mode in MODES:
            r = run_simulation(mode, tactic)
            nominal[(mode, tactic)] = r
            s = summarize(r)
            print(f'  [{tactic:>8} | {mode:>14}] {_status(r):>18}; path P {s["Lp"]:.2f} m, '
                  f'E {s["Le"]:.2f} m; detour steps {s["n_detour"]} (over/under {s["n_vert"]}); '
                  f'P z [{s["zp"][0]:.2f}, {s["zp"][1]:.2f}], E z [{s["ze"][0]:.2f}, {s["ze"][1]:.2f}]; '
                  f'E end ({r["e"][-1, 0]:.2f}, {r["e"][-1, 1]:.2f}, {r["e"][-1, 2]:.2f}); '
                  f'P slower than {STALL_SPEED} m/s for {s["stall"]:.2f} s')
        rr = nominal[('detour_3d', tactic)]
        pairs = [(OBST_NAMES[o], k) for k, o in zip(rr['kinds'], rr['obst']) if k is not None]
        print(f'  detour_3d choices ({tactic}), steps per (obstacle, side): ' +
              ', '.join(f'{o} {k} {pairs.count((o, k))}' for o, k in sorted(set(pairs))))
    ref = run_simulation('apf_3d', 'straight', obstacles_on=False)
    print(f'  obstacle-free reference (straight evader, same arena): {_status(ref)}, '
          f'path P {path_length(ref["p"]):.2f} m')
    e_in = all(((r['e'] >= ARENA_LO - 1e-9) & (r['e'] <= ARENA_HI + 1e-9)).all()
               for r in nominal.values())
    print(f'  evader stays inside the arena in every nominal run: {e_in}')

    print(f'\nMonte Carlo: {N_MC} evader starts x~U[1.5,4.5], y~U[-2.5,2.5], z=2 (seed {MC_SEED})')
    starts = sample_starts(N_MC, MC_SEED)
    mc = {}
    for tactic in TACTICS:
        for mode in MODES:
            mc[(mode, tactic)] = [summarize(run_simulation(mode, tactic, s)) for s in starts]
            rs = mc[(mode, tactic)]
            cap = [s for s in rs if s['outcome'] == 'captured']
            nvert = sum(s['n_vert'] > 0 for s in rs)
            outc = {o: sum(s['outcome'] == o for s in rs)
                    for o in ('captured', 'pursuer_crash', 'evader_crash', 'timeout')}
            tc = np.array([s['t'] for s in cap])
            lp = np.array([s['Lp'] for s in cap])
            if mode == 'detour_3d':
                runs = [run_simulation(mode, tactic, s0) for s0 in starts]
                firsts = sorted({OBST_NAMES[o] for rr in runs for o in rr['obst'] if o is not None})
                verts = sorted({(OBST_NAMES[o], k) for rr in runs
                                for k, o in zip(rr['kinds'], rr['obst']) if k in ('over', 'under')})
                print(f'    detour_3d over all starts: first-blocking obstacles {firsts}; '
                      f'over/under used at {verts}')
            print(f'  [{tactic:>8} | {mode:>14}] captured {outc["captured"]}/{N_MC}, '
                  f'P crash {outc["pursuer_crash"]}, E crash {outc["evader_crash"]}, '
                  f'timeout {outc["timeout"]}; capture time mean {tc.mean():.2f} s, '
                  f'median {np.median(tc):.2f} s; P path mean {lp.mean():.2f} m; '
                  f'runs with an over/under detour {nvert}')
        # paired comparison of the two detour modes on starts where both capture
        a, b = mc[('detour_fixed_z', tactic)], mc[('detour_3d', tactic)]
        both = [(x['t'], y['t']) for x, y in zip(a, b)
                if x['outcome'] == 'captured' and y['outcome'] == 'captured']
        if both:
            d = np.array([y - x for x, y in both])
            print(f'  paired detour_3d - detour_fixed_z capture time ({tactic}, {len(both)} starts '
                  f'both captured): mean {d.mean():+.2f} s, 3D faster in {(d < -1e-9).sum()}, '
                  f'equal in {(np.abs(d) <= 1e-9).sum()}, slower in {(d > 1e-9).sum()}')

    # Does hiding *low* matter? Repeat the shield Monte Carlo with other hiding altitudes.
    global SHIELD_Z
    z_nominal = SHIELD_Z
    print('\nShield altitude sweep (3D modes, same 40 starts): captured / mean capture time')
    for z in SHIELD_Z_SWEEP:
        SHIELD_Z = z
        row = []
        for mode in ('apf_3d', 'detour_3d'):
            rs = [run_simulation(mode, 'shield', s) for s in starts]
            tc = [r['t'] for r in rs if r['outcome'] == 'captured']
            row.append(f'{mode} {len(tc)}/{N_MC}, {np.mean(tc):.2f} s' if tc
                       else f'{mode} 0/{N_MC}')
        print(f'  hide at z = {z:.1f} m: ' + '; '.join(row))
    SHIELD_Z = z_nominal

    print('\nGenerating plots...')
    plot_trajectories(nominal, 'straight', 'trajectories_3d.png')
    plot_trajectories(nominal, 'shield', 'trajectories_shield.png')
    plot_altitude_profile(nominal)
    plot_path_length_comparison(nominal)
    plot_monte_carlo(mc)
    print('Generating animation...')
    save_animation(nominal)
    print('\nDone. All outputs saved to:', OUTPUT_DIR)


if __name__ == '__main__':
    main()
