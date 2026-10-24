"""
S065 Building 3D Modeling — Multi-Angle Scan Path
=================================================
Plan a photogrammetry flight around a 20 x 15 x 10 m building: facade-parallel strips
at a 3 m standoff (60 % forward / 30 % side overlap) on all four facades plus a nadir
grid over the roof at 13 m. The same 464 capture waypoints are ordered three ways:

  structured : rings — one lap around the building per strip altitude, then a roof
               boustrophedon
  NN-TSP     : nearest-neighbour tour over the pooled waypoints (card)
  NN + 2-opt : NN tour improved by 2-opt

Every leg that would enter the 1 m safety margin around the building is replaced by an
up-over-down detour at 14 m, and is counted. A GPS Monte Carlo then checks whether the
planned overlaps survive position noise at the capture points.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s065_3d_scan_path.py
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

# ── Parameters ────────────────────────────────────────────────────────────────
V_INSP   = 1.0                     # m/s
GPS_SIGMA = 0.1                    # m
A_MAX    = 1.0                     # m/s^2 acceleration limit for the trapezoidal leg time (added)
BLDG_L, BLDG_W, BLDG_H = 20.0, 15.0, 10.0
F_M, SW_M, W_PX = 16e-3, 17.6e-3, 4000
FOV_H, FOV_V = np.deg2rad(60.0), np.deg2rad(45.0)
GSD_MAX  = 0.02
D_SCAN   = 3.0
Z_MIN    = 1.0
P_X, P_Z = 0.60, 0.30
T_CAP    = 1.5
MARGIN   = 1.0                     # m  building inflation for the collision test (added)
Z_SAFE   = BLDG_H + 4.0            # m  detour altitude (added)
LAUNCH   = np.array([-(BLDG_L / 2 + D_SCAN + 5.0), 0.0, 1.5])

W_FOOT  = 2 * D_SCAN * math.tan(FOV_H / 2)
H_FOOT  = 2 * D_SCAN * math.tan(FOV_V / 2)
B_X     = W_FOOT * (1 - P_X)
DZ_STEP = H_FOOT * (1 - P_Z)
GSD_ACH = SW_M * D_SCAN / (F_M * W_PX)

GPS_LEVELS = [0.05, 0.1, 0.2, 0.5]
N_MC = 200
MC_SEED = 0
ANIM_STRIDE = 4

ZONES = ['South', 'East', 'North', 'West', 'Roof']
ZONE_COLORS = ['tab:red', 'tab:orange', 'tab:blue', 'tab:green', 'tab:purple']
ORDERS = ['structured', 'nn', 'nn2opt']
ORDER_LABELS = {'structured': 'Structured rings', 'nn': 'NN-TSP (card)',
                'nn2opt': 'NN + 2-opt'}
ORDER_COLORS = {'structured': '#1F77B4', 'nn': '#888888', 'nn2opt': '#E87722'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'outputs',
                          '04_industrial_agriculture', 's065_3d_scan_path')


# ── Waypoint generation (card) ────────────────────────────────────────────────

def facade_grid(fid):
    """Return (strips, normal axis info). strips: list of (n_pass, 3) arrays, bottom to top."""
    hl, hw = BLDG_L / 2, BLDG_W / 2
    cfg = {0: (1, -(hw + D_SCAN), -hl, hl),     # South: y fixed, sweep x
           1: (0, (hl + D_SCAN), -hw, hw),      # East:  x fixed, sweep y
           2: (1, (hw + D_SCAN), -hl, hl),      # North
           3: (0, -(hl + D_SCAN), -hw, hw)}[fid]  # West
    axis, fixed, a0, a1 = cfg
    n_strips = int(math.ceil((BLDG_H - Z_MIN) / DZ_STEP)) + 1
    zs = [Z_MIN + k * DZ_STEP for k in range(n_strips) if Z_MIN + k * DZ_STEP <= BLDG_H + H_FOOT / 2]
    n_pass = int(math.ceil((a1 - a0) / B_X)) + 1
    us = np.linspace(a0, a1, n_pass)
    strips = []
    for z in zs:
        if axis == 1:
            strips.append(np.column_stack([us, np.full(n_pass, fixed), np.full(n_pass, z)]))
        else:
            strips.append(np.column_stack([np.full(n_pass, fixed), us, np.full(n_pass, z)]))
    return strips, us


def roof_grid():
    h_roof = BLDG_H + D_SCAN
    d_row, b_fwd = W_FOOT * (1 - P_Z), W_FOOT * (1 - P_X)
    n_rows = int(math.ceil(BLDG_W / d_row)) + 1
    n_cols = int(math.ceil(BLDG_L / b_fwd)) + 1
    ys = np.linspace(-BLDG_W / 2, BLDG_W / 2, n_rows)
    xs = np.linspace(-BLDG_L / 2, BLDG_L / 2, n_cols)
    return [np.column_stack([xs, np.full(n_cols, y), np.full(n_cols, h_roof)]) for y in ys], xs, ys


def all_waypoints():
    """Pooled waypoints with zone labels; also returns the per-zone strip lists."""
    strips = {}
    for fid, name in enumerate(ZONES[:4]):
        strips[name] = facade_grid(fid)[0]
    strips['Roof'] = roof_grid()[0]
    pts, zone = [], []
    for zi, name in enumerate(ZONES):
        for s in strips[name]:
            pts.append(s)
            zone += [zi] * len(s)
    return np.vstack(pts), np.array(zone), strips


# ── Collision test and detours ────────────────────────────────────────────────

BOX_LO = np.array([-BLDG_L / 2 - MARGIN, -BLDG_W / 2 - MARGIN, -1.0])
BOX_HI = np.array([BLDG_L / 2 + MARGIN, BLDG_W / 2 + MARGIN, BLDG_H + MARGIN])


def seg_hits_box(p, q, lo=BOX_LO, hi=BOX_HI):
    """Slab test: does segment p-q pass through the axis-aligned box?"""
    d = q - p
    t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if p[k] < lo[k] or p[k] > hi[k]:
                return False
        else:
            ta, tb = (lo[k] - p[k]) / d[k], (hi[k] - p[k]) / d[k]
            if ta > tb:
                ta, tb = tb, ta
            t0, t1 = max(t0, ta), min(t1, tb)
            if t0 > t1:
                return False
    return True


def leg_path(p, q):
    """Polyline actually flown from p to q (straight, or up-over-down when blocked)."""
    if not seg_hits_box(p, q):
        return [p, q], False
    a = np.array([p[0], p[1], max(Z_SAFE, p[2])])
    b = np.array([q[0], q[1], max(Z_SAFE, q[2])])
    return [p, a, b, q], True


def leg_time(L):
    """Trapezoidal (or triangular) rest-to-rest profile with V_INSP and A_MAX."""
    if L >= V_INSP ** 2 / A_MAX:
        return L / V_INSP + V_INSP / A_MAX
    return 2 * math.sqrt(L / A_MAX)


BLDG_LO = np.array([-BLDG_L / 2, -BLDG_W / 2, 0.0])
BLDG_HI = np.array([BLDG_L / 2, BLDG_W / 2, BLDG_H])


def seg_box_clearance(p, q, n=4001):
    """Minimum distance between segment p-q and the (un-inflated) building box."""
    t = np.linspace(0.0, 1.0, n)[:, None]
    pts = p + t * (q - p)
    d = np.maximum(np.maximum(BLDG_LO - pts, 0.0), pts - BLDG_HI)
    return float(np.min(np.linalg.norm(d, axis=1)))


def evaluate(wps, order, zone):
    seq = wps[order]
    zz = zone[order]
    L_straight, L_flown, n_det, t_acc, max_leg, climb = 0.0, 0.0, 0, 0.0, 0.0, 0.0
    flagged = []
    flown = [seq[0]]
    for i, (p, q) in enumerate(zip(seq[:-1], seq[1:])):
        L_straight += float(np.linalg.norm(q - p))
        poly, det = leg_path(p, q)
        n_det += det
        if det:
            flagged.append((i, ZONES[zz[i]], ZONES[zz[i + 1]], p.copy(), q.copy(),
                            seg_box_clearance(p, q), seg_hits_box(p, q, BLDG_LO, BLDG_HI)))
        for a, b in zip(poly[:-1], poly[1:]):
            seg = float(np.linalg.norm(b - a))
            L_flown += seg
            t_acc += leg_time(seg)
            max_leg = max(max_leg, seg)
            climb += max(b[2] - a[2], 0.0)
            assert not seg_hits_box(a, b) or seg == 0.0, 'flown segment crosses the building'
            flown.append(b)
    N = len(seq)
    return {
        'L_straight': L_straight, 'L': L_flown, 'detours': n_det,
        'T_card': L_flown / V_INSP + N * T_CAP, 'T_acc': t_acc + N * T_CAP,
        'max_leg': max_leg, 'climb': climb, 'flown': np.array(flown), 'flagged': flagged,
    }


# ── Orderings ─────────────────────────────────────────────────────────────────

def order_nn(wps, start_pt=LAUNCH):
    n = len(wps)
    left = np.ones(n, dtype=bool)
    cur = int(np.argmin(np.linalg.norm(wps - start_pt, axis=1)))
    order = [cur]
    left[cur] = False
    for _ in range(n - 1):
        d = np.linalg.norm(wps - wps[cur], axis=1)
        d[~left] = np.inf
        cur = int(np.argmin(d))
        order.append(cur)
        left[cur] = False
    return np.array(order)


def two_opt(wps, order):
    """Open-path 2-opt (first point fixed), Euclidean cost, vectorised over j."""
    order = order.copy()
    n = len(order)
    improved = True
    passes = 0
    while improved:
        improved = False
        passes += 1
        for i in range(1, n - 1):
            P = wps[order]
            a, b = P[i - 1], P[i]
            c = P[i:n]                      # candidate c = P[j], j >= i
            d_next = np.vstack([P[i + 1:n], P[n - 1:n]])   # P[j+1] (last: repeat, no edge)
            has_next = np.arange(i, n) < n - 1
            old = np.linalg.norm(b - a) + np.where(has_next, np.linalg.norm(d_next - c, axis=1), 0.0)
            new = np.linalg.norm(c - a, axis=1) + np.where(has_next, np.linalg.norm(d_next - b, axis=1), 0.0)
            gain = old - new
            gain[0] = 0.0
            j_rel = int(np.argmax(gain))
            if gain[j_rel] > 1e-9:
                j = i + j_rel
                order[i:j + 1] = order[i:j + 1][::-1]
                improved = True
    return order, passes


def order_structured(wps, zone, strips, start_pt=LAUNCH):
    """Ring pattern: at each strip altitude fly once around the building counter-clockwise
    (West -y, South +x, East +y, North -x), climb to the next strip, and finally fly the
    roof grid as a boustrophedon starting from the corner nearest the last facade capture.
    """
    index = {tuple(np.round(p, 9)): i for i, p in enumerate(wps)}
    sort_key = {'West': (1, -1), 'South': (0, 1), 'East': (1, 1), 'North': (0, -1)}
    order = []
    n_strips = len(strips['West'])
    for k in range(n_strips):
        for name in ['West', 'South', 'East', 'North']:
            s = strips[name][k]
            ax_, sign = sort_key[name]
            for p in s[np.argsort(sign * s[:, ax_], kind='stable')]:
                order.append(index[tuple(np.round(p, 9))])
    cur = wps[order[-1]]
    rows = strips['Roof']
    best = None
    for flip_rows in (False, True):
        rr = rows[::-1] if flip_rows else rows
        for flip_dir in (False, True):
            first = rr[0][::-1] if flip_dir else rr[0]
            d = np.linalg.norm(first[0] - cur)
            if best is None or d < best[0]:
                best = (d, flip_rows, flip_dir)
    _, flip_rows, flip_dir = best
    rr = rows[::-1] if flip_rows else rows
    for k, s in enumerate(rr):
        seq = s[::-1] if (k % 2 == 0) == flip_dir else s
        for p in seq:
            order.append(index[tuple(np.round(p, 9))])
    return np.array(order)


# ── GPS overlap Monte Carlo ───────────────────────────────────────────────────

def overlap_mc(strips, sigma, rng):
    """Achieved forward / side overlap and GSD when each capture is displaced by GPS noise.

    Facade captures only. Forward overlap between neighbours on a strip:
    1 - |du| / mean footprint width; side overlap between vertically adjacent captures:
    1 - |dz| / mean footprint height; footprints scale with the actual standoff.
    """
    fwd, side, gsd = [], [], []
    for fid, name in enumerate(ZONES[:4]):
        S = np.array(strips[name])                       # (n_strips, n_pass, 3)
        noisy = S + rng.normal(0, sigma, S.shape)
        axis_along = 0 if fid in (0, 2) else 1
        axis_norm = 1 - axis_along
        fixed = S[0, 0, axis_norm]
        wall = math.copysign(BLDG_W / 2 if axis_norm == 1 else BLDG_L / 2, fixed)
        d = np.abs(noisy[..., axis_norm] - wall)        # actual standoff
        wf = 2 * d * math.tan(FOV_H / 2)
        hf = 2 * d * math.tan(FOV_V / 2)
        du = np.abs(np.diff(noisy[..., axis_along], axis=1))
        fwd.append((1 - du / (0.5 * (wf[:, 1:] + wf[:, :-1]))).ravel())
        dz = np.abs(np.diff(noisy[..., 2], axis=0))
        side.append((1 - dz / (0.5 * (hf[1:] + hf[:-1]))).ravel())
        gsd.append((SW_M * d / (F_M * W_PX)).ravel())
    return np.concatenate(fwd), np.concatenate(side), np.concatenate(gsd)


def run_simulation():
    wps, zone, strips = all_waypoints()
    orders = {'structured': order_structured(wps, zone, strips), 'nn': order_nn(wps)}
    orders['nn2opt'], passes = two_opt(wps, orders['nn'])
    for o in orders.values():
        assert sorted(o.tolist()) == list(range(len(wps)))
    ev = {k: evaluate(wps, v, zone) for k, v in orders.items()}
    rng = np.random.default_rng(MC_SEED)
    mc = {}
    for sg in GPS_LEVELS:
        f, s, g = [], [], []
        for _ in range(N_MC):
            a, b, c = overlap_mc(strips, sg, rng)
            f.append(a)
            s.append(b)
            g.append(c)
        mc[sg] = (np.concatenate(f), np.concatenate(s), np.concatenate(g))
    nominal = overlap_mc(strips, 0.0, np.random.default_rng(0))
    return wps, zone, strips, orders, ev, passes, mc, nominal


# ── Plotting ──────────────────────────────────────────────────────────────────

def draw_building(ax, alpha=0.12):
    hl, hw, h = BLDG_L / 2, BLDG_W / 2, BLDG_H
    v = [[[-hl, -hw, 0], [hl, -hw, 0], [hl, -hw, h], [-hl, -hw, h]],
         [[-hl, hw, 0], [hl, hw, 0], [hl, hw, h], [-hl, hw, h]],
         [[-hl, -hw, 0], [-hl, hw, 0], [-hl, hw, h], [-hl, -hw, h]],
         [[hl, -hw, 0], [hl, hw, 0], [hl, hw, h], [hl, -hw, h]],
         [[-hl, -hw, h], [hl, -hw, h], [hl, hw, h], [-hl, hw, h]]]
    ax.add_collection3d(Poly3DCollection(v, alpha=alpha, facecolor='grey', edgecolor='dimgrey',
                                         linewidth=0.6))


def _setup3d(ax):
    ax.set_xlim(-16, 16)
    ax.set_ylim(-14, 14)
    ax.set_zlim(0, 16)
    ax.set_box_aspect((32, 28, 16))
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.view_init(elev=24, azim=-55)


def plot_waypoints_paths(wps, zone, ev, out_dir):
    fig = plt.figure(figsize=(18, 6.5))
    for n, k in enumerate(ORDERS):
        ax = fig.add_subplot(1, 3, n + 1, projection='3d')
        draw_building(ax)
        for zi, (name, col) in enumerate(zip(ZONES, ZONE_COLORS)):
            m = zone == zi
            ax.scatter(*wps[m].T, s=4, c=col, depthshade=False, label=name)
        F = ev[k]['flown']
        ax.plot(*F.T, color='k', lw=0.6, alpha=0.6)
        ax.scatter(*LAUNCH, marker='*', s=150, color='k', label='launch')
        _setup3d(ax)
        e = ev[k]
        ax.set_title(f"{ORDER_LABELS[k]}\nL = {e['L']:.0f} m, {e['detours']} detours, "
                     f"T = {e['T_card'] / 60:.1f} min", fontsize=10)
        if n == 0:
            ax.legend(loc='upper left', fontsize=7, markerscale=2)
    fig.suptitle('S065 Capture waypoints (colour = zone) and the flown path of each ordering',
                 y=1.03)
    plt.tight_layout()
    path = os.path.join(out_dir, 'waypoints_paths.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_strip_layout(strips, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for ax, name, col in zip(axes.flat, ZONES[:4], ZONE_COLORS[:4]):
        S = np.array(strips[name])
        along = S[..., 0] if name in ('South', 'North') else S[..., 1]
        for k in range(S.shape[0]):
            for u in along[k]:
                ax.add_patch(plt.Rectangle((u - W_FOOT / 2, S[k, 0, 2] - H_FOOT / 2), W_FOOT, H_FOOT,
                                           fill=False, ec=col, lw=0.3, alpha=0.5))
        ax.scatter(along.ravel(), S[..., 2].ravel(), s=10, color=col, zorder=3)
        half = BLDG_L / 2 if name in ('South', 'North') else BLDG_W / 2
        ax.add_patch(plt.Rectangle((-half, 0), 2 * half, BLDG_H, fc='grey', alpha=0.15, ec='k'))
        ax.axhline(BLDG_H, color='grey', ls='--', lw=0.8)
        ax.axhline(Z_MIN, color='grey', ls=':', lw=0.8)
        ax.set_xlim(-half - 3, half + 3)
        ax.set_ylim(-1.5, BLDG_H + 2.5)
        ax.set_aspect('equal')
        ax.set_title(f'{name}: {S.shape[0]} strips × {S.shape[1]} captures = {S.shape[0] * S.shape[1]}',
                     fontsize=10)
        ax.set_xlabel('along facade (m)')
        ax.set_ylabel('z (m)')
    fig.suptitle(f'S065 Facade strip layout: capture centres and footprints '
                 f'({W_FOOT:.2f} × {H_FOOT:.2f} m at {D_SCAN:.0f} m); grey = facade')
    plt.tight_layout()
    path = os.path.join(out_dir, 'strip_layout.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_ordering_comparison(wps, zone, orders, ev, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.3))
    names = [ORDER_LABELS[k] for k in ORDERS]
    cols = [ORDER_COLORS[k] for k in ORDERS]
    for ax, (f, lab, scale) in zip(axes[:3], [('L', 'flown path length (m)', 1),
                                              ('T_acc', 'mission time incl. accel. (min)', 1 / 60),
                                              ('detours', 'legs inside the 1 m safety margin', 1)]):
        vals = [ev[k][f] * scale for k in ORDERS]
        b = ax.bar(range(3), vals, color=cols, edgecolor='k')
        ax.bar_label(b, fmt='%.1f' if f != 'detours' else '%d', fontsize=9)
        ax.set_xticks(range(3))
        ax.set_xticklabels(names, fontsize=7.5)
        ax.set_title(lab)
        ax.grid(axis='y', alpha=0.3)
    ax = axes[3]
    width = 0.27
    for n, k in enumerate(ORDERS):
        seq = wps[orders[k]]
        zz = zone[orders[k]]
        legs = np.linalg.norm(np.diff(seq, axis=0), axis=1)
        per = [legs[zz[:-1] == zi].sum() for zi in range(5)]
        ax.bar(np.arange(5) + (n - 1) * width, per, width, color=ORDER_COLORS[k], label=ORDER_LABELS[k])
    ax.set_xticks(range(5))
    ax.set_xticklabels(ZONES)
    ax.set_title('straight-line leg length by source zone (m)')
    ax.legend(fontsize=7)
    ax.grid(axis='y', alpha=0.3)
    fig.suptitle(f'S065 Ordering comparison ({len(wps)} waypoints, same capture set)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'ordering_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_gps_overlap(mc, nominal, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.5))
    for sg, c in zip(GPS_LEVELS, ['#2CA02C', '#1F77B4', '#E87722', '#D62728']):
        f, s, g = mc[sg]
        axes[0].hist(100 * f, bins=80, histtype='step', color=c, density=True, label=f'σ = {sg} m')
        axes[1].hist(100 * s, bins=80, histtype='step', color=c, density=True, label=f'σ = {sg} m')
        axes[2].hist(1000 * g, bins=80, histtype='step', color=c, density=True, label=f'σ = {sg} m')
    axes[0].axvline(100 * P_X, color='k', ls='--', lw=1, label='60 % requirement')
    axes[0].axvline(100 * nominal[0].min(), color='grey', ls=':', lw=1, label='planned (no noise)')
    axes[1].axvline(100 * P_Z, color='k', ls='--', lw=1, label='30 % requirement')
    axes[2].axvline(1000 * GSD_ACH, color='grey', ls=':', lw=1, label='planned')
    axes[0].set_xlabel('forward overlap between neighbouring captures (%)')
    axes[1].set_xlabel('side overlap between adjacent strips (%)')
    axes[2].set_xlabel('GSD (mm/px)')
    axes[0].set_xlim(-30, 101)
    axes[1].set_xlim(-60, 101)
    for ax in axes:
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)
    fig.suptitle(f'S065 Achieved overlap and GSD with GPS error at the capture points '
                 f'({N_MC} Monte Carlo draws per σ, facade captures)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'gps_overlap.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(wps, zone, orders, ev, best, out_dir):
    seq = wps[orders[best]]
    zz = zone[orders[best]]
    n = len(seq)
    n_frames = int(math.ceil((n - 1) / ANIM_STRIDE)) + 1
    fig = plt.figure(figsize=(6.5, 5.8))
    ax = fig.add_subplot(111, projection='3d')
    draw_building(ax)
    ax.scatter(*wps.T, s=2, c='lightgrey', depthshade=False)
    _setup3d(ax)
    line, = ax.plot([], [], [], color='red', lw=0.9)
    dot, = ax.plot([], [], [], 'o', color='red', ms=6)
    done = ax.scatter([], [], [], s=4)

    def update(fi):
        i = min(fi * ANIM_STRIDE, n - 1)
        line.set_data(seq[:i + 1, 0], seq[:i + 1, 1])
        line.set_3d_properties(seq[:i + 1, 2])
        dot.set_data([seq[i, 0]], [seq[i, 1]])
        dot.set_3d_properties([seq[i, 2]])
        done._offsets3d = (seq[:i + 1, 0], seq[:i + 1, 1], seq[:i + 1, 2])
        done.set_color([ZONE_COLORS[z] for z in zz[:i + 1]])
        ax.set_title(f'S065 {ORDER_LABELS[best]} — capture {i + 1} / {n} ({ZONES[zz[i]]})',
                     fontsize=10)
        return line, dot, done

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12)
    plt.close(fig)
    print(f'Saved: {path}  ({n_frames} frames)')


if __name__ == '__main__':
    wps, zone, strips, orders, ev, passes, mc, nominal = run_simulation()
    print('\n=== S065 Building 3D Modeling — Multi-Angle Scan Path — Results ===')
    fov_from_f = 2 * math.degrees(math.atan(SW_M / (2 * F_M)))
    print(f'  GSD at {D_SCAN:.0f} m: {GSD_ACH * 1000:.3f} mm/px = {GSD_ACH * 100:.4f} cm '
          f'(limit {GSD_MAX * 100:.0f} cm); max standoff for 2 cm GSD: '
          f'{GSD_MAX * F_M * W_PX / SW_M:.1f} m')
    print(f'  Horizontal FOV implied by f and sensor width: {fov_from_f:.1f} deg (card uses 60)')
    print(f'  Footprint {W_FOOT:.3f} x {H_FOOT:.3f} m; B_x = {B_X:.3f} m; dz = {DZ_STEP:.3f} m')
    for name in ZONES:
        S = strips[name]
        print(f'  {name:5s}: {len(S)} rows x {len(S[0])} = {len(S) * len(S[0])} waypoints')
    S0 = np.array(strips['South'])
    E0 = np.array(strips['East'])
    print(f"  Facade strip altitudes: {', '.join(f'{z:.2f}' for z in S0[:, 0, 2])} m")
    print(f'  Actual capture spacing: long facades {S0[0, 1, 0] - S0[0, 0, 0]:.3f} m, '
          f'short facades {E0[0, 1, 1] - E0[0, 0, 1]:.3f} m')
    print(f'  Total waypoints: {len(wps)}')
    print(f'  Planned overlap (no noise): forward min {100 * nominal[0].min():.1f} %, '
          f'side min {100 * nominal[1].min():.1f} %')
    print(f'  2-opt passes: {passes}')
    for k in ORDERS:
        e = ev[k]
        print(f'  [{ORDER_LABELS[k]}]')
        print(f"    Straight-line tour length   : {e['L_straight']:.1f} m "
              f"(card formula time {e['L_straight'] / V_INSP + len(wps) * T_CAP:.1f} s)")
        print(f"    Legs inside the 1 m margin  : {e['detours']}")
        for i, z0, z1, p, q, clr, hit in e['flagged']:
            print(f"      leg {i}: {z0} -> {z1}, ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) -> "
                  f"({q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f}); min clearance to the building "
                  f"{clr:.3f} m; crosses the building box: {hit}")
        print(f"    Flown length (with detours) : {e['L']:.1f} m")
        print(f"    Mission time, card formula  : {e['T_card']:.1f} s ({e['T_card'] / 60:.1f} min)")
        print(f"    Mission time, 1 m/s^2 accel : {e['T_acc']:.1f} s ({e['T_acc'] / 60:.1f} min)")
        print(f"    Longest flown segment       : {e['max_leg']:.1f} m; total climb {e['climb']:.1f} m")
    s, nn, n2 = ev['structured'], ev['nn'], ev['nn2opt']
    print(f"  NN vs structured: {100 * (nn['L'] / s['L'] - 1):+.1f} % flown length; "
          f"2-opt vs structured: {100 * (n2['L'] / s['L'] - 1):+.1f} %")
    print(f'  GPS Monte Carlo ({N_MC} draws, facade captures):')
    for sg in GPS_LEVELS:
        f, sd, g = mc[sg]
        print(f'    sigma {sg:.2f} m: forward pairs below 50 %: {int(np.sum(f < 0.5))} of {f.size} '
              f'(min {100 * f.min():.3f} %)')
        print(f'    sigma {sg:.2f} m: forward overlap < 60 %: {100 * np.mean(f < P_X):.1f} % of pairs '
              f'(< 50 %: {100 * np.mean(f < 0.5):.3f} %); '
              f'side overlap < 30 %: {100 * np.mean(sd < P_Z):.1f} % (min {100 * sd.min():.1f} %); '
              f'GSD max {1000 * g.max():.3f} mm')
    best = min(ORDERS, key=lambda k: (ev[k]['detours'] > 0, ev[k]['T_acc']))
    print(f'  Animated ordering: {ORDER_LABELS[best]}')

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_waypoints_paths(wps, zone, ev, out)
    plot_strip_layout(strips, out)
    plot_ordering_comparison(wps, zone, orders, ev, out)
    plot_gps_overlap(mc, nominal, out)
    save_animation(wps, zone, orders, ev, best, out)
