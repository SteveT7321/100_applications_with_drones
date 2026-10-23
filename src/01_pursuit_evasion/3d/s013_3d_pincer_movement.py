"""
S013 3D Upgrade — Pincer Movement (trident)
============================================
Three pursuers hold slots on a tilted equilateral triangle centred on the
evader (card):
    p_i^target = p_E(t − τ) + R(t)·R_x(ψ_tilt)·[cos(2πi/3), sin(2πi/3), 0]
    R(t) = max(R0 − v_shrink·t, R_MIN),  R_MIN = 0.1 m < r_capture = 0.15 m
with the card's tilt law ψ̇ = k_tilt·(atan2(z_E − z̄_P, d_xy) − ψ).  τ = 0 is
the card law (slots on the evader's current position); τ > 0 models a
tracking/communication delay.

Evader: K = 200 random directions per step, maximises the minimum angle to
the pursuers (card), with 20° switching hysteresis and the floor/ceiling
treated as blockers (no clip artefacts).

Comparisons (20 seeds each, common random numbers):
  trident (card, adaptive tilt) · trident with fixed 45° tilt ·
  2D two-pursuer pincer flown in 3D · 2D pincer in a planar world (the
  original 2D geometry) · tetrahedron (card extension 1, 4 pursuers),
  each at τ = 0, 0.05, 0.10 s.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s013_3d_pincer_movement.py
"""
import sys, os, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'outputs',
    '01_pursuit_evasion', '3d', 's013_3d_pincer_movement',
)
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters (card unless noted) ───────────────────────────
R0         = 3.0      # m
R_MIN      = 0.1      # m  (≤ capture radius, so a held formation captures)
V_SHRINK   = 0.3      # m/s
V_PURSUER  = 5.0
V_EVADER   = 3.5
PSI_INIT   = np.radians(45.0)
K_TILT     = 0.5      # 1/s
PSI_LIMIT  = np.pi / 3
Z_MIN, Z_MAX = 0.5, 6.0
DT         = 0.05     # s
MAX_TIME   = 30.0     # s
CAPTURE_R  = 0.15     # m
K_CAND     = 200
GAP_HYST   = np.radians(20.0)
WALL_MARGIN = 0.6     # m
TAU_LIST   = [0.0, 0.05, 0.10]
N_SEEDS    = 20
MAIN_SEED  = 0

INIT_E = np.array([2.0, 0.0, 2.0])
INIT_TRIDENT = np.array([[-4.0, -2.0, 4.0],    # P0: above-right
                         [-4.0,  3.0, 2.0],    # P1: lateral
                         [-4.0, -2.0, 0.5]])   # P2: below (card 0.5 m)
INIT_PINCER = np.array([[-4.0, -2.0, 2.0],     # 2D S013 starts
                        [-4.0,  2.0, 2.0]])
INIT_TETRA = np.vstack([INIT_TRIDENT, [[-4.0, 0.0, 3.0]]])

FORMATIONS = ['trident', 'trident_fixed', 'pincer3d', 'pincer2d', 'tetra']
FORM_LABEL = {'trident': '3D trident (card, adaptive tilt)',
              'trident_fixed': '3D trident, fixed 45° tilt',
              'pincer3d': '2D pincer (2 drones) in 3D',
              'pincer2d': '2D pincer, planar world',
              'tetra': 'Tetrahedron (4 drones, card ext. 1)'}
P_COLORS = ['red', 'darkorange', 'firebrick', 'saddlebrown']
TETRA_DIRS = np.array([[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]]) / np.sqrt(3)
MEAS_DIRS = None   # filled below


def fibonacci_sphere(n):
    i = np.arange(n); y = 1 - 2 * i / (n - 1); r = np.sqrt(np.maximum(1 - y * y, 0))
    ph = i * np.pi * (3 - np.sqrt(5))
    return np.column_stack([r * np.cos(ph), r * np.sin(ph), y])


MEAS_DIRS = fibonacci_sphere(2000)


# ── Geometry helpers ─────────────────────────────────────────

def rotation_x(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def slot_dirs(form, psi):
    if form in ('trident', 'trident_fixed'):
        a = 2 * np.pi * np.arange(3) / 3
        return (rotation_x(psi) @ np.vstack([np.cos(a), np.sin(a), np.zeros(3)])).T
    if form in ('pincer3d', 'pincer2d'):
        return np.array([[1.0, 0, 0], [-1.0, 0, 0]])
    return TETRA_DIRS


def init_positions(form):
    return {'trident': INIT_TRIDENT, 'trident_fixed': INIT_TRIDENT, 'pincer3d': INIT_PINCER,
            'pincer2d': INIT_PINCER, 'tetra': INIT_TETRA}[form].copy()


def blocking_plane_normal(pts):
    n = np.cross(pts[1] - pts[0], pts[2] - pts[0])
    nn = np.linalg.norm(n)
    return n / nn if nn > 1e-8 else np.array([0., 0., 1.])


def bearings(pos_e, pos_p):
    d = pos_p - pos_e
    return d / (np.linalg.norm(d, axis=1, keepdims=True) + 1e-12)


def wall_bearings(pos_e, R, planar):
    if planar:
        return np.zeros((0, 3))
    w = []
    if pos_e[2] - Z_MIN < R:
        w.append([0, 0, -1.0])
    if Z_MAX - pos_e[2] < R:
        w.append([0, 0, 1.0])
    return np.array(w, dtype=float).reshape(-1, 3)


def escape_solid_angle(pos_e, pos_p):
    """Measured: 4π × fraction of directions outside every pursuer's capture
    cone (half-angle asin(r_cap / d_i))."""
    b = bearings(pos_e, pos_p)
    d = np.linalg.norm(pos_p - pos_e, axis=1)
    half = np.arcsin(np.clip(CAPTURE_R / np.maximum(d, CAPTURE_R), 0, 1))
    ang = np.arccos(np.clip(MEAS_DIRS @ b.T, -1, 1))
    blocked = (ang <= half[np.newaxis, :]).any(axis=1)
    return 4 * np.pi * (1 - blocked.mean())


def card_solid_angle(R, n):
    return max(4 * np.pi - n * np.pi * (CAPTURE_R / R) ** 2, 0.0)


def max_gap_deg(pos_e, pos_p, R, planar):
    b = np.vstack([bearings(pos_e, pos_p), wall_bearings(pos_e, R, planar)])
    dirs = MEAS_DIRS if not planar else np.column_stack(
        [np.cos(np.linspace(0, 2 * np.pi, 720, endpoint=False)),
         np.sin(np.linspace(0, 2 * np.pi, 720, endpoint=False)), np.zeros(720)])
    return float(np.degrees(np.arccos(np.clip(dirs @ b.T, -1, 1)).min(axis=1).max()))


# ── Evader (card max-min angle, committed, wall-aware) ───────

def evader_direction(pos_e, pos_p, R, u_cur, rng, planar):
    b = np.vstack([bearings(pos_e, pos_p), wall_bearings(pos_e, R, planar)])
    u = rng.standard_normal((K_CAND, 3))
    if planar:
        u[:, 2] = 0.0
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    low = (pos_e[2] - Z_MIN) < WALL_MARGIN
    high = (Z_MAX - pos_e[2]) < WALL_MARGIN
    ok = ~((low & (u[:, 2] < 0)) | (high & (u[:, 2] > 0)))
    u = u[ok]
    score = np.arccos(np.clip(u @ b.T, -1, 1)).min(axis=1)
    best = int(np.argmax(score))
    if u_cur is not None and not ((low and u_cur[2] < 0) or (high and u_cur[2] > 0)):
        if score[best] <= np.arccos(np.clip(b @ u_cur, -1, 1)).min() + GAP_HYST:
            return u_cur
    return u[best]


# ── Simulation ───────────────────────────────────────────────

def run_simulation(form='trident', tau=0.0, seed=MAIN_SEED, static=False, log=True):
    rng = np.random.default_rng(seed)
    planar = form == 'pincer2d'
    lag = int(round(tau / DT))
    pos_p = init_positions(form)
    pos_e = INIT_E.copy()
    hist_e = [pos_e.copy()]
    psi = PSI_INIT
    u_cur = None
    L = dict(t=[0.0], te=[pos_e.copy()], tp=[pos_p.copy()], psi=[psi], R=[R0],
             om_meas=[escape_solid_angle(pos_e, pos_p)], om_card=[card_solid_angle(R0, len(pos_p))],
             normal=[blocking_plane_normal(pos_p) if len(pos_p) >= 3 else np.full(3, np.nan)],
             dplane=[np.nan], cos_vn=[np.nan], gap=[max_gap_deg(pos_e, pos_p, R0, planar)])
    captured, cap_time, cap_who = False, None, None
    max_gap = L['gap'][0]

    for step in range(1, int(round(MAX_TIME / DT)) + 1):
        t = step * DT
        R = max(R0 - V_SHRINK * t, R_MIN)
        # evader reacts to the pursuers' current positions
        if not static:
            u_cur = evader_direction(pos_e, pos_p, R, u_cur, rng, planar)
            pos_e = pos_e + V_EVADER * u_cur * DT
            pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)
        hist_e.append(pos_e.copy())
        # card tilt law (trident only)
        if form == 'trident':
            c = pos_p.mean(axis=0)
            psi_cmd = np.arctan2(pos_e[2] - c[2], np.linalg.norm(pos_e[:2] - c[:2]) + 1e-8)
            psi = float(np.clip(psi + K_TILT * (psi_cmd - psi) * DT, -PSI_LIMIT, PSI_LIMIT))
        centre = hist_e[max(0, len(hist_e) - 1 - lag)]          # p_E(t − τ)
        tg = centre + R * slot_dirs(form, psi)
        tg[:, 2] = np.clip(tg[:, 2], Z_MIN, Z_MAX)
        d = tg - pos_p
        dn = np.linalg.norm(d, axis=1, keepdims=True)
        pos_p = pos_p + d / np.maximum(dn, 1e-12) * np.minimum(V_PURSUER, dn / DT) * DT

        dists = np.linalg.norm(pos_p - pos_e, axis=1)
        g = max_gap_deg(pos_e, pos_p, R, planar)
        max_gap = max(max_gap, g)
        if log:
            n = blocking_plane_normal(pos_p) if len(pos_p) >= 3 else np.full(3, np.nan)
            v = (hist_e[-1] - hist_e[-2]) / DT
            L['t'].append(t); L['te'].append(pos_e.copy()); L['tp'].append(pos_p.copy())
            L['psi'].append(psi); L['R'].append(R); L['gap'].append(g)
            L['om_meas'].append(escape_solid_angle(pos_e, pos_p))
            L['om_card'].append(card_solid_angle(R, len(pos_p)))
            L['normal'].append(n)
            L['dplane'].append(float((pos_e - pos_p[0]) @ n) if len(pos_p) >= 3 else np.nan)
            L['cos_vn'].append(abs(float(v @ n)) / (np.linalg.norm(v) + 1e-12)
                               if len(pos_p) >= 3 and np.linalg.norm(v) > 0 else np.nan)
        if dists.min() < CAPTURE_R:
            captured, cap_time, cap_who = True, t, int(np.argmin(dists))
            break

    te = np.array(hist_e)
    out = dict(captured=captured, cap_time=cap_time, cap_who=cap_who, max_gap=max_gap,
               final_dmin=float(dists.min()), z_range=(float(te[:, 2].min()), float(te[:, 2].max())),
               floor_steps=int(np.sum(te[:, 2] <= Z_MIN + 1e-9)))
    if log:
        out.update({k: np.array(v) for k, v in L.items()})
    return out


def run_comparison():
    res = {}
    for form in FORMATIONS:
        for tau in TAU_LIST:
            rs = [run_simulation(form, tau, s, log=False) for s in range(N_SEEDS)]
            ct = [r['cap_time'] for r in rs if r['captured']]
            t_sched = (R0 - CAPTURE_R) / V_SHRINK
            res[(form, tau)] = dict(n_cap=len(ct), mean=float(np.mean(ct)) if ct else None,
                                    early=sum(c < t_sched - DT / 2 for c in ct),
                                    rng=(min(ct), max(ct)) if ct else None,
                                    sd=float(np.std(ct)) if ct else None,
                                    dmin=float(np.median([r['final_dmin'] for r in rs if not r['captured']]))
                                    if len(ct) < N_SEEDS else None,
                                    floor=sum(r['floor_steps'] > 0 for r in rs))
    return res


# ── Plots ────────────────────────────────────────────────────

def _status(r):
    return f"captured @ {r['cap_time']:.2f} s" if r['captured'] else 'not captured in 30 s'


def plot_trajectories_3d(runs, labels, out_dir):
    fig = plt.figure(figsize=(15, 7))
    for j, (r, lab) in enumerate(zip(runs, labels)):
        ax = fig.add_subplot(1, len(runs), j + 1, projection='3d'); ax.view_init(elev=20, azim=-60)
        te, tp = r['te'], r['tp']
        ax.plot(te[:, 0], te[:, 1], te[:, 2], color='royalblue', lw=2.0, label='Evader')
        ax.scatter(*te[0], color='blue', s=50)
        for i in range(tp.shape[1]):
            ax.plot(tp[:, i, 0], tp[:, i, 1], tp[:, i, 2], color=P_COLORS[i], lw=1.3, label=f'P{i}')
            ax.scatter(*tp[0, i], color=P_COLORS[i], s=40, marker='^')
        if tp.shape[1] >= 3:
            for ts, sc in zip([0.0, 3.0, 6.0], ['grey', 'peru', 'olive']):
                k = min(int(round(ts / DT)), len(tp) - 1)
                tri = Poly3DCollection([[tuple(p) for p in tp[k, :3]]], alpha=0.2, facecolor=sc, edgecolor=sc)
                ax.add_collection3d(tri)
                ax.text(*tp[k, :3].mean(axis=0), f't={r["t"][k]:.0f}s', fontsize=7, color=sc)
        if r['captured']:
            ax.scatter(*te[-1], color='black', s=120, marker='X', label='Capture')
        ax.set_zlim(Z_MIN - 0.2, Z_MAX + 0.2)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title(f'{lab}\n{_status(r)}', fontsize=9); ax.legend(fontsize=7, loc='upper left')
    fig.suptitle('S013 3D Pincer — trident trajectories (seed 0)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=110, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_altitude_time(r, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))
    for i in range(r['tp'].shape[1]):
        ax.plot(r['t'], r['tp'][:, i, 2], color=P_COLORS[i], lw=1.3, label=f'P{i} z')
    ax.plot(r['t'], r['te'][:, 2], color='royalblue', lw=2.6, ls=(0, (4, 2)), zorder=5, label='Evader z')
    ax2 = ax.twinx()
    ax2.plot(r['t'], np.degrees(r['psi']), color='grey', ls='--', lw=1.2, label='tilt ψ (right axis)')
    ax2.set_ylabel('Tilt ψ (deg)')
    ax.axhline(Z_MIN, color='grey', ls=':', lw=0.8); ax.axhline(Z_MAX, color='grey', ls=':', lw=0.8)
    if r['captured']:
        ax.axvline(r['cap_time'], color='black', ls='--', lw=1.0)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude z (m)')
    h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, ncol=3); ax.grid(True, alpha=0.3)
    ax.set_title(f'S013 3D Pincer — Altitude and Tilt vs Time (trident, τ=0, {_status(r)})', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_distance_time(runs, labels, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))
    for r, lab, ls in zip(runs, labels, ['-', '--']):
        d = np.linalg.norm(r['tp'] - r['te'][:, None, :], axis=2)
        for i in range(d.shape[1]):
            ax.plot(r['t'], d[:, i], color=P_COLORS[i], ls=ls, lw=1.4, label=f'P{i} — {lab}')
    ax.plot(runs[0]['t'], runs[0]['R'], color='steelblue', lw=1.0, label='R(t)')
    ax.axhline(CAPTURE_R, color='red', ls=':', lw=1.0, label=f'capture {CAPTURE_R} m')
    ax.set_yscale('log'); ax.set_xlabel('Time (s)'); ax.set_ylabel('Distance to evader (m)')
    ax.set_title('S013 3D Pincer — Pursuer Distances (trident)', fontsize=10)
    ax.legend(fontsize=7, ncol=2); ax.grid(True, alpha=0.3, which='both')
    plt.tight_layout()
    path = os.path.join(out_dir, 'distance_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_solid_angle(runs, labels, out_dir):
    fig, ax = plt.subplots(figsize=(11, 5))
    for r, lab, ls in zip(runs, labels, ['-', '--']):
        ax.plot(r['t'], r['om_meas'] / (4 * np.pi), color='royalblue', ls=ls, lw=1.6,
                label=f'measured Ω_escape/4π — {lab}')
    ax.plot(runs[0]['t'], runs[0]['om_card'] / (4 * np.pi), color='black', lw=1.2,
            label='card formula 1 − 3(r_cap/R)²/4')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Escape solid angle / 4π')
    ax.set_title('S013 3D Pincer — Escape Solid Angle (unblocked by capture cones)', fontsize=10)
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'solid_angle.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_blocking_normal(runs, labels, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    r = runs[0]
    for k, (c, lab) in enumerate(zip(['r', 'g', 'b'], ['n_x', 'n_y', 'n_z'])):
        axes[0].plot(r['t'], r['normal'][:, k], color=c, lw=1.3, label=lab)
    axes[0].set_ylabel('Blocking-plane\nnormal'); axes[0].legend(fontsize=8, ncol=3)
    axes[0].set_title(f'S013 3D Pincer — Blocking Plane ({labels[0]})', fontsize=10)
    for rr, lab, ls in zip(runs, labels, ['-', '--']):
        axes[1].plot(rr['t'], rr['cos_vn'], color='royalblue', ls=ls, lw=1.2, label=lab)
        axes[2].plot(rr['t'], rr['dplane'], color='royalblue', ls=ls, lw=1.2, label=lab)
    axes[1].set_ylabel('|cos ∠(v_E, n)|'); axes[1].set_ylim(-0.05, 1.05); axes[1].legend(fontsize=8)
    axes[2].axhline(0, color='black', lw=0.8)
    axes[2].set_ylabel('Signed distance to\nplane d_plane (m)'); axes[2].set_xlabel('Time (s)')
    axes[2].legend(fontsize=8)
    for ax in axes:
        ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'blocking_normal.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_comparison(comp, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    x = np.arange(len(FORMATIONS)); w = 0.25
    cols = ['seagreen', 'orange', 'firebrick']
    for j, (tau, c) in enumerate(zip(TAU_LIST, cols)):
        axes[0].bar(x + (j - 1) * w, [100 * comp[(f, tau)]['n_cap'] / N_SEEDS for f in FORMATIONS],
                    w, color=c, label=f'τ = {tau:.2f} s')
        m = [comp[(f, tau)]['mean'] if comp[(f, tau)]['mean'] is not None else np.nan for f in FORMATIONS]
        s = [comp[(f, tau)]['sd'] if comp[(f, tau)]['sd'] is not None else 0 for f in FORMATIONS]
        axes[1].bar(x + (j - 1) * w, m, w, yerr=s, color=c, capsize=3, label=f'τ = {tau:.2f} s')
    for ax in axes:
        ax.set_xticks(x); ax.set_xticklabels([FORM_LABEL[f] for f in FORMATIONS], rotation=15,
                                             ha='right', fontsize=8)
        ax.grid(True, alpha=0.3, axis='y'); ax.legend(fontsize=8)
    axes[0].set_ylabel(f'Capture rate (%) over {N_SEEDS} seeds'); axes[0].set_ylim(0, 105)
    axes[1].set_ylabel('Capture time (s), captured seeds')
    axes[1].axhline((R0 - CAPTURE_R) / V_SHRINK, color='black', ls=':', lw=1,
                    label='R(t) = r_cap (9.50 s)')
    axes[1].legend(fontsize=8)
    fig.suptitle('S013 3D — Formation Comparison (2D pincer vs 3D trident vs tetrahedron), '
                 'tracking delay τ', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'formation_comparison.png')
    plt.savefig(path, dpi=110, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def save_animation(r, out_dir):
    te, tp, t = r['te'], r['tp'], r['t']
    frames = list(range(0, len(t), 2))
    if frames[-1] != len(t) - 1:
        frames.append(len(t) - 1)
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d'); ax.view_init(elev=20, azim=-60)
    allp = np.vstack([te, tp.reshape(-1, 3)])
    lo, hi = allp.min(0) - 0.5, allp.max(0) + 0.5
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(Z_MIN - 0.2, Z_MAX + 0.2)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    n = tp.shape[1]
    lines = [ax.plot([], [], [], color=P_COLORS[i], lw=1.2)[0] for i in range(n)]
    dots = [ax.plot([], [], [], '^', color=P_COLORS[i], ms=8)[0] for i in range(n)]
    le, = ax.plot([], [], [], color='royalblue', lw=2.0)
    de, = ax.plot([], [], [], 'bs', ms=8)
    tri = [ax.plot([], [], [], '-', color='grey', lw=1.0)[0] for _ in range(3)]
    title = ax.set_title('')

    def update(fi):
        si = frames[fi]
        le.set_data(te[:si+1, 0], te[:si+1, 1]); le.set_3d_properties(te[:si+1, 2])
        de.set_data([te[si, 0]], [te[si, 1]]); de.set_3d_properties([te[si, 2]])
        for i in range(n):
            lines[i].set_data(tp[:si+1, i, 0], tp[:si+1, i, 1]); lines[i].set_3d_properties(tp[:si+1, i, 2])
            dots[i].set_data([tp[si, i, 0]], [tp[si, i, 1]]); dots[i].set_3d_properties([tp[si, i, 2]])
        for k, (a, b) in enumerate([(0, 1), (1, 2), (2, 0)]):
            tri[k].set_data([tp[si, a, 0], tp[si, b, 0]], [tp[si, a, 1], tp[si, b, 1]])
            tri[k].set_3d_properties([tp[si, a, 2], tp[si, b, 2]])
        title.set_text(f'S013 3D trident (τ=0)  t={t[si]:.2f}s  R={r["R"][si]:.2f}m'
                       + ('  CAPTURED' if fi == len(frames) - 1 and r['captured'] else ''))
        return [le, de] + lines + dots + tri + [title]

    ani = FuncAnimation(fig, update, frames=len(frames), interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=15), dpi=90); plt.close('all')
    print(f'Saved: {path}  ({len(frames)} frames)')


# ── Main ─────────────────────────────────────────────────────

def _describe(lab, r):
    i = len(r['t']) - 1
    print(f"  {lab}: {_status(r)}; max gap {r['max_gap']:.1f}°; evader z {r['z_range'][0]:.2f}–"
          f"{r['z_range'][1]:.2f} m (steps on floor {r['floor_steps']}); final nearest pursuer "
          f"{r['final_dmin']:.3f} m; R at end {r['R'][i]:.2f} m; tilt at end {np.degrees(r['psi'][i]):.1f}°; "
          f"mean |cos(v_E, n)| {np.nanmean(r['cos_vn']):.3f}; Ω_esc/4π at end measured "
          f"{r['om_meas'][i] / (4 * np.pi):.3f}, card formula {r['om_card'][i] / (4 * np.pi):.3f}")


if __name__ == '__main__':
    print('S013 3D Pincer (trident), seed 0')
    main = run_simulation('trident', 0.0)
    lagged = run_simulation('trident', 0.05)
    _describe('trident τ=0 (card law)', main)
    _describe('trident τ=0.05 s', lagged)
    st = run_simulation('trident', 0.0, static=True)
    print(f"  static evader, trident τ=0: {_status(st)} (R(t) = r_cap at "
          f"{(R0 - CAPTURE_R) / V_SHRINK:.2f} s)")
    print(f"  card capture-certainty radius r_cap·sqrt(3π) = {CAPTURE_R * np.sqrt(3 * np.pi):.3f} m; "
          f"card formula Ω_escape = 0 at R = r_cap·sqrt(3/4) = {CAPTURE_R * np.sqrt(0.75):.3f} m")

    print(f'\nFormation comparison: {N_SEEDS} seeds (common random numbers), τ = {TAU_LIST} s')
    comp = run_comparison()
    print(f"{'formation':<38} {'tau':>5} {'captured':>9} {'capture time (s)':>18} {'range (s)':>13} "
          f"{'before R=r_cap':>15} {'median final dist, uncaptured (m)':>34} {'runs touching floor':>20}")
    for f in FORMATIONS:
        for tau in TAU_LIST:
            c = comp[(f, tau)]
            ct = f"{c['mean']:.2f} ± {c['sd']:.2f}" if c['mean'] is not None else '-'
            dm = f"{c['dmin']:.3f}" if c['dmin'] is not None else '-'
            rg = f"{c['rng'][0]:.2f}–{c['rng'][1]:.2f}" if c['rng'] is not None else '-'
            print(f"{FORM_LABEL[f]:<38} {tau:>5.2f} {c['n_cap']:>5}/{N_SEEDS} {ct:>18} {rg:>13} "
                  f"{c['early']:>15} {dm:>34} {c['floor']:>17}/{N_SEEDS}")

    labels = ['trident τ=0 (card law)', 'trident τ=0.05 s']
    plot_trajectories_3d([main, lagged], labels, OUTPUT_DIR)
    plot_altitude_time(main, OUTPUT_DIR)
    plot_distance_time([main, lagged], labels, OUTPUT_DIR)
    plot_solid_angle([main, lagged], labels, OUTPUT_DIR)
    plot_blocking_normal([main, lagged], labels, OUTPUT_DIR)
    plot_comparison(comp, OUTPUT_DIR)
    save_animation(main, OUTPUT_DIR)
    print('All outputs saved to:', OUTPUT_DIR)
