"""
S049 3D Dynamic Zone Assignment
===============================
Four drones search a 400 x 400 x 60 m rubble volume (80 x 80 x 12 voxels of
5 m) for 5 survivors placed at voxel centres sampled from a 3D Gaussian-mixture
prior concentrated near the ground.  A voxel is scanned (and a survivor
detected) when the drone is within r_s = 8 m horizontally and r_z = 2.5 m
vertically.  Each drone sweeps its zone bottom-up, layer by layer (the 2D
"nearest of the TOP_M highest-prior unscanned cells" rule inside the current
layer).  Battery: SoC = 1 - (d_xy + gamma d_z) / D_max per pack; below 30 %
the drone flies home, swaps the pack and rejoins; a finder hovers 10 s to
confirm (the 2D rules).

Strategies (same survivor placements for all):
  static          - initial 3D Voronoi partition, never changed
  equal           - re-partition of all unscanned voxels by 3D Voronoi on the
                    current 3D positions at every event
  weighted        - same with SoC-weighted 3D distance |x - p_k|_3 / SoC_k
  weighted_column - added: SoC-weighted, horizontal distance only (the 2D
                    partition extruded through all layers)

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s049_3d_dynamic_zone.py
"""

import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
import multiprocessing as mp
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.spatial import cKDTree

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR   = os.path.join(PROJECT_ROOT, 'outputs', '03_environmental_sar', '3d', 's049_3d_dynamic_zone')

# -- Parameters (card unless marked) -----------------------------------------------------
K_DRONES   = 4
AREA       = 400.0
DXY, DZ    = 5.0, 5.0
NXY, NZ    = 80, 12
Z0         = 2.5
SPEED      = 6.0      # m/s horizontal
VZ         = 2.0      # m/s vertical
R_S, R_Z   = 8.0, 2.5
D_MAX      = 3000.0
GAMMA      = 2.0
SOC_THRESH = 0.30
N_SURV     = 5
DT         = 0.5
T_MAX      = 3600.0
T_SWAP     = 60.0     # 2D value
T_CONFIRM  = 10.0     # 2D value
TOP_M      = 20       # 2D value
N_MC       = 20       # 2D value: survivor placements 0..19
SHOW_SEED  = 0
N_WORKERS  = 10
FRAME_EVERY = 48      # 24 s per animation frame
PRIOR = [  # alpha, (mu_x, mu_y, mu_z), sigma_xy, sigma_z  (added: card gives only ranges)
    (0.4, (100.0, 300.0, 2.5), 60.0, 8.0),
    (0.3, (280.0, 150.0, 2.5), 50.0, 5.0),
    (0.2, (340.0, 320.0, 10.0), 40.0, 15.0),
    (0.1, (200.0, 80.0, 5.0), 30.0, 10.0),
]
HOMES = np.array([[20.0, 20.0, 0.0], [380.0, 20.0, 0.0], [20.0, 380.0, 0.0], [380.0, 380.0, 0.0]])
STRATEGIES = ['static', 'equal', 'weighted', 'weighted_column']
STYLE = {'static': ('--', '#888888', 'Static 3D Voronoi'),
         'equal': ('-.', '#2196F3', 'Dynamic equal-weight 3D'),
         'weighted': ('-', '#E63946', 'Dynamic battery-weighted 3D'),
         'weighted_column': (':', '#2a9d8f', 'Dynamic battery-weighted, columnar (2D distance)')}
DRONE_COLORS = ['#E63946', '#2196F3', '#4CAF50', '#FF9800']

XY = (np.arange(NXY) + 0.5) * DXY
ZC = Z0 + np.arange(NZ) * DZ
_gz, _gx, _gy = np.meshgrid(np.arange(NZ), np.arange(NXY), np.arange(NXY), indexing='ij')
VOX_K, VOX_I, VOX_J = _gz.ravel(), _gx.ravel(), _gy.ravel()     # voxel v = (k, i, j)
VOX = np.c_[XY[VOX_I], XY[VOX_J], ZC[VOX_K]]                     # (76800, 3) centres
N_VOX = len(VOX)
CELL = np.c_[XY[np.arange(NXY).repeat(NXY)], XY[np.tile(np.arange(NXY), NXY)]]
CELL_TREE = cKDTree(CELL)


def build_prior():
    P = np.zeros(N_VOX)
    for a, mu, sxy, sz in PRIOR:
        d2 = ((VOX[:, 0] - mu[0]) ** 2 + (VOX[:, 1] - mu[1]) ** 2) / sxy ** 2 + (VOX[:, 2] - mu[2]) ** 2 / sz ** 2
        P += a * np.exp(-0.5 * d2) / (sxy ** 2 * sz)
    return P / P.sum()


def scan_voxels(p):
    """Voxel indices within r_s horizontally and r_z vertically of point p."""
    cells = CELL_TREE.query_ball_point(p[:2], R_S)
    ks = np.flatnonzero(np.abs(ZC - p[2]) <= R_Z + 1e-9)
    if len(cells) == 0 or len(ks) == 0:
        return np.empty(0, int)
    return (ks[:, None] * NXY * NXY + np.asarray(cells)[None, :]).ravel()


def partition(unsc, gens, weights, column):
    """Owner (index into gens) for each unscanned voxel: argmin |x - p|/w (3D or horizontal)."""
    V = VOX[unsc]
    if column:
        d = np.linalg.norm(V[:, None, :2] - gens[None, :, :2], axis=2)
    else:
        d = np.linalg.norm(V[:, None, :] - gens[None, :, :], axis=2)
    return np.argmin(d / np.maximum(weights, 1e-3)[None, :], axis=1)


def jain(x):
    x = np.asarray(x, float)
    return 1.0 if len(x) == 0 or x.sum() == 0 else float(x.sum() ** 2 / (len(x) * (x ** 2).sum()))


class Drone:
    def __init__(self, k):
        self.k = k; self.home = HOMES[k].copy(); self.pos = HOMES[k].copy()
        self.pack = 0.0; self.soc = 1.0; self.mode = 'search'; self.timer = 0.0
        self.target = None; self.n_swaps = 0; self.dxy = 0.0; self.dz = 0.0
        self.hist = [self.pos.copy()]; self.layer_hist = []

    @property
    def available(self):
        return self.mode in ('search', 'idle')

    def _drain(self, dxy, dz):
        self.pack += dxy + GAMMA * dz
        self.soc = 1.0 - self.pack / D_MAX
        assert self.soc > 0.0, 'battery empty in flight'

    def fly_to(self, q):
        """Move horizontally (<= SPEED*DT) and vertically (<= VZ*DT) towards q; return 3D remainder."""
        dh = q[:2] - self.pos[:2]; nh = float(np.linalg.norm(dh))
        mh = min(SPEED * DT, nh)
        dzv = q[2] - self.pos[2]; mz = min(VZ * DT, abs(dzv))
        if nh > 1e-9:
            self.pos[:2] += dh / nh * mh
        self.pos[2] += np.sign(dzv) * mz
        self.dxy += mh; self.dz += mz
        self._drain(mh, mz)
        return float(np.linalg.norm(q - self.pos))

    def hover(self):
        self._drain(SPEED * DT, 0.0)      # hover drains like cruise (2D rule)


def choose_target(d, owner, scanned, P):
    """Lowest layer with unscanned voxels of the zone; nearest of its TOP_M highest-prior voxels."""
    mine = np.flatnonzero((owner == d.k) & ~scanned)
    if len(mine) == 0:
        return None
    kmin = VOX_K[mine].min()
    cand = mine[VOX_K[mine] == kmin]
    if len(cand) > TOP_M:
        cand = cand[np.argpartition(-P[cand], TOP_M - 1)[:TOP_M]]
    dist = np.linalg.norm(VOX[cand] - d.pos, axis=1)
    return int(cand[np.argmin(dist)])


def run_simulation(strategy='weighted', seed=SHOW_SEED, record=True, stop_when_found=True):
    P = build_prior()
    rng = np.random.default_rng(seed)
    surv_idx = rng.choice(N_VOX, size=N_SURV, replace=False, p=P)
    surv = VOX[surv_idx]
    det_t = [None] * N_SURV
    drones = [Drone(k) for k in range(K_DRONES)]
    scanned = np.zeros(N_VOX, bool)
    column = strategy == 'weighted_column'

    def do_partition(avail, weighted):
        owner = np.full(N_VOX, -1, np.int16)
        unsc = np.flatnonzero(~scanned)
        if len(avail) == 0 or len(unsc) == 0:
            return owner
        gens = np.array([d.pos for d in avail])
        w = np.array([max(d.soc, 1e-3) for d in avail]) if weighted else np.ones(len(avail))
        owner[unsc] = np.array([d.k for d in avail])[partition(unsc, gens, w, column)]
        return owner

    def fairness():
        av = [d.k for d in drones if d.available]
        if not av:
            return np.nan
        rem = np.bincount(owner[owner >= 0], minlength=K_DRONES)
        return jain([rem[k] for k in av])

    owner = do_partition(drones, weighted=False)
    n_differs_column = []
    for d in drones:
        d.target = choose_target(d, owner, scanned, P)
    events_log, reparts, snaps = [], [], [(0.0, owner.copy(), scanned.copy())]
    t_log, cov_log, fair_log, frames = [], [], [], []
    t, step = 0.0, 0
    while t < T_MAX - 1e-9 and not scanned.all():
        if stop_when_found and all(x is not None for x in det_t):
            break
        events = []
        for d in drones:
            if d.mode == 'search':
                if d.soc < SOC_THRESH:
                    d.mode = 'return'; d.target = None; events.append(('battery', d.k))
                elif d.target is None or scanned[d.target]:
                    d.target = choose_target(d, owner, scanned, P)
                    if d.target is None:
                        d.mode = 'idle'; events.append(('zone_done', d.k))
                if d.mode == 'search':
                    d.fly_to(VOX[d.target])
                    scanned[scan_voxels(d.pos)] = True
                    for i, s in enumerate(surv):
                        if (det_t[i] is None and np.linalg.norm(d.pos[:2] - s[:2]) <= R_S
                                and abs(d.pos[2] - s[2]) <= R_Z + 1e-9):
                            det_t[i] = t + DT
                            if d.mode == 'search':
                                d.mode = 'confirm'; d.timer = T_CONFIRM; events.append(('survivor', d.k))
            elif d.mode == 'confirm':
                d.hover(); d.timer -= DT
                if d.timer <= 1e-9:
                    d.mode = 'search'; d.target = None; events.append(('confirm_done', d.k))
            elif d.mode == 'return':
                if d.fly_to(d.home) <= 1e-9:
                    d.mode = 'swap'; d.timer = T_SWAP
            elif d.mode == 'swap':
                d.timer -= DT
                if d.timer <= 1e-9:
                    d.pack = 0.0; d.soc = 1.0; d.n_swaps += 1; d.mode = 'search'; d.target = None
                    events.append(('swap_done', d.k))
            assert d.pos[2] >= -1e-9, 'below ground'
            d.hist.append(d.pos.copy())
            d.layer_hist.append(VOX_K[d.target] if (d.mode == 'search' and d.target is not None) else -1)
        t += DT; step += 1
        owner[scanned] = -1
        for e in events:
            events_log.append((t, *e))
        if strategy != 'static' and events and not scanned.all() and any(d.available for d in drones):
            jb = fairness()
            avail = [d for d in drones if d.available]
            owner = do_partition(avail, weighted=(strategy != 'equal'))
            if strategy == 'weighted':     # how different is the columnar (2D) partition?
                unsc = np.flatnonzero(~scanned)
                gens = np.array([d.pos for d in avail])
                w = np.array([max(d.soc, 1e-3) for d in avail])
                o3 = partition(unsc, gens, w, False); o2 = partition(unsc, gens, w, True)
                n_differs_column.append(float(np.mean(o3 != o2)))
            for d in avail:
                d.target = choose_target(d, owner, scanned, P)
                d.mode = 'search' if d.target is not None else 'idle'
            reparts.append((t, jb, fairness(), [round(float(d.soc), 3) for d in avail],
                            [round(float(d.pos[2]), 1) for d in avail]))
            if record:
                snaps.append((t, owner.copy(), scanned.copy()))
        if record:
            t_log.append(t); cov_log.append(scanned.mean()); fair_log.append(fairness())
            if step % FRAME_EVERY == 0:
                frames.append((t, np.array([d.pos for d in drones]), [x is not None for x in det_t],
                               scanned.copy()))
    found = [x for x in det_t if x is not None]
    return dict(strategy=strategy, seed=seed, surv=surv, det_t=det_t, t_end=t,
                n_found=len(found), t_all=max(found) if len(found) == N_SURV else np.nan,
                t_mean=float(np.mean(found)) if found else np.nan,
                coverage=float(scanned.mean()), reparts=reparts, events=events_log, snaps=snaps,
                t_log=np.array(t_log), cov_log=np.array(cov_log), fair_log=np.array(fair_log),
                frames=frames, n_swaps=sum(d.n_swaps for d in drones),
                dxy=[d.dxy for d in drones], dz=[d.dz for d in drones],
                hist=[np.array(d.hist) for d in drones] if record else None,
                layer_hist=[np.array(d.layer_hist) for d in drones] if record else None,
                differs_column=n_differs_column, P=P if record else None,
                surv_layer=[int(VOX_K[i]) for i in surv_idx])


def _task(args):
    s, seed = args
    rec = seed == SHOW_SEED
    return run_simulation(s, seed, record=rec, stop_when_found=not rec)


def run_all():
    tasks = [(s, sd) for sd in range(N_MC) for s in STRATEGIES]
    with mp.Pool(N_WORKERS) as pool:
        outs = pool.map(_task, tasks)
    return dict(zip(tasks, outs))


# -- Plots -----------------------------------------------------------------------------------

def _save(fig, name):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    p = os.path.join(OUTPUT_DIR, name)
    fig.savefig(p, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {p}')


def plot_volume_zones(R):
    r = R[('weighted', SHOW_SEED)]
    fig = plt.figure(figsize=(15, 6.5))
    sub = np.arange(0, N_VOX, 7)
    picks = [(0, 'initial 3D Voronoi (t = 0)')]
    if len(r['snaps']) > 1:
        picks.append((1, f're-partition 1 (t = {r["snaps"][1][0]:.1f} s)'))
    for c, (si, title) in enumerate(picks):
        t_s, own, sc = r['snaps'][si]
        ax = fig.add_subplot(1, 2, c + 1, projection='3d')
        m = sub[own[sub] >= 0]
        ax.scatter(*VOX[m].T, c=[DRONE_COLORS[o] for o in own[m]], s=1.5, alpha=0.25)
        ax.scatter(*r['surv'].T, marker='*', s=160, c='gold', edgecolors='k', depthshade=False)
        ax.scatter(*HOMES.T, marker='^', s=80, c=DRONE_COLORS, edgecolors='k', depthshade=False)
        ax.set_xlim(0, AREA); ax.set_ylim(0, AREA); ax.set_zlim(0, 60)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
        ax.set_title(f'{title}: unscanned voxels by owner (every 7th)', fontsize=9)
    fig.suptitle(f'S049 3D - Battery-weighted 3D zones, placement seed {SHOW_SEED} (stars = survivors)',
                 fontsize=12, fontweight='bold')
    _save(fig, 'volume_zones_3d.png')


def plot_layer_snapshots(R):
    r = R[('weighted', SHOW_SEED)]
    picks = [r['snaps'][0], r['snaps'][1] if len(r['snaps']) > 1 else r['snaps'][0], r['snaps'][-1]]
    names = ['t = 0', f'1st re-part.\n{picks[1][0]:.1f} s', f'last re-part.\n{picks[2][0]:.1f} s']
    fig, axes = plt.subplots(3, NZ, figsize=(24, 6.6))
    for row, ((t_s, own, sc), nm) in enumerate(zip(picks, names)):
        for k in range(NZ):
            img = np.ones((NXY, NXY, 3))
            sl = slice(k * NXY * NXY, (k + 1) * NXY * NXY)
            o = own[sl].reshape(NXY, NXY); s = sc[sl].reshape(NXY, NXY)
            for q in range(K_DRONES):
                img[o == q] = mcolors.to_rgb(DRONE_COLORS[q])
            img[s] = (0.55, 0.55, 0.55)
            ax = axes[row, k]
            ax.imshow(np.transpose(img, (1, 0, 2)), origin='lower', extent=[0, AREA, 0, AREA])
            ax.set_xticks([]); ax.set_yticks([])
            if row == 0:
                ax.set_title(f'z = {ZC[k]:.1f} m', fontsize=8)
            if k == 0:
                ax.set_ylabel(nm, fontsize=8)
    fig.suptitle(f'S049 3D - Layer snapshots, battery-weighted, seed {SHOW_SEED} (grey = scanned, colour = owner)',
                 fontsize=12, fontweight='bold')
    _save(fig, 'layer_snapshots.png')


def plot_trajectories(R):
    r = R[('weighted', SHOW_SEED)]
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')
    for k, h in enumerate(r['hist']):
        ax.plot(*h[::4].T, color=DRONE_COLORS[k], lw=0.6, label=f'drone {k+1}')
    ax.scatter(*r['surv'].T, marker='*', s=160, c='gold', edgecolors='k', depthshade=False, label='survivors')
    ax.set_xlim(0, AREA); ax.set_ylim(0, AREA); ax.set_zlim(0, 60)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.legend(fontsize=8)
    ax.set_title(f'S049 3D - Battery-weighted 3D, seed {SHOW_SEED}: drone paths over {T_MAX:.0f} s',
                 fontsize=11, fontweight='bold')
    _save(fig, 'trajectories_3d.png')


def plot_coverage(R):
    fig, ax = plt.subplots(figsize=(10, 4.8))
    for s in STRATEGIES:
        r = R[(s, SHOW_SEED)]; ls, c, lab = STYLE[s]
        ax.plot(r['t_log'], 100 * r['cov_log'], ls=ls, color=c, lw=1.5,
                label=f'{lab}: {100*r["coverage"]:.1f}% at {r["t_end"]:.0f} s')
        if np.isfinite(r['t_all']):
            ax.axvline(r['t_all'], color=c, ls=ls, lw=0.8)
    ax.axhline(90, color='k', ls=':', lw=0.8, label='card target 90 %')
    ax.set_xlabel('time (s)'); ax.set_ylabel('volumetric coverage C_vol (%)')
    ax.set_ylim(0, 100); ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    ax.set_title(f'S049 3D - Volumetric coverage, seed {SHOW_SEED} (vertical lines: all 5 survivors found)',
                 fontsize=11, fontweight='bold')
    _save(fig, 'coverage_vs_time.png')


def plot_layer_gantt(R):
    r = R[('weighted', SHOW_SEED)]
    fig, ax = plt.subplots(figsize=(12, 4.2))
    for k, lh in enumerate(r['layer_hist']):
        t = (np.arange(len(lh)) + 1) * DT
        z = np.where(lh >= 0, ZC[np.clip(lh, 0, NZ - 1)], np.nan)
        ax.plot(t, z + (k - 1.5) * 0.4, color=DRONE_COLORS[k], lw=1.2, label=f'drone {k+1}')
    for (tr, *_r) in r['reparts']:
        ax.axvline(tr, color='gray', lw=0.3, alpha=0.6)
    ax.set_xlabel('time (s)'); ax.set_ylabel('layer being swept (centre z, m)')
    ax.set_yticks(ZC); ax.grid(True, alpha=0.3); ax.legend(fontsize=8, ncol=4)
    ax.set_title(f'S049 3D - Layer schedule, battery-weighted, seed {SHOW_SEED} '
                 '(gaps = return / swap / confirm; grey = re-partitions)', fontsize=11, fontweight='bold')
    _save(fig, 'layer_gantt.png')


def plot_detection(R, mc):
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8))
    x = np.arange(N_SURV); w = 0.2
    for i, s in enumerate(STRATEGIES):
        r = R[(s, SHOW_SEED)]; ls, c, lab = STYLE[s]
        dt = [v if v is not None else 0 for v in r['det_t']]
        b = axes[0].bar(x + (i - 1.5) * w, dt, w, color=c, label=lab)
        for xi, v in zip(x + (i - 1.5) * w, r['det_t']):
            if v is None:
                axes[0].text(xi, 30, 'not found', rotation=90, fontsize=6, ha='center')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels([f'S{i+1}\nz={R[("static", SHOW_SEED)]["surv"][i, 2]:.1f} m' for i in range(N_SURV)])
    axes[0].set_ylabel('detection time (s)'); axes[0].legend(fontsize=7)
    axes[0].set_title(f'Detection time per survivor, seed {SHOW_SEED}', fontsize=10)
    for i, s in enumerate(STRATEGIES):
        ls, c, lab = STYLE[s]
        nf = mc[s]['n_found']
        axes[1].bar(i, np.mean(nf), color=c)
        axes[1].text(i, np.mean(nf) + 0.05, f'{np.mean(nf):.2f}\nall 5: {int(np.sum(nf == N_SURV))}/{N_MC}',
                     ha='center', fontsize=8)
    axes[1].set_xticks(range(len(STRATEGIES))); axes[1].set_xticklabels([STYLE[s][2] for s in STRATEGIES],
                                                                       rotation=15, fontsize=7)
    axes[1].set_ylim(0, N_SURV + 0.8); axes[1].set_ylabel('survivors found within 3600 s (mean)')
    axes[1].set_title(f'{N_MC} survivor placements', fontsize=10)
    fig.suptitle('S049 3D - Survivor detection', fontsize=12, fontweight='bold')
    fig.tight_layout()
    _save(fig, 'detection_timeline.png')


def plot_jain(R):
    fig, ax = plt.subplots(figsize=(10, 4.2))
    for s in STRATEGIES:
        r = R[(s, SHOW_SEED)]; ls, c, lab = STYLE[s]
        ax.plot(r['t_log'], r['fair_log'], ls=ls, color=c, lw=1.2,
                label=f'{lab}: mean {np.nanmean(r["fair_log"]):.3f}')
    ax.set_xlabel('time (s)'); ax.set_ylabel('volumetric Jain index J_vol'); ax.set_ylim(0, 1.05)
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    ax.set_title(f'S049 3D - Load balance (remaining voxels per available drone), seed {SHOW_SEED}',
                 fontsize=11, fontweight='bold')
    _save(fig, 'jain_vs_time.png')


def save_animation(R):
    r = R[('weighted', SHOW_SEED)]
    frames = r['frames']
    fig = plt.figure(figsize=(8, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0, AREA); ax.set_ylim(0, AREA); ax.set_zlim(0, 60)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    sub = np.arange(0, N_VOX, 10)
    sc_pts = ax.scatter([], [], [], s=1, c='gray', alpha=0.25)
    dr = ax.scatter(*HOMES.T, s=60, c=DRONE_COLORS, marker='o', edgecolors='k', depthshade=False)
    sv = ax.scatter(*r['surv'].T, s=150, marker='*', c=['red'] * N_SURV, edgecolors='k', depthshade=False)
    ttl = ax.set_title('')

    def update(f):
        t, pos, found, scanned = frames[f]
        m = sub[scanned[sub]]
        sc_pts._offsets3d = (VOX[m, 0], VOX[m, 1], VOX[m, 2])
        dr._offsets3d = (pos[:, 0], pos[:, 1], pos[:, 2])
        sv.set_color(['green' if x else 'red' for x in found])
        ax.view_init(elev=25, azim=-60 + 1.5 * f)
        ttl.set_text(f'S049 3D - weighted 3D, t = {t:.0f} s, C_vol = {100*scanned.mean():.1f}%, '
                     f'found {sum(found)}/5')
        return sc_pts, dr, sv, ttl

    ani = animation.FuncAnimation(fig, update, frames=len(frames), blit=False)
    p = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(p, writer='pillow', fps=8, dpi=70)
    plt.close(fig)
    print(f'Saved: {p} ({len(frames)} frames)')


# -- Report ---------------------------------------------------------------------------------

def report(R):
    P = build_prior()
    pz = np.array([P[VOX_K == k].sum() for k in range(NZ)])
    print('\nPrior mass per layer (z = 2.5 ... 57.5 m): ' + ', '.join(f'{100*v:.1f}%' for v in pz))
    one_layer = AREA * AREA / (2 * R_S)
    print(f'Card-arithmetic check: one layer needs ~{one_layer:.0f} m of sweep at a {2*R_S:.0f} m swath, '
          f'all {NZ} layers {NZ*one_layer/1000:.0f} km; 4 drones x {SPEED} m/s x {T_MAX:.0f} s = '
          f'{K_DRONES*SPEED*T_MAX/1000:.1f} km of horizontal flight at most -> C_vol <= '
          f'{100*K_DRONES*SPEED*T_MAX/(NZ*one_layer):.0f}% (before swaps and climbs)')
    mc = {s: {} for s in STRATEGIES}
    for s in STRATEGIES:
        rs = [R[(s, sd)] for sd in range(N_MC)]
        mc[s]['n_found'] = np.array([r['n_found'] for r in rs])
        mc[s]['t_all'] = np.array([r['t_all'] for r in rs])
        mc[s]['t_mean'] = np.array([r['t_mean'] for r in rs])
        mc[s]['reparts'] = np.array([len(r['reparts']) for r in rs])
        mc[s]['swaps'] = np.array([r['n_swaps'] for r in rs])
        mc[s]['det'] = [r['det_t'] for r in rs]
    print(f'\n-- Monte Carlo, {N_MC} survivor placements (showcase seed runs to {T_MAX:.0f} s, others stop at 5/5) --')
    for s in STRATEGIES:
        m = mc[s]; ta = m['t_all']; ok = np.isfinite(ta)
        det = np.array([v for dl in m['det'] for v in dl if v is not None])
        print(f'  {STYLE[s][2]:48s}: found {m["n_found"].mean():.2f}/5 (all 5 in {ok.sum()}/{N_MC}) | '
              f'time to all 5 (when all found) {np.nanmean(ta) if ok.any() else np.nan:.1f} s, median '
              f'{np.nanmedian(ta) if ok.any() else np.nan:.1f} | mean detection time of found {det.mean():.1f} s | '
              f're-partitions {m["reparts"].mean():.1f} | swaps {m["swaps"].mean():.2f}')
    lay = np.array([R[('static', sd)]['surv_layer'] for sd in range(N_MC)])
    print(f'  survivor layers over all placements: ' + ', '.join(f'z={ZC[k]:.1f}: {int(np.sum(lay == k))}'
                                                         for k in range(NZ) if np.sum(lay == k)))
    for s in STRATEGIES:
        fl = np.array([[v is not None for v in dl] for dl in mc[s]['det']])
        by = [fl[lay == k].mean() if np.any(lay == k) else np.nan for k in range(NZ)]
        print(f'  {STYLE[s][2]:48s}: detection rate by survivor layer: ' +
              ', '.join(f'z={ZC[k]:.1f}: {100*by[k]:.0f}%' for k in range(NZ) if np.isfinite(by[k])))

    def h2h(a, b):
        fa, fb = mc[a]['n_found'], mc[b]['n_found']
        ta = np.where(np.isfinite(mc[a]['t_all']), mc[a]['t_all'], np.inf)
        tb = np.where(np.isfinite(mc[b]['t_all']), mc[b]['t_all'], np.inf)
        better = np.sum((fa > fb) | ((fa == fb) & (ta < tb)))
        worse = np.sum((fa < fb) | ((fa == fb) & (ta > tb)))
        return f'{better}/{worse}/{N_MC - better - worse}'
    print('  per placement better/worse/tie (more survivors found, then earlier last detection):')
    for a, b in (('equal', 'static'), ('weighted', 'static'), ('weighted', 'equal'),
                 ('weighted', 'weighted_column')):
        print(f'    {a} vs {b}: {h2h(a, b)}')
    dc = [x for sd in range(N_MC) for x in R[('weighted', sd)]['differs_column']]
    print(f'  weighted 3D vs columnar partition of the same generators: voxels assigned differently, '
          f'mean {100*np.mean(dc):.1f}% (max {100*np.max(dc):.1f}%) over {len(dc)} re-partitions')
    zs = [z for sd in range(N_MC) for rp in R[('weighted', sd)]['reparts'] for z in rp[4]]
    print(f'  generator altitudes at re-partitions (weighted): min {np.min(zs):.1f}, median {np.median(zs):.1f}, '
          f'max {np.max(zs):.1f} m')
    print(f'\n-- Showcase seed {SHOW_SEED} (runs to {T_MAX:.0f} s) --')
    print('  survivors: ' + '; '.join(f'({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})' for p in R[('static', SHOW_SEED)]['surv']))
    for s in STRATEGIES:
        r = R[(s, SHOW_SEED)]
        dts = ', '.join('-' if v is None else f'{v:.1f}' for v in r['det_t'])
        print(f'  {STYLE[s][2]:48s}: detections [{dts}] s | C_vol at {r["t_end"]:.0f} s {100*r["coverage"]:.1f}% | '
              f'reparts {len(r["reparts"])} | swaps {r["n_swaps"]} | d_xy {sum(r["dxy"]):.0f} m, d_z {sum(r["dz"]):.0f} m | '
              f'mean J_vol {np.nanmean(r["fair_log"]):.3f}')
        if s == 'weighted' and r['reparts']:
            rp = r['reparts'][0]
            print(f'    first re-partition t = {rp[0]:.1f} s: J {rp[1]:.3f} -> {rp[2]:.3f}, SoC {rp[3]}, z {rp[4]}')
    return mc


if __name__ == '__main__':
    print('Running S049 3D Dynamic Zone Assignment...')
    R = run_all()
    mc = report(R)
    plot_volume_zones(R)
    plot_layer_snapshots(R)
    plot_trajectories(R)
    plot_coverage(R)
    plot_layer_gantt(R)
    plot_detection(R, mc)
    plot_jain(R)
    save_animation(R)
    print('\nDone. Outputs in:', OUTPUT_DIR)
