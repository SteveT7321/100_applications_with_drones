"""
S034 Weather Rerouting
======================
A delivery drone crosses a 100 x 100 x 30 m voxel airspace (2 m voxels, 26-connected)
from (4, 4, 20) m to (96, 96, 20) m at 5 m/s. Eight spherical wind-hazard cells are
placed on or near the straight route; some are already active at take-off and expire
during the flight, others appear while the drone is en route. Hazard voxels inflate
edge costs, c = c_base (1 + alpha * sum w_k), and the drone replans with D* Lite.

Three variants fly the same weather (and the same plan at t = 0 for 2 and 3):
  blind      - shortest path ignoring weather, never replanned
  plan_once  - D* Lite plan using the cells active at t = 0, never replanned
  dstar      - same initial plan, event-triggered D* Lite replanning:
               * a newly active cell intersects the remaining path, or
               * an expiring cell lies within D_NEAR of the remaining path
               (cost changes are always pushed into D* Lite as UpdateVertex calls,
                for activations *and* expiries)

A Monte-Carlo study over N_SEEDS weather layouts reports mean hazard exposure and detour.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s034_weather_rerouting.py
"""

import os
import heapq
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ── Parameters ─────────────────────────────────────────────────────────────────
GRID_RES     = 2.0            # m voxel side
GX, GY, GZ   = 50, 50, 15     # voxels -> nodes at 0..98 m (x, y), 0..28 m (z)
V_MAX        = 5.0            # m/s
DT           = 0.05           # s
ALPHA        = 10.0           # hazard inflation factor
N_HAZARDS    = 8              # hazard cells per layout
R_RANGE      = (4.0, 12.0)    # m hazard radius
TAU_RANGE    = (30.0, 120.0)  # s hazard lifetime
W_RANGE      = (0.3, 1.0)     # severity
T_ON_RANGE   = (-60.0, 20.0)  # s activation time (negative = already active at take-off)
ROUTE_FRAC   = (0.15, 0.85)   # cell centre placed at this fraction of the straight route ...
LAT_OFFSET   = 10.0           # m ... +/- this horizontal offset from it ...
Z_OFFSET     = 4.0            # m ... and +/- this vertical offset from cruise altitude
D_NEAR       = 6.0            # m expiry trigger distance to the remaining path
DELIVERY_ALT = 20.0           # m cruise altitude (z-index 10)
P_START = np.array([4.0, 4.0, DELIVERY_ALT])
P_GOAL  = np.array([96.0, 96.0, DELIVERY_ALT])
T_MAX   = 200.0               # s simulation cap
SEED    = 9                   # weather layout shown in the plots (two in-flight activations)
N_SEEDS = 10                  # Monte-Carlo layouts (seeds 0 ... 9)

VARIANTS = ['blind', 'plan_once', 'dstar']
VAR_LABEL = {'blind': 'Blind (no weather info)', 'plan_once': 'Plan once at t=0',
             'dstar': 'D* Lite replanning'}
VAR_COLOR = {'blind': 'tab:gray', 'plan_once': 'tab:purple', 'dstar': 'tab:blue'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's034_weather_rerouting',
)


# ── Hazards ────────────────────────────────────────────────────────────────────
@dataclass
class HazardCell:
    center: np.ndarray
    radius: float
    severity: float
    t_on: float
    t_off: float


def is_active(cell, t):
    return cell.t_on <= t < cell.t_off


def generate_hazards(seed):
    """Cells on/near the straight route; activation times spread around the flight."""
    rng = np.random.default_rng(seed)
    d = (P_GOAL - P_START)[:2]
    u = d / np.linalg.norm(d)
    n = np.array([-u[1], u[0]])
    cells = []
    for _ in range(N_HAZARDS):
        s = rng.uniform(*ROUTE_FRAC)
        lat = rng.uniform(-LAT_OFFSET, LAT_OFFSET)
        xy = P_START[:2] + s * d + lat * n
        z = DELIVERY_ALT + rng.uniform(-Z_OFFSET, Z_OFFSET)
        r = rng.uniform(*R_RANGE)
        w = rng.uniform(*W_RANGE)
        t_on = rng.uniform(*T_ON_RANGE)
        tau = rng.uniform(*TAU_RANGE)
        cells.append(HazardCell(np.array([xy[0], xy[1], z]), r, w, t_on, t_on + tau))
    return cells


# ── Grid helpers ───────────────────────────────────────────────────────────────
_OFFSETS = [(di, dj, dk) for di in (-1, 0, 1) for dj in (-1, 0, 1) for dk in (-1, 0, 1)
            if (di, dj, dk) != (0, 0, 0)]
_OFF_LEN = [GRID_RES * math.sqrt(di * di + dj * dj + dk * dk) for di, dj, dk in _OFFSETS]


def world_to_vox(p):
    return tuple(int(np.clip(round(p[a] / GRID_RES), 0, lim - 1))
                 for a, lim in zip(range(3), (GX, GY, GZ)))


def vox_to_world(v):
    return np.array(v, dtype=float) * GRID_RES


def neighbours(v):
    i, j, k = v
    for (di, dj, dk), L in zip(_OFFSETS, _OFF_LEN):
        a, b, c = i + di, j + dj, k + dk
        if 0 <= a < GX and 0 <= b < GY and 0 <= c < GZ:
            yield (a, b, c), L


def hazard_weights(active_cells):
    """voxel -> sum of severities of the active cells containing it."""
    hw = {}
    for cell in active_cells:
        cv = world_to_vox(cell.center)
        rv = int(math.ceil(cell.radius / GRID_RES)) + 1
        for di in range(-rv, rv + 1):
            for dj in range(-rv, rv + 1):
                for dk in range(-rv, rv + 1):
                    v = (cv[0] + di, cv[1] + dj, cv[2] + dk)
                    if not (0 <= v[0] < GX and 0 <= v[1] < GY and 0 <= v[2] < GZ):
                        continue
                    if np.linalg.norm(vox_to_world(v) - cell.center) <= cell.radius:
                        hw[v] = hw.get(v, 0.0) + cell.severity
    return hw


# ── D* Lite (backward search from the goal) ────────────────────────────────────
class DStarLite:
    def __init__(self, start, goal, hw):
        self.start, self.goal, self.km = start, goal, 0.0
        self.hw = hw
        self.g, self.rhs = {}, {goal: 0.0}
        self.U = [(self._key(goal), goal)]
        self.expanded = 0

    def cost(self, L, v):
        return L * (1.0 + ALPHA * self.hw.get(v, 0.0))

    def _h(self, a, b):
        return GRID_RES * math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    def _key(self, s):
        m = min(self.g.get(s, math.inf), self.rhs.get(s, math.inf))
        # Keys rounded to 1e-6: along an optimal path k1 is constant up to float noise,
        # and without rounding that noise breaks the k2 tie-break and stops the search early.
        return (round(m + self._h(self.start, s) + self.km, 6), round(m, 6))

    def update_vertex(self, s):
        if s != self.goal:
            best = math.inf
            for nb, L in neighbours(s):
                val = self.cost(L, nb) + self.g.get(nb, math.inf)
                if val < best:
                    best = val
            self.rhs[s] = best
        if self.g.get(s, math.inf) != self.rhs.get(s, math.inf):
            heapq.heappush(self.U, (self._key(s), s))

    def compute_shortest_path(self):
        n0 = self.expanded
        while self.U:
            k_old, u = self.U[0]
            if not (k_old < self._key(self.start)
                    or self.rhs.get(self.start, math.inf) != self.g.get(self.start, math.inf)):
                break
            heapq.heappop(self.U)
            g_u, rhs_u = self.g.get(u, math.inf), self.rhs.get(u, math.inf)
            if g_u == rhs_u:
                continue                                   # stale entry
            k_new = self._key(u)
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
                continue
            self.expanded += 1
            if g_u > rhs_u:
                self.g[u] = rhs_u
                for nb, _ in neighbours(u):
                    self.update_vertex(nb)
            else:
                self.g[u] = math.inf
                self.update_vertex(u)
                for nb, _ in neighbours(u):
                    self.update_vertex(nb)
        return self.expanded - n0

    def update_costs(self, new_hw):
        """Apply a new hazard map: every voxel whose weight changed (activation OR expiry)
        changes the cost of all edges entering it, so its neighbours are updated."""
        changed = {v for v in set(self.hw) | set(new_hw)
                   if self.hw.get(v, 0.0) != new_hw.get(v, 0.0)}
        self.hw = new_hw
        touched = set()
        for v in changed:
            for nb, _ in neighbours(v):
                touched.add(nb)
        for s in sorted(touched):
            self.update_vertex(s)
        return len(changed)

    def move_start(self, new_start):
        if new_start != self.start:
            self.km += self._h(self.start, new_start)
            self.start = new_start

    def extract_path(self):
        """Follow argmin_s' [c(s, s') + g(s')] from start to goal."""
        path, s = [self.start], self.start
        for _ in range(GX * GY * GZ):
            if s == self.goal:
                break
            best, best_nb = math.inf, None
            for nb, L in neighbours(s):
                val = self.cost(L, nb) + self.g.get(nb, math.inf)
                if val < best:
                    best, best_nb = val, nb
            if best_nb is None or best_nb in path[-3:]:
                break
            path.append(best_nb)
            s = best_nb
        return path


# ── Simulation ─────────────────────────────────────────────────────────────────
def path_cells_hit(path_vox, cells, margin=0.0):
    """Indices of cells whose sphere (+margin) contains any voxel of the path."""
    if not path_vox:
        return set()
    P = np.array([vox_to_world(v) for v in path_vox])
    return {k for k, c in enumerate(cells)
            if np.min(np.linalg.norm(P - c.center, axis=1)) <= c.radius + margin}


def run_simulation(variant='dstar', seed=SEED, record=False):
    """Fly one variant through one weather layout."""
    cells = generate_hazards(seed)
    start_v, goal_v = world_to_vox(P_START), world_to_vox(P_GOAL)
    active0 = [c for c in cells if is_active(c, 0.0)]
    planner = DStarLite(start_v, goal_v, {} if variant == 'blind' else hazard_weights(active0))
    init_exp = planner.compute_shortest_path()
    waypoints = [vox_to_world(v) for v in planner.extract_path()[1:]]
    remaining_vox = planner.extract_path()

    pos, t = P_START.copy(), 0.0
    traj, times, n_active = [pos.copy()], [0.0], []
    replans, inc_exp, scratch_exp, g_err = [], [], [], []
    hazard_time = sev_exposure = 0.0
    cell_exposure, event_detour = {}, []     # s inside each cell; remaining-path change per replan
    planned_paths = [np.array([vox_to_world(v) for v in remaining_vox])]
    prev_active = {k for k, c in enumerate(cells) if is_active(c, 0.0)}

    while t < T_MAX and not np.allclose(pos, P_GOAL):
        active = {k for k, c in enumerate(cells) if is_active(c, t)}
        n_active.append(len(active))
        if variant == 'dstar' and active != prev_active:
            new_hw = hazard_weights([cells[k] for k in sorted(active)])
            planner.update_costs(new_hw)
            rem = [world_to_vox(pos)] + [world_to_vox(w) for w in waypoints]
            hit_new = path_cells_hit(rem, [cells[k] for k in sorted(active - prev_active)])
            near_exp = path_cells_hit(rem, [cells[k] for k in sorted(prev_active - active)], D_NEAR)
            if hit_new or near_exp:
                planner.move_start(world_to_vox(pos))
                inc_exp.append(planner.compute_shortest_path())
                fresh = DStarLite(planner.start, goal_v, new_hw)
                scratch_exp.append(fresh.compute_shortest_path())
                g_err.append(abs(planner.g.get(planner.start, math.inf) - fresh.g.get(planner.start, math.inf)))
                vox = planner.extract_path()
                old_rem = float(np.sum(np.linalg.norm(np.diff(np.array([pos] + waypoints), axis=0), axis=1)))
                waypoints = [vox_to_world(v) for v in vox[1:]]
                new_rem = float(np.sum(np.linalg.norm(np.diff(np.array([pos] + waypoints), axis=0), axis=1)))
                event_detour.append(round(new_rem - old_rem, 9) + 0.0)
                planned_paths.append(np.array([vox_to_world(v) for v in vox]))
                replans.append((t, 'activation' if hit_new else 'expiry'))
        prev_active = active

        # exposure (true geometry)
        inside = [cells[k] for k in active if np.linalg.norm(pos - cells[k].center) <= cells[k].radius]
        for k in active:
            if np.linalg.norm(pos - cells[k].center) <= cells[k].radius:
                cell_exposure[k] = cell_exposure.get(k, 0.0) + DT
        if inside:
            hazard_time += DT
            sev_exposure += DT * sum(c.severity for c in inside)

        # move with an exact distance budget
        budget = V_MAX * DT
        while budget > 1e-12 and waypoints:
            d = float(np.linalg.norm(waypoints[0] - pos))
            if d <= budget:
                pos = waypoints.pop(0).copy()
                budget -= d
            else:
                pos = pos + (waypoints[0] - pos) / d * budget
                budget = 0.0
        t = round(t + DT, 10)
        traj.append(pos.copy()); times.append(t)

    traj = np.array(traj)
    L = float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1)))
    J_base = float(np.linalg.norm(P_GOAL - P_START))
    L = round(L, 9) + 0.0
    return {
        'variant': variant, 'seed': seed, 'cells': cells, 'traj': traj,
        'times': np.array(times), 'n_active': np.array(n_active + n_active[-1:]),
        'replans': replans, 'inc_exp': inc_exp, 'scratch_exp': scratch_exp, 'g_err': g_err,
        'cell_exposure': cell_exposure, 'event_detour': event_detour,
        'init_exp': init_exp, 'J_base': J_base, 'L': L, 'J_detour': round(L - J_base, 9) + 0.0,
        'J_hazard': hazard_time, 'sev_exposure': sev_exposure,
        'mission_time': t, 'arrived': bool(np.allclose(pos, P_GOAL)),
        'planned_paths': planned_paths,
    }


# ── Plots ──────────────────────────────────────────────────────────────────────
def _sphere(ax, cell, color='orange', alpha=0.25):
    u = np.linspace(0, 2 * np.pi, 16); v = np.linspace(0, np.pi, 10)
    x = cell.center[0] + cell.radius * np.outer(np.cos(u), np.sin(v))
    y = cell.center[1] + cell.radius * np.outer(np.sin(u), np.sin(v))
    z = cell.center[2] + cell.radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_wireframe(x, y, z, color=color, alpha=alpha, linewidth=0.5)


def plot_trajectory_3d(runs, out_dir):
    """3D paths of the three variants with all cells active at some time in the flight."""
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    T = runs['dstar']['mission_time']
    for c in runs['dstar']['cells']:
        if c.t_on < T and c.t_off > 0:
            _sphere(ax, c, 'red' if c.t_on > 0 else 'orange')
    for var in VARIANTS:
        tr = runs[var]['traj']
        ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=VAR_COLOR[var], lw=2,
                label=f"{VAR_LABEL[var]} (L={runs[var]['L']:.1f} m)")
    for (tr_t, kind) in runs['dstar']['replans']:
        i = int(round(tr_t / DT))
        p = runs['dstar']['traj'][i]
        ax.scatter(*p, color='red', s=40, marker='x')
    ax.scatter(*P_START, c='green', s=80, label='Start')
    ax.scatter(*P_GOAL, c='black', marker='*', s=120, label='Goal')
    ax.set_xlim(0, 100); ax.set_ylim(0, 100); ax.set_zlim(0, 30)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title(f'S034 Weather Rerouting - 3D paths (seed {SEED})\n'
                 'orange = active at take-off, red = appears in flight, x = replan')
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def plot_timeline(runs, out_dir):
    """Active-hazard count, replans and hazard exposure vs time."""
    r = runs['dstar']
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axes[0].step(r['times'], r['n_active'], where='post', color='darkorange')
    axes[0].fill_between(r['times'], r['n_active'], step='post', color='orange', alpha=0.3)
    for (tr_t, kind) in r['replans']:
        axes[0].axvline(tr_t, color='red' if kind == 'activation' else 'green', lw=1.2)
    axes[0].set_ylabel('Active hazard cells'); axes[0].grid(alpha=0.3)
    axes[0].set_title(f'Weather activity and replans (seed {SEED}; red = replan on activation, '
                      'green = replan on expiry)')
    for var in VARIANTS:
        rr = runs[var]
        inside = np.zeros(len(rr['times']))
        for i, (tt, p) in enumerate(zip(rr['times'], rr['traj'])):
            inside[i] = sum(1 for c in rr['cells']
                            if is_active(c, tt) and np.linalg.norm(p - c.center) <= c.radius)
        axes[1].step(rr['times'], inside, where='post', color=VAR_COLOR[var], label=VAR_LABEL[var])
    axes[1].set_ylabel('Cells containing drone'); axes[1].set_xlabel('Time (s)')
    axes[1].legend(fontsize=8); axes[1].grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, 'timeline.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_altitude(runs, out_dir):
    fig, ax = plt.subplots(figsize=(12, 4))
    for var in VARIANTS:
        rr = runs[var]
        ax.plot(rr['times'], rr['traj'][:, 2], color=VAR_COLOR[var], lw=1.5, label=VAR_LABEL[var])
    ax.axhline(DELIVERY_ALT, color='k', ls=':', lw=1, label=f'Cruise {DELIVERY_ALT:.0f} m')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude (m)'); ax.set_ylim(0, 30)
    ax.set_title(f'Altitude profile (seed {SEED})'); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, 'altitude_profile.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_cost_breakdown(runs, mc, out_dir):
    """J_detour and J_hazard per variant: seed SEED and Monte-Carlo mean."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    x = np.arange(3); w = 0.38
    for ax, key, title in zip(axes, ['J_detour', 'J_hazard', 'sev_exposure'],
                              ['J_detour (m)', 'J_hazard: time inside cells (s)',
                               'Severity-weighted exposure (s)']):
        v0 = [runs[v][key] for v in VARIANTS]
        vm = [np.mean([r[key] for r in mc[v]]) for v in VARIANTS]
        b1 = ax.bar(x - w / 2, v0, w, color=[VAR_COLOR[v] for v in VARIANTS], alpha=0.5,
                    label=f'seed {SEED}')
        b2 = ax.bar(x + w / 2, vm, w, color=[VAR_COLOR[v] for v in VARIANTS],
                    label=f'mean of {N_SEEDS} layouts')
        ax.bar_label(b1, fmt='%.2f', fontsize=8); ax.bar_label(b2, fmt='%.2f', fontsize=8)
        ax.set_xticks(x); ax.set_xticklabels([VAR_LABEL[v] for v in VARIANTS], fontsize=8)
        ax.set_title(title, fontsize=10); ax.grid(axis='y', alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.suptitle(f'Cost breakdown (J_base = {runs["dstar"]["J_base"]:.2f} m for all)', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'cost_breakdown.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def save_animation(runs, out_dir):
    """Top-down view: hazard cells fade in/out, three variants flying (<= 150 frames)."""
    import matplotlib.animation as animation
    r = runs['dstar']
    n = max(len(runs[v]['times']) for v in VARIANTS)
    frames = list(range(0, n, max(1, int(math.ceil(n / 150)))))
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(0, 100); ax.set_ylim(0, 100); ax.set_aspect('equal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    circles = []
    for c in r['cells']:
        p = plt.Circle(c.center[:2], c.radius, color='orange', alpha=0.0, lw=0)
        ax.add_patch(p); circles.append(p)
    lines = {v: ax.plot([], [], '-', color=VAR_COLOR[v], lw=1.6, label=VAR_LABEL[v])[0]
             for v in VARIANTS}
    dots = {v: ax.plot([], [], 'o', color=VAR_COLOR[v], ms=6)[0] for v in VARIANTS}
    ax.plot(*P_START[:2], 'g^', ms=9); ax.plot(*P_GOAL[:2], 'k*', ms=12)
    ax.legend(loc='lower right', fontsize=8)
    ttl = ax.set_title('')

    def update(fi):
        f = frames[fi]
        t = f * DT
        for c, p in zip(r['cells'], circles):
            p.set_alpha(0.35 if is_active(c, t) else 0.0)
        for v in VARIANTS:
            tr = runs[v]['traj']; i = min(f, len(tr) - 1)
            lines[v].set_data(tr[:i + 1, 0], tr[:i + 1, 1])
            dots[v].set_data([tr[i, 0]], [tr[i, 1]])
        recent = any(0 <= t - rt < 1.0 for rt, _ in r['replans'])
        lines['dstar'].set_color('red' if recent else VAR_COLOR['dstar'])
        ttl.set_text(f'S034 Weather Rerouting (seed {SEED})  t = {t:.1f} s')
        return circles + list(lines.values()) + list(dots.values()) + [ttl]

    ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=80)
    plt.close()
    print(f'Saved: {path}  ({len(frames)} frames)')


# ── Entry point ────────────────────────────────────────────────────────────────
def _report(rr):
    rep = ', '.join(f'{t:.2f} s ({k})' for t, k in rr['replans']) or 'none'
    print(f"  {rr['variant']:10s} arrived={rr['arrived']}  L={rr['L']:7.2f} m  "
          f"J_detour={rr['J_detour']:6.2f} m  J_hazard={rr['J_hazard']:5.2f} s  "
          f"sev_exposure={rr['sev_exposure']:5.2f} s  time={rr['mission_time']:6.2f} s  "
          f"replans={len(rr['replans'])}: {rep}")


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)

    runs = {v: run_simulation(v, SEED, record=True) for v in VARIANTS}
    print(f'Weather layout seed {SEED}:')
    for k, c in enumerate(runs['dstar']['cells']):
        print(f'  cell {k}: centre=({c.center[0]:5.1f},{c.center[1]:5.1f},{c.center[2]:4.1f}) '
              f'r={c.radius:4.1f} m  w={c.severity:.2f}  active {c.t_on:6.1f} .. {c.t_off:6.1f} s')
    print(f"J_base = {runs['dstar']['J_base']:.2f} m (straight line, {runs['dstar']['J_base']/V_MAX:.2f} s at v_max)")
    for v in VARIANTS:
        _report(runs[v])
    d = runs['dstar']
    print(f"  D* Lite initial plan expansions: {d['init_exp']}")
    for (tt, k), a, b in zip(d['replans'], d['inc_exp'], d['scratch_exp']):
        print(f'  replan at {tt:.2f} s ({k}): incremental expansions={a}, from-scratch={b}')

    print(f'\nMonte Carlo over {N_SEEDS} weather layouts (seeds 0..{N_SEEDS-1}):')
    mc = {v: [run_simulation(v, s) for s in range(N_SEEDS)] for v in VARIANTS}
    for v in VARIANTS:
        rs = mc[v]
        print(f"  {v:10s} J_detour={np.mean([r['J_detour'] for r in rs]):6.2f} m  "
              f"J_hazard={np.mean([r['J_hazard'] for r in rs]):5.2f} s  "
              f"sev_exposure={np.mean([r['sev_exposure'] for r in rs]):5.2f} s  "
              f"layouts with exposure>0: {sum(r['J_hazard'] > 0 for r in rs)}/{N_SEEDS}  "
              f"replans={np.mean([len(r['replans']) for r in rs]):.1f}  "
              f"all arrived={all(r['arrived'] for r in rs)}")
    inc = [a for r in mc['dstar'] for a in r['inc_exp']]
    scr = [b for r in mc['dstar'] for b in r['scratch_exp']]
    kinds = [k for r in mc['dstar'] for _, k in r['replans']]
    n_exp_in_flight = sum(1 for r in mc['dstar'] for c in r['cells']
                          if 0.0 < c.t_off <= r['mission_time'])
    print(f"  cells expiring during the D* Lite flights: {n_exp_in_flight}; replans on activation="
          f"{kinds.count('activation')}, on expiry={kinds.count('expiry')}")
    if inc:
        print(f'  D* Lite replans: {len(inc)} total, mean expansions incremental={np.mean(inc):.0f} '
              f'vs from-scratch={np.mean(scr):.0f}; max |g_inc - g_scratch| at start = '
              f"{max(e for r in mc['dstar'] for e in r['g_err']):.1e} m")
    for kind in ('activation', 'expiry'):
        det = [dd for r in mc['dstar'] for (_, k), dd in zip(r['replans'], r['event_detour'])
               if k == kind]
        if det:
            print(f'  remaining-path change per replan on {kind}: mean {np.mean(det):+.2f} m '
                  f'(n={len(det)}, min {min(det):+.2f}, max {max(det):+.2f})')
    print('  exposure by cell origin (s inside cells active at take-off / appearing in flight):')
    for v in VARIANTS:
        rows = []
        for r in mc[v]:
            pre = sum(e for k, e in r['cell_exposure'].items() if r['cells'][k].t_on <= 0.0)
            post = sum(e for k, e in r['cell_exposure'].items() if r['cells'][k].t_on > 0.0)
            if pre > 0 or post > 0:
                rows.append(f"seed {r['seed']}: {pre:.2f}/{post:.2f}")
        n_post = sum(1 for r in mc[v] if any(r['cells'][k].t_on > 0.0 for k in r['cell_exposure']))
        print(f"    {v:10s} layouts hit by in-flight cells: {n_post}/{N_SEEDS}   " + ('; '.join(rows) or 'none'))

    plot_trajectory_3d(runs, out_dir)
    plot_timeline(runs, out_dir)
    plot_altitude(runs, out_dir)
    plot_cost_breakdown(runs, mc, out_dir)
    save_animation(runs, out_dir)
