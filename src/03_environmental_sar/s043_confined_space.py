"""
S043 Confined Space Exploration
================================
A rescue drone is deployed inside a collapsed building (30x30 m, 60x60 occupancy grid).
The drone starts with no map and builds one with a 36-beam, 2 m lidar (log-odds
occupancy grid). Three exploration strategies are compared over a 300 s mission:

  1. Random walk      — keeps a random heading for an exponentially distributed time
                        (mean 8 s) and re-draws it when the time is up or an obstacle
                        is within d_turn.
  2. Frontier-Nearest — goal = nearest frontier cell by shortest known-free path
                        (Dijkstra on the current map); the drone follows that path and
                        replans when the goal stops being a frontier or the path is blocked.
  3. Frontier + Bug2  — same frontier goal selection, but the drone gets there with
                        Bug2: straight along the M-line, right-hand wall following on
                        contact, leaving the wall only on the M-line closer to the goal.
                        Loops and timeouts blacklist the goal.

Coverage = fraction of ground-truth reachable free cells classified free (p < 0.35).
Both a map check (never enter p > 0.65) and a physical check (body radius vs. true
walls) guard every move.

Usage:
    conda activate drones
    python src/03_environmental_sar/s043_confined_space.py
"""

import os
import heapq
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from collections import deque

# ── Parameters ─────────────────────────────────
GRID_RES      = 0.5          # m — cell side length
GRID_W        = 60           # cells (30 m wide)
GRID_H        = 60           # cells (30 m tall)
SENSOR_RADIUS = 2.0          # m
N_BEAMS       = 36           # lidar rays
SIGMA_RANGE   = 0.05         # m — range noise std dev
L_OCC         = 0.85         # log-odds increment for hit cell
L_FREE        = -0.40        # log-odds decrement for free cells
L_MIN         = -2.0         # log-odds clamp minimum
L_MAX         = 3.5          # log-odds clamp maximum
P_OCC_THRESH  = 0.65         # occupied probability threshold
P_FREE_THRESH = 0.35         # free probability threshold
G_MIN         = 0.35         # m — minimum passable gap
DRONE_RADIUS  = 0.15         # m — body radius (width 0.3 m)
CLEARANCE     = 0.20         # m — body half-size + 0.05 m safety margin used against the map
DRONE_SPEED   = 0.5          # m/s
D_TURN        = 0.4          # m — obstacle detection distance
D_GOAL        = 0.3          # m — goal-reached tolerance
D_PROGRESS    = 0.1          # m — Bug2 leave condition (closer to goal than hit point)
D_WALL        = 0.75         # m — Bug2 wall-following stand-off (to nearest occupied cell centre)
K_WALL        = 1.0          # 1/m — wall stand-off correction gain
M_LINE_TOL    = 0.1          # m — "on the M-line" tolerance
GOAL_TIMEOUT  = 40.0         # s — blacklist a frontier goal not reached within this time
BLACKLIST_R   = 1.0          # m — frontier cells within this radius of a blacklisted goal are skipped
RW_MEAN_HOLD  = 8.0          # s — random walk: mean heading persistence time
T_MAX         = 300.0        # s — mission horizon
T_EXT         = 1200.0       # s — extended horizon (supplementary check that exploration completes)
DT            = 0.1          # s — simulation timestep
SEED          = 0
ANIM_EVERY    = 20           # record an animation frame every 20 steps (2 s)

START_M = np.array([2.0, 2.0])   # m

STRATEGIES = ['random', 'frontier', 'bug2']
LABELS = {'random': 'Random Walk', 'frontier': 'Frontier-Nearest', 'bug2': 'Frontier + Bug2'}
COLORS = {'random': 'tab:red', 'frontier': 'tab:orange', 'bug2': 'tab:blue'}

# Designed doorways: (name, orientation, wall index, first cell, last cell)
#   'h' = horizontal wall at row index, span over columns; 'v' = vertical wall at col index, span over rows
DOORWAYS = [('D1', 'h', 20, 24, 27), ('D2', 'h', 38, 29, 33), ('D3', 'h', 50, 14, 18),
            ('D4', 'v', 25, 9, 13), ('D5', 'v', 40, 27, 31), ('D6', 'v', 12, 39, 43)]

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's043_confined_space',
)

# ── Map helpers ────────────────────────────────

def log_odds_to_prob(l):
    """Convert log-odds value(s) to probability."""
    return 1.0 / (1.0 + np.exp(-l))


def world_to_cell(pos_m):
    """Convert world coordinates (m) to grid cell (row, col)."""
    col = int(pos_m[0] / GRID_RES)
    row = int(pos_m[1] / GRID_RES)
    return (min(max(row, 0), GRID_H - 1), min(max(col, 0), GRID_W - 1))


def cell_to_world(row, col):
    """Convert grid cell centre to world coordinates (m)."""
    return np.array([col * GRID_RES + GRID_RES / 2.0,
                     row * GRID_RES + GRID_RES / 2.0])


def build_ground_truth():
    """
    Create a 60x60 boolean occupancy grid representing a collapsed building.
    True = occupied (wall / debris).
    Layout: outer walls + inner rooms + narrow corridors + debris columns.
    """
    gt = np.zeros((GRID_H, GRID_W), dtype=bool)

    # Outer perimeter walls (1-cell thick)
    gt[0, :] = True
    gt[-1, :] = True
    gt[:, 0] = True
    gt[:, -1] = True

    # --- Horizontal internal walls with gaps ---
    gt[20, 5:24] = True      # row 20 (y=10 m), doorway cols 24-27
    gt[20, 28:55] = True
    gt[38, 10:29] = True     # row 38 (y=19 m), doorway cols 29-33
    gt[38, 34:50] = True
    gt[50, 2:14] = True      # row 50 (y=25 m), doorway cols 14-18
    gt[50, 19:45] = True

    # --- Vertical internal walls with gaps ---
    gt[2:9, 25] = True       # col 25 (x=12.5 m), doorway rows 9-13
    gt[14:20, 25] = True
    gt[21:38, 25] = True
    gt[2:27, 40] = True      # col 40 (x=20 m), doorway rows 27-31
    gt[32:50, 40] = True
    gt[21:39, 12] = True     # col 12 (x=6 m), doorway rows 39-43
    gt[44:58, 12] = True

    # --- Debris columns (2x2 blocks) ---
    debris_positions = [
        (8, 8), (8, 32), (8, 48),
        (28, 16), (28, 44),
        (42, 8), (42, 32), (42, 48),
        (54, 20), (54, 44),
        (15, 50), (30, 50),
    ]
    for r, c in debris_positions:
        gt[r:r+2, c:c+2] = True

    # --- Fallen beam: diagonal debris ---
    for k in range(8):
        r, c = 6 + k, 16 + k
        gt[r, c] = True
        gt[r, c + 1] = True
    return gt


def cast_ray(ground_truth, pos_m, angle, r_max):
    """Ray-cast on the ground-truth grid. Returns (true_range, hit_cell_or_None)."""
    d = np.array([np.cos(angle), np.sin(angle)])
    step = GRID_RES / 2.0
    r = step
    while r <= r_max:
        row, col = world_to_cell(pos_m + r * d)
        if ground_truth[row, col]:
            return r, (row, col)
        r += step
    return r_max, None


def update_map(log_odds, ground_truth, pos_m, rng):
    """Cast N_BEAMS rays with Gaussian range noise and apply the log-odds inverse sensor model."""
    for angle in np.linspace(0, 2.0 * np.pi, N_BEAMS, endpoint=False):
        true_range, hit_cell = cast_ray(ground_truth, pos_m, angle, SENSOR_RADIUS)
        noisy_range = float(np.clip(true_range + rng.normal(0, SIGMA_RANGE), 0.0, SENSOR_RADIUS))
        d = np.array([np.cos(angle), np.sin(angle)])
        step = GRID_RES / 2.0
        r = step
        touched = set()
        while r < noisy_range - step / 2.0:
            cell = world_to_cell(pos_m + r * d)
            if cell not in touched and cell != hit_cell:
                touched.add(cell)
                log_odds[cell] = min(max(log_odds[cell] + L_FREE, L_MIN), L_MAX)
            r += step
        if noisy_range < SENSOR_RADIUS and hit_cell is not None:
            log_odds[hit_cell] = min(max(log_odds[hit_cell] + L_OCC, L_MIN), L_MAX)


def frontier_mask(prob):
    """Free cells (p < 0.35) 4-adjacent to an unknown cell (0.35 <= p <= 0.65)."""
    free = prob < P_FREE_THRESH
    unk = (prob >= P_FREE_THRESH) & (prob <= P_OCC_THRESH)
    nb = np.zeros_like(unk)
    nb[1:, :] |= unk[:-1, :]
    nb[:-1, :] |= unk[1:, :]
    nb[:, 1:] |= unk[:, :-1]
    nb[:, :-1] |= unk[:, 1:]
    return free & nb


def compute_reachable(ground_truth):
    """BFS from the start over ground-truth free cells (4-connected). Returns a bool mask."""
    reach = np.zeros_like(ground_truth)
    start = world_to_cell(START_M)
    q = deque([start])
    reach[start] = True
    while q:
        r, c = q.popleft()
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if 0 <= nr < GRID_H and 0 <= nc < GRID_W and not ground_truth[nr, nc] and not reach[nr, nc]:
                reach[nr, nc] = True
                q.append((nr, nc))
    return reach


def coverage(prob, reach):
    return 100.0 * np.count_nonzero((prob < P_FREE_THRESH) & reach) / np.count_nonzero(reach)


NB8 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
       (-1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)), (1, -1, np.sqrt(2)), (1, 1, np.sqrt(2))]


def plan_to_nearest_frontier(prob, start_cell, fmask, inflate=True):
    """
    Dijkstra over known-free cells (8-connected, no corner cutting; with inflate=True cells
    4-adjacent to a mapped obstacle are avoided so the body keeps clearance) from start_cell to
    the nearest cell in fmask. The start cell itself is always expandable.
    Returns list of cells (path, start excluded) or None.
    """
    occ = prob > P_OCC_THRESH
    near_occ = occ.copy()
    near_occ[1:, :] |= occ[:-1, :]
    near_occ[:-1, :] |= occ[1:, :]
    near_occ[:, 1:] |= occ[:, :-1]
    near_occ[:, :-1] |= occ[:, 1:]
    free = (prob < P_FREE_THRESH) & ~near_occ if inflate else (prob < P_FREE_THRESH)
    free[start_cell] = True
    dist = {start_cell: 0.0}
    parent = {}
    pq = [(0.0, start_cell)]
    while pq:
        d, cell = heapq.heappop(pq)
        if d > dist.get(cell, np.inf):
            continue
        if fmask[cell] and cell != start_cell:
            path = [cell]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            return path[::-1][1:]
        r, c = cell
        for dr, dc, w in NB8:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < GRID_H and 0 <= nc < GRID_W) or not free[nr, nc]:
                continue
            if dr and dc and not (free[r + dr, c] and free[r, c + dc]):
                continue      # no corner cutting
            nd = d + w
            if nd < dist.get((nr, nc), np.inf):
                dist[(nr, nc)] = nd
                parent[(nr, nc)] = cell
                heapq.heappush(pq, (nd, (nr, nc)))
    return None


def body_clear(pos, grid_occ, r=None):
    """True if the square body footprint (half-size r) around pos overlaps no occupied cell."""
    r = CLEARANCE if r is None else r
    for dx in (-r, 0.0, r):
        for dy in (-r, 0.0, r):
            if grid_occ[world_to_cell(pos + np.array([dx, dy]))]:
                return False
    return True


def move_allowed(pos_new, occ_map, gt):
    """(map_ok, physical_ok): the body (plus margin) must not overlap a mapped occupied cell
    (p > 0.65), and the body itself must not overlap a true wall."""
    return body_clear(pos_new, occ_map), body_clear(pos_new, gt, DRONE_RADIUS)


def probe_blocked(pos, heading, prob):
    """True if the body would hit a mapped occupied cell within d_turn along heading."""
    occ = prob > P_OCC_THRESH
    for frac in (0.25, 0.5, 1.0):
        if not body_clear(pos + frac * D_TURN * heading, occ):
            return True
    return False


def rot(v, ang):
    c, s = np.cos(ang), np.sin(ang)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


# ── Bug2 ────────────────────────────────────────

class Bug2:
    """Bug2 local planner from the point where the goal was assigned (M-line start) to goal."""

    def __init__(self, start, goal):
        self.m0 = start.copy()
        self.goal = goal.copy()
        self.mode = 'MOTION_TO_GOAL'
        self.hit = None
        self.heading = None
        self.bf_dist = 0.0
        self.failed = False

    def m_line_offset(self, p):
        u = self.goal - self.m0
        n = np.linalg.norm(u)
        if n < 1e-9:
            return 0.0
        return (u[0] * (p[1] - self.m0[1]) - u[1] * (p[0] - self.m0[0])) / n

    def step(self, pos, prob):
        """Return desired unit heading (or None if goal reached / failed)."""
        to_goal = self.goal - pos
        dg = np.linalg.norm(to_goal)
        if dg < D_GOAL:
            return None
        if self.mode == 'MOTION_TO_GOAL':
            h = to_goal / dg
            if probe_blocked(pos, h, prob):
                self.mode = 'BOUNDARY_FOLLOW'
                self.hit = pos.copy()
                self.hit_dist = dg
                self.bf_dist = 0.0
                self.heading = rot(h, np.pi / 2)    # turn left: wall on the right
            else:
                return h
        # BOUNDARY_FOLLOW (right-hand rule: keep the wall on the right)
        if (self.bf_dist > 0.5 and abs(self.m_line_offset(pos)) < M_LINE_TOL
                and dg < self.hit_dist - D_PROGRESS and not probe_blocked(pos, to_goal / dg, prob)):
            self.mode = 'MOTION_TO_GOAL'
            return to_goal / dg
        if self.bf_dist > 2.0 and np.linalg.norm(pos - self.hit) < 0.3:
            self.failed = True                          # went round the obstacle: unreachable
            return None
        occ_r, occ_c = np.nonzero(prob > P_OCC_THRESH)
        if len(occ_r):
            pts = np.stack([occ_c * GRID_RES + GRID_RES / 2, occ_r * GRID_RES + GRID_RES / 2], axis=1)
            dd = np.hypot(pts[:, 0] - pos[0], pts[:, 1] - pos[1])
            k = np.argmin(dd)
            if dd[k] < 1.2:
                n_hat = (pts[k] - pos) / (dd[k] + 1e-9)      # toward wall
                tangent = rot(n_hat, np.pi / 2)            # wall on the right
                h = tangent + K_WALL * (dd[k] - D_WALL) * n_hat
                h /= np.linalg.norm(h)
            else:
                h = rot(self.heading, -0.3)                # lost the wall: curve right to find it
        else:
            h = self.heading
        for ang in np.linspace(0, np.pi, 9):               # rotate left until free
            hh = rot(h, ang)
            if not probe_blocked(pos, hh, prob):
                h = hh
                break
        self.heading = h
        self.bf_dist += DRONE_SPEED * DT
        return h


# ── Simulation ─────────────────────────────────

def run_strategy(gt, reach, strategy, t_max=T_MAX):
    rng_s = np.random.default_rng([SEED, 0])      # lidar noise (same stream for every strategy)
    rng_w = np.random.default_rng([SEED, 1])      # random-walk decisions
    log_odds = np.zeros((GRID_H, GRID_W))
    pos = START_M.copy()
    n_steps = int(round(t_max / DT)) + 1
    traj = [pos.copy()]
    cov_t, cov_v = [], []
    modes = []
    snaps, frames = {}, []
    events = []                 # doorway passages
    blacklist = []
    blocked = {'map': 0, 'physical': 0}
    goal, path, bug, goal_t0 = None, [], None, 0.0
    n_goals, n_blacklisted = 0, 0
    heading = None
    hold_until = 0.0
    finished_t = None

    for step in range(n_steps):
        t = step * DT
        update_map(log_odds, gt, pos, rng_s)
        prob = log_odds_to_prob(log_odds)
        cov_t.append(t)
        cov_v.append(coverage(prob, reach))
        if step in (0, 1000, 2000, n_steps - 1):
            snaps[int(round(t))] = (log_odds.copy(), pos.copy(), np.array(traj))
        if step % ANIM_EVERY == 0:
            frames.append((t, log_odds.copy(), pos.copy()))
        if step == n_steps - 1:
            break

        mode = 'IDLE'
        vel_dir = None
        occ_map = prob > P_OCC_THRESH
        if not body_clear(pos, occ_map):
            # newly mapped obstacle inside the safety box: back away from the nearest one
            rr, cc = np.nonzero(occ_map)
            pts = np.stack([cc * GRID_RES + 0.25, rr * GRID_RES + 0.25], axis=1)
            k = np.argmin(np.hypot(pts[:, 0] - pos[0], pts[:, 1] - pos[1]))
            away = pos - pts[k]
            new_pos = pos + DRONE_SPEED * DT * away / (np.linalg.norm(away) + 1e-9)
            if body_clear(new_pos, gt, DRONE_RADIUS):
                pos = new_pos
            modes.append('ESCAPE')
            traj.append(pos.copy())
            continue
        if strategy == 'random':
            mode = 'WALK'
            if heading is None or t >= hold_until or probe_blocked(pos, heading, prob):
                for _ in range(36):
                    a = rng_w.uniform(0, 2 * np.pi)
                    h = np.array([np.cos(a), np.sin(a)])
                    if not probe_blocked(pos, h, prob):
                        heading = h
                        break
                hold_until = t + rng_w.exponential(RW_MEAN_HOLD)
            vel_dir = heading
        else:
            fmask = frontier_mask(prob)
            for b in blacklist:
                rr, cc = np.ogrid[:GRID_H, :GRID_W]
                fmask &= np.hypot(cc * GRID_RES + 0.25 - b[0], rr * GRID_RES + 0.25 - b[1]) > BLACKLIST_R
            following_wall = bug is not None and bug.mode == 'BOUNDARY_FOLLOW' and not bug.failed
            need = (goal is None or np.linalg.norm(pos - goal) < D_GOAL
                    or (bug is not None and bug.failed)
                    or prob[world_to_cell(goal)] > P_OCC_THRESH
                    # a goal that stopped being a frontier is dropped, except in the middle of a
                    # Bug2 wall-follow (re-targeting there would reset the M-line every step)
                    or (not fmask[world_to_cell(goal)] and not following_wall))
            if goal is not None and t - goal_t0 > GOAL_TIMEOUT:
                blacklist.append(goal.copy())
                n_blacklisted += 1
                need = True
            if bug is not None and bug.failed:
                blacklist.append(goal.copy())
                n_blacklisted += 1
            if need:
                cell_path = plan_to_nearest_frontier(prob, world_to_cell(pos), fmask)
                if cell_path is None:
                    cell_path = plan_to_nearest_frontier(prob, world_to_cell(pos), fmask, inflate=False)
                if cell_path is None:
                    goal, path, bug = None, [], None
                    if not fmask.any() and finished_t is None:
                        finished_t = t
                else:
                    new_goal = cell_to_world(*cell_path[-1])
                    changed = goal is None or np.linalg.norm(new_goal - goal) > 1e-6
                    if changed:
                        goal_t0 = t
                        n_goals += 1
                    goal = new_goal
                    path = [cell_to_world(*c) for c in cell_path]
                    if strategy == 'bug2' and (changed or bug is None or bug.failed):
                        bug = Bug2(pos, goal)
            if goal is not None:
                if strategy == 'frontier':
                    mode = 'PATH_FOLLOW'
                    while path and np.linalg.norm(path[0] - pos) < DRONE_SPEED * DT:
                        path.pop(0)
                    if path and prob[world_to_cell(path[0])] > P_OCC_THRESH:
                        path = []
                        goal = None           # path blocked by new map info: replan next step
                    elif path:
                        v = path[0] - pos
                        vel_dir = v / np.linalg.norm(v)
                else:
                    if bug.failed:
                        vel_dir = None
                    else:
                        vel_dir = bug.step(pos, prob)
                        mode = bug.mode if vel_dir is not None else 'IDLE'

        if vel_dir is not None:
            new_pos = pos + DRONE_SPEED * DT * vel_dir
            map_ok, phys_ok = move_allowed(new_pos, prob > P_OCC_THRESH, gt)
            if not (map_ok and phys_ok):
                blocked['map' if not map_ok else 'physical'] += 1
                if strategy == 'random':
                    heading = None                       # re-draw a heading next step
                elif strategy == 'frontier' and goal is not None:
                    blacklist.append(goal.copy())        # path not passable with clearance
                    n_blacklisted += 1
                    goal, path = None, []
                elif bug is not None and bug.heading is not None:
                    bug.heading = rot(bug.heading, np.pi / 4)
            else:
                # doorway passage log
                for name, ori, idx, a0, a1 in DOORWAYS:
                    line = (idx + 0.5) * GRID_RES
                    k_cross, k_along = (1, 0) if ori == 'h' else (0, 1)
                    if (pos[k_cross] - line) * (new_pos[k_cross] - line) < 0 and \
                            a0 * GRID_RES <= new_pos[k_along] <= (a1 + 1) * GRID_RES:
                        # gap width estimate: contiguous run of mapped-free cells along the
                        # wall line containing the crossing cell
                        j0 = int(new_pos[k_along] / GRID_RES)
                        cell = (lambda j: (idx, j)) if ori == 'h' else (lambda j: (j, idx))
                        run = 0
                        for jj in list(range(j0, a1 + 4)):
                            if prob[cell(jj)] >= P_FREE_THRESH:
                                break
                            run += 1
                        for jj in range(j0 - 1, a0 - 4, -1):
                            if prob[cell(jj)] >= P_FREE_THRESH:
                                break
                            run += 1
                        events.append((t, name, run * GRID_RES, pos.copy(), new_pos.copy(), mode))
                pos = new_pos
        modes.append(mode)
        traj.append(pos.copy())

    traj = np.array(traj)
    return dict(traj=traj, cov_t=np.array(cov_t), cov=np.array(cov_v), modes=modes,
                snaps=snaps, frames=frames, events=events, blocked=blocked,
                n_goals=n_goals, n_blacklisted=n_blacklisted, finished_t=finished_t,
                path_len=float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))),
                final_lo=log_odds)


def run_simulation():
    gt = build_ground_truth()
    reach = compute_reachable(gt)
    print(f"Ground-truth reachable free cells: {reach.sum()} ({reach.sum() * GRID_RES ** 2:.0f} m^2)")
    data = {'gt': gt, 'reach': reach}
    for s in STRATEGIES:
        r = run_strategy(gt, reach, s)
        data[s] = r
        cov = r['cov']
        t50 = next((tt for tt, c in zip(r['cov_t'], cov) if c >= 50.0), float('nan'))
        t80 = next((tt for tt, c in zip(r['cov_t'], cov) if c >= 80.0), float('nan'))
        frac = {m: 100.0 * r['modes'].count(m) / len(r['modes']) for m in sorted(set(r['modes']))}
        print(f"[{LABELS[s]}]")
        print(f"  Coverage at 100/200/300 s: {np.interp(100, r['cov_t'], cov):.1f} / "
              f"{np.interp(200, r['cov_t'], cov):.1f} / {cov[-1]:.1f} %")
        print(f"  Time to 50% / 80% coverage: {t50:.1f} s / {t80:.1f} s")
        print(f"  Path length: {r['path_len']:.1f} m (max possible {DRONE_SPEED * T_MAX:.0f} m)")
        print(f"  Mode fractions: " + ", ".join(f"{m} {v:.1f}%" for m, v in frac.items()))
        print(f"  Blocked moves: map {r['blocked']['map']}, physical {r['blocked']['physical']}")
        if s != 'random':
            print(f"  Frontier goals: {r['n_goals']}, blacklisted: {r['n_blacklisted']}, "
                  f"frontiers exhausted at: {('%.1f s' % r['finished_t']) if r['finished_t'] is not None else 'never'}")
        names = sorted({e[1] for e in r['events']})
        print(f"  Doorway passages: {len(r['events'])} (doorways used: {', '.join(names) if names else 'none'})")
        for (te, name, w, p0, p1, m) in r['events']:
            print(f"    t={te:6.1f} s  {name}  gap est. {w:.1f} m  ({p0[0]:.2f},{p0[1]:.2f})->"
                  f"({p1[0]:.2f},{p1[1]:.2f})  mode {m}")
    print(f"--- Extended horizon {T_EXT:.0f} s (same strategies, supplementary) ---")
    for s in STRATEGIES:
        r = run_strategy(gt, reach, s, T_EXT)
        data[s + '_ext'] = r
        ts = {}
        for lvl in (50, 80, 90):
            ts[lvl] = next((tt for tt, c in zip(r['cov_t'], r['cov']) if c >= lvl), float('nan'))
        print(f"[{LABELS[s]}] coverage at 600/1200 s: {np.interp(600, r['cov_t'], r['cov']):.1f} / "
              f"{r['cov'][-1]:.1f} %, time to 50/80/90 %: {ts[50]:.1f} / {ts[80]:.1f} / {ts[90]:.1f} s, "
              f"frontiers exhausted at: {('%.1f s' % r['finished_t']) if r['finished_t'] is not None else 'never'}, "
              f"blocked moves {r['blocked']['map'] + r['blocked']['physical']}, blacklisted goals {r['n_blacklisted']}, "
              f"unmapped reachable cells {np.count_nonzero(reach & (log_odds_to_prob(r['final_lo']) >= P_FREE_THRESH))}")
    return data


# ── Plots ──────────────────────────────────────

def _map_image(log_odds):
    """Free = white, unknown = grey, occupied = black."""
    prob = log_odds_to_prob(log_odds)
    img = np.ones((GRID_H, GRID_W, 3))
    img[(prob >= P_FREE_THRESH) & (prob <= P_OCC_THRESH)] = [0.6, 0.6, 0.6]
    img[prob > P_OCC_THRESH] = [0.0, 0.0, 0.0]
    return img


def plot_map_evolution(data, out_dir):
    """Map snapshots at t = 0, 100, 200, 300 s, one row per strategy."""
    fig, axes = plt.subplots(3, 4, figsize=(17, 13))
    for row, s in enumerate(STRATEGIES):
        snaps = data[s]['snaps']
        for col, t in enumerate(sorted(snaps)):
            ax = axes[row, col]
            lo, pos, tr = snaps[t]
            img = _map_image(lo)
            img[frontier_mask(log_odds_to_prob(lo))] = [0.0, 0.85, 0.85]
            ax.imshow(img, origin='lower', extent=[0, 30, 0, 30])
            if len(tr) > 1:
                ax.plot(tr[:, 0], tr[:, 1], '-', color=COLORS[s], lw=0.8)
            ax.plot(pos[0], pos[1], 'ro', ms=6, mec='k')
            ax.set_title(f"{LABELS[s]} — t = {t} s ({data[s]['cov'][min(t * 10, len(data[s]['cov']) - 1)]:.0f}%)",
                         fontsize=10)
            ax.set_xlabel('x (m)')
            ax.set_ylabel('y (m)')
    handles = [mpatches.Patch(fc='white', ec='grey', label='Free'),
               mpatches.Patch(color='0.6', label='Unknown'),
               mpatches.Patch(color='black', label='Occupied'),
               mpatches.Patch(color=(0, 0.85, 0.85), label='Frontier'),
               plt.Line2D([0], [0], marker='o', color='r', ls='', label='Drone')]
    fig.legend(handles=handles, loc='lower center', ncol=5)
    fig.suptitle('S043 Confined Space Exploration — map evolution (percent = coverage of reachable free space)',
                 fontsize=13)
    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    path = os.path.join(out_dir, 'map_evolution.png')
    plt.savefig(path, dpi=110)
    plt.close()
    print(f'Saved: {path}')


def plot_trajectory_overlay(data, out_dir):
    """Final trajectories on the final maps, with ground-truth walls and doorways."""
    gt = data['gt']
    fig, axes = plt.subplots(1, 3, figsize=(18, 6.3))
    for ax, s in zip(axes, STRATEGIES):
        r = data[s]
        ax.imshow(_map_image(r['final_lo']), origin='lower', extent=[0, 30, 0, 30])
        rr, cc = np.nonzero(gt)
        ax.scatter(cc * GRID_RES + 0.25, rr * GRID_RES + 0.25, s=4, marker='s', c='saddlebrown', alpha=0.35)
        for name, ori, idx, a0, a1 in DOORWAYS:
            if ori == 'h':
                ax.add_patch(plt.Rectangle((a0 * GRID_RES, idx * GRID_RES), (a1 - a0 + 1) * GRID_RES,
                                           GRID_RES, fc='orange', alpha=0.5))
            else:
                ax.add_patch(plt.Rectangle((idx * GRID_RES, a0 * GRID_RES), GRID_RES,
                                           (a1 - a0 + 1) * GRID_RES, fc='orange', alpha=0.5))
        tr = r['traj']
        ax.plot(tr[:, 0], tr[:, 1], '-', color='royalblue', lw=1.0)
        ax.plot(*tr[0], 'go', ms=9, label='Start')
        ax.plot(*tr[-1], 'rx', ms=10, mew=2, label='End')
        ax.set_title(f"{LABELS[s]}: {r['cov'][-1]:.1f}% coverage, path {r['path_len']:.0f} m", fontsize=11)
        ax.set_xlim(0, 30)
        ax.set_ylim(0, 30)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
    axes[0].plot([], [], 's', color='saddlebrown', alpha=0.5, label='True walls')
    axes[0].add_patch(plt.Rectangle((0, 0), 0, 0, fc='orange', alpha=0.5, label='Doorways'))
    axes[0].legend(loc='upper right', fontsize=8)
    fig.suptitle('S043 trajectories on the final maps (white = mapped free, grey = unknown)', fontsize=13)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectory_overlay.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def plot_coverage_curve(data, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(15, 5))
    for s, ls in zip(STRATEGIES, ['--', '-.', '-']):
        a1.plot(data[s]['cov_t'], data[s]['cov'], ls=ls, color=COLORS[s], lw=2,
                label=f"{LABELS[s]} ({data[s]['cov'][-1]:.1f}%)")
        e = data[s + '_ext']
        a2.plot(e['cov_t'], e['cov'], ls=ls, color=COLORS[s], lw=2,
                label=f"{LABELS[s]} ({e['cov'][-1]:.1f}%)")
    for ax, tm, title in [(a1, T_MAX, f'Mission horizon {T_MAX:.0f} s'),
                          (a2, T_EXT, f'Extended horizon {T_EXT:.0f} s (supplementary)')]:
        ax.axhline(100, color='black', ls=':', lw=1.2, label='100% reachable')
        ax.set_xlim(0, tm)
        ax.set_ylim(0, 105)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Coverage of reachable free space (%)')
        ax.set_title(title)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)
    fig.suptitle('S043 coverage vs time — strategy comparison')
    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_curve.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_mode_timeline(data, out_dir):
    """Stacked bar of time fraction per navigation mode, per strategy."""
    mode_cols = {'WALK': 'tab:red', 'PATH_FOLLOW': 'tab:orange', 'MOTION_TO_GOAL': 'steelblue',
                 'BOUNDARY_FOLLOW': 'goldenrod', 'ESCAPE': 'purple', 'IDLE': '0.7'}
    fig, ax = plt.subplots(figsize=(10, 3.2))
    used = set()
    for i, s in enumerate(STRATEGIES):
        m = data[s]['modes']
        left = 0.0
        for mode, col in mode_cols.items():
            f = m.count(mode) / len(m)
            if f <= 0:
                continue
            ax.barh(i, f, left=left, color=col, label=mode if mode not in used else None)
            used.add(mode)
            if f > 0.04:
                ax.text(left + f / 2, i, f'{100 * f:.0f}%', ha='center', va='center', fontsize=9)
            left += f
    ax.set_yticks(range(3))
    ax.set_yticklabels([LABELS[s] for s in STRATEGIES])
    ax.set_xlim(0, 1)
    ax.set_xlabel('Fraction of time steps')
    ax.set_title('S043 navigation mode timeline')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.3), ncol=5, fontsize=8)
    path = os.path.join(out_dir, 'mode_timeline.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    """Three panels replaying the recorded runs (map, drone, path so far)."""
    import matplotlib.animation as animation
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.4))
    ims, dots, lines, titles = [], [], [], []
    for ax, s in zip(axes, STRATEGIES):
        ims.append(ax.imshow(np.ones((GRID_H, GRID_W, 3)), origin='lower', extent=[0, 30, 0, 30]))
        dots.append(ax.plot([], [], 'ro', ms=6, mec='k')[0])
        lines.append(ax.plot([], [], '-', color='royalblue', lw=0.8)[0])
        titles.append(ax.set_title(''))
        ax.set_xlim(0, 30)
        ax.set_ylim(0, 30)
        ax.set_xticks([])
        ax.set_yticks([])
    n_frames = len(data['bug2']['frames'])

    def update(i):
        for j, s in enumerate(STRATEGIES):
            t, lo, pos = data[s]['frames'][i]
            img = _map_image(lo)
            img[frontier_mask(log_odds_to_prob(lo))] = [0.0, 0.85, 0.85]
            ims[j].set_data(img)
            dots[j].set_data([pos[0]], [pos[1]])
            k = int(round(t / DT))
            tr = data[s]['traj'][:k + 1]
            lines[j].set_data(tr[:, 0], tr[:, 1])
            titles[j].set_text(f"{LABELS[s]}  t = {t:.0f} s  cov {data[s]['cov'][k]:.0f}%")
        return []

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12, dpi=70)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames)')


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    data = run_simulation()
    plot_map_evolution(data, out_dir)
    plot_trajectory_overlay(data, out_dir)
    plot_coverage_curve(data, out_dir)
    plot_mode_timeline(data, out_dir)
    save_animation(data, out_dir)
    print('Done.')
