"""
S034 Weather Rerouting
======================
A delivery drone flies a 3D voxel-grid airspace from depot to delivery point
at 20 m cruise altitude. Wind-hazard cells appear and expire stochastically;
when a cell intersects the planned path the drone replans in real-time using
D* Lite incremental replanning. The simulation tracks trajectory, replanning
events, cost breakdown (base / detour / hazard exposure), and produces a
top-down animation showing hazard cells fading in and out.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s034_weather_rerouting.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from dataclasses import dataclass
import heapq
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────────
GRID_RES     = 2.0            # m — voxel side length
GX, GY, GZ  = 50, 50, 15     # voxels  (100 m × 100 m × 30 m)
V_MAX        = 5.0            # m/s
DT           = 0.05           # s
ALPHA        = 10.0           # hazard inflation factor
N_HAZARDS    = 8              # total hazard cells over mission lifetime
DELIVERY_ALT = 20.0           # m nominal cruise altitude  → z-vox = 10
LANDING_THRESHOLD = 3.0       # m — arrival tolerance

# World-space start / goal
P_START = np.array([4.0,  4.0, DELIVERY_ALT])
P_GOAL  = np.array([96.0, 96.0, DELIVERY_ALT])

T_MAX   = 600.0               # s simulation cap

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's034_weather_rerouting',
)

RNG = np.random.default_rng(0)

# ── Hazard dataclass ────────────────────────────────────────────────────────────
@dataclass
class HazardCell:
    center: np.ndarray   # (3,) metres
    radius: float        # metres
    severity: float      # w_k ∈ [0, 1]
    t_on:  float         # activation time (s)
    t_off: float         # expiry time (s)

def is_active(cell: HazardCell, t: float) -> bool:
    return cell.t_on <= t < cell.t_off

# ── Grid helpers ───────────────────────────────────────────────────────────────
def world_to_vox(p: np.ndarray):
    """Convert world metres → voxel tuple (i,j,k), clamped to grid."""
    i = int(np.clip(p[0] / GRID_RES, 0, GX - 1))
    j = int(np.clip(p[1] / GRID_RES, 0, GY - 1))
    k = int(np.clip(p[2] / GRID_RES, 0, GZ - 1))
    return (i, j, k)

def vox_to_world(v) -> np.ndarray:
    return np.array([v[0] * GRID_RES, v[1] * GRID_RES, v[2] * GRID_RES])

def in_bounds(v):
    return 0 <= v[0] < GX and 0 <= v[1] < GY and 0 <= v[2] < GZ

_OFFSETS = [
    (di, dj, dk)
    for di in (-1, 0, 1) for dj in (-1, 0, 1) for dk in (-1, 0, 1)
    if not (di == 0 and dj == 0 and dk == 0)
]

def neighbours26(v):
    i, j, k = v
    result = []
    for di, dj, dk in _OFFSETS:
        nb = (i + di, j + dj, k + dk)
        if in_bounds(nb):
            result.append(nb)
    return result

def base_edge_cost(u, v):
    """Euclidean distance between voxel centres."""
    return np.linalg.norm(np.array(u, dtype=float) - np.array(v, dtype=float)) * GRID_RES

# ── Hazard voxel inflation ─────────────────────────────────────────────────────
def compute_hazard_cost_map(active_cells):
    """Return dict mapping voxel → extra hazard weight (sum w_k over active cells)."""
    hazard_weight = {}
    for cell in active_cells:
        cv = world_to_vox(cell.center)
        r_vox = int(np.ceil(cell.radius / GRID_RES)) + 1
        for di in range(-r_vox, r_vox + 1):
            for dj in range(-r_vox, r_vox + 1):
                for dk in range(-r_vox, r_vox + 1):
                    nb = (cv[0] + di, cv[1] + dj, cv[2] + dk)
                    if not in_bounds(nb):
                        continue
                    dist = np.linalg.norm(vox_to_world(nb) - cell.center)
                    if dist <= cell.radius:
                        hazard_weight[nb] = hazard_weight.get(nb, 0.0) + cell.severity
    return hazard_weight

def edge_cost(u, v, hazard_weight):
    """c(u,v,t) = c_base * (1 + alpha * sum_k w_k for v in H_k)."""
    base = base_edge_cost(u, v)
    w = hazard_weight.get(v, 0.0)
    return base * (1.0 + ALPHA * w)

# ── D* Lite ────────────────────────────────────────────────────────────────────
class DStarLite:
    """
    D* Lite: incremental heuristic search.
    Searches BACKWARDS from goal to start so that
    rhs(goal)=0 and path is extracted forward.
    """
    def __init__(self, start_vox, goal_vox):
        self.start = start_vox
        self.goal  = goal_vox
        self.km    = 0.0
        self.g   = {}   # cost-to-come (inf by default)
        self.rhs = {}   # one-step lookahead
        # Initialize goal
        self.rhs[goal_vox] = 0.0
        self.U = []
        heapq.heappush(self.U, (self._key(goal_vox), goal_vox))
        self._in_queue = {goal_vox}

    def _h(self, a, b):
        return np.linalg.norm(np.array(a, dtype=float) - np.array(b, dtype=float)) * GRID_RES

    def _key(self, s):
        g_s   = self.g.get(s, np.inf)
        rhs_s = self.rhs.get(s, np.inf)
        return (min(g_s, rhs_s) + self._h(self.start, s) + self.km,
                min(g_s, rhs_s))

    def update_vertex(self, s, hazard_weight):
        if s != self.goal:
            best = np.inf
            for nb in neighbours26(s):
                c = edge_cost(s, nb, hazard_weight)
                val = c + self.g.get(nb, np.inf)
                if val < best:
                    best = val
            self.rhs[s] = best
        if self.g.get(s, np.inf) != self.rhs.get(s, np.inf):
            heapq.heappush(self.U, (self._key(s), s))
            self._in_queue.add(s)

    def compute_shortest_path(self, hazard_weight):
        expanded = 0
        while self.U:
            k_top, u = self.U[0]
            k_start  = self._key(self.start)
            rhs_start = self.rhs.get(self.start, np.inf)
            g_start   = self.g.get(self.start, np.inf)
            if k_top >= k_start and rhs_start == g_start:
                break
            heapq.heappop(self.U)
            expanded += 1
            k_new = self._key(u)
            if k_top < k_new:
                # stale entry
                heapq.heappush(self.U, (k_new, u))
                continue
            g_u   = self.g.get(u, np.inf)
            rhs_u = self.rhs.get(u, np.inf)
            if g_u > rhs_u:
                self.g[u] = rhs_u
                for nb in neighbours26(u):
                    self.update_vertex(nb, hazard_weight)
            else:
                self.g[u] = np.inf
                self.update_vertex(u, hazard_weight)
                for nb in neighbours26(u):
                    self.update_vertex(nb, hazard_weight)
            if expanded > 200_000:
                break

    def extract_path(self):
        """Greedy descent from start to goal following min g(neighbour)."""
        path = [self.start]
        visited = {self.start}
        s = self.start
        for _ in range(GX * GY * GZ):
            if s == self.goal:
                break
            best_nb, best_g = None, np.inf
            for nb in neighbours26(s):
                if nb in visited:
                    continue
                g_nb = self.g.get(nb, np.inf)
                if g_nb < best_g:
                    best_g = g_nb
                    best_nb = nb
            if best_nb is None:
                break
            path.append(best_nb)
            visited.add(best_nb)
            s = best_nb
        return path

# ── Hazard generation ──────────────────────────────────────────────────────────
def generate_hazards():
    cells = []
    # Stagger activation times; first hazard activates early
    t_on_vals = np.sort(RNG.uniform(5, 350, N_HAZARDS))
    for t_on in t_on_vals:
        # Place along the rough path corridor but randomised
        cx = RNG.uniform(15, 85)
        cy = RNG.uniform(15, 85)
        cz = RNG.uniform(12, 28)
        r  = RNG.uniform(4, 12)
        w  = RNG.uniform(0.3, 1.0)
        tau = RNG.uniform(30, 120)
        cells.append(HazardCell(
            center=np.array([cx, cy, cz]),
            radius=r,
            severity=w,
            t_on=t_on,
            t_off=t_on + tau,
        ))
    return cells

# ── Path–hazard intersection check ────────────────────────────────────────────
def path_intersects_hazard(path_voxels, active_cells):
    if not active_cells:
        return False
    hw = compute_hazard_cost_map(active_cells)
    return any(v in hw for v in path_voxels)

# ── Drone step ────────────────────────────────────────────────────────────────
def step_drone(pos, waypoints, v_max, dt):
    """Advance drone one timestep toward next waypoint; pop waypoints as reached."""
    while waypoints:
        target = vox_to_world(waypoints[0])
        diff   = target - pos
        dist   = np.linalg.norm(diff)
        if dist < v_max * dt * 1.5:
            waypoints.pop(0)
            continue
        vel = v_max * diff / dist
        return pos + vel * dt
    return pos

# ── Main simulation ─────────────────────────────────────────────────────────────
def run_simulation():
    hazard_cells = generate_hazards()

    planner = DStarLite(world_to_vox(P_START), world_to_vox(P_GOAL))
    # Initial plan with no hazards
    planner.compute_shortest_path({})
    path_voxels = planner.extract_path()

    drone_pos    = P_START.copy()
    t            = 0.0
    traj         = [drone_pos.copy()]
    time_log     = [0.0]

    replan_times  = []
    replan_count  = 0
    active_log    = []          # active hazard count per timestep
    hazard_time   = 0.0         # seconds inside any hazard

    prev_active_set = set()
    waypoints = list(path_voxels)   # mutable working list

    while t < T_MAX:
        dist_to_goal = np.linalg.norm(drone_pos - P_GOAL)
        if dist_to_goal < LANDING_THRESHOLD:
            break

        # Determine active cells this step
        active_cells = [c for c in hazard_cells if is_active(c, t)]
        active_set   = set(id(c) for c in active_cells)
        active_log.append(len(active_cells))

        # Hazard exposure metric
        hw_now = compute_hazard_cost_map(active_cells)
        cur_vox = world_to_vox(drone_pos)
        if cur_vox in hw_now:
            hazard_time += DT

        # Check if replanning is needed
        set_changed = active_set != prev_active_set
        need_replan = set_changed and (
            path_intersects_hazard(waypoints[:40], active_cells) or
            path_intersects_hazard(waypoints[:40],
                [c for c in hazard_cells
                 if id(c) in prev_active_set and id(c) not in active_set])
        )

        if need_replan or (not waypoints):
            hw = compute_hazard_cost_map(active_cells)
            new_start = world_to_vox(drone_pos)
            if new_start != planner.start:
                planner.km    += planner._h(planner.start, new_start)
                planner.start  = new_start

            # Update only affected voxels
            changed_voxels = set()
            for cell in active_cells:
                cv = world_to_vox(cell.center)
                r_v = int(np.ceil(cell.radius / GRID_RES)) + 2
                for di in range(-r_v, r_v + 1):
                    for dj in range(-r_v, r_v + 1):
                        for dk in range(-r_v, r_v + 1):
                            nb = (cv[0]+di, cv[1]+dj, cv[2]+dk)
                            if in_bounds(nb):
                                changed_voxels.add(nb)

            for v in changed_voxels:
                planner.update_vertex(v, hw)

            planner.compute_shortest_path(hw)
            path_voxels = planner.extract_path()
            waypoints   = list(path_voxels)
            replan_count += 1
            replan_times.append(t)

        prev_active_set = active_set

        # Move drone
        drone_pos = step_drone(drone_pos, waypoints, V_MAX, DT)
        t        += DT
        traj.append(drone_pos.copy())
        time_log.append(t)

    traj_arr    = np.array(traj)
    time_arr    = np.array(time_log)
    active_arr  = np.array(active_log + [active_log[-1]] if active_log else [0])

    # ── Cost metrics ──────────────────────────────────────────────────────────
    J_base   = np.linalg.norm(P_GOAL - P_START)
    L_actual = float(np.sum(np.linalg.norm(np.diff(traj_arr, axis=0), axis=1)))
    J_detour = max(0.0, L_actual - J_base)
    J_hazard = hazard_time   # seconds

    print(f"Mission time       : {time_arr[-1]:.1f} s")
    print(f"Replanning events  : {replan_count}")
    print(f"J_base (m)         : {J_base:.2f}")
    print(f"J_detour (m)       : {J_detour:.2f}")
    print(f"J_hazard (s)       : {J_hazard:.3f}")
    print(f"Path length (m)    : {L_actual:.2f}")
    detour_pct = 100 * J_detour / J_base if J_base > 0 else 0
    print(f"Detour overhead    : {detour_pct:.1f} %")

    return (traj_arr, time_arr, active_arr, replan_times,
            hazard_cells, J_base, J_detour, J_hazard, L_actual)

# ── Plot helpers ───────────────────────────────────────────────────────────────
def _draw_hazard_sphere_3d(ax, cell, t, alpha=0.15):
    """Draw a wireframe sphere for a hazard cell (if active at t)."""
    if not is_active(cell, t):
        return
    u = np.linspace(0, 2 * np.pi, 12)
    v = np.linspace(0, np.pi, 8)
    r = cell.radius
    cx, cy, cz = cell.center
    x = cx + r * np.outer(np.cos(u), np.sin(v))
    y = cy + r * np.outer(np.sin(u), np.sin(v))
    z = cz + r * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_wireframe(x, y, z, color='orange', alpha=alpha, linewidth=0.5)

# ── Plot 1: 3D trajectory ──────────────────────────────────────────────────────
def plot_trajectory_3d(traj_arr, replan_times, hazard_cells, time_arr, out_dir):
    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')

    # Colour path segments by replanning interval
    colors = plt.cm.cool(np.linspace(0, 1, len(replan_times) + 1))
    bounds = [0] + [int(t / DT) for t in replan_times] + [len(traj_arr) - 1]
    for seg in range(len(bounds) - 1):
        s, e = bounds[seg], bounds[seg + 1]
        ax.plot(traj_arr[s:e+1, 0], traj_arr[s:e+1, 1], traj_arr[s:e+1, 2],
                color=colors[seg], linewidth=1.2, alpha=0.85)

    # Draw hazard cells at mid-mission snapshot (t = T/2)
    t_snap = time_arr[-1] / 2
    for cell in hazard_cells:
        _draw_hazard_sphere_3d(ax, cell, t_snap)

    ax.scatter(*P_START, c='green', s=80, zorder=5, label='Start')
    ax.scatter(*P_GOAL,  c='red',   s=80, zorder=5, label='Goal')

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('S034 – 3D Drone Trajectory with Weather Cells')
    ax.legend(loc='upper left', fontsize=8)
    ax.set_xlim(0, GX * GRID_RES); ax.set_ylim(0, GY * GRID_RES)
    ax.set_zlim(0, GZ * GRID_RES)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')

# ── Plot 2: timeline ─────────────────────────────────────────────────────────
def plot_timeline(time_arr, active_arr, replan_times, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

    # active hazard count
    t_plot = time_arr[:len(active_arr)]
    axes[0].fill_between(t_plot, active_arr, alpha=0.4, color='orange', label='Active hazards')
    axes[0].plot(t_plot, active_arr, color='darkorange', linewidth=0.8)
    axes[0].set_ylabel('Active hazard count')
    axes[0].set_title('Weather Activity and Replanning Events')
    axes[0].legend()

    # replanning events
    for rt in replan_times:
        axes[1].axvline(rt, color='red', linewidth=1.0, alpha=0.7)
    axes[1].set_xlim(0, time_arr[-1])
    axes[1].set_ylim(0, 2)
    axes[1].set_yticks([])
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Replan events')

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'timeline.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')

# ── Plot 3: altitude profile ──────────────────────────────────────────────────
def plot_altitude(traj_arr, time_arr, out_dir):
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(time_arr, traj_arr[:, 2], color='steelblue', linewidth=1.2)
    ax.axhline(DELIVERY_ALT, color='gray', linestyle='--', linewidth=0.8,
               label=f'Nominal {DELIVERY_ALT} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Altitude Time Series — Vertical Detour Manoeuvres')
    ax.set_ylim(0, GZ * GRID_RES)
    ax.legend()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'altitude_profile.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')

# ── Plot 4: cost breakdown ─────────────────────────────────────────────────────
def plot_cost_breakdown(J_base, J_detour, J_hazard, out_dir):
    labels = ['J_base\n(straight-line)', 'J_detour\n(rerouting)', 'J_hazard\n(exposure s)']
    values = [J_base, J_detour, J_hazard]
    colors = ['steelblue', 'coral', 'gold']

    fig, ax = plt.subplots(figsize=(7, 5))
    bars = ax.bar(labels, values, color=colors, edgecolor='black', linewidth=0.7)
    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                f'{val:.2f}', ha='center', va='bottom', fontsize=10)
    ax.set_title('Mission Cost Breakdown')
    ax.set_ylabel('Cost (m or s)')

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'cost_breakdown.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')

# ── Animation ─────────────────────────────────────────────────────────────────
def save_animation(traj_arr, time_arr, hazard_cells, replan_times, out_dir):
    import matplotlib.animation as animation

    STEP    = 6    # frame decimation
    frames  = range(0, len(traj_arr), STEP)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(0, GX * GRID_RES)
    ax.set_ylim(0, GY * GRID_RES)
    ax.set_aspect('equal')
    ax.set_title('S034 Weather Rerouting — Top-Down View')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')

    trail_line, = ax.plot([], [], 'b-', linewidth=1.0, alpha=0.7, label='Path')
    drone_dot,  = ax.plot([], [], 'bo', markersize=6)
    start_dot   = ax.plot(*P_START[:2], 'g^', markersize=9, label='Start')[0]
    goal_dot    = ax.plot(*P_GOAL[:2],  'r*', markersize=10, label='Goal')[0]
    time_text   = ax.text(2, GY * GRID_RES - 4, '', fontsize=9)
    ax.legend(loc='lower right', fontsize=8)

    hazard_patches = []

    def init():
        trail_line.set_data([], [])
        drone_dot.set_data([], [])
        time_text.set_text('')
        return [trail_line, drone_dot, time_text]

    def update(frame_idx):
        for p in hazard_patches:
            p.remove()
        hazard_patches.clear()

        i = frame_idx
        t = time_arr[i] if i < len(time_arr) else time_arr[-1]

        for cell in hazard_cells:
            if is_active(cell, t):
                c = plt.Circle(cell.center[:2], cell.radius,
                               color='orange', alpha=0.35, linewidth=0)
                ax.add_patch(c)
                hazard_patches.append(c)

        trail_x = traj_arr[:i+1, 0]
        trail_y = traj_arr[:i+1, 1]
        # colour last segment red if replanned recently
        last_replan = max((rt for rt in replan_times if rt <= t), default=-1)
        if last_replan > 0 and t - last_replan < 5.0:
            trail_line.set_color('red')
        else:
            trail_line.set_color('blue')
        trail_line.set_data(trail_x, trail_y)
        drone_dot.set_data([traj_arr[i, 0]], [traj_arr[i, 1]])
        time_text.set_text(f't = {t:.1f} s')
        return [trail_line, drone_dot, time_text] + hazard_patches

    frame_list = list(frames)
    ani = animation.FuncAnimation(
        fig, update, frames=frame_list, init_func=init,
        blit=False, interval=50
    )
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')

# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    (traj_arr, time_arr, active_arr, replan_times,
     hazard_cells, J_base, J_detour, J_hazard, L_actual) = run_simulation()

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectory_3d(traj_arr, replan_times, hazard_cells, time_arr, out_dir)
    plot_timeline(time_arr, active_arr, replan_times, out_dir)
    plot_altitude(traj_arr, time_arr, out_dir)
    plot_cost_breakdown(J_base, J_detour, J_hazard, out_dir)
    save_animation(traj_arr, time_arr, hazard_cells, replan_times, out_dir)
