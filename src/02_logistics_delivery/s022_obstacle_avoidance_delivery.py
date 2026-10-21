"""
S022 Obstacle Avoidance Delivery
=================================
RRT* path planning for a delivery drone navigating through a 3D urban
obstacle field. Compares a naive straight-line path (which collides) with
an RRT*-planned, post-shortcut collision-free route.

Outputs (saved to outputs/02_logistics_delivery/s022_obstacle_avoidance_delivery/):
  - trajectory_3d.png        : 3D scene with RRT* path, straight-line path, obstacles, drone track
  - metrics.png              : convergence curve + clearance over time + tracking error over time
  - animation.gif            : top-down + 3D animated drone flight
"""

import sys
import os
import time
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation

# ── Paths ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.insert(0, PROJECT_ROOT)
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "outputs", "02_logistics_delivery",
                          "s022_obstacle_avoidance_delivery")
os.makedirs(OUTPUT_DIR, exist_ok=True)

from src.base.drone_base import DroneBase

# ── Key Constants ──────────────────────────────────────────────────────────────
CRUISE_SPEED   = 3.0     # m/s
MAX_SPEED      = 4.0     # m/s
MAX_ITER       = 2000    # RRT* iterations
STEP_SIZE      = 1.0     # eta (m)
GAMMA_RRT      = 12.0    # gamma_RRT* coefficient
DRONE_RADIUS   = 0.15    # safety radius (m)
WP_THRESHOLD   = 0.3     # waypoint acceptance radius (m)
CTRL_FREQ      = 48      # Hz
DT             = 1.0 / CTRL_FREQ
MAX_SIM_TIME   = 120.0   # seconds

START = np.array([0.0,  0.0, 2.0])
GOAL  = np.array([18.0, 12.0, 2.0])

BOUNDS = np.array([[-1.0, 20.0],   # x
                   [-1.0, 14.0],   # y
                   [ 1.0,  5.0]])  # z  (card: flight altitude range 1.0-5.0 m)
GOAL_BIAS      = 0.10    # probability of sampling the goal directly
GOAL_CONNECT   = 1.5 * STEP_SIZE   # try a goal connection from nodes this close

# Obstacle layout: (cx, cy, radius, height)
OBSTACLES = [
    (4.0,  2.0,  1.2, 4.0),
    (7.0,  6.0,  1.0, 5.0),
    (9.0,  1.5,  0.8, 3.5),
    (12.0, 8.0,  1.3, 4.5),
    (14.5, 4.0,  0.9, 3.0),
    (16.0, 10.0, 1.1, 4.0),
]

RANDOM_SEED = 42


# ── Collision Checking ─────────────────────────────────────────────────────────

def _point_cylinder_dist2d(px, py, cx, cy, r):
    """2D distance from point to cylinder axis, minus radius."""
    return math.hypot(px - cx, py - cy) - r


def _seg_to_point_dist2d(ax, ay, bx, by, px, py):
    """Minimum 2D distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    t = 0.0
    denom = dx*dx + dy*dy
    if denom > 1e-12:
        t = ((px - ax)*dx + (py - ay)*dy) / denom
        t = max(0.0, min(1.0, t))
    closest_x = ax + t * dx
    closest_y = ay + t * dy
    return math.hypot(px - closest_x, py - closest_y)


def segment_collision_free(qa, qb, obstacles, drone_r=DRONE_RADIUS):
    """
    Return True if the segment qa->qb is free of all cylinder obstacles.
    Uses per-obstacle vertical range check + 2D segment-to-axis clearance.
    """
    ax, ay, az = qa
    bx, by, bz = qb
    for (cx, cy, r, h) in obstacles:
        # Vertical overlap: does the segment pass through [0, h]?
        z_lo = min(az, bz)
        z_hi = max(az, bz)
        if z_hi <= 0.0 or z_lo >= h:
            continue  # segment entirely above or below this cylinder
        # 2D minimum clearance from cylinder axis to segment
        d2d = _seg_to_point_dist2d(ax, ay, bx, by, cx, cy)
        if d2d < r + drone_r:
            return False
    return True


def point_in_bounds(q, bounds=BOUNDS):
    for i in range(3):
        if q[i] < bounds[i, 0] or q[i] > bounds[i, 1]:
            return False
    return True


def point_collision_free(q, obstacles=OBSTACLES, drone_r=DRONE_RADIUS):
    for (cx, cy, r, h) in obstacles:
        if q[2] < h:  # within height
            if math.hypot(q[0]-cx, q[1]-cy) < r + drone_r:
                return False
    return True


# ── RRT* ───────────────────────────────────────────────────────────────────────

class Node:
    __slots__ = ("pos", "parent", "cost", "children")

    def __init__(self, pos, parent=None, cost=0.0):
        self.pos = np.asarray(pos, dtype=float)
        self.parent = parent
        self.cost = cost
        self.children = []


class RRTStar:
    def __init__(self, start, goal, obstacles, bounds, rng,
                 step=STEP_SIZE, gamma=GAMMA_RRT, max_nodes=MAX_ITER + 1):
        self.start = Node(start, cost=0.0)
        self.goal_pos = np.asarray(goal, dtype=float)
        self.obstacles = obstacles
        self.bounds = bounds
        self.rng   = rng
        self.step  = step
        self.gamma = gamma
        self.nodes = [self.start]
        self._pos  = np.zeros((max_nodes, 3))      # vectorised node positions
        self._pos[0] = self.start.pos
        self.goal_parents = []    # tree nodes with a collision-free edge to the goal
        self.cost_history = []    # best goal cost after every iteration (inf = none yet)
        self.first_solution_iter = None
        self.n_accepted = 0

    # -- primitives ---------------------------------------------------------
    def _sample(self):
        if self.rng.random() < GOAL_BIAS:
            return self.goal_pos.copy()
        lo = self.bounds[:, 0]
        hi = self.bounds[:, 1]
        return self.rng.uniform(lo, hi)

    def _dists(self, q):
        n = len(self.nodes)
        return np.linalg.norm(self._pos[:n] - q, axis=1)

    def _nearest(self, q):
        return self.nodes[int(np.argmin(self._dists(q)))]

    def _steer(self, q_near, q_rand):
        diff = q_rand - q_near.pos
        d = np.linalg.norm(diff)
        if d < 1e-9:
            return q_near.pos.copy()
        return q_near.pos + (diff / d) * min(d, self.step)

    def near_radius(self):
        """Card: r_n = min(gamma * (log n / n)^(1/d), eta), d = 3."""
        n = max(len(self.nodes), 2)
        r = self.gamma * (math.log(n) / n) ** (1.0 / 3.0)
        return min(r, self.step)

    def _near_nodes(self, q_new):
        r = self.near_radius()
        idx = np.nonzero(self._dists(q_new) <= r)[0]
        return [self.nodes[i] for i in idx]

    def _choose_parent(self, q_new, near_nodes):
        q_min = None
        c_min = float("inf")
        for nd in near_nodes:
            c = nd.cost + np.linalg.norm(nd.pos - q_new)
            if c < c_min and segment_collision_free(nd.pos, q_new, self.obstacles):
                c_min = c
                q_min = nd
        return q_min, c_min

    @staticmethod
    def _propagate_cost(node):
        """After re-parenting, push the cost change down the subtree."""
        stack = [node]
        while stack:
            nd = stack.pop()
            for ch in nd.children:
                ch.cost = nd.cost + np.linalg.norm(ch.pos - nd.pos)
                stack.append(ch)

    def _set_parent(self, node, parent, cost):
        if node.parent is not None:
            node.parent.children.remove(node)
        node.parent = parent
        node.cost = cost
        parent.children.append(node)

    def _rewire(self, new_node, near_nodes):
        for nd in near_nodes:
            if nd is new_node.parent or nd is new_node:
                continue
            new_cost = new_node.cost + np.linalg.norm(new_node.pos - nd.pos)
            if new_cost < nd.cost - 1e-12:
                if segment_collision_free(new_node.pos, nd.pos, self.obstacles):
                    self._set_parent(nd, new_node, new_cost)
                    self._propagate_cost(nd)

    def best_goal(self):
        """(best total cost, parent node) over all goal-connected tree nodes."""
        best_c, best_nd = float("inf"), None
        for nd in self.goal_parents:
            c = nd.cost + np.linalg.norm(nd.pos - self.goal_pos)
            if c < best_c:
                best_c, best_nd = c, nd
        return best_c, best_nd

    def _extract_path(self, goal_parent):
        path = [self.goal_pos.copy()]
        nd = goal_parent
        while nd is not None:
            path.append(nd.pos.copy())
            nd = nd.parent
        path.reverse()
        return path

    # -- one iteration -------------------------------------------------------
    def _iterate(self):
        q_rand = self._sample()
        q_near = self._nearest(q_rand)
        q_new_pos = self._steer(q_near, q_rand)

        if not point_in_bounds(q_new_pos, self.bounds):
            return
        if not point_collision_free(q_new_pos, self.obstacles):
            return
        if not segment_collision_free(q_near.pos, q_new_pos, self.obstacles):
            return

        near_nodes = self._near_nodes(q_new_pos)
        q_min, c_min = self._choose_parent(q_new_pos, near_nodes)
        if q_min is None:
            # q_near is always collision-free-connected (checked above)
            q_min = q_near
            c_min = q_near.cost + np.linalg.norm(q_near.pos - q_new_pos)

        new_node = Node(q_new_pos)
        self._set_parent(new_node, q_min, c_min)
        self._pos[len(self.nodes)] = q_new_pos
        self.nodes.append(new_node)
        self.n_accepted += 1
        self._rewire(new_node, near_nodes)

        # Goal connection
        d_to_goal = np.linalg.norm(new_node.pos - self.goal_pos)
        if d_to_goal < GOAL_CONNECT and \
                segment_collision_free(new_node.pos, self.goal_pos, self.obstacles):
            self.goal_parents.append(new_node)

    def plan(self, max_iter=MAX_ITER):
        for i in range(max_iter):
            self._iterate()
            best_c, _ = self.best_goal()
            if self.first_solution_iter is None and np.isfinite(best_c):
                self.first_solution_iter = i + 1
            self.cost_history.append(best_c)

        best_c, best_nd = self.best_goal()
        if best_nd is None:
            return None
        return self._extract_path(best_nd)


# ── Path Post-processing ───────────────────────────────────────────────────────

def shortcut_path(path, obstacles=OBSTACLES):
    """Greedy shortcut pass: remove waypoints that can be skipped."""
    path = list(path)
    i = 0
    while i < len(path) - 2:
        if segment_collision_free(path[i], path[i + 2], obstacles):
            path.pop(i + 1)
        else:
            i += 1
    return path


def path_length(path):
    return sum(np.linalg.norm(np.array(path[k+1]) - np.array(path[k]))
               for k in range(len(path) - 1))


# ── Waypoint Tracker ───────────────────────────────────────────────────────────

def run_tracker(waypoints, cruise=CRUISE_SPEED, max_speed=MAX_SPEED,
                wp_thresh=WP_THRESHOLD, dt=DT, max_time=MAX_SIM_TIME,
                obstacles=OBSTACLES):
    """
    Fly a DroneBase through the waypoint list.
    Returns trajectory array, clearance array, tracking-error array.
    """
    drone = DroneBase(init_pos=waypoints[0], max_speed=max_speed, dt=dt)
    wp_idx = 1
    clearances = []
    track_errors = []

    # Build a dense reference trajectory (one point per waypoint segment)
    ref_pts = _interpolate_reference(waypoints, dt, cruise)

    t = 0.0
    step_count = 0
    while t < max_time and wp_idx < len(waypoints):
        target = np.array(waypoints[wp_idx])
        diff = target - drone.pos
        dist = np.linalg.norm(diff)

        if dist < wp_thresh:
            wp_idx += 1
            if wp_idx >= len(waypoints):
                break
            continue

        v_cmd = cruise * diff / (dist + 1e-9)
        drone.step(v_cmd)

        # Clearance to nearest obstacle
        cl = _min_clearance(drone.pos, obstacles)
        clearances.append(cl)

        # Tracking error vs time-indexed constant-speed reference
        ref_idx = min(step_count + 1, len(ref_pts) - 1)
        err = np.linalg.norm(drone.pos - ref_pts[ref_idx])
        track_errors.append(err)

        t += dt
        step_count += 1

    traj = drone.get_trajectory()
    return traj, np.array(clearances), np.array(track_errors)


def _interpolate_reference(waypoints, dt, speed):
    """Dense point-cloud reference path at constant speed."""
    pts = [np.array(waypoints[0])]
    for k in range(1, len(waypoints)):
        seg = np.array(waypoints[k]) - np.array(waypoints[k-1])
        seg_len = np.linalg.norm(seg)
        if seg_len < 1e-9:
            continue
        n_steps = max(1, int(round(seg_len / (speed * dt))))
        for j in range(1, n_steps + 1):
            pts.append(np.array(waypoints[k-1]) + seg * (j / n_steps))
    return pts


def _dist_to_polyline(p, path):
    """3D distance from point p to the planned waypoint polyline (cross-track)."""
    best = float("inf")
    for k in range(len(path) - 1):
        a = np.asarray(path[k]); b = np.asarray(path[k + 1])
        ab = b - a
        t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(p - (a + t * ab))))
    return best


def _min_clearance(pos, obstacles):
    best = float("inf")
    for (cx, cy, r, h) in obstacles:
        if pos[2] < h:
            d = math.hypot(pos[0]-cx, pos[1]-cy) - r
            best = min(best, d)
    return best


# ── Visualisation Helpers ──────────────────────────────────────────────────────

def _draw_cylinder(ax, cx, cy, r, h, color="grey", alpha=0.25, n=30):
    """Add a vertical cylinder to a 3D axes."""
    theta = np.linspace(0, 2*np.pi, n)
    z_base = np.array([0.0, h])
    # Side surface
    xs = cx + r * np.outer(np.cos(theta), np.ones(2))
    ys = cy + r * np.outer(np.sin(theta), np.ones(2))
    zs = np.outer(np.ones(n), z_base)
    ax.plot_surface(xs, ys, zs, color=color, alpha=alpha, linewidth=0)
    # Top cap
    x_top = [cx + r * math.cos(t) for t in theta]
    y_top = [cy + r * math.sin(t) for t in theta]
    z_top = [h] * n
    verts = [list(zip(x_top, y_top, z_top))]
    poly = Poly3DCollection(verts, color=color, alpha=alpha + 0.1)
    ax.add_collection3d(poly)


def plot_trajectory_3d(rrt_path, straight_path, drone_traj,
                       tree_nodes, obstacles, start, goal, out_dir, rng):
    """Plot 1: 3D scene overview."""
    fig = plt.figure(figsize=(14, 9))
    ax = fig.add_subplot(111, projection="3d")

    # Draw obstacles
    for (cx, cy, r, h) in obstacles:
        _draw_cylinder(ax, cx, cy, r, h)

    # Draw RRT* tree (sample of nodes)
    tree_arr = np.array([n.pos for n in tree_nodes])
    sample_idx = rng.choice(len(tree_arr),
                                  size=min(500, len(tree_arr)), replace=False)
    ax.scatter(tree_arr[sample_idx, 0], tree_arr[sample_idx, 1],
               tree_arr[sample_idx, 2],
               c="lightgrey", s=4, alpha=0.5, label="RRT* tree (sample)")

    # Straight-line path (dashed red)
    sl = np.array(straight_path)
    ax.plot(sl[:, 0], sl[:, 1], sl[:, 2],
            "r--", lw=2, label="Straight-line (collides)")

    # RRT* path (green)
    rp = np.array(rrt_path)
    ax.plot(rp[:, 0], rp[:, 1], rp[:, 2],
            "g-", lw=2.5, label="RRT* path")

    # Drone trajectory (blue)
    dt_arr = np.array(drone_traj)
    ax.plot(dt_arr[:, 0], dt_arr[:, 1], dt_arr[:, 2],
            "b-", lw=1.5, alpha=0.8, label="Drone flight")

    # Start / Goal
    ax.scatter(*start, c="green", s=120, marker="o", zorder=5, label="Start")
    ax.scatter(*goal,  c="gold",  s=120, marker="*", zorder=5, label="Goal")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("S022 Obstacle Avoidance Delivery — 3D Scene")
    ax.legend(loc="upper left", fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, "trajectory_3d.png")
    plt.savefig(path, dpi=120)
    plt.close()
    print(f"  Saved: {path}")


def plot_metrics(cost_history, clearances, track_errors, drone_radius, out_dir,
                 first_iter=None, cross_track=None):
    """Plot 2: convergence curve, clearance over time, tracking error."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 11))

    # --- convergence ---
    ax = axes[0]
    finite_mask = np.isfinite(cost_history)
    iters = np.arange(1, len(cost_history) + 1)
    if finite_mask.any():
        ax.plot(iters[finite_mask], np.array(cost_history)[finite_mask],
                color="steelblue", lw=1.5, label="Best raw-tree path length")
    if first_iter is not None:
        ax.axvline(first_iter, color="grey", ls=":", lw=1.2,
                   label=f"First solution (iter {first_iter})")
        ax.legend(fontsize=8)
    ax.set_xlabel("RRT* Iteration")
    ax.set_ylabel("Best Path Length (m)")
    ax.set_title("RRT* Cost Convergence")
    ax.grid(True, alpha=0.4)

    # --- clearance ---
    ax = axes[1]
    t_cl = np.arange(len(clearances)) / CTRL_FREQ
    ax.plot(t_cl, clearances, color="darkorange", lw=1.2, label="Clearance")
    ax.axhline(drone_radius, color="red", ls="--", lw=1.5,
               label=f"Drone radius ({drone_radius} m)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Clearance to nearest obstacle (m)")
    ax.set_title("Minimum Obstacle Clearance During Flight")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.4)

    # --- tracking error ---
    ax = axes[2]
    t_te = np.arange(len(track_errors)) / CTRL_FREQ
    ax.plot(t_te, track_errors, color="purple", lw=1.2,
            label="|p(t) - p_ref(t)| (time-indexed, card metric)")
    if cross_track is not None:
        ax.plot(np.arange(len(cross_track)) / CTRL_FREQ, cross_track,
                color="teal", lw=1.2, label="Distance to planned polyline")
    ax.axhline(0.1, color="red", ls="--", lw=1.5, label="0.1 m threshold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Instantaneous tracking error (m)")
    ax.set_title("Tracking Error vs Constant-Speed Reference "
                 "(kinematic follower: only corner-cutting at r_wp)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.4)

    plt.suptitle("S022 — Path Planning & Flight Metrics", fontsize=13,
                 fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    path = os.path.join(out_dir, "metrics.png")
    plt.savefig(path, dpi=120)
    plt.close()
    print(f"  Saved: {path}")


def save_animation(rrt_path, drone_traj, obstacles, start, goal, out_dir):
    """GIF: top-down view showing drone flying along RRT* path."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    ax_top = axes[0]
    ax_3d  = fig.add_subplot(122, projection="3d")
    axes[1].remove()

    traj = np.array(drone_traj)
    rp   = np.array(rrt_path)

    # Static elements — top-down
    for (cx, cy, r, h) in obstacles:
        circle = plt.Circle((cx, cy), r, color="grey", alpha=0.4)
        ax_top.add_patch(circle)
    ax_top.plot(rp[:, 0], rp[:, 1], "g-", lw=1.5, alpha=0.7, label="RRT* path")
    ax_top.scatter(*start[:2], c="green", s=100, zorder=5)
    ax_top.scatter(*goal[:2],  c="gold",  s=100, marker="*", zorder=5)
    ax_top.set_xlim(-1, 20); ax_top.set_ylim(-1, 14)
    ax_top.set_aspect("equal")
    ax_top.set_xlabel("X (m)"); ax_top.set_ylabel("Y (m)")
    ax_top.set_title("Top-Down View")
    ax_top.grid(True, alpha=0.3)

    drone_dot_top,  = ax_top.plot([], [], "bo", ms=8, zorder=10)
    trail_top,      = ax_top.plot([], [], "b-", lw=1.2, alpha=0.6)

    # Static elements — 3D
    for (cx, cy, r, h) in obstacles:
        _draw_cylinder(ax_3d, cx, cy, r, h)
    ax_3d.plot(rp[:, 0], rp[:, 1], rp[:, 2], "g-", lw=1.5, alpha=0.7)
    ax_3d.scatter(*start, c="green", s=80)
    ax_3d.scatter(*goal,  c="gold",  s=80, marker="*")
    ax_3d.set_xlabel("X"); ax_3d.set_ylabel("Y"); ax_3d.set_zlabel("Z")
    ax_3d.set_title("3D View")

    drone_dot_3d,  = ax_3d.plot([], [], [], "bo", ms=8, zorder=10)
    trail_3d,      = ax_3d.plot([], [], [], "b-", lw=1.2, alpha=0.6)

    # Sub-sample trajectory for animation
    n_frames = min(150, len(traj))
    frame_idx = np.linspace(0, len(traj) - 1, n_frames, dtype=int)

    def init():
        drone_dot_top.set_data([], [])
        trail_top.set_data([], [])
        drone_dot_3d.set_data([], [])
        drone_dot_3d.set_3d_properties([])
        trail_3d.set_data([], [])
        trail_3d.set_3d_properties([])
        return drone_dot_top, trail_top, drone_dot_3d, trail_3d

    def update(frame):
        idx = frame_idx[frame]
        x, y, z = traj[idx]
        # top-down
        drone_dot_top.set_data([x], [y])
        trail_top.set_data(traj[:idx+1, 0], traj[:idx+1, 1])
        # 3D
        drone_dot_3d.set_data([x], [y])
        drone_dot_3d.set_3d_properties([z])
        trail_3d.set_data(traj[:idx+1, 0], traj[:idx+1, 1])
        trail_3d.set_3d_properties(traj[:idx+1, 2])
        return drone_dot_top, trail_top, drone_dot_3d, trail_3d

    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  init_func=init, blit=False, interval=50)
    gif_path = os.path.join(out_dir, "animation.gif")
    ani.save(gif_path, writer="pillow", fps=20)
    plt.close()
    print(f"  Saved: {gif_path}")


# ── Main Simulation ────────────────────────────────────────────────────────────

def run_simulation():
    print("=" * 60)
    print("S022 Obstacle Avoidance Delivery")
    print("=" * 60)

    straight_line_dist = np.linalg.norm(GOAL - START)
    print(f"  Start : {START}")
    print(f"  Goal  : {GOAL}")
    print(f"  Straight-line distance: {straight_line_dist:.2f} m")

    # ── RRT* planning ─────────────────────────────────────────────────────────
    print("\n[Step 1] Running RRT* planner …")
    rng = np.random.default_rng(RANDOM_SEED)
    t0 = time.perf_counter()
    planner = RRTStar(START, GOAL, OBSTACLES, BOUNDS, rng)
    raw_path = planner.plan(max_iter=MAX_ITER)
    planning_time = time.perf_counter() - t0
    raw_len = path_length(raw_path) if raw_path is not None else float("nan")

    if raw_path is None:
        print("  ERROR: RRT* failed to find a path. Increase MAX_ITER.")
        sys.exit(1)

    print(f"  Iterations    : {MAX_ITER}  (accepted nodes: {planner.n_accepted})")
    print(f"  First solution: iteration {planner.first_solution_iter}, "
          f"cost {planner.cost_history[planner.first_solution_iter - 1]:.2f} m")
    print(f"  Final near radius r_n : {planner.near_radius():.3f} m")
    print(f"  RRT* nodes    : {len(planner.nodes)}")
    print(f"  Raw waypoints : {len(raw_path)}")
    print(f"  Raw path length       : {raw_len:.2f} m")
    print(f"  (wall-clock planning time, machine-dependent, not printed "
          f"so the console output is reproducible)")

    # ── Post-processing ────────────────────────────────────────────────────────
    print("\n[Step 2] Shortcutting path …")
    short_path = shortcut_path(list(raw_path))
    rrt_len    = path_length(short_path)
    detour     = rrt_len / straight_line_dist
    print(f"  Waypoints after shortcut : {len(short_path)}")
    print(f"  Path length              : {rrt_len:.2f} m")
    print(f"  Detour ratio ρ           : {detour:.3f}")

    # Verify collision-free
    all_free = all(
        segment_collision_free(short_path[k], short_path[k+1], OBSTACLES)
        for k in range(len(short_path) - 1)
    )
    print(f"  All segments collision-free: {all_free}")

    # ── Straight-line path ─────────────────────────────────────────────────────
    straight_path = [START, GOAL]
    straight_free = segment_collision_free(START, GOAL, OBSTACLES)
    print(f"\n[Step 3] Straight-line collision-free: {straight_free} "
          f"(expected False)")

    # ── Drone simulation ───────────────────────────────────────────────────────
    print("\n[Step 4] Simulating drone flight along RRT* path …")
    drone_traj, clearances, track_errors = run_tracker(short_path)

    # Metrics
    min_clearance = float(np.min(clearances)) if len(clearances) > 0 else 0.0
    track_rmse    = float(np.sqrt(np.mean(track_errors**2))) if len(track_errors) > 0 else 0.0
    track_max     = float(np.max(track_errors)) if len(track_errors) > 0 else 0.0
    flight_time   = (len(drone_traj) - 1) * DT
    final_err     = float(np.linalg.norm(drone_traj[-1] - GOAL))
    cross_track   = np.array([_dist_to_polyline(p, short_path) for p in drone_traj[1:]])
    xt_max        = float(np.max(cross_track))

    print(f"  Flight simulation steps  : {len(drone_traj)}")
    print(f"  Flight time              : {flight_time:.2f} s")
    print(f"  Final distance to goal   : {final_err:.3f} m  (r_wp = {WP_THRESHOLD} m)")
    print(f"  Min clearance to obs.    : {min_clearance:.3f} m  (drone r = {DRONE_RADIUS} m)")
    print(f"  Tracking RMSE            : {track_rmse:.4f} m  (max {track_max:.4f} m, "
          f"final {track_errors[-1]:.4f} m)")
    print(f"  Max dist. to planned path: {xt_max:.4f} m  (corner cutting)")

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY METRICS")
    print("=" * 60)
    print(f"  Straight-line distance   : {straight_line_dist:.2f} m")
    print(f"  RRT* raw path length     : {raw_len:.2f} m")
    print(f"  RRT* path length (short) : {rrt_len:.2f} m")
    print(f"  Detour ratio ρ           : {detour:.3f}")
    print(f"  RRT* iterations          : {MAX_ITER} (first solution @ {planner.first_solution_iter})")
    print(f"  Min obstacle clearance   : {min_clearance:.3f} m")
    tracking_ok = "OK < 0.1 m" if track_rmse < 0.1 else "FAIL > 0.1 m"
    print(f"  Tracking RMSE            : {track_rmse:.4f} m  ({tracking_ok})")
    print(f"  Max dist. to planned path: {xt_max:.4f} m")
    print(f"  Straight-line free       : {straight_free}")
    print(f"  RRT* path free           : {all_free}")
    print("=" * 60)

    # ── Plots ──────────────────────────────────────────────────────────────────
    print("\n[Step 5] Generating plots …")
    plot_trajectory_3d(short_path, straight_path, drone_traj,
                       planner.nodes, OBSTACLES, START, GOAL, OUTPUT_DIR,
                       np.random.default_rng(RANDOM_SEED + 1))
    plot_metrics(planner.cost_history, clearances, track_errors,
                 DRONE_RADIUS, OUTPUT_DIR, planner.first_solution_iter,
                 cross_track)
    save_animation(short_path, drone_traj, OBSTACLES, START, GOAL, OUTPUT_DIR)

    print("\nAll outputs saved to:")
    print(f"  {OUTPUT_DIR}")

    return {
        "straight_line_dist": straight_line_dist,
        "rrt_path_length": rrt_len,
        "detour_ratio": detour,
        "planning_time_s": planning_time,   # wall-clock, machine-dependent
        "first_solution_iter": planner.first_solution_iter,
        "raw_path_length": raw_len,
        "min_clearance_m": min_clearance,
        "tracking_rmse_m": track_rmse,
        "max_cross_track_m": xt_max,
        "rrt_nodes": len(planner.nodes),
        "waypoints": len(short_path),
        "flight_time_s": flight_time,
        "straight_line_free": straight_free,
        "rrt_path_free": all_free,
    }


if __name__ == "__main__":
    results = run_simulation()
