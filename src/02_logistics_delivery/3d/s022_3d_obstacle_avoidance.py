"""
S022 3D Obstacle Avoidance Delivery
=====================================
A single delivery drone must fly from a depot to a drop zone through a 3D
urban-like environment containing finite-height cylindrical building obstacles.
The drone can fly *over* short obstacles if that is cheaper.  Two planners are
compared head-to-head:

  1. Straight-line (collision path)  — naive direct flight that collides
  2. RRT* with shortcutting          — asymptotically-optimal, collision-free

The drone then tracks the RRT* waypoints with a proportional velocity
controller and clearance is monitored over the entire flight.

Outputs (saved to outputs/02_logistics_delivery/3d/s022_3d_obstacle_avoidance/):
  - path_3d.png            — 3D scene: obstacles, both paths, drone trajectory
  - rrt_tree.png           — 2D top-down view of the RRT* exploration tree
  - convergence.png        — best-path length vs. RRT* iteration
  - clearance.png          — min clearance to any obstacle vs. time
  - tracking_error.png     — waypoint-tracking error vs. time
  - animation.gif          — animated 3D flight

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s022_3d_obstacle_avoidance.py
"""

import matplotlib
matplotlib.use("Agg")

import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))

# ── Output directory ───────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..",
    "outputs", "02_logistics_delivery", "3d", "s022_3d_obstacle_avoidance",
))

# ── Reproducibility ───────────────────────────────────────────────────────────
RNG = np.random.default_rng(0)

# ── Scenario parameters ───────────────────────────────────────────────────────
START      = np.array([0.0,  0.0, 2.0])   # m
GOAL       = np.array([18.0, 12.0, 2.0])  # m
BOUNDS     = np.array([[0.0, 20.0],        # x
                        [0.0, 14.0],       # y
                        [1.0,  5.0]])      # z  (flight altitude range)

CRUISE_SPEED  = 3.0     # m/s
MAX_ITER      = 2000    # RRT* iterations
STEP_SIZE     = 1.0     # m   — eta
GAMMA_RRT     = 12.0    # γ_RRT* coefficient
DRONE_RADIUS  = 0.15    # m   — collision safety radius
WP_THRESHOLD  = 0.3     # m   — waypoint acceptance radius
DT            = 1.0 / 48  # s   — control frequency 48 Hz
GOAL_BIAS     = 0.10    # probability of sampling the goal directly

# ── Obstacle layout (x_c, y_c, radius, height) ────────────────────────────────
OBSTACLES = np.array([
    [4.0,  2.0,  1.2, 4.0],
    [7.0,  6.0,  1.0, 5.0],
    [9.0,  1.5,  0.8, 3.5],
    [12.0, 8.0,  1.3, 4.5],
    [14.5, 4.0,  0.9, 3.0],
    [16.0, 10.0, 1.1, 4.0],
])  # columns: cx, cy, r, h


# ═══════════════════════════════════════════════════════════════════════════════
# Collision checking
# ═══════════════════════════════════════════════════════════════════════════════

def _seg2d_min_dist(ax, ay, bx, by, cx, cy):
    """Minimum 2-D distance from point (cx,cy) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    len2 = dx * dx + dy * dy
    if len2 < 1e-12:
        return np.hypot(cx - ax, cy - ay)
    t = max(0.0, min(1.0, ((cx - ax) * dx + (cy - ay) * dy) / len2))
    px, py = ax + t * dx, ay + t * dy
    return np.hypot(cx - px, cy - py)


def segment_free(qa, qb, obstacles=OBSTACLES, r_drone=DRONE_RADIUS):
    """Return True iff the 3-D segment qa→qb is collision-free."""
    for (cx, cy, r, h) in obstacles:
        # Vertical overlap: segment must overlap [0, h] in z
        z_lo = min(qa[2], qb[2])
        z_hi = max(qa[2], qb[2])
        if z_lo > h:           # segment entirely above obstacle
            continue
        # Horizontal clearance
        d = _seg2d_min_dist(qa[0], qa[1], qb[0], qb[1], cx, cy)
        if d <= r + r_drone:
            return False
    return True


def point_free(q, obstacles=OBSTACLES, r_drone=DRONE_RADIUS):
    """Return True iff point q is outside all obstacles."""
    for (cx, cy, r, h) in obstacles:
        if q[2] > h:
            continue
        if np.hypot(q[0] - cx, q[1] - cy) <= r + r_drone:
            return False
    return True


def clearance_to_obstacles(q, obstacles=OBSTACLES):
    """Minimum clearance (m) from q to the nearest obstacle surface."""
    min_cl = np.inf
    for (cx, cy, r, h) in obstacles:
        if q[2] > h:
            # above cylinder: distance is 3-D to top rim — simplify to horizontal
            horiz = max(0.0, np.hypot(q[0] - cx, q[1] - cy) - r)
            vert  = q[2] - h
            cl = np.hypot(horiz, vert)
        else:
            cl = max(0.0, np.hypot(q[0] - cx, q[1] - cy) - r)
        min_cl = min(min_cl, cl)
    return min_cl


# ═══════════════════════════════════════════════════════════════════════════════
# RRT* planner
# ═══════════════════════════════════════════════════════════════════════════════

class Node:
    __slots__ = ("pos", "parent", "cost")

    def __init__(self, pos):
        self.pos    = np.asarray(pos, dtype=float)
        self.parent = None
        self.cost   = 0.0


def plan_rrt_star(start, goal,
                  bounds=BOUNDS,
                  max_iter=MAX_ITER,
                  step=STEP_SIZE,
                  gamma=GAMMA_RRT,
                  goal_bias=GOAL_BIAS):
    """Run RRT* and return (path, nodes, best_lengths)."""
    root = Node(start)
    nodes = [root]
    goal_node  = None
    best_cost  = np.inf
    best_lengths = []      # best path length after each iteration

    for it in range(max_iter):
        # ── Sample ──────────────────────────────────────────────────────────
        if RNG.random() < goal_bias:
            q_rand = goal.copy()
        else:
            q_rand = np.array([
                RNG.uniform(bounds[0, 0], bounds[0, 1]),
                RNG.uniform(bounds[1, 0], bounds[1, 1]),
                RNG.uniform(bounds[2, 0], bounds[2, 1]),
            ])

        # ── Nearest ─────────────────────────────────────────────────────────
        dists  = [np.linalg.norm(nd.pos - q_rand) for nd in nodes]
        q_near = nodes[int(np.argmin(dists))]

        # ── Steer ───────────────────────────────────────────────────────────
        diff = q_rand - q_near.pos
        dist = np.linalg.norm(diff)
        if dist < 1e-9:
            continue
        q_new_pos = q_near.pos + step * diff / dist
        # Clamp to bounds
        q_new_pos = np.clip(q_new_pos, bounds[:, 0], bounds[:, 1])

        if not point_free(q_new_pos):
            continue
        if not segment_free(q_near.pos, q_new_pos):
            continue

        # ── Near-node radius ─────────────────────────────────────────────────
        n  = len(nodes)
        rn = min(gamma * (np.log(max(n, 2)) / n) ** (1.0 / 3), step)

        near_nodes = [nd for nd in nodes
                      if np.linalg.norm(nd.pos - q_new_pos) < rn]

        # ── Choose best parent ───────────────────────────────────────────────
        q_new = Node(q_new_pos)
        best_parent = q_near
        best_c      = q_near.cost + np.linalg.norm(q_near.pos - q_new_pos)
        for nd in near_nodes:
            c = nd.cost + np.linalg.norm(nd.pos - q_new_pos)
            if c < best_c and segment_free(nd.pos, q_new_pos):
                best_parent = nd
                best_c      = c
        q_new.parent = best_parent
        q_new.cost   = best_c
        nodes.append(q_new)

        # ── Rewire ───────────────────────────────────────────────────────────
        for nd in near_nodes:
            new_c = q_new.cost + np.linalg.norm(q_new.pos - nd.pos)
            if new_c < nd.cost and segment_free(q_new.pos, nd.pos):
                nd.parent = q_new
                nd.cost   = new_c

        # ── Goal check ───────────────────────────────────────────────────────
        if (np.linalg.norm(q_new_pos - goal) < step
                and segment_free(q_new_pos, goal)):
            total = q_new.cost + np.linalg.norm(q_new_pos - goal)
            if total < best_cost:
                best_cost  = total
                goal_node  = q_new

        best_lengths.append(best_cost if best_cost < np.inf else np.nan)

    # ── Extract path ─────────────────────────────────────────────────────────
    if goal_node is None:
        raise RuntimeError("RRT* failed to find a path — increase MAX_ITER")

    path = [goal.copy()]
    nd = goal_node
    while nd is not None:
        path.append(nd.pos.copy())
        nd = nd.parent
    path.reverse()

    return path, nodes, best_lengths


def shortcut_path(path):
    """Greedy shortcut pass: remove waypoints whose skipping is collision-free."""
    path = list(path)
    i = 0
    while i < len(path) - 2:
        if segment_free(path[i], path[i + 2]):
            path.pop(i + 1)
        else:
            i += 1
    return path


# ═══════════════════════════════════════════════════════════════════════════════
# Waypoint-tracking controller
# ═══════════════════════════════════════════════════════════════════════════════

def track_waypoints(waypoints, dt=DT, v_cruise=CRUISE_SPEED,
                    wp_thresh=WP_THRESHOLD):
    """Simulate drone tracking the waypoint list. Return trajectory array."""
    pos  = waypoints[0].copy()
    wp_idx = 1
    traj   = [pos.copy()]
    errors = []
    clearances = []

    while wp_idx < len(waypoints):
        target = waypoints[wp_idx]
        diff   = target - pos
        dist   = np.linalg.norm(diff)

        if dist < wp_thresh:
            wp_idx += 1
            continue

        vel  = v_cruise * diff / (dist + 1e-9)
        pos  = pos + vel * dt

        # Reference position: linear interpolation on the planned path
        traj.append(pos.copy())
        errors.append(dist)
        clearances.append(clearance_to_obstacles(pos))

    traj_arr = np.array(traj)
    return traj_arr, np.array(errors), np.array(clearances)


# ═══════════════════════════════════════════════════════════════════════════════
# Main simulation
# ═══════════════════════════════════════════════════════════════════════════════

def run_simulation():
    print("Planning RRT* path …")
    t0 = time.perf_counter()
    rrt_path_raw, nodes, best_lengths = plan_rrt_star(START, GOAL)
    plan_time = time.perf_counter() - t0
    print(f"  Planning time : {plan_time:.2f} s  ({len(nodes)} nodes)")

    rrt_path = shortcut_path(rrt_path_raw)
    print(f"  Waypoints (after shortcut): {len(rrt_path)}")

    # Path length
    path_len = sum(np.linalg.norm(rrt_path[k + 1] - rrt_path[k])
                   for k in range(len(rrt_path) - 1))
    straight = np.linalg.norm(GOAL - START)
    detour   = path_len / straight

    print(f"  Path length   : {path_len:.2f} m")
    print(f"  Straight dist : {straight:.2f} m")
    print(f"  Detour ratio  : {detour:.3f}")

    print("Tracking waypoints …")
    traj, errors, clearances = track_waypoints(rrt_path)
    tracking_rmse = np.sqrt(np.mean(errors ** 2)) if len(errors) else 0.0
    min_clearance = clearances.min() if len(clearances) else 0.0

    print(f"  Tracking RMSE : {tracking_rmse:.4f} m")
    print(f"  Min clearance : {min_clearance:.3f} m  (drone radius={DRONE_RADIUS} m)")

    return {
        "rrt_path"    : rrt_path,
        "rrt_path_raw": rrt_path_raw,
        "nodes"       : nodes,
        "best_lengths": best_lengths,
        "traj"        : traj,
        "errors"      : errors,
        "clearances"  : clearances,
        "path_len"    : path_len,
        "straight"    : straight,
        "detour"      : detour,
        "plan_time"   : plan_time,
        "tracking_rmse": tracking_rmse,
        "min_clearance": min_clearance,
    }


# ═══════════════════════════════════════════════════════════════════════════════
# Cylinder drawing helper
# ═══════════════════════════════════════════════════════════════════════════════

def _draw_cylinder_3d(ax, cx, cy, r, h, color="grey", alpha=0.25, n=30):
    """Draw a filled cylinder on a 3-D axes."""
    theta = np.linspace(0, 2 * np.pi, n)
    z_lo, z_hi = 0.0, h
    # Side surface
    xs = cx + r * np.cos(theta)
    ys = cy + r * np.sin(theta)
    Z_side = np.array([[z_lo, z_hi]] * n).T
    X_side = np.tile(xs, (2, 1))
    Y_side = np.tile(ys, (2, 1))
    ax.plot_surface(X_side, Y_side, Z_side, color=color, alpha=alpha,
                    linewidth=0, antialiased=False)
    # Top cap
    r_range = np.linspace(0, r, 5)
    T, R = np.meshgrid(theta, r_range)
    ax.plot_surface(cx + R * np.cos(T), cy + R * np.sin(T),
                    np.full_like(T, h), color=color, alpha=alpha + 0.1,
                    linewidth=0, antialiased=False)


# ═══════════════════════════════════════════════════════════════════════════════
# Plots
# ═══════════════════════════════════════════════════════════════════════════════

def plot_path_3d(data, out_dir):
    fig = plt.figure(figsize=(12, 8))
    ax  = fig.add_subplot(111, projection="3d")

    # Obstacles
    for (cx, cy, r, h) in OBSTACLES:
        _draw_cylinder_3d(ax, cx, cy, r, h)

    # Straight-line (collision)
    sl = np.array([START, GOAL])
    ax.plot(sl[:, 0], sl[:, 1], sl[:, 2], "r--", lw=1.5,
            label="Straight line (collision)")

    # RRT* raw path
    rp = np.array(data["rrt_path_raw"])
    ax.plot(rp[:, 0], rp[:, 1], rp[:, 2], "g-", lw=1.2, alpha=0.5,
            label="RRT* raw path")

    # Shortcut path
    sp = np.array(data["rrt_path"])
    ax.plot(sp[:, 0], sp[:, 1], sp[:, 2], "lime",
            lw=2.5, label="RRT* shortcut")

    # Tracked trajectory
    tr = data["traj"]
    ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], "b-", lw=1.5,
            label="Tracked trajectory")

    # Start / goal markers
    ax.scatter(*START, color="cyan",    s=80, zorder=5, label="Start")
    ax.scatter(*GOAL,  color="magenta", s=80, zorder=5, label="Goal")

    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("S022 — 3D RRT* Obstacle Avoidance Delivery")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_xlim(BOUNDS[0]); ax.set_ylim(BOUNDS[1]); ax.set_zlim([0, 6])

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "path_3d.png")
    plt.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def plot_rrt_tree(data, out_dir):
    fig, ax = plt.subplots(figsize=(10, 8))

    # Obstacle circles (top-down)
    for (cx, cy, r, h) in OBSTACLES:
        circ = plt.Circle((cx, cy), r, color="grey", alpha=0.4)
        ax.add_patch(circ)
        ax.text(cx, cy, f"h={h}m", ha="center", va="center", fontsize=7)

    # Tree edges
    for nd in data["nodes"]:
        if nd.parent is not None:
            ax.plot([nd.parent.pos[0], nd.pos[0]],
                    [nd.parent.pos[1], nd.pos[1]],
                    color="lightgrey", lw=0.5, zorder=1)

    # Straight line
    ax.plot([START[0], GOAL[0]], [START[1], GOAL[1]],
            "r--", lw=1.5, label="Straight line", zorder=3)

    # RRT* shortcut path
    sp = np.array(data["rrt_path"])
    ax.plot(sp[:, 0], sp[:, 1], "g-", lw=2.5,
            label="RRT* path", zorder=4)

    ax.scatter(*START[:2], color="cyan",    s=80, zorder=5, label="Start")
    ax.scatter(*GOAL[:2],  color="magenta", s=80, zorder=5, label="Goal")

    ax.set_xlim(BOUNDS[0]); ax.set_ylim(BOUNDS[1])
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"RRT* Exploration Tree ({len(data['nodes'])} nodes)")
    ax.legend(); ax.set_aspect("equal"); ax.grid(True, alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "rrt_tree.png")
    plt.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def plot_convergence(data, out_dir):
    bl = np.array(data["best_lengths"])
    # Replace nan with previous best for plotting
    for i in range(1, len(bl)):
        if np.isnan(bl[i]):
            bl[i] = bl[i - 1]

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(np.arange(1, len(bl) + 1), bl, color="steelblue", lw=1.5)
    ax.axhline(data["path_len"], color="green", ls="--", lw=1,
               label=f"Final path: {data['path_len']:.2f} m")
    ax.axhline(data["straight"], color="red",   ls="--", lw=1,
               label=f"Straight dist: {data['straight']:.2f} m")
    ax.set_xlabel("RRT* Iteration"); ax.set_ylabel("Best path length (m)")
    ax.set_title("RRT* Path-Length Convergence")
    ax.legend(); ax.grid(True, alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "convergence.png")
    plt.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def plot_clearance(data, out_dir):
    cl = data["clearances"]
    t  = np.arange(len(cl)) * DT

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, cl, color="darkorange", lw=1.5, label="Min clearance")
    ax.axhline(DRONE_RADIUS, color="red", ls="--", lw=1,
               label=f"Drone radius = {DRONE_RADIUS} m")
    ax.fill_between(t, 0, cl, alpha=0.15, color="darkorange")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Clearance (m)")
    ax.set_title("Minimum Clearance to Nearest Obstacle")
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_ylim(bottom=0)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "clearance.png")
    plt.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def plot_tracking_error(data, out_dir):
    err = data["errors"]
    t   = np.arange(len(err)) * DT

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, err, color="purple", lw=1.2, label="Distance to target wp")
    ax.axhline(WP_THRESHOLD, color="red", ls="--", lw=1,
               label=f"WP threshold = {WP_THRESHOLD} m")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Distance (m)")
    ax.set_title("Waypoint-Tracking Error vs. Time")
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_ylim(bottom=0)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "tracking_error.png")
    plt.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def save_animation(data, out_dir):
    import matplotlib.animation as animation

    traj = data["traj"]
    rrt  = np.array(data["rrt_path"])
    step = max(1, len(traj) // 200)   # decimate for smooth GIF

    fig = plt.figure(figsize=(10, 7))
    ax  = fig.add_subplot(111, projection="3d")

    for (cx, cy, r, h) in OBSTACLES:
        _draw_cylinder_3d(ax, cx, cy, r, h)

    sl = np.array([START, GOAL])
    ax.plot(sl[:, 0], sl[:, 1], sl[:, 2], "r--", lw=1.2, alpha=0.6)
    ax.plot(rrt[:, 0], rrt[:, 1], rrt[:, 2], "lime", lw=2, label="RRT* path")
    ax.scatter(*START, color="cyan",    s=80)
    ax.scatter(*GOAL,  color="magenta", s=80)

    traj_line, = ax.plot([], [], [], "b-",  lw=1.5)
    drone_dot, = ax.plot([], [], [], "bo",  ms=8)

    ax.set_xlim(BOUNDS[0]); ax.set_ylim(BOUNDS[1]); ax.set_zlim([0, 6])
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("S022 — 3D Obstacle Avoidance Delivery")

    frames = range(0, len(traj), step)

    def update(i):
        sub = traj[:i + 1]
        traj_line.set_data(sub[:, 0], sub[:, 1])
        traj_line.set_3d_properties(sub[:, 2])
        drone_dot.set_data([sub[-1, 0]], [sub[-1, 1]])
        drone_dot.set_3d_properties([sub[-1, 2]])
        return traj_line, drone_dot

    ani = animation.FuncAnimation(fig, update, frames=frames,
                                  blit=False, interval=50)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "animation.gif")
    ani.save(path, writer="pillow", fps=20, dpi=100)
    plt.close()
    print(f"Saved: {path}")


# ═══════════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    data = run_simulation()

    print("\n── Summary ──────────────────────────────────────────────────")
    print(f"  Path length     : {data['path_len']:.2f} m")
    print(f"  Straight dist   : {data['straight']:.2f} m")
    print(f"  Detour ratio    : {data['detour']:.3f}")
    print(f"  Min clearance   : {data['min_clearance']:.3f} m")
    print(f"  Tracking RMSE   : {data['tracking_rmse']:.4f} m")
    print(f"  Planning time   : {data['plan_time']:.2f} s")
    print(f"  RRT* nodes      : {len(data['nodes'])}")
    print(f"  Waypoints (sc.) : {len(data['rrt_path'])}")
    print("─────────────────────────────────────────────────────────────\n")

    out = OUTPUT_DIR
    plot_path_3d(data, out)
    plot_rrt_tree(data, out)
    plot_convergence(data, out)
    plot_clearance(data, out)
    plot_tracking_error(data, out)
    save_animation(data, out)

    print("\nAll outputs saved to:", out)
