"""
S022 3D Obstacle Avoidance Delivery
=====================================
A delivery drone flies from a ground-level depot (0, 0, 2) m to an elevated
drop zone (10, 10, 8) m (card values) through finite-height cylindrical
buildings.  A low, wide warehouse sits on the direct line and is cheaper to
fly OVER than around; a tower taller than the flight ceiling also blocks the
line and can only be flown AROUND.  The planner works in full (x, y, z).

  1. Straight line            -- naive direct flight, collides
  2. RRT* (5000 it., 0.5 m step) + greedy shortcut -- collision-free

The shortcut path is then flown by a PD acceleration controller that chases a
constant-speed reference moving along the path.

Outputs (saved to outputs/02_logistics_delivery/3d/s022_3d_obstacle_avoidance/):
  - path_3d.png            -- 3D scene: obstacles, both paths, drone trajectory
  - rrt_tree.png           -- top-down view of the RRT* tree + altitude side view
  - altitude_profile.png   -- flight altitude vs. along-track distance, roofs below
  - convergence.png        -- best goal cost vs. RRT* iteration (all iterations)
  - clearance.png          -- min 3D clearance to any obstacle vs. time
  - tracking_error.png     -- cross-track error and reference error vs. time
  - animation.gif          -- animated 3D flight

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s022_3d_obstacle_avoidance.py
"""

import matplotlib
matplotlib.use("Agg")

import math
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# -- Output directory ----------------------------------------------------------
OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..",
    "outputs", "02_logistics_delivery", "3d", "s022_3d_obstacle_avoidance",
))

RANDOM_SEED = 0

# -- Scenario parameters (card values unless noted) ----------------------------
START      = np.array([0.0,  0.0, 2.0])   # m  (card)
GOAL       = np.array([10.0, 10.0, 8.0])  # m  (card)
BOUNDS     = np.array([[-1.0, 12.0],      # x  sampling volume
                       [-1.0, 12.0],      # y
                       [ 0.5, 11.0]])     # z  (flight ceiling 11 m)

MAX_ITER      = 5000    # RRT* iterations (card)
STEP_SIZE     = 0.5     # m   -- eta (card)
GOAL_TOL      = 0.5     # m   -- goal tolerance (card): goal-connection radius
GAMMA_RRT     = 12.0    # gamma_RRT* coefficient
R_NEAR_MAX    = 1.5     # m   -- cap on the rewiring radius (= 3 eta, see README)
GOAL_BIAS     = 0.10    # probability of sampling the goal directly
DRONE_RADIUS  = 0.15    # m   -- collision safety radius (horizontal AND vertical)

CRUISE_SPEED  = 2.0     # m/s -- speed of the moving reference
KP            = 16.0    # 1/s^2  PD position gain
KD            = 8.0     # 1/s    PD velocity gain (critically damped: 2 sqrt(KP))
A_MAX         = 8.0     # m/s^2  acceleration limit
DT            = 1.0 / 48  # s   -- control frequency 48 Hz
T_SETTLE      = 3.0     # s   -- extra time after the reference reaches the goal

# -- Obstacle layout (x_c, y_c, radius, height) --------------------------------
OBSTACLES = np.array([
    [3.5, 3.5, 2.5,  5.0],   # wide low warehouse on the direct line -> overfly
    [7.0, 7.0, 1.0, 12.0],   # tower taller than the 11 m ceiling   -> go around
    [2.0, 8.0, 1.0,  9.0],   # flanking towers
    [8.0, 2.0, 1.0,  9.0],
    [9.5, 5.5, 1.0,  4.0],   # low block off the direct line
])
OBSTACLE_NAMES = ["warehouse", "tower", "NW tower", "SE tower", "low block"]


# ==============================================================================
# Collision checking (exact for a vertical cylinder inflated by DRONE_RADIUS)
# ==============================================================================

def _seg2d_min_dist(ax, ay, bx, by, cx, cy):
    """Minimum 2-D distance from point (cx,cy) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    len2 = dx * dx + dy * dy
    if len2 < 1e-12:
        return math.hypot(cx - ax, cy - ay)
    t = max(0.0, min(1.0, ((cx - ax) * dx + (cy - ay) * dy) / len2))
    return math.hypot(cx - (ax + t * dx), cy - (ay + t * dy))


def segment_free(qa, qb, obstacles=OBSTACLES, r_drone=DRONE_RADIUS):
    """True iff segment qa->qb keeps DRONE_RADIUS from every inflated cylinder.

    The segment is clipped to the part below the inflated roof (z <= h + r),
    and only that part is tested horizontally, so a segment that climbs over a
    roof is accepted once it is high enough.
    """
    for (cx, cy, r, h) in obstacles:
        h_inf = h + r_drone
        za, zb = qa[2], qb[2]
        if za > h_inf and zb > h_inf:
            continue
        # parameter range [t0, t1] where z(t) <= h_inf
        if abs(zb - za) < 1e-12:
            t0, t1 = 0.0, 1.0
        else:
            tc = (h_inf - za) / (zb - za)
            if za <= h_inf and zb <= h_inf:
                t0, t1 = 0.0, 1.0
            elif za <= h_inf:
                t0, t1 = 0.0, tc
            else:
                t0, t1 = tc, 1.0
        pa = qa + t0 * (qb - qa)
        pb = qa + t1 * (qb - qa)
        if _seg2d_min_dist(pa[0], pa[1], pb[0], pb[1], cx, cy) <= r + r_drone:
            return False
    return True


def point_free(q, obstacles=OBSTACLES, r_drone=DRONE_RADIUS):
    """True iff point q is outside every inflated cylinder."""
    for (cx, cy, r, h) in obstacles:
        if q[2] > h + r_drone:
            continue
        if math.hypot(q[0] - cx, q[1] - cy) <= r + r_drone:
            return False
    return True


def clearance_to_obstacles(q, obstacles=OBSTACLES):
    """Minimum 3-D distance (m) from q to the nearest cylinder surface."""
    min_cl = np.inf
    for (cx, cy, r, h) in obstacles:
        horiz = max(0.0, math.hypot(q[0] - cx, q[1] - cy) - r)
        vert = max(0.0, q[2] - h)
        min_cl = min(min_cl, math.hypot(horiz, vert))
    return min_cl


# ==============================================================================
# RRT* planner (with subtree cost propagation)
# ==============================================================================

class Node:
    __slots__ = ("pos", "parent", "cost", "children")

    def __init__(self, pos):
        self.pos = np.asarray(pos, dtype=float)
        self.parent = None
        self.cost = 0.0
        self.children = []


def _set_parent(node, parent, cost):
    if node.parent is not None:
        node.parent.children.remove(node)
    node.parent = parent
    node.cost = cost
    parent.children.append(node)


def _propagate_cost(node):
    """After re-parenting, push the cost change down the whole subtree."""
    stack = [node]
    while stack:
        nd = stack.pop()
        for ch in nd.children:
            ch.cost = nd.cost + float(np.linalg.norm(ch.pos - nd.pos))
            stack.append(ch)


def near_radius(n, gamma=GAMMA_RRT):
    n = max(n, 2)
    return min(gamma * (math.log(n) / n) ** (1.0 / 3.0), R_NEAR_MAX)


def plan_rrt_star(start, goal, rng, bounds=BOUNDS, max_iter=MAX_ITER,
                  step=STEP_SIZE, goal_bias=GOAL_BIAS):
    """Run RRT*; return (raw path, nodes, best cost after every iteration, info)."""
    root = Node(start)
    nodes = [root]
    pos = np.zeros((max_iter + 1, 3))
    pos[0] = root.pos
    goal_parents = []
    best_lengths = []          # one entry per iteration (inf until first solution)
    first_iter = None
    n_accepted = 0

    def best_goal():
        best_c, best_nd = np.inf, None
        for nd in goal_parents:
            c = nd.cost + float(np.linalg.norm(nd.pos - goal))
            if c < best_c:
                best_c, best_nd = c, nd
        return best_c, best_nd

    for it in range(max_iter):
        if rng.random() < goal_bias:
            q_rand = goal.copy()
        else:
            q_rand = rng.uniform(bounds[:, 0], bounds[:, 1])

        n = len(nodes)
        d_all = np.linalg.norm(pos[:n] - q_rand, axis=1)
        q_near = nodes[int(np.argmin(d_all))]
        diff = q_rand - q_near.pos
        dist = float(np.linalg.norm(diff))
        if dist > 1e-9:
            q_new_pos = q_near.pos + diff / dist * min(dist, step)
            in_bounds = np.all(q_new_pos >= bounds[:, 0]) and np.all(q_new_pos <= bounds[:, 1])
            if (in_bounds and point_free(q_new_pos)
                    and segment_free(q_near.pos, q_new_pos)):
                rn = near_radius(n)
                idx = np.nonzero(np.linalg.norm(pos[:n] - q_new_pos, axis=1) <= rn)[0]
                near_nodes = [nodes[i] for i in idx]

                best_parent = q_near
                best_c = q_near.cost + float(np.linalg.norm(q_near.pos - q_new_pos))
                for nd in near_nodes:
                    c = nd.cost + float(np.linalg.norm(nd.pos - q_new_pos))
                    if c < best_c - 1e-12 and segment_free(nd.pos, q_new_pos):
                        best_parent, best_c = nd, c
                q_new = Node(q_new_pos)
                _set_parent(q_new, best_parent, best_c)
                pos[n] = q_new_pos
                nodes.append(q_new)
                n_accepted += 1

                for nd in near_nodes:
                    if nd is best_parent:
                        continue
                    new_c = q_new.cost + float(np.linalg.norm(q_new.pos - nd.pos))
                    if new_c < nd.cost - 1e-12 and segment_free(q_new.pos, nd.pos):
                        _set_parent(nd, q_new, new_c)
                        _propagate_cost(nd)

                if (np.linalg.norm(q_new_pos - goal) <= GOAL_TOL
                        and segment_free(q_new_pos, goal)):
                    goal_parents.append(q_new)

        best_c, _ = best_goal()
        if first_iter is None and np.isfinite(best_c):
            first_iter = it + 1
        best_lengths.append(best_c)

    best_c, goal_node = best_goal()
    if goal_node is None:
        raise RuntimeError("RRT* failed to find a path -- increase MAX_ITER")

    path = [goal.copy()]
    nd = goal_node
    while nd is not None:
        path.append(nd.pos.copy())
        nd = nd.parent
    path.reverse()
    info = {"first_iter": first_iter, "n_accepted": n_accepted,
            "final_rn": near_radius(len(nodes))}
    return path, nodes, best_lengths, info


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


def path_length(path):
    return float(sum(np.linalg.norm(np.asarray(path[k + 1]) - np.asarray(path[k]))
                     for k in range(len(path) - 1)))


def dist_to_polyline(p, poly):
    best = np.inf
    for a, b in zip(poly[:-1], poly[1:]):
        ab = b - a
        t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(p - (a + t * ab))))
    return best


# ==============================================================================
# PD tracking of a constant-speed reference along the path
# ==============================================================================

def _reference(poly, s):
    """Point and unit tangent at arc length s along polyline poly."""
    seg = np.diff(poly, axis=0)
    lens = np.linalg.norm(seg, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(lens)])
    s = min(max(s, 0.0), cum[-1])
    k = min(int(np.searchsorted(cum, s, side="right") - 1), len(lens) - 1)
    u = seg[k] / lens[k]
    return poly[k] + u * (s - cum[k]), u


def track_path(waypoints, dt=DT, v_ref=CRUISE_SPEED):
    """Fly the PD-controlled point mass along the path.

    Returns trajectory, reference trajectory, cross-track error (distance to the
    planned polyline), reference error |p - p_ref(t)|, and clearance per step.
    """
    poly = np.asarray(waypoints, dtype=float)
    L = path_length(poly)
    t_end = L / v_ref + T_SETTLE
    p = poly[0].copy()
    v = np.zeros(3)
    traj, refs, xtrack, ref_err, clear = [p.copy()], [p.copy()], [], [], []
    n_steps = int(round(t_end / dt))
    for k in range(1, n_steps + 1):
        t = k * dt
        s = v_ref * t
        p_ref, u = _reference(poly, s)
        v_ff = v_ref * u if s < L else np.zeros(3)
        a = KP * (p_ref - p) + KD * (v_ff - v)
        a_n = np.linalg.norm(a)
        if a_n > A_MAX:
            a = a * A_MAX / a_n
        v = v + a * dt
        p = p + v * dt
        traj.append(p.copy())
        refs.append(p_ref.copy())
        xtrack.append(dist_to_polyline(p, poly))
        ref_err.append(float(np.linalg.norm(p - p_ref)))
        clear.append(clearance_to_obstacles(p))
    return (np.array(traj), np.array(refs), np.array(xtrack),
            np.array(ref_err), np.array(clear), L / v_ref)


# ==============================================================================
# Main simulation
# ==============================================================================

def overflight_report(traj):
    """For each obstacle, min vertical clearance while horizontally over it."""
    out = []
    for name, (cx, cy, r, h) in zip(OBSTACLE_NAMES, OBSTACLES):
        horiz = np.hypot(traj[:, 0] - cx, traj[:, 1] - cy)
        over = horiz < r
        if over.any():
            out.append((name, h, int(over.sum()), float((traj[over, 2] - h).min())))
    return out


def run_simulation():
    rng = np.random.default_rng(RANDOM_SEED)
    straight = float(np.linalg.norm(GOAL - START))
    straight_free = segment_free(START, GOAL)
    print("S022 3D Obstacle Avoidance Delivery")
    print(f"  Start {START.tolist()}  Goal {GOAL.tolist()}  straight-line {straight:.2f} m")
    print(f"  Straight line collision-free: {straight_free}")
    blocked = [nm for nm, ob in zip(OBSTACLE_NAMES, OBSTACLES)
               if not segment_free(START, GOAL, obstacles=ob[None, :])]
    print(f"  Straight line blocked by: {', '.join(blocked)}")

    print("Planning RRT* path ...")
    rrt_path_raw, nodes, best_lengths, info = plan_rrt_star(START, GOAL, rng)
    raw_len = path_length(rrt_path_raw)

    # bookkeeping check: stored cost == true cost-to-root for every node
    def true_cost(nd):
        c = 0.0
        while nd.parent is not None:
            c += float(np.linalg.norm(nd.pos - nd.parent.pos))
            nd = nd.parent
        return c
    stale = max(abs(nd.cost - true_cost(nd)) for nd in nodes)
    assert stale < 1e-9, f"stale RRT* cost {stale}"
    assert abs(best_lengths[-1] - raw_len) < 1e-9

    print(f"  Iterations {MAX_ITER}, accepted nodes {info['n_accepted']}, "
          f"final near radius {info['final_rn']:.3f} m")
    print(f"  First solution at iteration {info['first_iter']}, "
          f"cost {best_lengths[info['first_iter'] - 1]:.2f} m")
    print(f"  Final best cost {best_lengths[-1]:.2f} m = raw path length {raw_len:.2f} m "
          f"(max stale node cost {stale:.1e} m)")

    rrt_path = [np.asarray(p) for p in shortcut_path(rrt_path_raw)]
    path_len = path_length(rrt_path)
    detour = path_len / straight
    all_free = all(segment_free(rrt_path[k], rrt_path[k + 1])
                   for k in range(len(rrt_path) - 1))
    sp = np.array(rrt_path)
    print(f"  Shortcut path: {len(rrt_path)} waypoints, {path_len:.2f} m, "
          f"detour ratio {detour:.3f}, collision-free {all_free}")
    print("  Waypoints (x, y, z):")
    for w in sp:
        print(f"    ({w[0]:6.2f}, {w[1]:6.2f}, {w[2]:5.2f})")

    print("Tracking path with PD controller ...")
    traj, refs, xtrack, ref_err, clear, t_ref = track_path(rrt_path)
    xt_rms = float(np.sqrt(np.mean(xtrack ** 2)))
    xt_max = float(xtrack.max())
    ref_rms = float(np.sqrt(np.mean(ref_err ** 2)))
    min_cl = float(clear.min())
    final_err = float(np.linalg.norm(traj[-1] - GOAL))
    over = overflight_report(traj)
    print(f"  Reference duration {t_ref:.2f} s (+{T_SETTLE:.1f} s settle)")
    print(f"  Altitude range {traj[:, 2].min():.2f}-{traj[:, 2].max():.2f} m")
    print(f"  Cross-track error (distance to path): RMS {xt_rms:.4f} m, max {xt_max:.4f} m")
    print(f"  Reference error |p - p_ref(t)|     : RMS {ref_rms:.4f} m, max {ref_err.max():.4f} m")
    print(f"  Min 3D clearance {min_cl:.3f} m (drone radius {DRONE_RADIUS} m)")
    print(f"  Final distance to goal {final_err:.4f} m")
    for name, h, nsamp, dz in over:
        print(f"  Overflies {name} (h = {h:.1f} m): {nsamp} samples above roof, "
              f"min height above roof {dz:.3f} m")
    if not over:
        print("  No obstacle overflown")

    return {
        "rrt_path": rrt_path, "rrt_path_raw": rrt_path_raw, "nodes": nodes,
        "best_lengths": best_lengths, "info": info, "traj": traj, "refs": refs,
        "errors": xtrack, "ref_err": ref_err, "clearances": clear,
        "path_len": path_len, "raw_len": raw_len, "straight": straight,
        "detour": detour, "tracking_rmse": xt_rms, "min_clearance": min_cl,
        "overflights": over, "straight_free": straight_free,
    }


# ==============================================================================
# Drawing helpers
# ==============================================================================

def _draw_cylinder_3d(ax, cx, cy, r, h, color="grey", alpha=0.25, n=30):
    theta = np.linspace(0, 2 * np.pi, n)
    xs = cx + r * np.cos(theta)
    ys = cy + r * np.sin(theta)
    ax.plot_surface(np.tile(xs, (2, 1)), np.tile(ys, (2, 1)),
                    np.array([[0.0] * n, [h] * n]), color=color, alpha=alpha,
                    linewidth=0, antialiased=False)
    T, R = np.meshgrid(theta, np.linspace(0, r, 5))
    ax.plot_surface(cx + R * np.cos(T), cy + R * np.sin(T), np.full_like(T, h),
                    color=color, alpha=alpha + 0.1, linewidth=0, antialiased=False)


def _setup_3d(ax):
    ax.set_xlim(BOUNDS[0]); ax.set_ylim(BOUNDS[1]); ax.set_zlim([0, 12])
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.view_init(elev=24, azim=-60)


# ==============================================================================
# Plots
# ==============================================================================

def plot_path_3d(data, out_dir):
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")
    for (cx, cy, r, h) in OBSTACLES:
        _draw_cylinder_3d(ax, cx, cy, r, h)
    sl = np.array([START, GOAL])
    ax.plot(sl[:, 0], sl[:, 1], sl[:, 2], "r--", lw=1.5, label="Straight line (collides)")
    rp = np.array(data["rrt_path_raw"])
    ax.plot(rp[:, 0], rp[:, 1], rp[:, 2], color="olive", lw=1.0, alpha=0.6,
            label=f"RRT* raw path ({data['raw_len']:.2f} m)")
    sp = np.array(data["rrt_path"])
    ax.plot(sp[:, 0], sp[:, 1], sp[:, 2], "g-", lw=2.5,
            label=f"RRT* shortcut ({data['path_len']:.2f} m)")
    tr = data["traj"]
    ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], "b-", lw=1.2, label="Flown trajectory")
    ax.scatter(*START, color="black", s=60, label="Start")
    ax.scatter(*GOAL, color="green", marker="*", s=200, label="Goal")
    _setup_3d(ax)
    ax.set_title("S022 3D -- RRT* over the warehouse, around the tower")
    ax.legend(loc="upper left", fontsize=8)
    path = os.path.join(out_dir, "path_3d.png")
    plt.savefig(path, dpi=130, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def plot_rrt_tree(data, out_dir):
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 6.5))
    for (cx, cy, r, h), nm in zip(OBSTACLES, OBSTACLE_NAMES):
        ax.add_patch(plt.Circle((cx, cy), r, color="grey", alpha=0.4))
        ax.text(cx, cy, f"{nm}\nh={h:g} m", ha="center", va="center", fontsize=7)
    for nd in data["nodes"]:
        if nd.parent is not None:
            ax.plot([nd.parent.pos[0], nd.pos[0]], [nd.parent.pos[1], nd.pos[1]],
                    color="lightgrey", lw=0.4, zorder=1)
    ax.plot([START[0], GOAL[0]], [START[1], GOAL[1]], "r--", lw=1.5,
            label="Straight line", zorder=3)
    sp = np.array(data["rrt_path"])
    ax.plot(sp[:, 0], sp[:, 1], "g-", lw=2.5, label="RRT* shortcut", zorder=4)
    ax.scatter(*START[:2], color="black", s=60, zorder=5, label="Start")
    ax.scatter(*GOAL[:2], color="green", marker="*", s=200, zorder=5, label="Goal")
    ax.set_xlim(BOUNDS[0]); ax.set_ylim(BOUNDS[1])
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"Top view: RRT* tree ({len(data['nodes'])} nodes)")
    ax.legend(loc="lower right", fontsize=8); ax.set_aspect("equal"); ax.grid(True, alpha=0.3)

    # side view: node altitude vs. distance along the start->goal ground line
    gdir = (GOAL[:2] - START[:2]) / np.linalg.norm(GOAL[:2] - START[:2])
    allp = np.array([nd.pos for nd in data["nodes"]])
    ax2.scatter((allp[:, :2] - START[:2]) @ gdir, allp[:, 2], s=2, color="lightgrey",
                label="Tree nodes")
    labels = {}
    for (cx, cy, r, h), nm in zip(OBSTACLES, OBSTACLE_NAMES):
        c_al = (np.array([cx, cy]) - START[:2]) @ gdir
        ax2.add_patch(plt.Rectangle((c_al - r, 0), 2 * r, h, color="grey", alpha=0.35))
        key = (round(float(c_al), 1), h)
        labels[key] = labels.get(key, []) + [nm]
    for (c_al, h), nms in labels.items():
        ax2.text(c_al, min(h, 11.5) - 0.4, " + ".join(nms), ha="center", fontsize=7)
    ax2.plot((sp[:, :2] - START[:2]) @ gdir, sp[:, 2], "g-o", lw=2, ms=4,
             label="RRT* shortcut")
    ax2.plot([0, np.linalg.norm(GOAL[:2] - START[:2])], [START[2], GOAL[2]], "r--",
             lw=1.5, label="Straight line")
    ax2.axhline(BOUNDS[2, 1], color="k", ls=":", lw=1, label="Flight ceiling")
    ax2.set_xlabel("Distance along start-goal ground line (m)")
    ax2.set_ylabel("Z (m)"); ax2.set_ylim(0, 12.5)
    ax2.set_title("Side view (obstacles projected onto the start-goal line)")
    ax2.legend(loc="upper left", fontsize=8); ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, "rrt_tree.png")
    plt.savefig(path, dpi=130, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def plot_altitude_profile(data, out_dir):
    tr = data["traj"]
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(tr[:, :2], axis=0), axis=1))])
    roof = np.zeros(len(tr))
    for (cx, cy, r, h) in OBSTACLES:
        under = np.hypot(tr[:, 0] - cx, tr[:, 1] - cy) < r
        roof = np.where(under, np.maximum(roof, h), roof)
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.fill_between(s, 0, roof, color="grey", alpha=0.45, step="mid",
                    label="Roof directly below the drone")
    ax.plot(s, tr[:, 2], "b-", lw=1.8, label="Drone altitude")
    ax.plot(s, np.where(roof > 0, roof + DRONE_RADIUS, np.nan), color="red", ls=":", lw=1,
            label=f"Roof + drone radius ({DRONE_RADIUS} m)")
    ax.axhline(GOAL[2], color="green", ls="--", lw=1, label=f"Goal altitude {GOAL[2]:g} m")
    ax.set_xlabel("Horizontal distance flown (m)"); ax.set_ylabel("Z (m)")
    ax.set_title("Altitude profile: the drone climbs over the 5 m warehouse")
    ax.legend(loc="lower right", fontsize=8); ax.grid(True, alpha=0.3); ax.set_ylim(0, 10)
    path = os.path.join(out_dir, "altitude_profile.png")
    plt.savefig(path, dpi=130, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def plot_convergence(data, out_dir):
    bl = np.array(data["best_lengths"], dtype=float)
    it = np.arange(1, len(bl) + 1)
    fin = np.isfinite(bl)
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(it[fin], bl[fin], color="steelblue", lw=1.5, label="Best goal cost (raw tree)")
    ax.axvline(data["info"]["first_iter"], color="grey", ls=":", lw=1.2,
               label=f"First solution (iteration {data['info']['first_iter']})")
    ax.axhline(data["path_len"], color="green", ls="--", lw=1,
               label=f"After shortcut: {data['path_len']:.2f} m")
    ax.axhline(data["straight"], color="red", ls="--", lw=1,
               label=f"Straight line (infeasible): {data['straight']:.2f} m")
    ax.set_xlim(0, MAX_ITER)
    ax.set_xlabel("RRT* iteration (all samples, accepted or not)")
    ax.set_ylabel("Path length (m)")
    ax.set_title("RRT* Path-Length Convergence")
    ax.legend(loc="upper right"); ax.grid(True, alpha=0.3)
    path = os.path.join(out_dir, "convergence.png")
    plt.savefig(path, dpi=130, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def plot_clearance(data, out_dir):
    cl = data["clearances"]
    t = np.arange(1, len(cl) + 1) * DT
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, cl, color="darkorange", lw=1.5, label="Min 3D clearance to any obstacle")
    ax.axhline(DRONE_RADIUS, color="red", ls="--", lw=1,
               label=f"Drone radius = {DRONE_RADIUS} m")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Clearance (m)")
    ax.set_title("Minimum Clearance to Nearest Obstacle Surface")
    ax.legend(loc="lower right"); ax.grid(True, alpha=0.3); ax.set_ylim(bottom=0)
    path = os.path.join(out_dir, "clearance.png")
    plt.savefig(path, dpi=130, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def plot_tracking_error(data, out_dir):
    xt = data["errors"]
    re = data["ref_err"]
    t = np.arange(1, len(xt) + 1) * DT
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, xt, color="purple", lw=1.4, label="Cross-track: distance to planned path")
    ax.plot(t, re, color="teal", lw=1.0, alpha=0.8, label="|p - p_ref(t)| (incl. along-track lag)")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Error (m)")
    ax.set_title("PD Path-Tracking Error vs. Time")
    ax.legend(loc="upper center"); ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 1.35 * max(xt.max(), re.max()))
    path = os.path.join(out_dir, "tracking_error.png")
    plt.savefig(path, dpi=130, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def save_animation(data, out_dir):
    import matplotlib.animation as animation
    traj = data["traj"]
    rrt = np.array(data["rrt_path"])
    step = max(1, len(traj) // 150)
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    for (cx, cy, r, h) in OBSTACLES:
        _draw_cylinder_3d(ax, cx, cy, r, h)
    sl = np.array([START, GOAL])
    ax.plot(sl[:, 0], sl[:, 1], sl[:, 2], "r--", lw=1.2, alpha=0.6)
    ax.plot(rrt[:, 0], rrt[:, 1], rrt[:, 2], "g-", lw=2)
    ax.scatter(*START, color="black", s=50)
    ax.scatter(*GOAL, color="green", marker="*", s=160)
    traj_line, = ax.plot([], [], [], "b-", lw=1.5)
    drone_dot, = ax.plot([], [], [], "bo", ms=7)
    _setup_3d(ax)
    title = ax.set_title("")
    frames = list(range(0, len(traj), step))
    if frames[-1] != len(traj) - 1:
        frames.append(len(traj) - 1)

    def update(i):
        sub = traj[:i + 1]
        traj_line.set_data(sub[:, 0], sub[:, 1])
        traj_line.set_3d_properties(sub[:, 2])
        drone_dot.set_data([sub[-1, 0]], [sub[-1, 1]])
        drone_dot.set_3d_properties([sub[-1, 2]])
        title.set_text(f"S022 3D -- t = {i * DT:5.2f} s, z = {sub[-1, 2]:.2f} m")
        return traj_line, drone_dot

    ani = animation.FuncAnimation(fig, update, frames=frames, blit=False, interval=60)
    path = os.path.join(out_dir, "animation.gif")
    ani.save(path, writer="pillow", fps=15, dpi=80)
    plt.close()
    print(f"Saved: {path}")


if __name__ == "__main__":
    data = run_simulation()
    out = OUTPUT_DIR
    os.makedirs(out, exist_ok=True)
    plot_path_3d(data, out)
    plot_rrt_tree(data, out)
    plot_altitude_profile(data, out)
    plot_convergence(data, out)
    plot_clearance(data, out)
    plot_tracking_error(data, out)
    save_animation(data, out)
    print("\nAll outputs saved to:", out)
