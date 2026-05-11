# S078 Harvester Cooperative Guidance

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Moving-Horizon Scout + A* Replanning with Latency | **Dimension**: 2D

---

## Problem Definition

**Setup**: An autonomous harvester advances along a 200 × 50 m rectangular field at a constant
speed of $v_{harvester} = 1.5$ m/s, following a pre-planned straight-line route along the field's
long axis. A scouting drone flies $L_{lead} = 45$ m ahead of the harvester, continuously scanning a
10 m wide corridor centred on the harvester's planned track. The field contains $N_{obs}$ randomly
placed obstacles (rocks, waterlogged patches) each of radius $r_{obs} = 1.0$ m; obstacles are not
known in advance to either agent. The drone sweeps the corridor with a 3-pass lawnmower
(boustrophedon) sub-pattern of strip width $w_{scan} = 3.3$ m, detecting any obstacle within sensor
range $r_{sensor} = 2.0$ m. Detections are relayed to the harvester's onboard A* path planner with
a communication latency of $T_{latency} = 2$ s. Once the harvester reaches within 5 m of the far
boundary, the drone loops back to the near boundary to re-scout and maintain lead coverage.

**Roles**:
- **Scout drone**: lawnmower-pattern sensor platform flying at $v_{drone} = 3.0$ m/s; no actuator
  authority over the harvester; acts only as a sensor relay.
- **Harvester**: ground vehicle advancing at fixed $v_{harvester} = 1.5$ m/s; replans its 2D path
  via A* on a 0.5 m grid each time a new obstacle message arrives; inflates each detected obstacle
  cell by $r_{harvester} = 1.5$ m (vehicle half-width) to guarantee clearance.

**Objective**: Quantify how lead distance $L_{lead}$, sensor range $r_{sensor}$, and communication
latency $T_{latency}$ interact to determine (1) the fraction of obstacles detected before the
harvester reaches them, (2) the number of replanning events, and (3) the extra path length
(detour cost) accumulated due to late or missed detections.

---

## Mathematical Model

### Drone Lead Position

The scout maintains a nominal target position $T_{lead}$ ahead of the harvester along its current
heading $\hat{v}_{harvester}$:

$$\mathbf{p}_{scout}(t) = \mathbf{p}_{harvester}(t) + L_{lead} \cdot \hat{v}_{harvester}$$

When $\mathbf{p}_{scout}$ reaches within $d_{turnaround} = 5$ m of the far field boundary the
drone reverses to the near boundary. The loop period is:

$$T_{loop} = \frac{2 \cdot (L_{field} - L_{lead})}{v_{drone}} = \frac{2 \times 155}{3.0} \approx 103\,\text{s}$$

### Scout Lawnmower Sub-Pattern

Within the 10 m wide corridor centred on $y_{track}$ (the harvester's lateral position), the drone
executes 3 parallel passes at strip centres:

$$y_{strip,i} = y_{track} + \left(i - 1\right) \cdot w_{scan} - \frac{w_{scan}}{2}, \quad i \in \{0, 1, 2\}$$

where $w_{scan} = 3.3$ m ensures the three strips tile the 10 m corridor with 0.1 m overlap on
each side. Each strip is traversed left-to-right or right-to-left alternately (boustrophedon). The
effective sensor sweep width per strip equals $2 r_{sensor} = 4.0$ m, so the strips provide a
$\frac{4.0}{3.3} \approx 1.2\times$ oversampling ratio for robustness against GPS error.

### Obstacle Detection

An obstacle at position $\mathbf{p}_{obs,j}$ is detected at time $t_{det,j}$ when the drone's
current position $\mathbf{p}_{drone}(t)$ satisfies:

$$\left\| \mathbf{p}_{drone}(t) - \mathbf{p}_{obs,j} \right\| \leq r_{sensor}$$

Detection is a discrete event (first crossing of the threshold). Immediately after detection a
message is queued; it arrives at the harvester's planner at time:

$$t_{relay,j} = t_{det,j} + T_{latency}$$

### Time-to-Collision at Detection

When the relayed obstacle message arrives, the harvester is at position $\mathbf{p}_{harvester}(t_{relay,j})$.
The time remaining before the harvester would reach the obstacle (absent replanning) is:

$$\Delta t_{ttc,j} = \frac{\left\| \mathbf{p}_{obs,j} - \mathbf{p}_{harvester}(t_{relay,j}) \right\|}{v_{harvester}}$$

A detection is classified as **timely** if $\Delta t_{ttc,j} > T_{replan}$ (sufficient replanning
margin), **marginal** if $0 < \Delta t_{ttc,j} \leq T_{replan}$, and **late** if
$\Delta t_{ttc,j} \leq 0$ (the harvester has already reached or passed the obstacle).

### A* Replanning on Occupancy Grid

The field is discretised into a 2D grid of cell size $\Delta_{grid} = 0.5$ m, giving
$400 \times 100$ cells. Each relayed obstacle inflates a circle of radius
$r_{inflate} = r_{obs} + r_{harvester} = 1.0 + 1.5 = 2.5$ m around the detected centre; all cells
within $r_{inflate}$ are marked blocked. A* searches a 4-connected (Manhattan) graph from the
harvester's current cell to the field exit cell:

$$f(n) = g(n) + h(n), \quad h(n) = \left\| \mathbf{p}(n) - \mathbf{p}_{exit} \right\|_1 \cdot \Delta_{grid}$$

The additional path length introduced by replanning around obstacle $j$ is:

$$\delta L_j = L_{replan,j} - L_{straight,j}$$

where $L_{straight,j}$ is the straight-line distance to the exit from the replanning trigger point
and $L_{replan,j}$ is the A* path length after inflating obstacle $j$.

### Extra Path Length Due to Latency

If the same obstacle had been detected $T_{latency}$ seconds earlier (zero-latency scenario), the
harvester would trigger replanning from a position $\Delta x_{latency}$ further back:

$$\Delta x_{latency} = v_{harvester} \cdot T_{latency} = 1.5 \times 2 = 3.0\,\text{m}$$

The latency detour penalty for obstacle $j$ is:

$$\delta L_{latency,j} = L_{replan}(t_{relay,j}) - L_{replan}(t_{relay,j} - T_{latency})$$

The total latency cost over all $N_{det}$ detected obstacles is:

$$\Delta L_{latency} = \sum_{j=1}^{N_{det}} \delta L_{latency,j}$$

### GPS Localisation Error

Both the drone and harvester positions carry independent Gaussian noise at each timestep:

$$\tilde{\mathbf{p}}(t) = \mathbf{p}(t) + \boldsymbol{\epsilon}(t), \quad \boldsymbol{\epsilon}(t) \sim \mathcal{N}(\mathbf{0},\, \sigma_{GPS}^2 \mathbf{I})$$

with $\sigma_{GPS} = \texttt{GPS\_ERROR} = 0.1$ m. The detection threshold is widened
conservatively to $r_{sensor} - \sigma_{GPS}$ to avoid false negatives due to noise.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from collections import defaultdict
import heapq

# --- Constants ---
FIELD_X        = 200.0    # m — field length (harvester travel axis)
FIELD_Y        =  50.0    # m — field width
V_HARVESTER    =   1.5    # m/s — harvester ground speed
V_DRONE        =   3.0    # m/s — scout drone speed
L_LEAD         =  45.0    # m — nominal lead distance ahead of harvester
T_HORIZON      =  30.0    # s — scouting time horizon (L_lead / v_harvester)
CORRIDOR_W     =  10.0    # m — scan corridor width
N_PASSES       =   3      # lawnmower passes within corridor
W_SCAN         =   3.3    # m — strip width (corridor / passes)
R_SENSOR       =   2.0    # m — obstacle detection radius
R_OBS          =   1.0    # m — obstacle physical radius
R_HARVESTER    =   1.5    # m — harvester half-width (inflation)
R_INFLATE      = R_OBS + R_HARVESTER   # 2.5 m
T_LATENCY      =   2.0    # s — relay communication latency
GPS_ERROR      =   0.1    # m — position noise std dev
INSPECTION_SPEED = 1.0    # m/s (domain constant, referenced)
GRID_RES       =   0.5    # m — A* grid cell size
DT             =   0.1    # s — simulation timestep
D_TURNAROUND   =   5.0    # m — loopback trigger from boundary
T_REPLAN_MIN   =   5.0    # s — minimum replanning margin for "timely" detection

NX = int(FIELD_X / GRID_RES)   # 400 cells
NY = int(FIELD_Y / GRID_RES)   # 100 cells

# Reproducible random obstacle field
RNG = np.random.default_rng(42)
N_OBS = 15
OBS_POSITIONS = RNG.uniform(
    [20.0, 5.0], [FIELD_X - 20.0, FIELD_Y - 5.0], size=(N_OBS, 2)
)


# --- A* Path Planner ---

def astar_2d(occ_grid, start_cell, goal_cell):
    """A* on 4-connected grid; returns list of (ix, iy) cells."""
    def h(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    open_heap = [(h(start_cell, goal_cell), 0, start_cell, [start_cell])]
    visited = set()
    while open_heap:
        f, g, node, path = heapq.heappop(open_heap)
        if node == goal_cell:
            return path
        if node in visited:
            continue
        visited.add(node)
        ix, iy = node
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx_, ny_ = ix+dx, iy+dy
            if 0 <= nx_ < NX and 0 <= ny_ < NY:
                if not occ_grid[nx_, ny_] and (nx_,ny_) not in visited:
                    g2 = g + 1
                    heapq.heappush(open_heap,
                        (g2 + h((nx_,ny_), goal_cell), g2, (nx_,ny_),
                         path + [(nx_,ny_)]))
    return [start_cell, goal_cell]  # fallback straight line


def inflate_obstacle(occ_grid, obs_pos, r_inflate, grid_res):
    """Mark all cells within r_inflate of obs_pos as blocked."""
    cx = int(obs_pos[0] / grid_res)
    cy = int(obs_pos[1] / grid_res)
    r_cells = int(np.ceil(r_inflate / grid_res))
    for dx in range(-r_cells, r_cells+1):
        for dy in range(-r_cells, r_cells+1):
            nx_, ny_ = cx+dx, cy+dy
            if 0 <= nx_ < NX and 0 <= ny_ < NY:
                if np.sqrt(dx**2+dy**2)*grid_res <= r_inflate:
                    occ_grid[nx_, ny_] = True
    return occ_grid


def path_length_m(path_cells, grid_res):
    """Convert cell path to physical length (m)."""
    return (len(path_cells) - 1) * grid_res


# --- Scout Drone Lawnmower Controller ---

def drone_lawnmower_target(t, p_harvester, field_x, field_y,
                            l_lead, corridor_w, n_passes, w_scan, v_drone):
    """
    Return instantaneous waypoint for the drone given harvester position.
    Scout executes a 3-pass boustrophedon in the 10m corridor ahead.
    The along-track position within the corridor is driven by time.
    """
    y_track = p_harvester[1]   # harvester lateral position (assumed centred)
    x_lead  = np.clip(p_harvester[0] + l_lead, 0.0, field_x - D_TURNAROUND)

    # Lawnmower phase within [x_lead - corridor_w, x_lead]
    strip_idx = int((t * v_drone / (corridor_w / n_passes)) % n_passes)
    strip_idx = strip_idx % n_passes
    y_strip   = y_track - corridor_w/2 + (strip_idx + 0.5) * w_scan

    # Alternate direction per pass
    x_phase = (t * v_drone) % (2 * corridor_w)
    if x_phase < corridor_w:
        x_drone = x_lead - corridor_w + x_phase
    else:
        x_drone = x_lead - (x_phase - corridor_w)

    x_drone = np.clip(x_drone, 0.0, field_x)
    y_strip = np.clip(y_strip, 0.0, field_y)
    return np.array([x_drone, y_strip])


# --- Main Simulation ---

def run_simulation():
    occ_grid      = np.zeros((NX, NY), dtype=bool)
    p_harvester   = np.array([0.0, FIELD_Y / 2.0])
    p_drone       = np.array([L_LEAD, FIELD_Y / 2.0])

    # State tracking
    detected_obs      = set()
    pending_relays    = []   # (t_relay, obs_idx)
    replan_events     = []   # (t, p_harvester_at_replan, delta_L)
    harvester_path    = []
    drone_path        = []
    timely_count = marginal_count = late_count = 0
    total_extra_length = 0.0

    # Compute initial straight-line reference path
    goal_cell = (NX - 1, int(p_harvester[1] / GRID_RES))

    t = 0.0
    while p_harvester[0] < FIELD_X - 1.0 and t < 300.0:
        # --- Noise ---
        p_h_noisy = p_harvester + RNG.normal(0, GPS_ERROR, 2)
        p_d_noisy = p_drone     + RNG.normal(0, GPS_ERROR, 2)

        harvester_path.append(p_harvester.copy())
        drone_path.append(p_drone.copy())

        # --- Drone detection sweep ---
        for j, obs in enumerate(OBS_POSITIONS):
            if j in detected_obs:
                continue
            if np.linalg.norm(p_d_noisy - obs) <= (R_SENSOR - GPS_ERROR):
                detected_obs.add(j)
                pending_relays.append((t + T_LATENCY, j))

        # --- Process relayed messages ---
        for relay_t, j in list(pending_relays):
            if t >= relay_t:
                pending_relays.remove((relay_t, j))
                obs = OBS_POSITIONS[j]
                # Classify timing
                dist_to_obs = np.linalg.norm(obs - p_harvester)
                ttc = dist_to_obs / V_HARVESTER
                if ttc > T_REPLAN_MIN:
                    timely_count += 1
                elif ttc > 0:
                    marginal_count += 1
                else:
                    late_count += 1

                # A* replan before and after inflation (latency cost)
                start_cell = (int(p_harvester[0]/GRID_RES),
                               int(p_harvester[1]/GRID_RES))

                # Path before this obstacle was known (counterfactual)
                L_before = path_length_m(
                    astar_2d(occ_grid.copy(), start_cell, goal_cell), GRID_RES)

                # Inflate and replan
                occ_grid = inflate_obstacle(occ_grid, obs, R_INFLATE, GRID_RES)
                new_path  = astar_2d(occ_grid, start_cell, goal_cell)
                L_after   = path_length_m(new_path, GRID_RES)

                # Extra length vs. straight to goal
                straight  = np.linalg.norm(
                    np.array(goal_cell)*GRID_RES - p_harvester)
                delta_L   = max(0.0, L_after - straight)
                latency_penalty = max(0.0, L_after - L_before)
                total_extra_length += delta_L

                replan_events.append({
                    "t": t, "obs_idx": j, "ttc": ttc,
                    "delta_L": delta_L, "latency_penalty": latency_penalty,
                    "p_harvester": p_harvester.copy(),
                })

        # --- Advance harvester ---
        p_harvester[0] += V_HARVESTER * DT

        # --- Advance drone ---
        wp = drone_lawnmower_target(
            t, p_harvester, FIELD_X, FIELD_Y,
            L_LEAD, CORRIDOR_W, N_PASSES, W_SCAN, V_DRONE)
        to_wp   = wp - p_drone
        dist_wp = np.linalg.norm(to_wp)
        step    = V_DRONE * DT
        if dist_wp > step:
            p_drone += (to_wp / dist_wp) * step
        else:
            p_drone = wp.copy()

        t += DT

    harvester_path = np.array(harvester_path)
    drone_path     = np.array(drone_path)

    n_detected    = len(detected_obs)
    n_replanned   = len(replan_events)
    detect_rate   = 100.0 * n_detected / N_OBS

    print(f"Obstacles placed       : {N_OBS}")
    print(f"Obstacles detected     : {n_detected}  ({detect_rate:.1f}%)")
    print(f"Replanning events      : {n_replanned}")
    print(f"  Timely detections    : {timely_count}")
    print(f"  Marginal detections  : {marginal_count}")
    print(f"  Late detections      : {late_count}")
    print(f"Total extra path length: {total_extra_length:.2f} m")
    latency_total = sum(e["latency_penalty"] for e in replan_events)
    print(f"Latency penalty (extra): {latency_total:.2f} m")

    return {
        "harvester_path": harvester_path,
        "drone_path":     drone_path,
        "detected_obs":   detected_obs,
        "replan_events":  replan_events,
        "occ_grid":       occ_grid,
        "n_detected":     n_detected,
        "detect_rate":    detect_rate,
        "n_replanned":    n_replanned,
        "total_extra_length": total_extra_length,
        "latency_total":  latency_total,
        "timely_count":   timely_count,
        "marginal_count": marginal_count,
        "late_count":     late_count,
    }


def plot_results(res):
    hp  = res["harvester_path"]
    dp  = res["drone_path"]
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle("S078 Harvester Cooperative Guidance", fontsize=13, fontweight="bold")

    # --- Plot 1: Field overview (top-down) ---
    ax = axes[0, 0]
    ax.set_title("Field Overview — Scout + Harvester Paths")
    ax.set_xlim(0, FIELD_X); ax.set_ylim(0, FIELD_Y)
    ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")

    for j, obs in enumerate(OBS_POSITIONS):
        colour = "grey" if j not in res["detected_obs"] else "darkorange"
        circ   = plt.Circle(obs, R_OBS, color=colour, alpha=0.6, zorder=3)
        ax.add_patch(circ)
    ax.plot(hp[:, 0], hp[:, 1], "r-",  lw=1.5, label="Harvester", zorder=4)
    ax.plot(dp[:, 0], dp[:, 1], "b-",  lw=0.8, alpha=0.5, label="Scout drone", zorder=4)
    for ev in res["replan_events"]:
        ax.plot(*ev["p_harvester"], "r^", ms=6, zorder=5)
    ax.legend(fontsize=8, loc="upper left")
    ax.text(2, 2, f"Detected: {res['n_detected']}/{N_OBS} ({res['detect_rate']:.0f}%)",
            fontsize=8, color="black")

    # --- Plot 2: Detection classification bar chart ---
    ax = axes[0, 1]
    ax.set_title("Detection Classification")
    categories = ["Timely\n(TTC > 5 s)", "Marginal\n(0 < TTC ≤ 5 s)", "Late\n(TTC ≤ 0)"]
    counts     = [res["timely_count"], res["marginal_count"], res["late_count"]]
    colours    = ["green", "gold", "red"]
    ax.bar(categories, counts, color=colours, edgecolor="black")
    ax.set_ylabel("Count")
    ax.set_xlabel("Detection category")
    for i, c in enumerate(counts):
        ax.text(i, c + 0.05, str(c), ha="center", fontsize=10)

    # --- Plot 3: Cumulative extra path length vs time ---
    ax = axes[1, 0]
    ax.set_title("Cumulative Extra Path Length vs Time")
    if res["replan_events"]:
        times   = [ev["t"]       for ev in res["replan_events"]]
        dLs     = [ev["delta_L"] for ev in res["replan_events"]]
        cum_dL  = np.cumsum(dLs)
        ax.step(times, cum_dL, where="post", color="red",    label="Total extra length")
        lat_pen = [ev["latency_penalty"] for ev in res["replan_events"]]
        cum_lat = np.cumsum(lat_pen)
        ax.step(times, cum_lat, where="post", color="orange",
                linestyle="--", label="Latency penalty component")
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("Cumulative extra path length (m)")
    ax.legend(fontsize=8)

    # --- Plot 4: Occupancy grid (blocked cells after all detections) ---
    ax = axes[1, 1]
    ax.set_title("A* Occupancy Grid — Inflated Obstacles")
    ax.imshow(res["occ_grid"].T, origin="lower", cmap="Greys",
              extent=[0, FIELD_X, 0, FIELD_Y], aspect="auto")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    for j, obs in enumerate(OBS_POSITIONS):
        marker = "x" if j not in res["detected_obs"] else "+"
        colour = "grey" if j not in res["detected_obs"] else "cyan"
        ax.plot(*obs, marker, color=colour, ms=8, mew=2)

    plt.tight_layout()
    plt.savefig("outputs/04_industrial_agriculture/s078_harvester_guidance/field_overview.png",
                dpi=120, bbox_inches="tight")
    plt.show()


def animate_mission(res):
    """Top-down animation of scout + harvester traversal."""
    hp = res["harvester_path"]
    dp = res["drone_path"]
    stride = 10   # animate every 10th frame

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.set_xlim(0, FIELD_X); ax.set_ylim(0, FIELD_Y)
    ax.set_aspect("equal")
    ax.set_title("S078 Scout–Harvester Animation")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")

    for j, obs in enumerate(OBS_POSITIONS):
        colour = "grey" if j not in res["detected_obs"] else "darkorange"
        ax.add_patch(plt.Circle(obs, R_OBS, color=colour, alpha=0.5))

    harv_dot,  = ax.plot([], [], "rs", ms=10, label="Harvester", zorder=5)
    drone_dot, = ax.plot([], [], "b^", ms=8,  label="Scout",     zorder=5)
    harv_trail, = ax.plot([], [], "r-", lw=1.0, alpha=0.6)
    drone_trail, = ax.plot([], [], "b-", lw=0.6, alpha=0.4)
    time_text = ax.text(5, FIELD_Y - 4, "", fontsize=9)
    ax.legend(fontsize=8, loc="upper right")

    indices = range(0, min(len(hp), len(dp)), stride)

    def update(frame):
        i = frame
        harv_dot.set_data([hp[i, 0]], [hp[i, 1]])
        drone_dot.set_data([dp[i, 0]], [dp[i, 1]])
        harv_trail.set_data(hp[:i+1, 0], hp[:i+1, 1])
        drone_trail.set_data(dp[:i+1, 0], dp[:i+1, 1])
        time_text.set_text(f"t = {i * DT * stride:.1f} s")
        return harv_dot, drone_dot, harv_trail, drone_trail, time_text

    anim = FuncAnimation(fig, update, frames=list(indices),
                         interval=30, blit=True)
    anim.save(
        "outputs/04_industrial_agriculture/s078_harvester_guidance/mission_animation.gif",
        writer="pillow", fps=20)
    plt.show()


if __name__ == "__main__":
    results = run_simulation()
    plot_results(results)
    animate_mission(results)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Field dimensions | 200 × 50 m |
| Harvester speed $v_{harvester}$ | 1.5 m/s |
| Scout drone speed $v_{drone}$ | 3.0 m/s |
| Lead distance $L_{lead}$ | 45 m |
| Scouting time horizon $T_{horizon}$ | 30 s |
| Corridor width | 10 m |
| Lawnmower passes $N_{passes}$ | 3 |
| Strip width $w_{scan}$ | 3.3 m |
| Sensor detection radius $r_{sensor}$ | 2.0 m |
| Obstacle physical radius $r_{obs}$ | 1.0 m |
| Harvester half-width $r_{harvester}$ | 1.5 m |
| A* inflation radius $r_{inflate}$ | 2.5 m |
| Communication latency $T_{latency}$ | 2 s |
| Latency position penalty $\Delta x_{latency}$ | 3.0 m |
| A* grid resolution $\Delta_{grid}$ | 0.5 m |
| Grid size | 400 × 100 cells |
| Number of obstacles $N_{obs}$ | 15 |
| GPS position error $\sigma_{GPS}$ | 0.1 m |
| Simulation timestep $\Delta t$ | 0.1 s |
| Loopback trigger distance $d_{turnaround}$ | 5 m |
| Timely detection threshold $T_{replan}$ | 5 s TTC |
| Domain inspection speed | 1.0 m/s (reference) |

---

## Expected Output

- **Field overview (top-down)**: 200 × 50 m field with all obstacles drawn as circles
  (grey = undetected, orange = detected); the harvester's trajectory as a red line with red
  triangle markers at each replanning event; the scout drone's lawnmower path as a thin blue
  trace; the lead distance $L_{lead}$ illustrated with a dashed bracket.
- **Detection classification bar chart**: three grouped bars showing the count of timely
  (green), marginal (gold), and late (red) detections, enabling quick assessment of lead-distance
  and latency adequacy.
- **Cumulative extra path length vs time**: step-plot of total extra path length accumulated
  over the mission (red solid line) and the latency-attributable penalty component (orange
  dashed line); x-axis is simulation time in seconds; vertical dotted lines mark each replanning
  event.
- **A* occupancy grid heatmap**: the final 400 × 100 blocked-cell grid displayed as a grey
  image; detected obstacles shown as cyan "+" symbols, undetected as grey "×" symbols; illustrates
  the inflation footprint relative to raw obstacle sizes.
- **Mission animation (GIF)**: top-down animated view of the harvester (red square) and scout
  drone (blue triangle) traversing the field together; each detected obstacle changes colour from
  grey to orange when the relay message arrives; a live time counter and replanning event counter
  are displayed in the corner.
- **Console metrics table**:

  | Metric | Value |
  |--------|-------|
  | Obstacles placed | 15 |
  | Obstacles detected | … |
  | Detection rate (%) | … |
  | Replanning events | … |
  | Timely detections | … |
  | Marginal detections | … |
  | Late detections | … |
  | Total extra path length (m) | … |
  | Latency penalty (m) | … |

---

## Extensions

1. **Lead distance sensitivity sweep**: run the simulation for
   $L_{lead} \in \{15, 30, 45, 60, 75\}$ m and plot detection rate and extra path length as
   functions of $L_{lead}$; identify the optimal lead that maximises detection rate before the
   harvester arrives while keeping drone battery endurance feasible.
2. **Variable latency**: sweep $T_{latency} \in \{0, 1, 2, 4, 8\}$ s and measure the latency
   penalty $\Delta L_{latency}$ vs. latency curve; fit a linear regression to quantify the
   marginal detour cost per second of communication delay.
3. **Multi-obstacle density**: vary $N_{obs} \in \{5, 10, 20, 40\}$ and plot the fraction of
   obstacles missed (drone never passes within $r_{sensor}$) to identify the density threshold
   above which the 3-pass lawnmower is insufficient and additional passes or wider sensor range
   are needed.
4. **Dual-drone configuration**: add a second scout drone covering the opposite half of the
   field width; compare total detection rate and extra path length with the single-drone
   baseline; quantify the coordination overhead (waypoint handoff, no-fly zone between drones).
5. **Adaptive replanning horizon**: instead of replanning to the field exit, replan only over a
   rolling $H_{plan} = 20$ m horizon; re-solve every $T_{replan} = 5$ s; compare computational
   cost and path quality with the full-horizon A* baseline.
6. **3D terrain extension**: add a Digital Elevation Model (DEM) for sloped agricultural land;
   the harvester's speed varies with slope (energy model); the drone must maintain a fixed
   above-ground altitude using the DEM; connect to [S065 3D Scan Path Planning](S065_3d_scan_path.md).

---

## Related Scenarios

- Prerequisites: [S068 Large-Scale Farmland Cooperative Spraying](S068_large_field_spray.md), [S070 Crop Health Monitoring](S070_crop_health_monitoring.md)
- Follow-up: [S077 Seeding Path Planning](S077_seeding_path.md) (same air-ground coordination framework, seed delivery)
- Algorithmic cross-reference: [S048 Full-Area Coverage Scan (Lawnmower)](../../03_environmental_sar/S048_lawnmower.md) (boustrophedon strip geometry), [S042 Missing Person Search](../../03_environmental_sar/S042_missing_person_search.md) (A* replanning on occupancy grid)
