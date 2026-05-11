# S070 Swarm Weeding Task

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Hungarian Task Assignment + PID Precision Hover | **Dimension**: 2D

---

## Problem Definition

**Setup**: A $40 \times 40$ m crop field contains $W = 30$ weed plants scattered at random positions
among uniform crop rows. A swarm of $N = 3$ drones must locate every weed and apply a targeted
herbicide treatment. The mission proceeds in two sequential phases:

**Phase 1 — Detection Sweep**: Each drone is assigned one of three equal-width parallel sub-strips
($40 \times 13.3$ m each). All three drones fly synchronised lawnmower (boustrophedon) patterns at
a cruise speed of $v = 1.0$ m/s (domain inspection speed). A downward-facing multispectral sensor
with detection radius $r_s = 1.5$ m and per-pass detection probability $P_{detect} = 0.85$ flags
candidate weed positions using a Bernoulli model. Weeds not detected in one pass may be detected on
the next overlapping pass (the sweep spacing is set to $2 r_s$, so each weed is covered exactly
once per sweep).

**Phase 2 — Treatment**: After all three drones complete their sweep strips, the set of detected
weeds $\mathcal{W}_{det}$ is assembled. A global cost matrix is constructed and the Hungarian
algorithm solves the minimum-cost assignment of $|\mathcal{W}_{det}|$ weeds across the three
drones (one drone may treat multiple weeds sequentially). Each drone then navigates to its assigned
weeds in order and hovers for $T_{treat} = 2$ s at each weed centroid, applying herbicide. A PID
controller holds position within $\epsilon_{pos} = 0.15$ m RMS to ensure treatment accuracy.
GPS noise is modelled as $\mathcal{N}(0, \sigma_{GPS}^2)$ with $\sigma_{GPS} = 0.10$ m.

**Roles**:
- **Drones** ($N = 3$): homogeneous platforms, one per sub-strip during Phase 1; reassigned to
  optimal weed subsets during Phase 2.
- **Weeds** ($W = 30$): fixed positions drawn uniformly at random within the field; each weed
  requires exactly one confirmed treatment hover.

**Objective**: Treat the maximum number of weeds (primary) while minimising total travel distance
in Phase 2 (secondary). Two assignment strategies are compared:

1. **Hungarian assignment** — solve the linear sum assignment problem on the full cost matrix;
   globally minimises total drone–weed travel distance.
2. **Greedy nearest-drone assignment** — for each unassigned weed, pick the closest available
   drone; fast but suboptimal; serves as the baseline.

---

## Mathematical Model

### Phase 1 Detection Sweep

Discretise each sub-strip into a boustrophedon path with strip spacing $\delta_s = 2 r_s = 3.0$ m.
For sub-strip $i$ (width $L_{strip} = 40/3$ m, length $L_{field} = 40$ m), the number of passes is:

$$n_{passes}^{(i)} = \left\lceil \frac{L_{strip}}{\delta_s} \right\rceil$$

Weed $j$ at position $\mathbf{w}_j$ is detected by drone $i$ at position $\mathbf{p}_i(t)$ during
the sweep if two conditions both hold:

$$\|\mathbf{p}_i(t) - \mathbf{w}_j\| \leq r_s \quad \text{and} \quad b_{ij} = 1$$

where $b_{ij} \sim \mathrm{Bernoulli}(P_{detect})$ is drawn independently for each (drone, weed,
pass) encounter. Because each weed falls within exactly one drone's sub-strip and is covered once
per sweep, the probability of detection after a single sweep pass is $P_{detect} = 0.85$.

GPS positioning error during the sweep is modelled as additive noise:

$$\tilde{\mathbf{p}}_i(t) = \mathbf{p}_i(t) + \boldsymbol{\eta}, \qquad
  \boldsymbol{\eta} \sim \mathcal{N}\!\left(\mathbf{0},\, \sigma_{GPS}^2 \mathbf{I}_2\right)$$

### Phase 2 Hungarian Task Assignment

Let $\mathcal{D} = \{1, 2, 3\}$ be the drone set and $\mathcal{W}_{det} = \{1, \ldots, M\}$ the
set of $M = |\mathcal{W}_{det}|$ detected weeds. Construct the $N \times M$ cost matrix:

$$C_{ij} = \|\mathbf{p}_i^{end} - \mathbf{w}_j\|, \qquad i \in \mathcal{D},\; j \in \mathcal{W}_{det}$$

where $\mathbf{p}_i^{end}$ is drone $i$'s position at the end of its Phase 1 sweep strip. When
$M > N$ the assignment problem is rectangular; the standard formulation pads $C$ to a square
$\max(N, M) \times \max(N, M)$ matrix with zero-cost dummy entries.

The optimal assignment $\mathbf{X}^* = \{x_{ij}^*\} \in \{0,1\}^{N \times M}$ minimises total
travel cost subject to covering every detected weed exactly once:

$$\min_{\mathbf{X}} \sum_{i \in \mathcal{D}} \sum_{j \in \mathcal{W}_{det}} C_{ij}\, x_{ij}$$

$$\text{subject to} \quad \sum_{j} x_{ij} \leq K_{max}, \;\; \forall\, i; \qquad
  \sum_{i} x_{ij} = 1, \;\; \forall\, j; \qquad x_{ij} \in \{0,1\}$$

where $K_{max} = \lceil M / N \rceil$ limits per-drone weed load. This is solved in $O(M^3)$ time
using the Kuhn–Munkres (Hungarian) algorithm via `scipy.optimize.linear_sum_assignment`.

For the **greedy baseline**, weeds are assigned iteratively: at each step the unassigned weed
nearest to any drone's current end-of-strip position is assigned to that drone; the drone's
notional position is then advanced to the assigned weed's location.

### Phase 2 PID Precision Hover

During treatment, each drone must hold position within $\epsilon_{pos} = 0.15$ m RMS. A
discrete-time PID controller drives lateral position error to zero:

$$\mathbf{u}(t) = K_p\, \mathbf{e}(t) + K_i \sum_{\tau=0}^{t} \mathbf{e}(\tau)\,\Delta t
                + K_d\, \frac{\mathbf{e}(t) - \mathbf{e}(t-1)}{\Delta t}$$

where $\mathbf{e}(t) = \mathbf{w}_{j}^{target} - \tilde{\mathbf{p}}_i(t)$ is the position error
including GPS noise. The closed-loop position variance in steady state is:

$$\sigma_{hover}^2 = \frac{\sigma_{GPS}^2}{1 + 2 K_p / K_d}$$

With $K_p = 2.0$, $K_i = 0.1$, $K_d = 0.5$ and $\sigma_{GPS} = 0.10$ m, the steady-state RMS
error is approximately $0.06$ m, well within the $0.15$ m treatment threshold.

### Weeding Success Rate

A weed is successfully treated if (i) it was detected in Phase 1 and (ii) the drone's hover
position error during treatment satisfies $\|\mathbf{e}\| \leq \epsilon_{pos}$. The joint
success probability per weed is:

$$P_{success} = P_{detect} \cdot P\!\left(\|\mathbf{e}\| \leq \epsilon_{pos}\right)
             = P_{detect} \cdot \left[1 - \exp\!\left(-\frac{\epsilon_{pos}^2}{2\,\sigma_{hover}^2}\right)\right]$$

For the nominal parameters ($P_{detect} = 0.85$, $\sigma_{hover} \approx 0.06$ m,
$\epsilon_{pos} = 0.15$ m):

$$P_{success} \approx 0.85 \times 0.997 \approx 0.847$$

Expected number of weeds treated: $\mathbb{E}[W_{treated}] = W \cdot P_{success} \approx 25.4$.

### Phase 2 Travel Cost Comparison

Define the total Phase 2 travel distance for assignment strategy $s$:

$$D_s = \sum_{i \in \mathcal{D}} \sum_{j \in \mathcal{W}_{det}^{(i)}} \|\mathbf{p}_{i,j-1} - \mathbf{w}_j\|$$

where $\mathbf{p}_{i,0} = \mathbf{p}_i^{end}$ and $\mathbf{p}_{i,j}$ is the drone's position after
treating its $j$-th assigned weed. The assignment cost ratio (efficiency gain of Hungarian over
greedy) is:

$$\rho = \frac{D_{greedy}}{D_{hungarian}} \geq 1$$

Theoretical worst-case improvement: for $M$ uniformly scattered tasks and $N$ agents,
$\mathbb{E}[\rho] \approx 1 + O(1/\sqrt{N})$ (bounded gain for small $N$, larger for clustered
weeds).

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.optimize import linear_sum_assignment

# ── Simulation constants ──────────────────────────────────────────────────────
N_DRONES       = 3
FIELD_SIZE     = 40.0       # m — square field side length
N_WEEDS        = 30         # total weed plants
SENSOR_RADIUS  = 1.5        # m — detection footprint radius
P_DETECT       = 0.85       # Bernoulli detection probability per pass
INSPECTION_SPD = 1.0        # m/s — Phase 1 cruise speed (domain constant)
GPS_ERROR      = 0.10       # m std dev — positioning noise (domain constant)
T_TREAT        = 2.0        # s — hover duration per weed
POS_THRESHOLD  = 0.15       # m — required hover accuracy (RMS)
DT             = 0.05       # s — simulation timestep

# PID gains for precision hover
KP, KI, KD    = 2.0, 0.1, 0.5

STRIP_WIDTH    = FIELD_SIZE / N_DRONES          # m per sub-strip
SWEEP_SPACING  = 2.0 * SENSOR_RADIUS            # m between lawnmower passes


# ── Lawnmower path generator ──────────────────────────────────────────────────

def lawnmower_strip(drone_idx, field_size=FIELD_SIZE, spacing=SWEEP_SPACING):
    """
    Generate boustrophedon waypoints for sub-strip assigned to drone_idx.
    x-axis: along crop rows (0 to field_size)
    y-axis: cross-row (each drone covers [drone_idx*STRIP_WIDTH, (drone_idx+1)*STRIP_WIDTH])
    Returns: (n_waypoints, 2) array of (x, y) positions.
    """
    y_min = drone_idx * STRIP_WIDTH
    y_max = (drone_idx + 1) * STRIP_WIDTH
    y_passes = np.arange(y_min + spacing / 2, y_max, spacing)
    waypoints = []
    for k, y in enumerate(y_passes):
        if k % 2 == 0:
            waypoints += [[0.0, y], [field_size, y]]
        else:
            waypoints += [[field_size, y], [0.0, y]]
    return np.array(waypoints)


# ── Bernoulli detection model ─────────────────────────────────────────────────

def detect_weeds(drone_path, weed_positions, rng):
    """
    Simulate weed detection during a lawnmower sweep.
    drone_path      : (n_wp, 2) — waypoint sequence (drone moves at constant speed)
    weed_positions  : (W, 2) — true weed locations
    Returns: boolean mask (W,) — True for each weed detected.
    """
    W = len(weed_positions)
    detected = np.zeros(W, dtype=bool)
    step = INSPECTION_SPD * DT

    pos = drone_path[0].copy()
    wp_idx = 1

    while wp_idx < len(drone_path):
        target = drone_path[wp_idx]
        diff   = target - pos
        dist   = np.linalg.norm(diff)

        if dist < 1e-6:
            wp_idx += 1
            continue

        move = min(step, dist)
        pos += (diff / dist) * move

        # GPS noise on observed position
        obs_pos = pos + rng.normal(0, GPS_ERROR, size=2)

        # Check proximity to each undetected weed
        for j in range(W):
            if not detected[j]:
                if np.linalg.norm(obs_pos - weed_positions[j]) <= SENSOR_RADIUS:
                    if rng.random() < P_DETECT:
                        detected[j] = True

        if dist <= move:
            wp_idx += 1

    return detected


# ── Hungarian vs greedy assignment ───────────────────────────────────────────

def build_cost_matrix(drone_end_positions, weed_positions):
    """
    Cost matrix C[i, j] = Euclidean distance from drone i end position to weed j.
    drone_end_positions : (N, 2)
    weed_positions      : (M, 2)
    Returns: (N, M) cost matrix.
    """
    N = len(drone_end_positions)
    M = len(weed_positions)
    C = np.zeros((N, M))
    for i in range(N):
        for j in range(M):
            C[i, j] = np.linalg.norm(drone_end_positions[i] - weed_positions[j])
    return C


def hungarian_assign(drone_end_positions, weed_positions):
    """
    Solve rectangular assignment using scipy linear_sum_assignment.
    Returns: list of length N, each element is a list of weed indices for that drone.
    """
    N = len(drone_end_positions)
    M = len(weed_positions)
    if M == 0:
        return [[] for _ in range(N)]

    # Expand cost matrix: each weed must appear once; drones may have multiple rows
    # Strategy: replicate each drone row ceil(M/N) times, solve, then collect assignments
    K = int(np.ceil(M / N))
    C_exp = np.tile(build_cost_matrix(drone_end_positions, weed_positions),
                    (K, 1))   # (N*K, M)
    row_ind, col_ind = linear_sum_assignment(C_exp)

    assignment = [[] for _ in range(N)]
    for r, c in zip(row_ind, col_ind):
        if c < M:
            drone_id = r % N
            assignment[drone_id].append(c)
    return assignment


def greedy_assign(drone_end_positions, weed_positions):
    """
    Greedy nearest-drone assignment: iteratively assign each weed to the
    closest drone (by current end position, updated after each assignment).
    Returns: list of length N, each element is a list of weed indices.
    """
    N = len(drone_end_positions)
    M = len(weed_positions)
    assignment  = [[] for _ in range(N)]
    cur_pos     = [p.copy() for p in drone_end_positions]
    unassigned  = list(range(M))

    while unassigned:
        # Find the (drone, weed) pair with minimum distance
        best_cost  = np.inf
        best_drone = 0
        best_weed  = unassigned[0]
        for i in range(N):
            for j in unassigned:
                d = np.linalg.norm(cur_pos[i] - weed_positions[j])
                if d < best_cost:
                    best_cost  = d
                    best_drone = i
                    best_weed  = j
        assignment[best_drone].append(best_weed)
        cur_pos[best_drone] = weed_positions[best_weed].copy()
        unassigned.remove(best_weed)

    return assignment


# ── PID precision hover ───────────────────────────────────────────────────────

def pid_hover(target, rng, t_treat=T_TREAT, dt=DT):
    """
    Simulate PID hover at target position for t_treat seconds.
    Returns: (n_steps, 2) position trajectory and RMS position error.
    """
    pos     = target + rng.normal(0, 0.3, size=2)   # start offset
    integral= np.zeros(2)
    prev_e  = target - pos
    traj    = [pos.copy()]

    for _ in range(int(t_treat / dt)):
        obs_pos  = pos + rng.normal(0, GPS_ERROR, size=2)
        e        = target - obs_pos
        integral += e * dt
        deriv    = (e - prev_e) / dt
        u        = KP * e + KI * integral + KD * deriv
        pos      = pos + u * dt
        prev_e   = e
        traj.append(pos.copy())

    traj = np.array(traj)
    rms  = np.sqrt(np.mean(np.sum((traj - target)**2, axis=1)))
    return traj, rms


# ── Phase 2 travel distance ───────────────────────────────────────────────────

def compute_phase2_distance(drone_end_positions, weed_positions, assignment):
    """Total travel distance for Phase 2 given an assignment."""
    total = 0.0
    for i, weed_list in enumerate(assignment):
        pos = drone_end_positions[i].copy()
        for j in weed_list:
            total += np.linalg.norm(pos - weed_positions[j])
            pos    = weed_positions[j].copy()
    return total


# ── Main simulation ───────────────────────────────────────────────────────────

def run_simulation(seed=42):
    rng = np.random.default_rng(seed)

    # Place weeds uniformly at random in field
    weed_pos = rng.uniform(0, FIELD_SIZE, size=(N_WEEDS, 2))

    # Phase 1: sweep and detect
    paths       = [lawnmower_strip(i) for i in range(N_DRONES)]
    detected    = np.zeros(N_WEEDS, dtype=bool)
    end_pos     = np.zeros((N_DRONES, 2))

    for i in range(N_DRONES):
        det_i      = detect_weeds(paths[i], weed_pos, rng)
        detected  |= det_i
        end_pos[i] = paths[i][-1]

    detected_weeds = weed_pos[detected]
    n_detected     = detected.sum()
    print(f"Phase 1: {n_detected}/{N_WEEDS} weeds detected "
          f"(expected {N_WEEDS * P_DETECT:.1f})")

    # Phase 2: assignment
    assign_h = hungarian_assign(end_pos, detected_weeds)
    assign_g = greedy_assign(end_pos, detected_weeds)

    dist_h = compute_phase2_distance(end_pos, detected_weeds, assign_h)
    dist_g = compute_phase2_distance(end_pos, detected_weeds, assign_g)
    print(f"Phase 2 travel — Hungarian: {dist_h:.2f} m | Greedy: {dist_g:.2f} m "
          f"| ratio: {dist_g / max(dist_h, 1e-9):.3f}")

    # Phase 2: hover simulation (Hungarian assignment only)
    hover_rms_all = []
    treated_count = 0
    for i, weed_list in enumerate(assign_h):
        for j in weed_list:
            _, rms = pid_hover(detected_weeds[j], rng)
            hover_rms_all.append(rms)
            if rms <= POS_THRESHOLD:
                treated_count += 1

    mean_hover_rms = np.mean(hover_rms_all) if hover_rms_all else 0.0
    print(f"Treatment — weeds treated: {treated_count}/{n_detected} "
          f"| mean hover RMS: {mean_hover_rms:.4f} m")

    return {
        "weed_pos":        weed_pos,
        "detected":        detected,
        "detected_weeds":  detected_weeds,
        "paths":           paths,
        "end_pos":         end_pos,
        "assign_h":        assign_h,
        "assign_g":        assign_g,
        "dist_h":          dist_h,
        "dist_g":          dist_g,
        "hover_rms_all":   hover_rms_all,
        "treated_count":   treated_count,
        "mean_hover_rms":  mean_hover_rms,
    }


# ── Plotting ──────────────────────────────────────────────────────────────────

DRONE_COLORS = ["tab:red", "tab:blue", "tab:green"]


def plot_field_overview(res):
    """Top-down field map: sweep strips, weed positions, detection status."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    for ax_idx, (title, assign) in enumerate(
        [("Hungarian Assignment", res["assign_h"]),
         ("Greedy Assignment",    res["assign_g"])]
    ):
        ax = axes[ax_idx]
        ax.set_facecolor("#f5f5e8")

        # Sub-strip backgrounds
        for i in range(N_DRONES):
            y_min = i * STRIP_WIDTH
            y_max = (i + 1) * STRIP_WIDTH
            ax.axhspan(y_min, y_max, alpha=0.08, color=DRONE_COLORS[i])

        # Phase 1 sweep paths
        for i, path in enumerate(res["paths"]):
            ax.plot(path[:, 0], path[:, 1], color=DRONE_COLORS[i],
                    lw=0.8, alpha=0.5, label=f"Drone {i+1} sweep")

        # All weeds
        ax.scatter(res["weed_pos"][:, 0], res["weed_pos"][:, 1],
                   c="gray", s=30, zorder=3, label="Weed (missed)")
        # Detected weeds
        ax.scatter(res["detected_weeds"][:, 0], res["detected_weeds"][:, 1],
                   c="gold", s=60, edgecolors="k", lw=0.5, zorder=4, label="Weed (detected)")

        # Phase 2 travel paths
        for i, weed_list in enumerate(assign):
            pos = res["end_pos"][i].copy()
            for j in weed_list:
                target = res["detected_weeds"][j]
                ax.annotate("", xy=target, xytext=pos,
                            arrowprops=dict(arrowstyle="->", color=DRONE_COLORS[i],
                                            lw=1.4, alpha=0.85))
                pos = target.copy()

        # Drone end-of-strip positions
        for i in range(N_DRONES):
            ax.plot(*res["end_pos"][i], marker="^", ms=9,
                    color=DRONE_COLORS[i], zorder=5)

        ax.set_xlim(-1, FIELD_SIZE + 1)
        ax.set_ylim(-1, FIELD_SIZE + 1)
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        dist_key = "dist_h" if ax_idx == 0 else "dist_g"
        ax.set_title(f"{title}\nPhase 2 travel = {res[dist_key]:.1f} m")
        if ax_idx == 0:
            ax.legend(fontsize=7, loc="upper right")

    plt.suptitle("S070 Swarm Weeding — Field Overview", fontsize=13, fontweight="bold")
    plt.tight_layout()
    return fig


def plot_metrics(res):
    """Three-panel metrics figure: hover accuracy, cost comparison, detection."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))

    # Panel 1: hover RMS distribution
    ax = axes[0]
    ax.hist(res["hover_rms_all"], bins=12, color="steelblue", edgecolor="white", alpha=0.85)
    ax.axvline(POS_THRESHOLD, color="red", lw=1.5, ls="--", label=f"Threshold {POS_THRESHOLD} m")
    ax.axvline(res["mean_hover_rms"], color="orange", lw=1.5, ls="-",
               label=f"Mean {res['mean_hover_rms']:.3f} m")
    ax.set_xlabel("Hover RMS error (m)")
    ax.set_ylabel("Count")
    ax.set_title("PID Hover Accuracy\n(Phase 2, Hungarian)")
    ax.legend(fontsize=8)

    # Panel 2: assignment cost comparison (bar chart)
    ax = axes[1]
    strategies = ["Hungarian", "Greedy"]
    costs      = [res["dist_h"], res["dist_g"]]
    colors     = ["steelblue", "salmon"]
    bars = ax.bar(strategies, costs, color=colors, width=0.5, edgecolor="k", lw=0.7)
    for bar, val in zip(bars, costs):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                f"{val:.1f} m", ha="center", va="bottom", fontsize=9)
    ax.set_ylabel("Total Phase 2 travel (m)")
    ax.set_title(f"Assignment Cost Comparison\n"
                 f"Gain ratio: {res['dist_g'] / max(res['dist_h'], 1e-9):.3f}")
    ax.set_ylim(0, max(costs) * 1.15)

    # Panel 3: weeding outcome summary
    ax = axes[2]
    n_total    = N_WEEDS
    n_missed   = n_total - res["detected"].sum()
    n_detected = res["detected"].sum()
    n_treated  = res["treated_count"]
    n_untreated = n_detected - n_treated

    labels = ["Missed\n(not detected)", "Detected but\nhover error", "Successfully\ntreated"]
    sizes  = [n_missed, n_untreated, n_treated]
    colors_pie = ["#cccccc", "#ffaa55", "#66cc66"]
    wedge_props = {"edgecolor": "white", "linewidth": 1.5}
    ax.pie(sizes, labels=labels, colors=colors_pie, autopct="%1.0f%%",
           wedgeprops=wedge_props, startangle=90, textprops={"fontsize": 8})
    ax.set_title(f"Weeding Outcome\n(W={n_total} plants)")

    plt.suptitle("S070 Swarm Weeding — Performance Metrics", fontsize=13, fontweight="bold")
    plt.tight_layout()
    return fig


def make_animation(res, filename="outputs/04_industrial_agriculture/s070_swarm_weeding/s070_weeding.gif"):
    """
    Top-down animation: Phase 1 sweep (drones advancing) then Phase 2 travel + hover.
    Saves GIF to filename.
    """
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-1, FIELD_SIZE + 1)
    ax.set_ylim(-1, FIELD_SIZE + 1)
    ax.set_aspect("equal")
    ax.set_facecolor("#f5f5e8")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    # Static weed markers
    ax.scatter(res["weed_pos"][:, 0], res["weed_pos"][:, 1],
               c="gray", s=25, zorder=2, alpha=0.6)

    drone_markers = [
        ax.plot([], [], marker="^", ms=10, color=DRONE_COLORS[i], lw=0)[0]
        for i in range(N_DRONES)
    ]
    trail_lines = [
        ax.plot([], [], color=DRONE_COLORS[i], lw=1.2, alpha=0.5)[0]
        for i in range(N_DRONES)
    ]
    detected_scatter = ax.scatter([], [], c="gold", s=55, edgecolors="k",
                                  lw=0.5, zorder=4)
    title_obj = ax.set_title("Phase 1 — Detection Sweep", fontsize=11)

    # Pre-compute per-drone Phase 1 trajectory at INSPECTION_SPD
    def dense_path(path, speed=INSPECTION_SPD, dt=DT):
        traj = []
        pos = path[0].copy()
        for wp in path[1:]:
            diff = wp - pos
            dist = np.linalg.norm(diff)
            steps = max(1, int(dist / (speed * dt)))
            for s in range(steps):
                traj.append(pos + diff * s / steps)
            pos = wp.copy()
        traj.append(path[-1].copy())
        return np.array(traj)

    phase1_trajs = [dense_path(p) for p in res["paths"]]
    max_p1 = max(len(t) for t in phase1_trajs)

    # Build Phase 2 trajectory: travel to each assigned weed, then hover T_treat
    def build_phase2(end_pos, detected_weeds, assignment, speed=INSPECTION_SPD, dt=DT):
        trajs = []
        for i, weed_list in enumerate(assignment):
            traj = []
            pos = end_pos[i].copy()
            for j in weed_list:
                target = detected_weeds[j]
                diff   = target - pos
                dist   = np.linalg.norm(diff)
                steps  = max(1, int(dist / (speed * dt)))
                for s in range(steps):
                    traj.append(pos + diff * s / steps)
                hover_steps = int(T_TREAT / dt)
                traj.extend([target.copy()] * hover_steps)
                pos = target.copy()
            trajs.append(np.array(traj) if traj else np.array([end_pos[i]]))
        return trajs

    phase2_trajs = build_phase2(res["end_pos"], res["detected_weeds"], res["assign_h"])
    max_p2 = max(len(t) for t in phase2_trajs)

    total_frames = max_p1 + max_p2 + 10  # 10-frame pause between phases
    histories = [[] for _ in range(N_DRONES)]

    def init():
        for m in drone_markers:
            m.set_data([], [])
        for l in trail_lines:
            l.set_data([], [])
        detected_scatter.set_offsets(np.empty((0, 2)))
        return drone_markers + trail_lines + [detected_scatter, title_obj]

    def update(frame):
        if frame < max_p1:
            # Phase 1
            title_obj.set_text(f"Phase 1 — Detection Sweep  (t={frame * DT:.1f} s)")
            for i in range(N_DRONES):
                idx = min(frame, len(phase1_trajs[i]) - 1)
                pos = phase1_trajs[i][idx]
                drone_markers[i].set_data([pos[0]], [pos[1]])
                histories[i].append(pos.copy())
                hist = np.array(histories[i])
                trail_lines[i].set_data(hist[:, 0], hist[:, 1])
        elif frame < max_p1 + 10:
            title_obj.set_text("Phase 1 complete — Running Hungarian assignment…")
            det = res["detected_weeds"]
            if len(det):
                detected_scatter.set_offsets(det)
        else:
            f2 = frame - max_p1 - 10
            title_obj.set_text(f"Phase 2 — Treatment  (t={f2 * DT:.1f} s)")
            det = res["detected_weeds"]
            if len(det):
                detected_scatter.set_offsets(det)
            for i in range(N_DRONES):
                idx = min(f2, len(phase2_trajs[i]) - 1)
                pos = phase2_trajs[i][idx]
                drone_markers[i].set_data([pos[0]], [pos[1]])
                histories[i].append(pos.copy())
                hist = np.array(histories[i])
                trail_lines[i].set_data(hist[:, 0], hist[:, 1])

        return drone_markers + trail_lines + [detected_scatter, title_obj]

    ani = animation.FuncAnimation(fig, update, frames=total_frames,
                                  init_func=init, blit=True, interval=40)
    ani.save(filename, writer="pillow", fps=25)
    plt.close(fig)
    print(f"Animation saved → {filename}")
    return ani


if __name__ == "__main__":
    import os
    os.makedirs("outputs/04_industrial_agriculture/s070_swarm_weeding", exist_ok=True)

    res = run_simulation(seed=42)

    fig1 = plot_field_overview(res)
    fig1.savefig("outputs/04_industrial_agriculture/s070_swarm_weeding/s070_field_overview.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig1)

    fig2 = plot_metrics(res)
    fig2.savefig("outputs/04_industrial_agriculture/s070_swarm_weeding/s070_metrics.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig2)

    make_animation(res, "outputs/04_industrial_agriculture/s070_swarm_weeding/s070_weeding.gif")
    print("Done.")
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Field size | 40 × 40 m |
| Number of drones $N$ | 3 |
| Number of weeds $W$ | 30 |
| Sensor detection radius $r_s$ | 1.5 m |
| Sweep strip spacing $\delta_s$ | 3.0 m ($= 2 r_s$) |
| Detection probability per pass $P_{detect}$ | 0.85 |
| Inspection speed $v$ (domain constant) | 1.0 m/s |
| GPS positioning error $\sigma_{GPS}$ (domain constant) | 0.10 m |
| Treatment hover duration $T_{treat}$ | 2.0 s |
| Hover accuracy threshold $\epsilon_{pos}$ | 0.15 m RMS |
| PID gains $(K_p, K_i, K_d)$ | (2.0, 0.1, 0.5) |
| Steady-state hover RMS (theoretical) | $\approx$ 0.06 m |
| Expected weeds detected | $\approx$ 25.5 / 30 |
| Expected weeds successfully treated | $\approx$ 25.4 / 30 |
| Simulation timestep $\Delta t$ | 0.05 s |
| Assignment algorithms compared | Hungarian, Greedy nearest-drone |
| Reference $P_{success}$ per weed | $\approx$ 0.847 |
| Random seed | 42 |

---

## Expected Output

- **Field overview (2-panel figure)**: top-down $40 \times 40$ m map for each assignment strategy
  side-by-side; sub-strip backgrounds colour-coded per drone; Phase 1 lawnmower paths drawn as
  thin coloured lines; all 30 weeds shown as grey dots; detected weeds highlighted in gold;
  Phase 2 travel routes shown as coloured arrows from drone end-of-strip positions to assigned
  weeds; drone end positions marked as triangles.
- **Metrics figure (3-panel)**: (i) histogram of hover RMS errors across all treated weeds with
  the $\epsilon_{pos} = 0.15$ m threshold and mean annotated; (ii) bar chart comparing total Phase 2
  travel distance for Hungarian vs. greedy assignment with per-bar labels; (iii) pie chart of
  weeding outcome (missed, detected but hover error, successfully treated) with percentages.
- **Animation** (`s070_weeding.gif`): top-down view of the full mission; Phase 1 shows three drones
  sweeping their sub-strips with trails accumulating; detected weeds appear as gold dots when
  triggered; 10-frame pause labelled "Running Hungarian assignment"; Phase 2 shows drones flying to
  their assigned weeds and hovering; frame rate 25 fps.
- **Console metrics reported**:
  - Weeds detected / total (Phase 1)
  - Phase 2 travel distance — Hungarian vs. greedy (m), and ratio
  - Weeds successfully treated / detected (Phase 2)
  - Mean hover RMS error (m) across all treatment hovers

---

## Extensions

1. **Multi-pass detection**: run two detection sweeps with $50\%$ offset between passes so each
   weed is covered twice; model the joint detection probability as $1 - (1 - P_{detect})^2$ and
   measure the improvement in $P_{success}$ vs. the extra flight time cost.
2. **Dynamic re-tasking**: integrate Phases 1 and 2; when a drone finishes its sub-strip early,
   dispatch it immediately to treat nearby detected weeds instead of waiting for all three to
   complete Phase 1; compare mission completion time against the two-phase baseline.
3. **Heterogeneous treatment payloads**: assign different herbicide volumes per weed (based on
   detected weed size from sensor area); model depletion of the drone's tank and add refuelling
   waypoints to the Phase 2 plan; extend the Hungarian formulation to include capacity constraints.
4. **3D canopy penetration**: lift the simulation from 2D to 3D and model crop row height
   variation using a Gaussian random field; drones must descend into the canopy to within
   $h_{treat} = 0.5$ m of detected weeds; add vertical PID to the hover controller.
5. **Persistent weed map with EKF**: fuse multi-pass sensor observations into a per-cell occupancy
   grid using an Extended Kalman Filter; compare map-based assignment (using weed probability
   per cell) against the direct detection-list approach for sparse and dense weed distributions.
6. **Online replanning under missed detections**: after Phase 2, estimate false-negative rate from
   treated/untreated ratio; if the estimate exceeds a threshold, dispatch a targeted re-survey of
   high-probability sub-regions identified from the sweep coverage map.

---

## Related Scenarios

- Prerequisites: [S063 Precision Hover Inspection](S063_precision_hover_inspection.md) (PID hover, GPS noise model), [S067 Crop Row Lawnmower Coverage](S067_crop_row_coverage.md) (boustrophedon sweep, strip assignment), [S068 Multi-Drone Field Mapping](S068_multi_drone_field_mapping.md) (parallel sub-strip division)
- Follow-ups: [S071 Variable-Rate Spraying](S071_variable_rate_spraying.md) (dosage optimisation after detection), [S075 Greenhouse Autonomous Inspection](S075_greenhouse_inspection.md) (3D canopy navigation)
- Algorithmic cross-reference: [S018 Multi-Target Interception](../01_pursuit_evasion/S018_multi_target_interception.md) (Hungarian algorithm for multi-target assignment), [S040 Fleet Load Balancing](../02_logistics_delivery/S040_fleet_load_balancing.md) (equitable task distribution), [S042 Missing Person Localization](../03_environmental_sar/S042_missing_person.md) (Bernoulli sensor model, coverage sweep)
