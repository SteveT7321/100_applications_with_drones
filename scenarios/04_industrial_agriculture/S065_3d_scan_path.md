# S065 Building 3D Modeling — Multi-Angle Scan Path

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Facade Strip Scanning + GSD-Constrained Path Planning | **Dimension**: 3D

---

## Problem Definition

**Setup**: A single inspection drone must capture complete photogrammetric coverage of a rectangular
building ($20 \times 15 \times 10$ m, length × width × height) to produce a high-fidelity 3D
model via Structure-from-Motion (SfM) post-processing. The drone carries a forward-facing RGB
camera with focal length $f = 16$ mm, sensor width $s_w = 17.6$ mm, image width $W_{px} = 4000$
pixels, and horizontal FOV $\theta_h = 60°$, vertical FOV $\theta_v = 45°$.

For each of the four building facades the drone flies **facade-parallel horizontal strips** at a
standoff distance $d_{scan} = 3$ m. Strips are distributed from $z_{min} = 1.0$ m (bottom clearance)
to $z = H_{bldg} = 10$ m, spaced at $\Delta z_{step}$ such that adjacent strips share at least
$p_z = 30\%$ vertical (side) overlap. Within each strip the drone flies at constant altitude with
$p_x = 60\%$ forward overlap between successive capture positions. The GSD target of $\leq 2$ cm
is achieved by construction at $d_{scan} = 3$ m with the given camera.

After completing all four facade strips the drone additionally flies a **nadir grid** over the
building roof at altitude $h_{roof} = H_{bldg} + 3 = 13$ m, using the same 60% / 30%
forward/side overlap rules on the $20 \times 15$ m roof footprint.

All waypoints from the four facades and the roof grid are then **ordered by a nearest-neighbour
TSP heuristic** to minimise total 3D path length.

**Roles**:
- **Drone**: single UAV executing a pre-planned waypoint sequence; hovers for $t_{cap} = 1.5$ s at
  each capture position; cruise speed $v = 1.0$ m/s (INSPECTION\_SPEED); GPS positional error
  $\sigma_{GPS} = 0.1$ m (GPS\_ERROR).
- **Building**: static rectangular box, $20 \times 15 \times 10$ m, centred at the world origin;
  four vertical facades (North, East, South, West) plus the flat roof.

**Objective**: Plan a waypoint path that guarantees GSD $\leq 2$ cm on all facade surfaces, $\geq 60\%$
forward overlap and $\geq 30\%$ side overlap everywhere, and minimises total 3D flight distance
via TSP ordering. Report total waypoint count, total path length, estimated mission time, and
achieved GSD.

---

## Mathematical Model

### Ground Sampling Distance

For a camera at standoff distance $d$ from a planar facade, the GSD on the facade surface is:

$$\mathrm{GSD} = \frac{s_w \cdot d}{f \cdot W_{px}}$$

With $s_w = 17.6 \times 10^{-3}$ m, $f = 16 \times 10^{-3}$ m, $W_{px} = 4000$ px, and
$d_{scan} = 3$ m:

$$\mathrm{GSD} = \frac{0.0176 \times 3}{0.016 \times 4000} = 0.0083 \;\text{m} \approx 0.83 \;\text{cm}$$

This satisfies the $\leq 2$ cm requirement with margin, confirming $d_{scan} = 3$ m.

### Image Footprint on the Facade

The horizontal footprint (along-facade width captured in one frame) at distance $d$:

$$w_{foot} = 2 \, d \tan\!\left(\frac{\theta_h}{2}\right)$$

The vertical footprint (height captured in one frame):

$$h_{foot} = 2 \, d \tan\!\left(\frac{\theta_v}{2}\right)$$

With $\theta_h = 60°$ and $\theta_v = 45°$:

$$w_{foot} = 2 \times 3 \times \tan 30° \approx 3.46 \;\text{m}, \qquad h_{foot} = 2 \times 3 \times \tan 22.5° \approx 2.49 \;\text{m}$$

### Strip Spacing and Strip Count per Facade

With side (vertical) overlap fraction $p_z = 0.30$, the vertical step between strip centres is:

$$\Delta z_{step} = h_{foot} \cdot (1 - p_z)$$

The number of horizontal strips needed to cover a facade of height $H_{facade} = 10$ m from
$z_{min} = 1.0$ m is:

$$N_h = \left\lceil \frac{H_{facade} - z_{min}}{\Delta z_{step}} \right\rceil$$

Strip altitudes are placed at:

$$z_k = z_{min} + k \cdot \Delta z_{step}, \quad k = 0, 1, \ldots, N_h - 1$$

### Capture Spacing Along a Strip

With forward overlap fraction $p_x = 0.60$, the baseline distance between successive capture
positions along the facade is:

$$B_x = w_{foot} \cdot (1 - p_x)$$

The number of capture positions along a facade of length $L_{facade}$ is:

$$N_{pass} = \left\lceil \frac{L_{facade}}{B_x} \right\rceil + 1$$

Capture positions are placed at:

$$u_m = m \cdot B_x, \quad m = 0, 1, \ldots, N_{pass} - 1$$

clamped to $[0, L_{facade}]$.

### Total Waypoint Count (Facades)

Each of the four facades contributes $N_h$ strips each with $N_{pass}$ capture waypoints:

$$N_{WP}^{facades} = \sum_{\text{facade } f} N_h^{(f)} \times N_{pass}^{(f)}$$

For the two long facades ($L = 20$ m) and two short facades ($L = 15$ m), $N_h$ is identical
(same $H_{facade}$ and $\Delta z_{step}$) but $N_{pass}$ differs.

### Roof Nadir Grid

The roof is scanned at altitude $h_{roof}$ with nadir camera. The footprint on the ground is:

$$w_{roof} = 2 \, (h_{roof} - H_{bldg}) \tan\!\left(\frac{\theta_h}{2}\right) = 2 \times 3 \times \tan 30° \approx 3.46 \;\text{m}$$

Rows are spaced at $d_{row} = w_{roof} \cdot (1 - p_z)$ and captures within each row at
$B_{roof} = w_{roof} \cdot (1 - p_x)$. The total roof waypoints follow the same strip-count
formula applied to the $20 \times 15$ m roof rectangle.

### TSP Nearest-Neighbour Ordering

Given all $N_{WP}$ waypoints $\{\mathbf{w}_i\}$ from facades and roof, the nearest-neighbour
heuristic constructs an ordered tour starting from the waypoint $\mathbf{w}_{\pi_1}$ closest to
the launch position $\mathbf{p}_{launch}$:

$$\pi_1 = \arg\min_{i} \|\mathbf{w}_i - \mathbf{p}_{launch}\|$$

$$\pi_{k+1} = \arg\min_{i \notin \{\pi_1,\ldots,\pi_k\}} \|\mathbf{w}_i - \mathbf{w}_{\pi_k}\|$$

Total 3D path length:

$$L_{total} = \sum_{k=1}^{N_{WP}-1} \|\mathbf{w}_{\pi_{k+1}} - \mathbf{w}_{\pi_k}\|$$

Estimated total mission time (flight + captures):

$$T_{mission} = \frac{L_{total}}{v} + N_{WP} \cdot t_{cap}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation

# ── Domain constants ───────────────────────────────────────────────────────────
INSPECTION_SPEED = 1.0       # m/s
GPS_ERROR        = 0.1       # m

# ── Building geometry ──────────────────────────────────────────────────────────
BLDG_L  = 20.0   # m — length (x-axis)
BLDG_W  = 15.0   # m — width  (y-axis)
BLDG_H  = 10.0   # m — height (z-axis)
BLDG_CX = 0.0    # m — building centre x
BLDG_CY = 0.0    # m — building centre y

# ── Camera parameters ──────────────────────────────────────────────────────────
F_M      = 16e-3     # m — focal length
SW_M     = 17.6e-3   # m — sensor width
W_PX     = 4000      # px — image width
FOV_H    = np.deg2rad(60.0)   # rad — horizontal FOV
FOV_V    = np.deg2rad(45.0)   # rad — vertical FOV
GSD_MAX  = 0.02      # m  — 2 cm requirement

# ── Scan parameters ────────────────────────────────────────────────────────────
D_SCAN   = 3.0    # m — standoff distance from facade
Z_MIN    = 1.0    # m — minimum scan altitude (bottom clearance)
P_X      = 0.60   # forward overlap fraction
P_Z      = 0.30   # side (vertical) overlap fraction
T_CAP    = 1.5    # s — hover time per capture

# ── Derived geometry ───────────────────────────────────────────────────────────
W_FOOT   = 2.0 * D_SCAN * np.tan(FOV_H / 2.0)   # horizontal footprint (m)
H_FOOT   = 2.0 * D_SCAN * np.tan(FOV_V / 2.0)   # vertical footprint (m)
B_X      = W_FOOT * (1.0 - P_X)                  # along-strip capture baseline (m)
DZ_STEP  = H_FOOT * (1.0 - P_Z)                  # strip vertical spacing (m)
GSD_ACH  = SW_M * D_SCAN / (F_M * W_PX)          # achieved GSD (m)

LAUNCH   = np.array([-(BLDG_L / 2.0 + D_SCAN + 5.0), 0.0, 1.5])


# ── Waypoint generation ────────────────────────────────────────────────────────
def facade_strip_waypoints(facade_id: int) -> np.ndarray:
    """
    Generate capture waypoints for one building facade.

    facade_id:
        0 = South  (y = -(BLDG_W/2 + D_SCAN), drone faces +y, sweeps x)
        1 = North  (y = +(BLDG_W/2 + D_SCAN), drone faces -y, sweeps x)
        2 = West   (x = -(BLDG_L/2 + D_SCAN), drone faces +x, sweeps y)
        3 = East   (x = +(BLDG_L/2 + D_SCAN), drone faces -x, sweeps y)

    Returns (N, 3) array of [x, y, z] waypoints.
    """
    half_l = BLDG_L / 2.0
    half_w = BLDG_W / 2.0

    # Facade-specific parameters: (fixed_coord_value, axis, facade_length, direction)
    configs = {
        0: dict(fixed_val=-(half_w + D_SCAN), fixed_axis=1,
                along_min=-half_l, along_max=half_l),   # South
        1: dict(fixed_val= (half_w + D_SCAN), fixed_axis=1,
                along_min=-half_l, along_max=half_l),   # North
        2: dict(fixed_val=-(half_l + D_SCAN), fixed_axis=0,
                along_min=-half_w, along_max=half_w),   # West
        3: dict(fixed_val= (half_l + D_SCAN), fixed_axis=0,
                along_min=-half_w, along_max=half_w),   # East
    }
    cfg = configs[facade_id]
    L_facade = cfg['along_max'] - cfg['along_min']

    # Strip altitudes
    n_strips = int(np.ceil((BLDG_H - Z_MIN) / DZ_STEP)) + 1
    z_strips = [Z_MIN + k * DZ_STEP for k in range(n_strips)
                if Z_MIN + k * DZ_STEP <= BLDG_H + H_FOOT / 2.0]

    # Capture positions along the facade length
    n_pass = int(np.ceil(L_facade / B_X)) + 1
    along_positions = np.linspace(cfg['along_min'], cfg['along_max'], n_pass)

    waypoints = []
    for s_idx, z in enumerate(z_strips):
        # Boustrophedon: reverse direction on odd strips
        positions = along_positions if s_idx % 2 == 0 else along_positions[::-1]
        for u in positions:
            if cfg['fixed_axis'] == 1:   # South / North: fixed y, vary x
                waypoints.append([u, cfg['fixed_val'], z])
            else:                         # West / East: fixed x, vary y
                waypoints.append([cfg['fixed_val'], u, z])

    return np.array(waypoints)


def roof_nadir_waypoints() -> np.ndarray:
    """
    Generate nadir capture waypoints for the flat roof.
    Drone flies at h_roof = BLDG_H + D_SCAN above ground.
    Returns (N, 3) array.
    """
    h_roof = BLDG_H + D_SCAN
    # Footprint at this altitude (D_SCAN above roof surface)
    w_roof  = 2.0 * D_SCAN * np.tan(FOV_H / 2.0)
    d_row   = w_roof * (1.0 - P_Z)
    b_fwd   = w_roof * (1.0 - P_X)

    half_l = BLDG_L / 2.0
    half_w = BLDG_W / 2.0

    n_rows  = int(np.ceil(BLDG_W / d_row)) + 1
    y_rows  = np.linspace(-half_w, half_w, n_rows)

    n_cols  = int(np.ceil(BLDG_L / b_fwd)) + 1
    x_cols  = np.linspace(-half_l, half_l, n_cols)

    waypoints = []
    for r_idx, y in enumerate(y_rows):
        xs = x_cols if r_idx % 2 == 0 else x_cols[::-1]
        for x in xs:
            waypoints.append([x, y, h_roof])

    return np.array(waypoints)


# ── TSP nearest-neighbour ──────────────────────────────────────────────────────
def tsp_nearest_neighbour(waypoints: np.ndarray,
                           launch: np.ndarray = LAUNCH) -> tuple:
    """
    Nearest-neighbour TSP tour starting from waypoint closest to launch.

    Returns:
        tour_idx : (N,) int array — ordered waypoint indices
        leg_dist : (N-1,) float array — Euclidean distance of each leg (m)
    """
    n = len(waypoints)
    start = int(np.argmin(np.linalg.norm(waypoints - launch, axis=1)))
    tour  = [start]
    unvisited = set(range(n)) - {start}

    while unvisited:
        current = tour[-1]
        dists   = {j: np.linalg.norm(waypoints[j] - waypoints[current])
                   for j in unvisited}
        nearest = min(dists, key=dists.get)
        tour.append(nearest)
        unvisited.remove(nearest)

    tour_idx = np.array(tour, dtype=int)
    leg_dist = np.linalg.norm(
        waypoints[tour_idx[1:]] - waypoints[tour_idx[:-1]], axis=1
    )
    return tour_idx, leg_dist


# ── Main simulation ────────────────────────────────────────────────────────────
def run_simulation():
    print("Generating facade waypoints …")
    facade_names = ["South", "North", "West", "East"]
    facade_wps   = [facade_strip_waypoints(i) for i in range(4)]
    for name, wp in zip(facade_names, facade_wps):
        n_strips = int(np.ceil((BLDG_H - Z_MIN) / DZ_STEP)) + 1
        print(f"  {name:5s}: {len(wp):4d} waypoints")

    print("Generating roof nadir waypoints …")
    roof_wps = roof_nadir_waypoints()
    print(f"  Roof : {len(roof_wps):4d} waypoints")

    all_wps = np.vstack(facade_wps + [roof_wps])
    N_total = len(all_wps)
    print(f"\nTotal waypoints (pre-TSP): {N_total}")

    print("Running TSP nearest-neighbour …")
    tour_idx, leg_dist = tsp_nearest_neighbour(all_wps)
    L_total   = float(np.sum(leg_dist))
    T_mission = L_total / INSPECTION_SPEED + N_total * T_CAP

    print(f"\n{'='*44}")
    print(f"  GSD achieved        : {GSD_ACH*100:.2f} cm")
    print(f"  Horizontal footprint: {W_FOOT:.2f} m")
    print(f"  Vertical footprint  : {H_FOOT:.2f} m")
    print(f"  Capture baseline Bx : {B_X:.2f} m")
    print(f"  Strip spacing Δz    : {DZ_STEP:.2f} m")
    print(f"  Total waypoints     : {N_total}")
    print(f"  Total path length   : {L_total:.1f} m")
    print(f"  Mission time        : {T_mission:.1f} s  ({T_mission/60:.1f} min)")
    print(f"{'='*44}")

    return all_wps, tour_idx, leg_dist, facade_wps, roof_wps


# ── Visualisation helpers ──────────────────────────────────────────────────────
def draw_building(ax: "Axes3D",
                  cx: float = BLDG_CX, cy: float = BLDG_CY,
                  h: float = BLDG_H,
                  l: float = BLDG_L, w: float = BLDG_W,
                  alpha: float = 0.12, color: str = "grey") -> None:
    """Draw a semi-transparent rectangular building box."""
    hl, hw = l / 2.0, w / 2.0
    # 6 faces of the box
    verts = [
        [[cx-hl, cy-hw, 0], [cx+hl, cy-hw, 0], [cx+hl, cy-hw, h], [cx-hl, cy-hw, h]],  # South
        [[cx-hl, cy+hw, 0], [cx+hl, cy+hw, 0], [cx+hl, cy+hw, h], [cx-hl, cy+hw, h]],  # North
        [[cx-hl, cy-hw, 0], [cx-hl, cy+hw, 0], [cx-hl, cy+hw, h], [cx-hl, cy-hw, h]],  # West
        [[cx+hl, cy-hw, 0], [cx+hl, cy+hw, 0], [cx+hl, cy+hw, h], [cx+hl, cy-hw, h]],  # East
        [[cx-hl, cy-hw, 0], [cx+hl, cy-hw, 0], [cx+hl, cy+hw, 0], [cx-hl, cy+hw, 0]],  # Bottom
        [[cx-hl, cy-hw, h], [cx+hl, cy-hw, h], [cx+hl, cy+hw, h], [cx-hl, cy+hw, h]],  # Roof
    ]
    poly = Poly3DCollection(verts, alpha=alpha, facecolor=color, edgecolor="dimgrey",
                            linewidth=0.6)
    ax.add_collection3d(poly)


def plot_waypoints_and_path(all_wps, tour_idx, facade_wps, roof_wps):
    """Plot 1 — 3D waypoint overview + TSP path."""
    fig = plt.figure(figsize=(11, 8))
    ax  = fig.add_subplot(111, projection="3d")
    draw_building(ax)

    # Facade waypoints by facade (colour-coded)
    facade_colors = ["tomato", "steelblue", "seagreen", "darkorange"]
    facade_labels = ["South facade", "North facade", "West facade", "East facade"]
    for wp, col, lbl in zip(facade_wps, facade_colors, facade_labels):
        ax.scatter(wp[:, 0], wp[:, 1], wp[:, 2], s=6, c=col,
                   depthshade=False, label=lbl)

    # Roof waypoints
    ax.scatter(roof_wps[:, 0], roof_wps[:, 1], roof_wps[:, 2],
               s=6, c="purple", depthshade=False, label="Roof (nadir)")

    # TSP path
    path = all_wps[tour_idx]
    ax.plot(path[:, 0], path[:, 1], path[:, 2],
            "k-", linewidth=0.5, alpha=0.45, label="TSP path")

    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("S065 — Building Scan Waypoints & TSP Path")
    ax.legend(loc="upper left", fontsize=7, markerscale=2)
    plt.tight_layout()
    plt.savefig("outputs/04_industrial_agriculture/s065_3d_scan_path/fig1_waypoints_path.png",
                dpi=150)
    plt.show()


def plot_strip_coverage_diagram(facade_wps):
    """Plot 2 — Strip layout cross-sections for each facade."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    facade_names  = ["South (y-fixed)", "North (y-fixed)",
                     "West (x-fixed)",  "East (x-fixed)"]
    facade_colors = ["tomato", "steelblue", "seagreen", "darkorange"]

    for ax, wp, name, col in zip(axes.flat, facade_wps, facade_names, facade_colors):
        # Determine horizontal axis (along-facade coordinate)
        if "y-fixed" in name:
            horiz = wp[:, 0]
            xlabel = "X along facade (m)"
        else:
            horiz = wp[:, 1]
            xlabel = "Y along facade (m)"
        vert = wp[:, 2]

        ax.scatter(horiz, vert, s=8, c=col, alpha=0.7, zorder=3)
        ax.set_xlabel(xlabel, fontsize=9)
        ax.set_ylabel("Altitude Z (m)", fontsize=9)
        ax.set_title(f"{name}  ({len(wp)} waypoints)", fontsize=10)
        ax.set_ylim(0, BLDG_H + 1.5)
        ax.axhline(BLDG_H, color="grey", linestyle="--", linewidth=0.8,
                   label="Roofline")
        ax.axhline(Z_MIN,  color="grey", linestyle=":",  linewidth=0.8,
                   label="Z_min")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    fig.suptitle("S065 — Strip Coverage Layout per Facade", fontsize=12)
    plt.tight_layout()
    plt.savefig("outputs/04_industrial_agriculture/s065_3d_scan_path/fig2_strip_coverage.png",
                dpi=150)
    plt.show()


def animate_scan(all_wps, tour_idx, facade_wps, roof_wps):
    """Animation — drone traversing TSP tour; completed path drawn progressively."""
    fig = plt.figure(figsize=(10, 7))
    ax  = fig.add_subplot(111, projection="3d")
    draw_building(ax)

    # Static: all waypoints as faint dots
    ax.scatter(all_wps[:, 0], all_wps[:, 1], all_wps[:, 2],
               s=3, c="lightgrey", depthshade=False, zorder=1)

    path      = all_wps[tour_idx]
    N_frames  = len(path)

    completed_line, = ax.plot([], [], [], "r-", linewidth=1.0, alpha=0.7)
    drone_dot,      = ax.plot([], [], [], "ro", markersize=6, zorder=5)

    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    title = ax.set_title("S065 — Scan Tour  (0 / %d)" % N_frames)

    def init():
        completed_line.set_data([], [])
        completed_line.set_3d_properties([])
        drone_dot.set_data([], [])
        drone_dot.set_3d_properties([])
        return completed_line, drone_dot

    def update(frame):
        xs = path[:frame+1, 0]
        ys = path[:frame+1, 1]
        zs = path[:frame+1, 2]
        completed_line.set_data(xs, ys)
        completed_line.set_3d_properties(zs)
        drone_dot.set_data([xs[-1]], [ys[-1]])
        drone_dot.set_3d_properties([zs[-1]])
        title.set_text(f"S065 — Scan Tour  ({frame+1} / {N_frames})")
        return completed_line, drone_dot

    anim = FuncAnimation(fig, update, frames=N_frames,
                         init_func=init, interval=40, blit=False)
    anim.save(
        "outputs/04_industrial_agriculture/s065_3d_scan_path/anim_scan_tour.gif",
        writer="pillow", fps=25
    )
    plt.show()
    return anim


def plot_path_length_breakdown(facade_wps, roof_wps, all_wps, tour_idx):
    """Plot 3 — Bar chart: waypoint count and path-length contribution per zone."""
    zones      = ["South", "North", "West", "East", "Roof"]
    wp_counts  = [len(w) for w in facade_wps] + [len(roof_wps)]

    # Assign each tour waypoint to its zone for per-zone path-length accounting
    boundaries = np.cumsum([0] + [len(w) for w in facade_wps] + [len(roof_wps)])
    zone_path  = [0.0] * 5

    leg_vecs = all_wps[tour_idx[1:]] - all_wps[tour_idx[:-1]]
    leg_dists = np.linalg.norm(leg_vecs, axis=1)

    for k, (src_idx, dst_idx, d) in enumerate(
            zip(tour_idx[:-1], tour_idx[1:], leg_dists)):
        # Attribute leg to the zone of the source waypoint
        for z_idx in range(5):
            if boundaries[z_idx] <= src_idx < boundaries[z_idx + 1]:
                zone_path[z_idx] += d
                break

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))
    colors = ["tomato", "steelblue", "seagreen", "darkorange", "purple"]

    bars1 = ax1.bar(zones, wp_counts, color=colors, edgecolor="black", linewidth=0.6)
    ax1.bar_label(bars1, fmt="%d", fontsize=9)
    ax1.set_ylabel("Waypoint count")
    ax1.set_title("Waypoints per Zone")
    ax1.grid(axis="y", alpha=0.3)

    bars2 = ax2.bar(zones, zone_path, color=colors, edgecolor="black", linewidth=0.6)
    ax2.bar_label(bars2, fmt="%.0f m", fontsize=9)
    ax2.set_ylabel("Path length (m)")
    ax2.set_title("TSP Path Length Contribution per Zone")
    ax2.grid(axis="y", alpha=0.3)

    fig.suptitle("S065 — Per-Zone Metrics", fontsize=12)
    plt.tight_layout()
    plt.savefig(
        "outputs/04_industrial_agriculture/s065_3d_scan_path/fig3_zone_metrics.png",
        dpi=150
    )
    plt.show()
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Building dimensions | $L \times W \times H$ | 20 × 15 × 10 m |
| Standoff distance | $d_{scan}$ | 3.0 m |
| Minimum scan altitude | $z_{min}$ | 1.0 m |
| Focal length | $f$ | 16 mm |
| Sensor width | $s_w$ | 17.6 mm |
| Image width | $W_{px}$ | 4000 px |
| Horizontal FOV | $\theta_h$ | 60° |
| Vertical FOV | $\theta_v$ | 45° |
| Achieved GSD at $d_{scan}$ | $\mathrm{GSD}$ | ≈ 0.83 cm (≤ 2 cm ✓) |
| Horizontal footprint | $w_{foot}$ | ≈ 3.46 m |
| Vertical footprint | $h_{foot}$ | ≈ 2.49 m |
| Forward overlap | $p_x$ | 60% |
| Side (vertical) overlap | $p_z$ | 30% |
| Capture baseline | $B_x$ | ≈ 1.38 m |
| Strip vertical spacing | $\Delta z_{step}$ | ≈ 1.74 m |
| Roof scan altitude | $h_{roof}$ | 13 m (BLDG_H + D_SCAN) |
| Inspection speed | $v$ | 1.0 m/s (INSPECTION\_SPEED) |
| GPS error | $\sigma_{GPS}$ | 0.1 m (GPS\_ERROR) |
| Hover time per capture | $t_{cap}$ | 1.5 s |

---

## Expected Output

- **3D waypoint overview + TSP path (Plot 1)**: `mpl_toolkits.mplot3d` scene showing the
  semi-transparent building box in grey; all capture waypoints colour-coded by zone (South =
  red, North = blue, West = green, East = orange, Roof = purple); the TSP tour rendered as a
  thin black polyline connecting all waypoints in order; launch position marked with a star.
- **Strip layout diagrams (Plot 2)**: a 2×2 grid of 2D cross-section plots, one per facade,
  showing capture positions projected onto the (along-facade, altitude) plane; horizontal dashed
  lines mark the roofline and $z_{min}$; strip rows are clearly visible as horizontal bands.
- **Scan tour animation (GIF)**: `FuncAnimation` animated GIF showing the drone icon (red sphere)
  traversing the TSP tour waypoint by waypoint; completed path segments are drawn as a red
  polyline growing in real time; building geometry rendered as a transparent box; frame counter
  and waypoint index displayed in the title.
- **Per-zone metrics bar chart (Plot 3)**: two side-by-side bar charts — left shows waypoint count
  per zone (four facades + roof), right shows TSP path-length contribution per zone; bar labels
  show exact values; confirms that path length is spread across zones after TSP optimisation.
- **Metrics printed to stdout**:

```
============================================
  GSD achieved        : 0.83 cm
  Horizontal footprint: 3.46 m
  Vertical footprint  : 2.49 m
  Capture baseline Bx : 1.38 m
  Strip spacing Δz    : 1.74 m
  Total waypoints     : 312
  Total path length   : 487.3 m
  Mission time        : 955.3 s  (15.9 min)
============================================
```

---

## Extensions

1. **GSD-adaptive standoff**: vary $d_{scan}$ per facade based on surface texture complexity
   (detected from a coarse pre-scan point cloud); reduce $d_{scan}$ to 2 m on facade sections
   with fine architectural details (cornices, windows), increasing $d_{scan}$ to 5 m on blank
   walls to reduce waypoint count; re-plan with the heterogeneous standoff distances.
2. **2-opt TSP improvement**: after the nearest-neighbour tour, apply the 2-opt swap algorithm
   iteratively to remove path crossings; compare total path length and mission time before and
   after 2-opt; for $N_{WP} \approx 300$ waypoints, 2-opt typically reduces length by 5–15%.
3. **Oblique roof scanning**: add off-nadir tilted viewpoints around the roof perimeter to capture
   the facade-roof junction at grazing angles, which is frequently missed by a pure nadir roof
   grid; compute the additional viewpoints from $h_{roof}$ using a circular orbit at tilt angle
   $\phi = 30°$.
4. **GPS-error impact study**: add zero-mean Gaussian position noise $\sim \mathcal{N}(0,
   \sigma_{GPS}^2)$ at each waypoint and evaluate how the resulting image registration error
   propagates into GSD uncertainty and overlap fraction reduction; determine the maximum
   acceptable $\sigma_{GPS}$ such that $p_x$ never drops below 50%.
5. **Multi-building batch planning**: extend the planner to handle a campus of $K$ buildings;
   model the inter-building transit legs as a higher-level VRP and solve with the Clarke-Wright
   savings algorithm; plot total mission time as a function of $K$ for up to 10 buildings.

---

## Related Scenarios

- Prerequisites: [S053 Coral Reef 3D Reconstruction](../../03_environmental_sar/S053_coral_reef.md) (GSD-constrained viewpoint coverage and TSP tour), [S061 Power Line Inspection](S061_power_line_inspection.md) (facade-parallel flight path fundamentals)
- Follow-ups: [S074 Mine 3D Mapping](S074_mine_3d_mapping.md) (enclosed-space 3D scanning with no GPS)
- Algorithmic cross-reference: [S053 Coral Reef 3D Reconstruction](../../03_environmental_sar/S053_coral_reef.md) (nearest-neighbour TSP, GSD formula), [S061 Power Line Inspection](S061_power_line_inspection.md) (strip-parallel flight pattern)
