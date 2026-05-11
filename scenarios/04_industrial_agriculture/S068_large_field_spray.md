# S068 Large-Scale Farmland Cooperative Spraying

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Field Partitioning + Battery-Aware Lawnmower Replanning | **Dimension**: 2D

---

## Problem Definition

**Setup**: A $200 \times 100$ m rectangular farmland must be fully sprayed by $N = 4$ cooperative
drones. The field is partitioned into $N$ equal-area strips of $50 \times 100$ m, one strip per
drone. Each drone departs from a common charging station located at the field edge, flies a
boustrophedon (lawnmower) path within its assigned strip, and delivers pesticide through a
downward-facing spray nozzle of fixed width $w_{spray} = 2.0$ m. All drones cruise at
$v = 3$ m/s.

Battery capacity is limited to $T_{bat} = 600$ s of flight at cruise speed. Before the battery
is exhausted, each drone must return to the charging station to swap batteries
($T_{charge} = 120$ s). The drone then resumes spraying from exactly the waypoint where it
left off. The mission ends when every point in all four strips has been sprayed at least once.

**Roles**:
- **Drones** ($N = 4$): homogeneous agricultural UAVs, each assigned one $50 \times 100$ m strip.
  Each drone maintains a resume waypoint and a remaining-waypoint queue that persists across
  battery swaps.
- **Charging station**: single ground station at position $(0, 50)$ m (left edge, field mid-height).
  Battery swaps take $T_{charge} = 120$ s; the station has unlimited capacity (no queuing in the
  base scenario).
- **Field**: static $200 \times 100$ m rectangle, discretised into $0.5 \times 0.5$ m cells for
  coverage accounting.

**Objective**: Minimise **total mission time** $T_{mission}$ — the time from first take-off to
the moment all field cells have been sprayed — while minimising the total number of battery-swap
trips across all drones.

**Comparison strategies**:
1. **Centralised equal partition** — field split into 4 fixed $50 \times 100$ m strips; no
   load rebalancing between drones; battery-aware replanning within each strip.
2. **Sequential single-drone baseline** — one drone sprays the entire field alone, making as many
   battery swaps as necessary; used to quantify the cooperative speedup.

---

## Mathematical Model

### Strip Geometry and Spray Parameters

The field has total width $W_{total} = 200$ m (x-axis) and depth $D = 100$ m (y-axis). With
$N = 4$ drones, each strip has width:

$$W_{strip} = \frac{W_{total}}{N} = 50 \text{ m}$$

Strip $i$ ($i = 0, 1, 2, 3$) occupies $x \in [i \cdot W_{strip},\; (i+1) \cdot W_{strip}]$.

The spray nozzle covers a swath of width $w_{spray} = 2.0$ m with a lateral overlap of
$\delta_{overlap} = 0.1$ m between adjacent swaths to avoid unsprayed gaps. The effective
centre-to-centre spacing between parallel spray runs is therefore:

$$d_{run} = w_{spray} - \delta_{overlap} = 1.9 \text{ m}$$

The number of spray runs per strip is:

$$N_{runs} = \left\lceil \frac{W_{strip}}{d_{run}} \right\rceil$$

### Battery-Limited Coverage Distance

The return-flight time from any position within strip $i$ to the charging station is bounded by
the worst-case return distance. For conservatism, use the maximum one-way distance from strip $i$
to the station at position $\mathbf{p}_{sta}$:

$$t_{return,i} = \frac{\|\mathbf{p}_{current} - \mathbf{p}_{sta}\|}{v}$$

The battery-aware return trigger fires when elapsed flight time satisfies:

$$t_{elapsed} + t_{return} \geq T_{bat}$$

Once triggered, the drone records its current waypoint index as the **resume index**, flies
directly to the station, waits $T_{charge} = 120$ s, then resumes from the recorded waypoint.

### Maximum Spray Distance per Charge

The maximum along-track distance a drone can spray on a single charge (ignoring return flight)
is estimated as:

$$L_{max} = \left(T_{bat} - t_{return,i}\right) \cdot v$$

where $t_{return,i}$ is the expected return-flight time for strip $i$ (approximated by the
centroid-to-station distance at planning time):

$$t_{return,i} = \frac{\left|(i + 0.5) \cdot W_{strip} - p_{sta,x}\right|}{v}$$

This gives the number of complete spray runs achievable per charge:

$$N_{runs,charge} = \left\lfloor \frac{L_{max}}{D} \right\rfloor$$

where $D = 100$ m is the along-track strip length.

### Mission Time Model

Let drone $i$ make $M_i$ battery-swap trips to complete its strip. The total time for drone $i$
is:

$$T_i = T_{fly,i} + M_i \cdot T_{charge} + T_{ferry,i}$$

where $T_{fly,i} = L_{total,i} / v$ is pure spray flight time, $L_{total,i}$ is the total
path length within the strip (spray runs plus intra-strip transitions), and $T_{ferry,i}$ is
the total time spent flying to/from the station across all $M_i$ trips.

The overall mission time is:

$$T_{mission} = \max_{i \in \{0,\ldots,N-1\}} T_i$$

### Coverage Progress

The field is discretised into $M_x \times M_y$ cells of size $\Delta = 0.5$ m. Cell
$(m, n)$ is marked **sprayed** when the drone's nozzle centre passes within $w_{spray}/2$
of the cell centre. The cumulative coverage fraction at time $t$ is:

$$C(t) = \frac{\bigl|\{(m,n) : \text{cell }(m,n)\text{ sprayed by time }t\}\bigr|}{M_x \cdot M_y}$$

The mission is complete when $C(T_{mission}) = 1.0$ (all cells sprayed).

### Total Battery Trips

The total number of battery-swap trips across all drones is:

$$N_{trips} = \sum_{i=0}^{N-1} M_i$$

A lower $N_{trips}$ is preferable because each swap adds $T_{charge} = 120$ s of idle time
and a ferry round-trip penalty.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

# ── Field and spray constants ────────────────────────────────────────────────
FIELD_W      = 200.0    # m — field width  (x-axis)
FIELD_D      = 100.0    # m — field depth  (y-axis)
N_DRONES     = 4        # number of cooperative drones
SPRAY_WIDTH  = 2.0      # m — nozzle swath width
SPRAY_OVERLAP= 0.1      # m — lateral overlap between adjacent runs
SPEED        = 3.0      # m/s — cruise speed
T_BAT        = 600.0    # s — battery life at cruise speed
T_CHARGE     = 120.0    # s — battery swap time at station
STATION_POS  = np.array([0.0, 50.0])  # charging station (left edge, mid-height)
CELL_SIZE    = 0.5      # m — coverage grid resolution
DT           = 0.2      # s — simulation timestep

# ── Derived constants ────────────────────────────────────────────────────────
STRIP_W      = FIELD_W / N_DRONES          # 50 m per strip
RUN_SPACING  = SPRAY_WIDTH - SPRAY_OVERLAP  # 1.9 m centre-to-centre
N_RUNS       = int(np.ceil(STRIP_W / RUN_SPACING))  # runs per strip

Mx = int(FIELD_W / CELL_SIZE)   # coverage grid cells in x
My = int(FIELD_D / CELL_SIZE)   # coverage grid cells in y


def generate_strip_waypoints(strip_idx):
    """Return ordered (x, y) waypoints for a boustrophedon sweep of strip i."""
    x_start = strip_idx * STRIP_W
    waypoints = []
    for r in range(N_RUNS):
        x_centre = x_start + (r + 0.5) * RUN_SPACING
        x_centre = min(x_centre, x_start + STRIP_W - SPRAY_WIDTH / 2)
        if r % 2 == 0:
            waypoints.append(np.array([x_centre, 0.0]))
            waypoints.append(np.array([x_centre, FIELD_D]))
        else:
            waypoints.append(np.array([x_centre, FIELD_D]))
            waypoints.append(np.array([x_centre, 0.0]))
    return waypoints


class DroneAgent:
    """Battery-aware agricultural drone that resumes after each swap."""

    def __init__(self, idx):
        self.idx         = idx
        self.pos         = STATION_POS.copy()
        self.t_elapsed   = 0.0    # flight time on current charge
        self.t_total     = 0.0    # wall-clock time including charges/ferry
        self.trips       = 0      # number of completed battery swaps
        self.charging    = False
        self.charge_until= 0.0
        self.returning   = False
        self.done        = False

        # Pre-generate full waypoint queue; pop from front as drone advances
        full_wpts = generate_strip_waypoints(idx)
        # Prepend ferry waypoint from station to first spray point
        self.waypoints   = full_wpts.copy()
        self.resume_idx  = 0
        self.history     = [STATION_POS.copy()]
        self.trip_log    = []      # (t_depart, t_return) pairs

    def return_time(self):
        """Estimated seconds to fly from current position back to station."""
        return np.linalg.norm(self.pos - STATION_POS) / SPEED

    def step(self, dt, coverage_grid):
        """Advance simulation by dt seconds. Update position and coverage grid."""
        if self.done:
            return

        # ── Charging phase ────────────────────────────────────────────────
        if self.charging:
            if self.t_total >= self.charge_until:
                self.charging    = False
                self.t_elapsed   = 0.0
                self.returning   = False
                # Resume from stored waypoint index
            else:
                self.t_total += dt
            return

        # ── Return-to-station phase ───────────────────────────────────────
        if self.returning:
            target = STATION_POS
            diff   = target - self.pos
            dist   = np.linalg.norm(diff)
            move   = SPEED * dt
            if dist <= move:
                self.pos         = target.copy()
                self.t_total    += dist / SPEED
                self.t_elapsed  += dist / SPEED
                self.trips      += 1
                self.charge_until = self.t_total + T_CHARGE
                self.charging    = True
            else:
                step_vec         = (diff / dist) * move
                self.pos        += step_vec
                self.t_total    += dt
                self.t_elapsed  += dt
            self.history.append(self.pos.copy())
            return

        # ── Spray phase ───────────────────────────────────────────────────
        if self.resume_idx >= len(self.waypoints):
            self.done = True
            return

        target = self.waypoints[self.resume_idx]
        diff   = target - self.pos
        dist   = np.linalg.norm(diff)
        move   = SPEED * dt

        # Check battery trigger before moving
        if self.t_elapsed + self.return_time() >= T_BAT - 1e-6:
            self.returning = True
            return

        if dist <= move:
            self.pos         = target.copy()
            self.t_total    += dist / SPEED
            self.t_elapsed  += dist / SPEED
            self.resume_idx += 1
        else:
            step_vec         = (diff / dist) * move
            self.pos        += step_vec
            self.t_total    += dt
            self.t_elapsed  += dt

        self.history.append(self.pos.copy())
        _mark_coverage(self.pos, coverage_grid)


def _mark_coverage(pos, grid):
    """Mark all cells within SPRAY_WIDTH/2 of pos as sprayed."""
    x_lo = max(0, int((pos[0] - SPRAY_WIDTH / 2) / CELL_SIZE))
    x_hi = min(Mx, int((pos[0] + SPRAY_WIDTH / 2) / CELL_SIZE) + 1)
    y_lo = max(0, int((pos[1] - SPRAY_WIDTH / 2) / CELL_SIZE))
    y_hi = min(My, int((pos[1] + SPRAY_WIDTH / 2) / CELL_SIZE) + 1)
    grid[x_lo:x_hi, y_lo:y_hi] = True


def run_simulation():
    coverage_grid = np.zeros((Mx, My), dtype=bool)
    drones        = [DroneAgent(i) for i in range(N_DRONES)]

    coverage_log  = []   # (t, coverage_fraction)
    t             = 0.0

    while True:
        all_done = all(d.done for d in drones)
        if all_done:
            break

        for d in drones:
            d.step(DT, coverage_grid)

        cov = coverage_grid.sum() / (Mx * My)
        coverage_log.append((t, cov))
        t += DT

        if t > 10_000.0:  # safety cutoff
            break

    T_mission = max(d.t_total for d in drones)
    trips     = [d.trips for d in drones]
    return drones, coverage_grid, coverage_log, T_mission, trips


def plot_results(drones, coverage_grid, coverage_log, T_mission, trips):
    t_arr = np.array([r[0] for r in coverage_log])
    c_arr = np.array([r[1] for r in coverage_log]) * 100.0

    drone_colors = ['tab:red', 'tab:orange', 'tab:blue', 'tab:purple']

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        f"S068 – Cooperative Farmland Spraying  "
        f"(N={N_DRONES}, T_mission={T_mission:.0f} s)",
        fontsize=13, fontweight='bold'
    )

    # ── Panel 1: Coverage map ─────────────────────────────────────────────
    ax = axes[0, 0]
    ax.imshow(
        coverage_grid.T, origin='lower', cmap='YlGn',
        extent=[0, FIELD_W, 0, FIELD_D], aspect='auto', alpha=0.8
    )
    for i, d in enumerate(drones):
        hist = np.array(d.history)
        ax.plot(hist[:, 0], hist[:, 1], color=drone_colors[i],
                lw=0.8, alpha=0.7, label=f'Drone {i} ({trips[i]} swaps)')
    # Draw strip boundaries
    for i in range(1, N_DRONES):
        ax.axvline(i * STRIP_W, color='k', lw=0.8, ls='--', alpha=0.5)
    ax.scatter(*STATION_POS, marker='*', s=200, color='gold',
               zorder=5, label='Station')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Flight Paths + Coverage Map')
    ax.legend(fontsize=7, loc='upper right')

    # ── Panel 2: Coverage fraction vs time ───────────────────────────────
    ax = axes[0, 1]
    ax.plot(t_arr, c_arr, color='tab:green', lw=2)
    ax.axhline(100.0, color='grey', ls='--', lw=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Coverage (%)')
    ax.set_title('Coverage Progress vs Time')
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.3)

    # ── Panel 3: Per-drone total time breakdown ───────────────────────────
    ax = axes[1, 0]
    drone_labels = [f'Drone {i}' for i in range(N_DRONES)]
    total_times  = [d.t_total for d in drones]
    charge_times = [d.trips * T_CHARGE for d in drones]
    fly_times    = [tt - ct for tt, ct in zip(total_times, charge_times)]

    x_pos = np.arange(N_DRONES)
    ax.bar(x_pos, fly_times,  color=drone_colors, alpha=0.85, label='Flight time')
    ax.bar(x_pos, charge_times, bottom=fly_times, color='lightgrey',
           edgecolor='k', lw=0.5, label='Charging time')
    ax.axhline(T_mission, color='crimson', ls='--', lw=1.2,
               label=f'T_mission = {T_mission:.0f} s')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(drone_labels)
    ax.set_ylabel('Time (s)')
    ax.set_title('Per-Drone Time Breakdown')
    ax.legend(fontsize=8)
    ax.grid(axis='y', alpha=0.3)

    # ── Panel 4: Battery trips per drone (bar) ───────────────────────────
    ax = axes[1, 1]
    ax.bar(x_pos, trips, color=drone_colors, alpha=0.85, edgecolor='k', lw=0.5)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(drone_labels)
    ax.set_ylabel('Battery swap trips')
    ax.set_title(f'Battery Trips per Drone  (total = {sum(trips)})')
    for xi, mi in zip(x_pos, trips):
        ax.text(xi, mi + 0.05, str(mi), ha='center', va='bottom', fontsize=10)
    ax.set_ylim(0, max(trips) + 1.5)
    ax.grid(axis='y', alpha=0.3)

    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s068_large_field_spray/coverage_summary.png',
                dpi=150, bbox_inches='tight')
    plt.show()


def animate_simulation(drones, coverage_grid_final):
    """Generate a top-down animation of all drones spraying the field."""
    # Sub-sample history arrays to a common time axis
    max_frames = min(300, min(len(d.history) for d in drones))
    step = max(1, max(len(d.history) for d in drones) // max_frames)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_xlim(0, FIELD_W)
    ax.set_ylim(0, FIELD_D)
    ax.set_aspect('equal')
    ax.set_title('S068 – Cooperative Spraying Animation')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    # Static elements
    for i in range(1, N_DRONES):
        ax.axvline(i * STRIP_W, color='k', lw=0.8, ls='--', alpha=0.4)
    ax.scatter(*STATION_POS, marker='*', s=250, color='gold', zorder=6)

    drone_colors = ['tab:red', 'tab:orange', 'tab:blue', 'tab:purple']
    traj_lines   = [ax.plot([], [], color=c, lw=0.8, alpha=0.6)[0]
                    for c in drone_colors]
    drone_dots   = [ax.plot([], [], 'o', color=c, ms=6)[0]
                    for c in drone_colors]
    coverage_im  = ax.imshow(
        np.zeros((My, Mx), dtype=float), origin='lower', cmap='YlGn',
        extent=[0, FIELD_W, 0, FIELD_D], aspect='auto', alpha=0.5,
        vmin=0, vmax=1
    )

    grid_accum = np.zeros((Mx, My), dtype=bool)

    def update(frame):
        for i, d in enumerate(drones):
            idx = min(frame * step, len(d.history) - 1)
            hist = np.array(d.history[:idx + 1])
            traj_lines[i].set_data(hist[:, 0], hist[:, 1])
            drone_dots[i].set_data([hist[-1, 0]], [hist[-1, 1]])
            _mark_coverage(hist[-1], grid_accum)
        coverage_im.set_data(grid_accum.T.astype(float))
        return traj_lines + drone_dots + [coverage_im]

    ani = animation.FuncAnimation(
        fig, update, frames=max_frames, interval=50, blit=True
    )
    ani.save(
        'outputs/04_industrial_agriculture/s068_large_field_spray/spray_animation.gif',
        writer='pillow', fps=20
    )
    plt.close(fig)
    return ani


if __name__ == '__main__':
    import os
    os.makedirs(
        'outputs/04_industrial_agriculture/s068_large_field_spray', exist_ok=True
    )
    drones, coverage_grid, coverage_log, T_mission, trips = run_simulation()
    print(f"Mission complete in {T_mission:.1f} s")
    print(f"Battery swaps per drone: {trips}  (total {sum(trips)})")
    print(f"Final coverage: {coverage_grid.sum() / (Mx * My) * 100:.2f}%")
    plot_results(drones, coverage_grid, coverage_log, T_mission, trips)
    animate_simulation(drones, coverage_grid)
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Field dimensions | $W_{total} \times D$ | 200 × 100 m |
| Number of drones | $N$ | 4 |
| Strip width per drone | $W_{strip}$ | 50 m |
| Spray swath width | $w_{spray}$ | 2.0 m |
| Lateral overlap | $\delta_{overlap}$ | 0.1 m |
| Effective run spacing | $d_{run}$ | 1.9 m |
| Spray runs per strip | $N_{runs}$ | $\lceil 50 / 1.9 \rceil = 27$ |
| Cruise speed | $v$ | 3.0 m/s |
| Battery flight time | $T_{bat}$ | 600 s |
| Battery swap time | $T_{charge}$ | 120 s |
| Charging station position | $\mathbf{p}_{sta}$ | $(0, 50)$ m |
| Coverage grid resolution | $\Delta$ | 0.5 m ($400 \times 200$ cells) |
| Simulation timestep | $\Delta t$ | 0.2 s |
| Safety cutoff | — | 10 000 s |

---

## Expected Output

- **Coverage map + trajectories**: top-down 2D plot of the full $200 \times 100$ m field with a
  green heatmap showing sprayed cells; boustrophedon flight paths for all four drones overlaid in
  distinct colours (red, orange, blue, purple); strip boundaries drawn as dashed vertical lines;
  charging station marked with a gold star.
- **Coverage progress curve**: $C(t)$ (percentage) plotted against wall-clock time $t$ for the
  cooperative 4-drone strategy; a horizontal dashed line at 100%; the curve should reach full
  coverage before the single-drone baseline.
- **Per-drone time breakdown**: stacked bar chart showing flight time and accumulated charging
  idle time for each drone; a horizontal dashed line marking $T_{mission}$; the bottleneck drone
  (longest total time) determines mission completion.
- **Battery trips bar chart**: number of battery-swap trips per drone; total trip count annotated
  above each bar; used to assess charging-station load.
- **Animation (GIF)**: real-time top-down view of all four drones flying their lawnmower patterns;
  sprayed cells fill in progressively on the green heatmap; drone icons move along their paths;
  strip boundaries visible throughout.
- **Console metrics** (printed at run end):
  - Mission complete time $T_{mission}$ (s)
  - Battery swaps per drone and total
  - Final field coverage percentage (should equal 100%)

---

## Extensions

1. **Unequal strip assignment**: when drones have different battery capacities or start from
   different positions, partition the field into strips of unequal width proportional to each
   drone's battery-limited range; formulate as an integer programming problem minimising
   $T_{mission}$.
2. **Shared charging queue**: add a second drone per charging station slot; implement a FIFO or
   priority queue (battery-lowest-first) and measure queuing delay; compare with a distributed
   charging topology where each drone has its own station at the edge of its strip.
3. **Wind-drift compensation**: model a constant crosswind $v_{wind}$ that displaces the spray
   plume laterally; adjust the planned run spacing $d_{run}$ as a function of wind speed and
   direction to maintain the target overlap fraction.
4. **Non-rectangular field geometry**: replace the $200 \times 100$ m rectangle with an arbitrary
   convex polygon (e.g., an L-shaped or trapezoidal farm boundary); use a boustrophedon
   decomposition algorithm to partition the polygon into monotone sub-regions, then assign
   sub-regions to drones using the Hungarian algorithm.
5. **Variable spray-rate control**: allow each drone to modulate spray flow rate based on crop
   density (sensed via a downward-facing NDVI camera); formulate a rate-vs-speed trade-off and
   show that slowing down in dense regions increases battery consumption; update $t_{return}$
   trigger accordingly.
6. **Multi-chemical zoning**: divide the field into zones requiring different agrochemicals;
   drones must return to swap both battery and chemical tank when crossing zone boundaries;
   model the additional swap overhead and minimise total number of combined battery + chemical
   swaps.

---

## Related Scenarios

- Prerequisites: [S063 Orchard Row Spraying](S063_orchard_row_spraying.md), [S067 Spray Overlap Optimisation](S067_spray_overlap_optimization.md)
- Follow-ups: [S070 Precision Variable-Rate Spraying](S070_precision_variable_rate_spraying.md) (sensor-driven spray control)
- Algorithmic cross-reference: [S048 Full-Area Coverage Scan (Lawnmower)](../../03_environmental_sar/S048_lawnmower_coverage.md) (boustrophedon geometry), [S049 Dynamic Zone Assignment](../../03_environmental_sar/S049_dynamic_zone.md) (multi-drone zone partitioning), [S032 Charging Queue](../../02_logistics_delivery/S032_charging_queue.md) (battery swap scheduling)
