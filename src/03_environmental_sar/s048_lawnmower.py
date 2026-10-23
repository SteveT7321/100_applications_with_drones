"""
S048 Full-Area Coverage Scan (Lawnmower)
=========================================
A single drone surveys a 200x200 m rectangular search area using a boustrophedon
(lawnmower) pattern at a fixed altitude. The drone carries a nadir-pointing sensor
with a circular footprint of radius r_sensor = 5 m. Three strip-width configurations
are compared: under-lap (d=12 m, gaps in coverage), optimal (d=10 m, exact full
coverage), and over-lap (d=7 m, redundant re-scanning). Key metrics — coverage rate,
total path length, energy consumed, and mean overlap — quantify the trade-off between
coverage completeness and energy efficiency.

Usage:
    conda activate drones
    python src/03_environmental_sar/s048_lawnmower.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.colors as mcolors

# ── Parameters ─────────────────────────────────────────────────────────────────
AREA_W    = 200.0   # m  — cross-track width of search area
AREA_H    = 200.0   # m  — along-track height of search area
R_SENSOR  = 5.0     # m  — sensor footprint radius
V_CRUISE  = 10.0    # m/s — cruise speed
P_CRUISE  = 150.0   # W  — motor power during level cruise
P_HOVER   = 200.0   # W  — elevated motor power during heading reversals
T_TURN    = 4.0     # s  — time to execute one 180° heading reversal
DT        = 0.5     # s  — position update step (footprint is swept between steps)
FINE_RES  = 0.2     # m  — cell size of the fine-grid cross-check of the overlap metrics
ANIM_MAX_FRAMES = 150  # GIF frame budget

# Three comparison configurations: (label, overlap_ratio rho = 1 - d / (2 r_s), strip_d_m)
CONFIGS = [
    ("Under-lap\n(d=12 m, 2 m gaps)",   -0.20, 12.0),
    ("Optimal\n(d=10 m, 0% overlap)",   0.0,  10.0),
    ("Over-lap\n(d=7 m, 30% overlap)",  0.30,  7.0),
]

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's048_lawnmower',
)

# ── Helpers ────────────────────────────────────────────────────────────────────

def generate_lawnmower_waypoints(area_w, area_h, strip_d, x0=0.0, y0=0.0):
    """Return ordered list of (x, y) waypoints for a boustrophedon scan.

    Strips run along the x (along-track) direction.
    The y axis is the cross-track direction.
    """
    n_strips = int(np.ceil(area_w / strip_d))
    waypoints = []
    for i in range(n_strips):
        y_centre = y0 + (i + 0.5) * strip_d
        if i % 2 == 0:    # left to right
            waypoints.append((x0,          y_centre))
            waypoints.append((x0 + area_h, y_centre))
        else:              # right to left
            waypoints.append((x0 + area_h, y_centre))
            waypoints.append((x0,          y_centre))
    return waypoints, n_strips


def _capsule_mask(cx, cy, p0, p1, r):
    """Cells whose centre lies within r of the segment p0-p1 (swept footprint)."""
    seg = p1 - p0
    L2 = float(seg @ seg)
    if L2 < 1e-12:
        t = np.zeros_like(cx)
    else:
        t = np.clip(((cx - p0[0]) * seg[0] + (cy - p0[1]) * seg[1]) / L2, 0.0, 1.0)
    dx = cx - (p0[0] + t * seg[0])
    dy = cy - (p0[1] + t * seg[1])
    return dx * dx + dy * dy <= r * r


def simulate_coverage(waypoints, area_w, area_h, r_sensor, dt, v_cruise,
                      snapshot_every=None):
    """Fly the waypoint sequence at v_cruise and sweep the sensor footprint.

    The drone position is advanced every dt; between two consecutive
    positions the footprint is swept continuously (all cells within r_sensor
    of the connecting segment are marked), so the coverage result does not
    depend on dt.  Each straight leg between two waypoints (scan strip or
    turn leg) counts as one *pass*; ``passes[c]`` is the number of distinct
    passes whose swept footprint contains cell c.

    Returns:
        trajectory : (T, 2) drone positions
        passes     : (area_w, area_h) int32 distinct-pass count per cell
        dist_log   : (T,) cumulative distance flown
        cov_log    : (T,) cumulative coverage (%) of the area
        snaps      : list of (step index, pass-count-so-far) if snapshot_every
    """
    xs = np.arange(0.5, area_h, 1.0)    # along-track cell centres
    ys = np.arange(0.5, area_w, 1.0)    # cross-track cell centres
    cx, cy = np.meshgrid(xs, ys)
    passes = np.zeros(cx.shape, dtype=np.int32)
    covered = np.zeros(cx.shape, dtype=bool)

    pos = np.array(waypoints[0], dtype=float)
    covered |= _capsule_mask(cx, cy, pos, pos, r_sensor)
    trajectory, dist_log = [pos.copy()], [0.0]
    cov_log = [100.0 * covered.mean()]
    snaps = []
    cum_dist = 0.0
    for wp in waypoints[1:]:
        target = np.array(wp, dtype=float)
        dist = float(np.linalg.norm(target - pos))
        if dist < 1e-9:
            continue
        unit = (target - pos) / dist
        steps = max(1, int(np.ceil(dist / (v_cruise * dt))))
        step_dist = dist / steps
        this_pass = np.zeros(cx.shape, dtype=bool)
        for _ in range(steps):
            new = pos + unit * step_dist
            m = _capsule_mask(cx, cy, pos, new, r_sensor)
            this_pass |= m
            covered |= m
            pos = new
            cum_dist += step_dist
            trajectory.append(pos.copy())
            dist_log.append(cum_dist)
            cov_log.append(100.0 * covered.mean())
            if snapshot_every and (len(trajectory) - 1) % snapshot_every == 0:
                snaps.append((len(trajectory) - 1, passes + this_pass))
        passes += this_pass
    if snapshot_every and snaps[-1][0] != len(trajectory) - 1:
        snaps.append((len(trajectory) - 1, passes.copy()))
    return (np.array(trajectory), passes, np.array(dist_log),
            np.array(cov_log), snaps)


def compute_metrics(n_strips, strip_d, area_h, passes, v_cruise=V_CRUISE,
                    p_cruise=P_CRUISE, p_hover=P_HOVER, t_turn=T_TURN):
    """Path length, energy, coverage rate and overlap metrics."""
    L = n_strips * area_h + (n_strips - 1) * strip_d
    E = (p_cruise * (L / v_cruise)
         + (p_hover - p_cruise) * t_turn * (n_strips - 1))
    cov = 100.0 * np.mean(passes > 0)
    mean_ov = float(np.mean(passes))                 # card: sum n / (W H)
    mean_ov_scanned = float(passes[passes > 0].mean())
    multi = 100.0 * np.mean(passes >= 2)
    return L, E, cov, mean_ov, mean_ov_scanned, multi


def fine_grid_metrics(waypoints, area_w, area_h, r_sensor, res=FINE_RES):
    """Coverage / distinct-pass metrics on a finer res x res grid.  Each straight
    leg is swept as one capsule (exact for straight-line flight), so this is
    cheap even for a 0.2 m grid.  Returns (coverage %, mean passes, % cells >= 2)."""
    xs = np.arange(res / 2, area_h, res)
    ys = np.arange(res / 2, area_w, res)
    cx, cy = np.meshgrid(xs, ys)
    passes = np.zeros(cx.shape, dtype=np.int16)
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        passes += _capsule_mask(cx, cy, np.array(a, float), np.array(b, float), r_sensor)
    return (100.0 * np.mean(passes > 0), float(np.mean(passes)),
            100.0 * np.mean(passes >= 2))


# ── Simulation ─────────────────────────────────────────────────────────────────

def run_simulation():
    """Run lawnmower simulation for all three configurations."""
    results = []
    for label, rho, strip_d in CONFIGS:
        waypoints, n_strips = generate_lawnmower_waypoints(
            AREA_W, AREA_H, strip_d)
        traj, grid, dist_log, cov_log, _ = simulate_coverage(
            waypoints, AREA_W, AREA_H, R_SENSOR, DT, V_CRUISE)
        L, E, cov, mean_ov, mean_ov_sc, multi = compute_metrics(
            n_strips, strip_d, AREA_H, grid)
        results.append({
            "label":          label,
            "strip_d":        strip_d,
            "n_strips":       n_strips,
            "waypoints":      waypoints,
            "trajectory":     traj,
            "grid":           grid,
            "dist_log":       dist_log,
            "cov_log":        cov_log,
            "path_length_m":  L,
            "energy_J":       E,
            "coverage_pct":   cov,
            "mean_overlap":   mean_ov,
            "mean_overlap_scanned": mean_ov_sc,
            "multi_pass_pct": multi,
            "fine": fine_grid_metrics(waypoints, AREA_W, AREA_H, R_SENSOR),
            "flight_time_s":  L / V_CRUISE + T_TURN * (n_strips - 1),
        })
        short_label = label.replace("\n", " ")
        print(f"{short_label}")
        print(f"  Strips: {n_strips}  |  Path: {L:.0f} m  |  Energy: {E/1000:.2f} kJ")
        print(f"  Coverage: {cov:.2f}%  |  uncovered cells: {int(np.sum(grid == 0))}")
        print(f"  Mean passes per cell (all cells): {mean_ov:.3f}  |  "
              f"per scanned cell: {mean_ov_sc:.3f}  |  cells seen by >=2 passes: {multi:.2f}%")
        fc, fm, fmu = results[-1]["fine"]
        print(f"  {FINE_RES} m grid check: coverage {fc:.2f}%  |  mean passes {fm:.3f}  |  "
              f"cells seen by >=2 passes: {fmu:.2f}%")
        print(f"  Max passes on one cell: {int(grid.max())}  |  "
              f"distance to reach final coverage: {dist_log[np.argmax(cov_log >= cov_log[-1] - 1e-9)]:.0f} m")
        print()
    return results


# ── Plots ───────────────────────────────────────────────────────────────────────

def plot_coverage_maps(results, out_dir):
    """2D heatmap showing scan-count per cell for each configuration."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle("Coverage Maps — Distinct Passes per Cell (swept footprint)",
                 fontsize=14, fontweight='bold')

    # Shared colour scale: 0 = white, 1 = light blue, 2+ = dark blue
    cmap = mcolors.LinearSegmentedColormap.from_list(
        "scan_cmap", ["white", "#74b9ff", "#0984e3", "#2d3436"], N=256)

    for ax, res in zip(axes, results):
        grid = res["grid"]
        traj = res["trajectory"]
        wp   = res["waypoints"]

        # Clip display to max = 3 for readability
        display = np.clip(grid, 0, 3)
        im = ax.imshow(
            display,
            origin='lower',
            extent=[0, AREA_H, 0, AREA_W],
            vmin=0, vmax=3,
            cmap=cmap,
            aspect='equal',
            interpolation='nearest',
        )

        # Trajectory overlay
        ax.plot(traj[:, 0], traj[:, 1], color='#e17055', lw=0.8, alpha=0.9,
                label='Flight path')

        # Draw sensor footprint circles at evenly spaced waypoints
        n_wp = len(wp)
        sample_every = max(1, n_wp // 8)
        for k in range(0, n_wp, sample_every * 2):
            x_wp, y_wp = wp[k]
            circle = plt.Circle((x_wp, y_wp), R_SENSOR,
                                  fill=False, edgecolor='gold',
                                  linewidth=0.8, linestyle='--', alpha=0.7)
            ax.add_patch(circle)

        short = res["label"].replace("\n", " ")
        ax.set_title(
            f"{short}\nCoverage={res['coverage_pct']:.2f}%  "
            f"d={res['strip_d']:.0f} m  N={res['n_strips']}",
            fontsize=9,
        )
        ax.set_xlabel("Along-track x (m)")
        ax.set_ylabel("Cross-track y (m)")
        ax.set_xlim(0, AREA_H)
        ax.set_ylim(0, AREA_W)

        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label("Distinct passes")
        cbar.set_ticks([0, 1, 2, 3])
        cbar.set_ticklabels(['0', '1', '2', '3+'])

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'coverage_maps.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_flight_paths(results, out_dir):
    """Top-down boustrophedon path coloured by strip index."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle("Boustrophedon Flight Paths by Strip Index", fontsize=14, fontweight='bold')

    for ax, res in zip(axes, results):
        wp       = res["waypoints"]
        n_strips = res["n_strips"]
        cmap_s   = plt.get_cmap('tab20', n_strips)

        for i in range(n_strips):
            idx = i * 2
            if idx + 1 >= len(wp):
                break
            x0, y0 = wp[idx]
            x1, y1 = wp[idx + 1]
            color = cmap_s(i)
            # Scan strip (solid)
            ax.plot([x0, x1], [y0, y1], color=color, lw=2.0,
                    label=f"Strip {i}" if i < 5 else "")
            ax.plot(x0, y0, 'o', color=color, ms=4)
            ax.plot(x1, y1, 's', color=color, ms=4)
            # Turn leg to next strip (dashed)
            if idx + 2 < len(wp):
                xn, yn = wp[idx + 2]
                ax.plot([x1, xn], [y1, yn], color='grey',
                        lw=1.0, linestyle='--', alpha=0.6)

        short = res["label"].replace("\n", " ")
        ax.set_title(
            f"{short}\nL={res['path_length_m']:.0f} m  "
            f"E={res['energy_J']/1000:.1f} kJ",
            fontsize=9,
        )
        ax.set_xlabel("Along-track x (m)")
        ax.set_ylabel("Cross-track y (m)")
        ax.set_xlim(-5, AREA_H + 5)
        ax.set_ylim(-5, AREA_W + 5)
        ax.set_aspect('equal')
        ax.add_patch(patches.Rectangle(
            (0, 0), AREA_H, AREA_W,
            linewidth=1.5, edgecolor='black', facecolor='none', linestyle='-'))

    plt.tight_layout(rect=(0, 0, 1, 0.94))
    path = os.path.join(out_dir, 'flight_paths.png')
    plt.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def plot_comparison_bars(results, out_dir):
    """Grouped bar chart: coverage %, path length, energy for each config."""
    labels = [r["label"] for r in results]
    cov    = [r["coverage_pct"]      for r in results]
    paths  = [r["path_length_m"]     for r in results]
    energy = [r["energy_J"] / 1000.0 for r in results]   # kJ

    x = np.arange(3)
    width = 0.25
    colors = ['#e17055', '#00b894', '#0984e3']

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle("Configuration Comparison", fontsize=14, fontweight='bold')

    # Coverage rate
    axes[0].bar(x, cov, color=colors, width=0.5, edgecolor='black', linewidth=0.8)
    axes[0].axhline(100, color='grey', linestyle='--', lw=1.2, label='100% target')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(labels, fontsize=8)
    axes[0].set_ylabel("Coverage Rate (%)")
    axes[0].set_title("Coverage Rate")
    axes[0].set_ylim(0, 110)
    for i, v in enumerate(cov):
        axes[0].text(i, v + 1.0, f"{v:.1f}%", ha='center', va='bottom', fontsize=9)
    axes[0].legend(fontsize=8)

    # Path length
    axes[1].bar(x, paths, color=colors, width=0.5, edgecolor='black', linewidth=0.8)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, fontsize=8)
    axes[1].set_ylabel("Total Path Length (m)")
    axes[1].set_title("Total Path Length")
    for i, v in enumerate(paths):
        axes[1].text(i, v + 20, f"{v:.0f} m", ha='center', va='bottom', fontsize=9)

    # Energy
    axes[2].bar(x, energy, color=colors, width=0.5, edgecolor='black', linewidth=0.8)
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels, fontsize=8)
    axes[2].set_ylabel("Energy (kJ)")
    axes[2].set_title("Energy Consumed")
    for i, v in enumerate(energy):
        axes[2].text(i, v + 0.5, f"{v:.1f} kJ", ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    path = os.path.join(out_dir, 'comparison_bars.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_coverage_vs_distance(results, out_dir):
    """Cumulative coverage rate vs distance flown for each configuration."""
    fig, ax = plt.subplots(figsize=(10, 6))
    colors = ['#e17055', '#00b894', '#0984e3']
    linestyles = ['--', '-', '-.']

    for res, color, ls in zip(results, colors, linestyles):
        short = res["label"].replace("\n", " ")
        ax.plot(res["dist_log"], res["cov_log"],
                color=color, lw=2.0, linestyle=ls, label=short)

    ax.axhline(100, color='black', linestyle=':', lw=1.2, alpha=0.6,
               label='100% target')
    ax.set_xlabel("Distance Flown (m)", fontsize=12)
    ax.set_ylabel("Cumulative Coverage Rate (%)", fontsize=12)
    ax.set_title("Coverage Rate vs Distance Flown", fontsize=13, fontweight='bold')
    ax.set_ylim(0, 105)
    ax.set_xlim(0, max(r["dist_log"][-1] for r in results) * 1.02)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_vs_distance.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_mean_overlap(results, out_dir):
    """Bar chart of mean distinct passes per cell (redundancy metric)."""
    labels     = [r["label"]       for r in results]
    mean_ov    = [r["mean_overlap"] for r in results]
    colors     = ['#e17055', '#00b894', '#0984e3']

    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.arange(len(results))
    bars = ax.bar(x, mean_ov, color=colors, width=0.5,
                  edgecolor='black', linewidth=0.8)
    ax.axhline(1.0, color='grey', linestyle='--', lw=1.2,
               label='1 pass per cell')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel("Mean distinct passes per cell", fontsize=11)
    ax.set_title("Mean Overlap (Redundancy) per Configuration",
                 fontsize=12, fontweight='bold')
    ax.set_ylim(0, max(mean_ov) * 1.3)
    for i, v in enumerate(mean_ov):
        ax.text(i, v + 0.02, f"{v:.3f}×", ha='center', va='bottom', fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(axis='y', alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'mean_overlap.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def save_animation(results, out_dir):
    """Animate the optimal configuration (d = 10 m): drone position and the
    distinct-pass count per cell as it builds up (swept footprint)."""
    import matplotlib.animation as animation

    res = results[1]
    wp = res["waypoints"]
    n_steps = len(res["trajectory"]) - 1
    every = int(np.ceil(n_steps / ANIM_MAX_FRAMES))
    traj, _, dist_log, cov_log, snaps = simulate_coverage(
        wp, AREA_W, AREA_H, R_SENSOR, DT, V_CRUISE, snapshot_every=every)

    cmap = mcolors.LinearSegmentedColormap.from_list(
        "scan_cmap", ["white", "#74b9ff", "#0984e3", "#2d3436"], N=256)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(0, AREA_H); ax.set_ylim(0, AREA_W)
    ax.set_xlabel("Along-track x (m)"); ax.set_ylabel("Cross-track y (m)")
    ax.set_title(f"Optimal Lawnmower Coverage (d = {res['strip_d']:.0f} m)")
    im = ax.imshow(np.clip(snaps[0][1], 0, 3), origin='lower',
                   extent=[0, AREA_H, 0, AREA_W], vmin=0, vmax=3, cmap=cmap,
                   aspect='equal', interpolation='nearest')
    drone_dot, = ax.plot([], [], 'ro', ms=6, zorder=5, label='Drone')
    trail_line, = ax.plot([], [], '-', color='#e17055', lw=1.0, alpha=0.7, zorder=4)
    cov_text = ax.text(0.02, 0.97, '', transform=ax.transAxes, fontsize=9, va='top',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
    cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label='Distinct passes')
    cbar.set_ticks([0, 1, 2, 3]); cbar.set_ticklabels(['0', '1', '2', '3+'])
    ax.legend(loc='lower right', fontsize=8)

    def update(k):
        fi, grid_k = snaps[k]
        im.set_data(np.clip(grid_k, 0, 3))
        drone_dot.set_data([traj[fi, 0]], [traj[fi, 1]])
        t0 = max(0, fi - 100)
        trail_line.set_data(traj[t0:fi + 1, 0], traj[t0:fi + 1, 1])
        cov_text.set_text(f"Coverage: {cov_log[fi]:.1f}%\nDist: {dist_log[fi]:.0f} m")
        return im, drone_dot, trail_line, cov_text

    ani = animation.FuncAnimation(fig, update, frames=len(snaps), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=80)
    plt.close(fig)
    print(f"Saved: {path} ({len(snaps)} frames, every {every} steps of {DT} s)")


# ── Main ────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print("=" * 60)
    print("S048 Full-Area Coverage Scan (Lawnmower)")
    print("=" * 60)
    results = run_simulation()

    # Summary table
    print("-" * 60)
    print(f"{'Config':<30} {'N':>4} {'L (m)':>8} {'E (kJ)':>8} {'Cov%':>7} {'Passes':>8}")
    print("-" * 60)
    for r in results:
        short = r["label"].replace("\n", " ")
        print(f"{short:<30} {r['n_strips']:>4} "
              f"{r['path_length_m']:>8.0f} {r['energy_J']/1000:>8.2f} "
              f"{r['coverage_pct']:>7.2f} {r['mean_overlap']:>8.3f}")
    print("=" * 60)

    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)

    plot_coverage_maps(results, out_dir)
    plot_flight_paths(results, out_dir)
    plot_comparison_bars(results, out_dir)
    plot_coverage_vs_distance(results, out_dir)
    plot_mean_overlap(results, out_dir)
    save_animation(results, out_dir)

    print("\nAll outputs saved to:", out_dir)
