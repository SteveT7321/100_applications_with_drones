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

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.colors as mcolors
from matplotlib.collections import LineCollection

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────────
AREA_W    = 200.0   # m  — cross-track width of search area
AREA_H    = 200.0   # m  — along-track height of search area
R_SENSOR  = 5.0     # m  — sensor footprint radius
V_CRUISE  = 10.0    # m/s — cruise speed
P_CRUISE  = 150.0   # W  — motor power during level cruise
P_HOVER   = 200.0   # W  — elevated motor power during heading reversals
T_TURN    = 4.0     # s  — time to execute one 180° heading reversal
DT        = 0.5     # s  — simulation timestep

# Three comparison configurations: (label, overlap_ratio, strip_d_m)
CONFIGS = [
    ("Under-lap\n(d=12 m, 0% overlap)", 0.0,  12.0),
    ("Optimal\n(d=10 m, 0% overlap)",   0.0,  10.0),
    ("Over-lap\n(d=7 m, 30% overlap)",  0.30,  7.0),
]

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's048_lawnmower',
)

RNG = np.random.default_rng(0)

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


def simulate_coverage(waypoints, area_w, area_h, r_sensor, dt, v_cruise,
                      x0=0.0, y0=0.0):
    """Simulate drone flight along waypoints.

    Returns:
        trajectory : (T, 2) array of drone positions
        grid       : (area_w, area_h) int32 scan-count grid
        dist_log   : (T,) cumulative distance flown at each step
        cov_log    : (T,) cumulative coverage fraction (0–100) at each step
    """
    # Discretise area into 1x1 m cells
    xs = np.arange(x0 + 0.5, x0 + area_h + 0.5, 1.0)   # along-track centres
    ys = np.arange(y0 + 0.5, y0 + area_w + 0.5, 1.0)    # cross-track centres
    total_cells = len(xs) * len(ys)
    cx, cy = np.meshgrid(xs, ys)   # shape (area_w, area_h)
    grid = np.zeros_like(cx, dtype=np.int32)

    trajectory = []
    dist_log   = []
    cov_log    = []

    pos       = np.array(waypoints[0], dtype=float)
    cum_dist  = 0.0
    trajectory.append(pos.copy())

    # Initial coverage snapshot
    d2c = np.sqrt((cx - pos[0])**2 + (cy - pos[1])**2)
    grid[d2c <= r_sensor] += 1
    dist_log.append(0.0)
    cov_log.append(100.0 * np.sum(grid > 0) / total_cells)

    for wp in waypoints[1:]:
        target    = np.array(wp, dtype=float)
        direction = target - pos
        dist      = np.linalg.norm(direction)
        if dist < 1e-6:
            continue
        unit  = direction / dist
        steps = max(1, int(np.ceil(dist / (v_cruise * dt))))
        step_dist = dist / steps
        for _ in range(steps):
            pos      += unit * step_dist
            cum_dist += step_dist
            trajectory.append(pos.copy())
            d2c = np.sqrt((cx - pos[0])**2 + (cy - pos[1])**2)
            grid[d2c <= r_sensor] += 1
            dist_log.append(cum_dist)
            cov_log.append(100.0 * np.sum(grid > 0) / total_cells)

    return (np.array(trajectory),
            grid,
            np.array(dist_log),
            np.array(cov_log))


def compute_metrics(n_strips, strip_d, area_h, grid, v_cruise=V_CRUISE,
                    p_cruise=P_CRUISE, p_hover=P_HOVER, t_turn=T_TURN):
    """Compute path length, energy, coverage rate, mean overlap."""
    L      = n_strips * area_h + (n_strips - 1) * strip_d
    E      = (p_cruise * (L / v_cruise)
              + (p_hover - p_cruise) * t_turn * (n_strips - 1))
    total  = grid.size
    cov    = 100.0 * np.sum(grid > 0) / total
    mean_ov = np.sum(grid) / total
    return L, E, cov, mean_ov


# ── Simulation ─────────────────────────────────────────────────────────────────

def run_simulation():
    """Run lawnmower simulation for all three configurations."""
    results = []
    for label, rho, strip_d in CONFIGS:
        waypoints, n_strips = generate_lawnmower_waypoints(
            AREA_W, AREA_H, strip_d)
        traj, grid, dist_log, cov_log = simulate_coverage(
            waypoints, AREA_W, AREA_H, R_SENSOR, DT, V_CRUISE)
        L, E, cov, mean_ov = compute_metrics(
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
        })
        short_label = label.replace("\n", " ")
        print(f"{short_label}")
        print(f"  Strips: {n_strips}  |  Path: {L:.0f} m  |  Energy: {E/1000:.2f} kJ")
        print(f"  Coverage: {cov:.2f}%  |  Mean overlap: {mean_ov:.3f}x")
        print()
    return results


# ── Plots ───────────────────────────────────────────────────────────────────────

def plot_coverage_maps(results, out_dir):
    """2D heatmap showing scan-count per cell for each configuration."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle("Coverage Maps — Scan Count per Cell", fontsize=14, fontweight='bold')

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
            f"{short}\nCoverage={res['coverage_pct']:.1f}%  "
            f"d={res['strip_d']:.0f} m  N={res['n_strips']}",
            fontsize=9,
        )
        ax.set_xlabel("Along-track x (m)")
        ax.set_ylabel("Cross-track y (m)")
        ax.set_xlim(0, AREA_H)
        ax.set_ylim(0, AREA_W)

        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label("Scan count")
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
        cmap_s   = plt.cm.get_cmap('tab20', n_strips)

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

    plt.tight_layout()
    path = os.path.join(out_dir, 'flight_paths.png')
    plt.savefig(path, dpi=150)
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
    """Bar chart of mean scan count per cell (redundancy metric)."""
    labels     = [r["label"]       for r in results]
    mean_ov    = [r["mean_overlap"] for r in results]
    colors     = ['#e17055', '#00b894', '#0984e3']

    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.arange(len(results))
    bars = ax.bar(x, mean_ov, color=colors, width=0.5,
                  edgecolor='black', linewidth=0.8)
    ax.axhline(1.0, color='grey', linestyle='--', lw=1.2,
               label='Ideal (1× overlap)')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel("Mean Scan Count per Cell", fontsize=11)
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
    """Animate the optimal configuration (d=10 m) showing drone progress and
    cumulative coverage heatmap."""
    import matplotlib.animation as animation

    # Use optimal config (index 1)
    res = results[1]
    traj = res["trajectory"]
    wp   = res["waypoints"]

    # Precompute cumulative coverage grid at each frame
    xs = np.arange(0.5, AREA_H + 0.5, 1.0)
    ys = np.arange(0.5, AREA_W + 0.5, 1.0)
    cx, cy = np.meshgrid(xs, ys)
    grid_running = np.zeros_like(cx, dtype=np.int32)

    # Decimate frames for animation size
    step = max(1, len(traj) // 200)
    frame_indices = list(range(0, len(traj), step))
    if frame_indices[-1] != len(traj) - 1:
        frame_indices.append(len(traj) - 1)

    # Build list of grid snapshots at each frame
    snapshots = []
    pos_prev_idx = 0
    for fi in frame_indices:
        for k in range(pos_prev_idx, fi + 1):
            p = traj[k]
            d2c = np.sqrt((cx - p[0])**2 + (cy - p[1])**2)
            grid_running[d2c <= R_SENSOR] += 1
        pos_prev_idx = fi + 1
        snapshots.append(np.clip(grid_running.copy(), 0, 3))

    cmap = mcolors.LinearSegmentedColormap.from_list(
        "scan_cmap", ["white", "#74b9ff", "#0984e3", "#2d3436"], N=256)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect('equal')
    ax.set_xlim(0, AREA_H)
    ax.set_ylim(0, AREA_W)
    ax.set_xlabel("Along-track x (m)")
    ax.set_ylabel("Cross-track y (m)")
    ax.set_title("Optimal Lawnmower Coverage (d=10 m)\nFrames: animated")

    im = ax.imshow(
        snapshots[0],
        origin='lower',
        extent=[0, AREA_H, 0, AREA_W],
        vmin=0, vmax=3,
        cmap=cmap,
        aspect='equal',
        interpolation='nearest',
        animated=True,
    )
    drone_dot, = ax.plot([], [], 'ro', ms=6, zorder=5, label='Drone')
    trail_line, = ax.plot([], [], '-', color='#e17055', lw=1.0, alpha=0.7,
                          zorder=4)
    cov_text = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                       fontsize=9, va='top', color='black',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                                 alpha=0.7))
    plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label='Scan count')
    ax.legend(loc='lower right', fontsize=8)

    def init():
        im.set_data(snapshots[0])
        drone_dot.set_data([], [])
        trail_line.set_data([], [])
        cov_text.set_text('')
        return im, drone_dot, trail_line, cov_text

    def update(frame_num):
        fi = frame_indices[frame_num]
        im.set_data(snapshots[frame_num])
        drone_dot.set_data([traj[fi, 0]], [traj[fi, 1]])
        trail_start = max(0, fi - 100)
        trail_line.set_data(traj[trail_start:fi+1, 0],
                            traj[trail_start:fi+1, 1])
        covered = np.sum(snapshots[frame_num] > 0)
        cov_pct = 100.0 * covered / snapshots[frame_num].size
        dist = res["dist_log"][fi]
        cov_text.set_text(f"Coverage: {cov_pct:.1f}%\nDist: {dist:.0f} m")
        return im, drone_dot, trail_line, cov_text

    ani = animation.FuncAnimation(
        fig, update, frames=len(frame_indices),
        init_func=init, blit=True, interval=50,
    )

    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f"Saved: {path}")


# ── Main ────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print("=" * 60)
    print("S048 Full-Area Coverage Scan (Lawnmower)")
    print("=" * 60)
    results = run_simulation()

    # Summary table
    print("-" * 60)
    print(f"{'Config':<30} {'N':>4} {'L (m)':>8} {'E (kJ)':>8} {'Cov%':>7} {'Overlap':>8}")
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
