"""
S044 Wall Crack Inspection
===========================
A single inspection drone flies a boustrophedon (serpentine) path along a 50 m x 20 m
reinforced-concrete building facade. The drone maintains a constant standoff distance of
1.5 m from the wall via a PD controller and acquires camera images at each timestep.
Synthetic cracks are placed at known positions; detection is probabilistic, based on
angular resolution of the camera sensor and slant range. The simulation measures wall
coverage fraction, crack detection rate by width bin, and total flight distance.

Usage:
    conda activate drones
    python src/03_environmental_sar/s044_wall_crack.py
"""

import sys
import os
import matplotlib
matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection
from scipy.special import ndtr

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────────
WALL_WIDTH   = 50.0    # m  — y axis (horizontal along wall)
WALL_HEIGHT  = 20.0    # m  — z axis (vertical)
D_STANDOFF   = 1.5     # m  — perpendicular to wall (x axis)
V_CRUISE     = 2.0     # m/s — along-wall cruise speed
FOV_DEG      = 60.0    # degrees — total camera FOV
OVERLAP      = 0.20    # strip overlap fraction
CAM_RES_PX   = 2048    # pixels across FOV
SIGMA_NOISE  = 0.05    # mm — crack width detection noise
Z_MIN        = 0.5     # m  — minimum safe altitude
Z_MAX        = 19.5    # m  — maximum altitude
DT           = 0.1     # s  — simulation timestep
N_CRACKS     = 40      # synthetic crack count

# Derived geometry
ALPHA    = np.radians(FOV_DEG / 2.0)          # camera half-angle (rad)
H_STRIP  = D_STANDOFF * np.tan(ALPHA)          # ~0.866 m  strip half-width
W_STRIP  = 2.0 * H_STRIP                       # ~1.732 m  full strip width
DELTA_Z  = W_STRIP * (1.0 - OVERLAP)           # ~1.386 m  vertical step
N_PASS   = int(np.ceil(WALL_HEIGHT / DELTA_Z)) # 15 passes

# PD controller gains
KP_X = 16.0;  KD_X = 8.0    # standoff (x) control  omega_n = 4 rad/s
KP_Z =  9.0;  KD_Z = 6.0    # height (z) control

SIGMA_X = 0.05  # m  — wall-distance sensor noise std

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's044_wall_crack',
)

RNG = np.random.default_rng(0)   # fixed seed for reproducibility


# ── Helpers ────────────────────────────────────────────────────────────────────

def generate_pass_heights():
    """Compute the centre height of each horizontal scan pass."""
    z_start = Z_MIN + H_STRIP          # centre of first strip above Z_MIN
    z_end   = Z_MAX - H_STRIP          # centre of last strip below Z_MAX
    return list(np.linspace(z_start, z_end, N_PASS))


def build_waypoints(pass_heights):
    """Generate boustrophedon waypoint list as (y, z) tuples."""
    wps = []
    for k, z in enumerate(pass_heights):
        if k % 2 == 0:                 # odd pass (1-indexed): left → right
            wps += [(0.0, z), (WALL_WIDTH, z)]
        else:                          # even pass: right → left
            wps += [(WALL_WIDTH, z), (0.0, z)]
    return wps


def generate_cracks(n, seed=42):
    """Random crack positions and widths on the wall face."""
    rng = np.random.default_rng(seed)
    y = rng.uniform(1.0, WALL_WIDTH - 1.0, n)
    z = rng.uniform(1.0, WALL_HEIGHT - 1.0, n)
    w = rng.uniform(0.1, 2.0, n)      # crack width in mm
    return np.stack([y, z, w], axis=1)  # shape (n, 3)


def w_min_at_range(r):
    """Minimum detectable crack width (mm) at slant range r (m)."""
    return (2.0 * r * np.tan(ALPHA) / CAM_RES_PX) * 1000.0


def detection_prob(crack_w_mm, slant_range):
    """Probabilistic detection based on angular resolution and sensor noise."""
    w_min = w_min_at_range(slant_range)
    return float(ndtr((crack_w_mm - w_min) / SIGMA_NOISE))


# ── Simulation ─────────────────────────────────────────────────────────────────

def run_simulation():
    """
    Run the boustrophedon wall-scan simulation.

    Returns
    -------
    traj       : (T, 2) array of (y, z) drone positions
    x_history  : (T,)   array of standoff-x positions
    t_history  : (T,)   array of simulation times
    cracks     : (N_CRACKS, 3)  [y, z, width_mm]
    detected   : (N_CRACKS,)    boolean detection flags
    coverage   : float           wall coverage fraction
    detect_rate: float           fraction of cracks detected
    path_len   : float           total flight distance (m)
    pass_coverage : list of floats  cumulative coverage after each pass
    pass_heights  : list of floats  pass centre heights
    """
    rng_det = np.random.default_rng(0)       # separate RNG for detection draws

    pass_heights = generate_pass_heights()
    cracks       = generate_cracks(N_CRACKS)
    waypoints    = build_waypoints(pass_heights)

    # State: [y, z, vy, vz]
    state = np.array([waypoints[0][0], waypoints[0][1], 0.0, 0.0], dtype=float)

    # Standoff x state with PD control noise
    x_pos   = D_STANDOFF
    x_vel   = 0.0

    wp_idx  = 1
    traj    = [state[:2].copy()]
    x_hist  = [x_pos]
    t_hist  = [0.0]
    detected = np.zeros(N_CRACKS, dtype=bool)

    # Coverage grid  (ny x nz booleans)
    GRID_RES = 0.5   # m
    ny = int(WALL_WIDTH  / GRID_RES)
    nz = int(WALL_HEIGHT / GRID_RES)
    covered = np.zeros((ny, nz), dtype=bool)

    # Pass-by-pass coverage checkpoints (waypoint pair index = completed pass)
    pass_coverage = []
    current_pass  = 0
    t             = 0.0

    while wp_idx < len(waypoints):
        wp     = np.array(waypoints[wp_idx])
        to_wp  = wp - state[:2]
        dist   = np.linalg.norm(to_wp)

        if abs(to_wp[0]) <= 0.3 and abs(to_wp[1]) <= 0.3:
            # Check if we just completed a full horizontal pass
            if wp_idx % 2 == 0:          # every 2 waypoints = 1 complete pass
                frac = covered.sum() / (ny * nz)
                pass_coverage.append(frac)
                print(f'  Pass {len(pass_coverage):2d}/{N_PASS} complete — coverage {frac*100:.1f}%')
            wp_idx += 1
            continue

        # ── Velocity command ────────────────────────────────────────────
        dir_y = np.sign(to_wp[0]) if abs(to_wp[0]) > 0.3 else 0.0
        dir_z = np.sign(to_wp[1]) if abs(to_wp[1]) > 0.3 else 0.0
        state[2] = dir_y * V_CRUISE
        state[3] = dir_z * V_CRUISE

        # ── Integrate position ──────────────────────────────────────────
        state[:2] += state[2:] * DT
        state[0]   = np.clip(state[0], 0.0, WALL_WIDTH)
        state[1]   = np.clip(state[1], Z_MIN, Z_MAX)
        t         += DT

        # ── PD standoff controller with noise ──────────────────────────
        noise    = rng_det.normal(0, SIGMA_X)
        x_meas   = x_pos + noise
        ax       = KP_X * (D_STANDOFF - x_meas) - KD_X * x_vel
        x_vel   += ax * DT
        x_pos   += x_vel * DT

        traj.append(state[:2].copy())
        x_hist.append(x_pos)
        t_hist.append(t)

        # ── Coverage update ─────────────────────────────────────────────
        y_pos, z_pos = state[0], state[1]
        j0 = max(0, int((y_pos - H_STRIP) / GRID_RES))
        j1 = min(ny, int((y_pos + H_STRIP) / GRID_RES) + 1)
        k0 = max(0, int((z_pos - H_STRIP) / GRID_RES))
        k1 = min(nz, int((z_pos + H_STRIP) / GRID_RES) + 1)
        covered[j0:j1, k0:k1] = True

        # ── Crack detection (vectorised) ────────────────────────────────
        undetected = np.where(~detected)[0]
        if len(undetected):
            dy = y_pos - cracks[undetected, 0]
            dz = z_pos - cracks[undetected, 1]
            in_f = (np.abs(dy) <= H_STRIP) & (np.abs(dz) <= H_STRIP)
            cands = undetected[in_f]
            if len(cands):
                slant = np.sqrt(dy[in_f]**2 + dz[in_f]**2 + D_STANDOFF**2)
                p = ndtr((cracks[cands, 2] - w_min_at_range(slant)) / SIGMA_NOISE)
                hits = rng_det.random(len(cands)) < p
                detected[cands[hits]] = True

    # Final coverage if last pass wasn't captured
    if len(pass_coverage) < N_PASS:
        pass_coverage.append(covered.sum() / (ny * nz))

    traj     = np.array(traj)
    x_hist   = np.array(x_hist)
    t_hist   = np.array(t_hist)
    coverage = covered.sum() / (ny * nz)
    det_rate = detected.sum() / N_CRACKS
    path_len = np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))

    return (traj, x_hist, t_hist, cracks, detected,
            coverage, det_rate, path_len, pass_coverage, pass_heights)


# ── Plots ───────────────────────────────────────────────────────────────────────

def plot_trajectory(traj, cracks, detected, pass_heights, path_len, out_dir):
    """Boustrophedon trajectory overlaid on wall with crack markers."""
    fig, ax = plt.subplots(figsize=(14, 6))

    # Colour trajectory by pass (segments between waypoints)
    wps    = build_waypoints(pass_heights)
    cmap   = plt.get_cmap('tab20')
    y_arr  = traj[:, 0]
    z_arr  = traj[:, 1]

    ax.plot(y_arr, z_arr, color='steelblue', lw=0.8, alpha=0.7, label='Drone path')

    # Mark pass waypoints and annotate pass numbers
    for k, z in enumerate(pass_heights):
        ax.axhline(z, color='lightgrey', lw=0.5, ls='--')
        ax.text(0.5, z + 0.05, f'P{k+1}', fontsize=6, color='dimgrey')

    # Strip half-width bands (first pass only for legend)
    for k, z in enumerate(pass_heights):
        ax.axhspan(z - H_STRIP, z + H_STRIP, color='lightblue', alpha=0.08)

    # Turnaround markers
    for i, (wy, wz) in enumerate(wps):
        ax.plot(wy, wz, 'k^', ms=4, zorder=5)

    # Cracks
    for ci, crack in enumerate(cracks):
        if detected[ci]:
            ax.plot(crack[0], crack[1], 'r+', ms=8, mew=1.5, zorder=6)
        else:
            ax.plot(crack[0], crack[1], 'o', ms=5, color='grey',
                    alpha=0.5, zorder=5)

    ax.set_xlim(-1, WALL_WIDTH + 1)
    ax.set_ylim(-0.5, WALL_HEIGHT + 0.5)
    ax.set_xlabel('y (m) — along wall', fontsize=11)
    ax.set_ylabel('z (m) — height', fontsize=11)
    ax.set_title(f'Boustrophedon Scan Trajectory  |  Total path: {path_len:.1f} m', fontsize=12)

    red_cross   = mpatches.Patch(color='red',   label='Detected crack')
    grey_circle = mpatches.Patch(color='grey',  label='Undetected crack')
    ax.legend(handles=[red_cross, grey_circle], loc='upper right', fontsize=9)

    ax.set_aspect('equal')
    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectory.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_coverage_map(traj, cracks, detected, out_dir):
    """2D heatmap of wall coverage (visit count per cell) with crack overlays."""
    GRID_RES = 0.5
    ny = int(WALL_WIDTH  / GRID_RES)
    nz = int(WALL_HEIGHT / GRID_RES)
    visit_count = np.zeros((ny, nz), dtype=int)

    for pos in traj:
        j0 = max(0, int((pos[0] - H_STRIP) / GRID_RES))
        j1 = min(ny, int((pos[0] + H_STRIP) / GRID_RES) + 1)
        k0 = max(0, int((pos[1] - H_STRIP) / GRID_RES))
        k1 = min(nz, int((pos[1] + H_STRIP) / GRID_RES) + 1)
        visit_count[j0:j1, k0:k1] += 1

    fig, ax = plt.subplots(figsize=(14, 6))
    im = ax.imshow(
        visit_count.T,
        origin='lower',
        extent=[0, WALL_WIDTH, 0, WALL_HEIGHT],
        cmap='YlGn',
        aspect='auto',
        interpolation='nearest',
    )
    plt.colorbar(im, ax=ax, label='Visit count (image frames)', shrink=0.8)

    # Crack overlays
    for ci, crack in enumerate(cracks):
        if detected[ci]:
            ax.plot(crack[0], crack[1], 'r+', ms=10, mew=2, zorder=5)
        else:
            size = 4 + crack[2] * 4   # scale marker by crack width
            ax.plot(crack[0], crack[1], 'o', ms=size,
                    color='dimgrey', alpha=0.6, zorder=5)

    ax.set_xlabel('y (m) — along wall', fontsize=11)
    ax.set_ylabel('z (m) — height', fontsize=11)
    ax.set_title('Wall Coverage Map (image footprint visit count)', fontsize=12)

    red_cross   = mpatches.Patch(color='red',    label='Detected crack')
    grey_circle = mpatches.Patch(color='dimgrey', label='Undetected (size ~ width)')
    ax.legend(handles=[red_cross, grey_circle], loc='upper right', fontsize=9)

    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'coverage_map.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_standoff_error(x_hist, t_hist, out_dir):
    """PD standoff controller error vs time."""
    error = x_hist - D_STANDOFF
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_hist, error, color='steelblue', lw=0.7, label='Standoff error x(t) - d_standoff')
    ax.axhspan(-SIGMA_X, SIGMA_X, color='orange', alpha=0.25, label=f'±1σ noise ({SIGMA_X} m)')
    ax.axhline(0, color='black', lw=0.8, ls='--')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Standoff error (m)', fontsize=11)
    ax.set_title('PD Standoff Controller Error vs Time', fontsize=12)
    ax.set_ylim(-0.3, 0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'standoff_error.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_detection_curve(cracks, detected, out_dir):
    """Detection probability curve vs crack width, scatter of simulated cracks."""
    w_range = np.linspace(0.01, 2.1, 300)
    slant   = D_STANDOFF                       # fixed slant = standoff
    prob    = ndtr((w_range - w_min_at_range(slant)) / SIGMA_NOISE)

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(w_range, prob, 'k-', lw=2, label='P_det(w) at d_standoff')
    ax.axvline(w_min_at_range(slant), color='grey', ls='--', lw=1.2,
               label=f'w_min = {w_min_at_range(slant):.4f} mm')

    # Scatter simulated cracks with jitter in y for visibility
    rng_jitter = np.random.default_rng(7)
    for ci, crack in enumerate(cracks):
        col = 'red' if detected[ci] else 'grey'
        y_j = detection_prob(crack[2], D_STANDOFF) + rng_jitter.uniform(-0.02, 0.02)
        ax.scatter(crack[2], np.clip(y_j, 0, 1), color=col, s=30, zorder=5, alpha=0.7)

    red_dot  = mpatches.Patch(color='red',  label='Detected crack')
    grey_dot = mpatches.Patch(color='grey', label='Undetected crack')
    ax.legend(handles=[ax.get_lines()[0], ax.get_lines()[1], red_dot, grey_dot],
              fontsize=9, loc='lower right')
    ax.set_xlabel('Crack width (mm)', fontsize=11)
    ax.set_ylabel('Detection probability', fontsize=11)
    ax.set_title('Detection Probability vs Crack Width', fontsize=12)
    ax.set_xlim(0, 2.2)
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'detection_curve.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_coverage_vs_pass(pass_coverage, out_dir):
    """Cumulative coverage fraction after each completed horizontal pass."""
    passes = list(range(1, len(pass_coverage) + 1))
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.bar(passes, [c * 100 for c in pass_coverage], color='steelblue',
           edgecolor='white', width=0.7)
    ax.axhline(99, color='red', ls='--', lw=1.2, label='99% coverage target')
    ax.set_xlabel('Pass number', fontsize=11)
    ax.set_ylabel('Cumulative coverage (%)', fontsize=11)
    ax.set_title('Wall Coverage Fraction vs Pass Number', fontsize=12)
    ax.set_ylim(0, 105)
    ax.set_xticks(passes)
    ax.legend(fontsize=9)
    ax.grid(True, axis='y', alpha=0.3)
    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'coverage_vs_pass.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_detection_by_width_bin(cracks, detected, out_dir):
    """Stacked bar chart of detected vs missed cracks by width bin."""
    bins   = [0.0, 0.5, 1.0, 1.5, 2.0]
    labels = ['0–0.5', '0.5–1.0', '1.0–1.5', '1.5–2.0']
    det_counts  = []
    miss_counts = []
    for lo, hi in zip(bins[:-1], bins[1:]):
        mask      = (cracks[:, 2] >= lo) & (cracks[:, 2] < hi)
        det_counts.append(int(detected[mask].sum()))
        miss_counts.append(int((~detected[mask]).sum()))

    x    = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(8, 5))
    bars_d = ax.bar(x, det_counts,  color='steelblue', label='Detected')
    bars_m = ax.bar(x, miss_counts, bottom=det_counts, color='lightcoral', label='Missed')

    for i, (d, m) in enumerate(zip(det_counts, miss_counts)):
        total = d + m
        if total > 0:
            ax.text(i, total + 0.1, str(total), ha='center', va='bottom', fontsize=9)

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=10)
    ax.set_xlabel('Crack width bin (mm)', fontsize=11)
    ax.set_ylabel('Number of cracks', fontsize=11)
    ax.set_title('Detection Rate by Crack Width Bin', fontsize=12)
    ax.legend(fontsize=10)
    ax.set_ylim(0, max(sum(z) for z in zip(det_counts, miss_counts)) + 3)
    ax.grid(True, axis='y', alpha=0.3)
    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'detection_by_bin.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(traj, cracks, detected, pass_heights, out_dir):
    """Animate the drone scanning the wall with crack detection events."""
    import matplotlib.animation as animation

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.set_xlim(-1, WALL_WIDTH + 1)
    ax.set_ylim(-0.5, WALL_HEIGHT + 0.5)
    ax.set_xlabel('y (m)', fontsize=10)
    ax.set_ylabel('z (m)', fontsize=10)
    ax.set_title('S044 Wall Crack Inspection — Boustrophedon Scan', fontsize=11)

    # Static: pass lines
    for z in pass_heights:
        ax.axhline(z, color='lightgrey', lw=0.5, ls='--')

    # Wall boundary
    wall_rect = mpatches.FancyBboxPatch(
        (0, 0), WALL_WIDTH, WALL_HEIGHT,
        boxstyle='square,pad=0', fill=False, edgecolor='black', lw=2
    )
    ax.add_patch(wall_rect)

    # Crack circles (static, coloured at detection)
    crack_markers = []
    for ci, crack in enumerate(cracks):
        marker, = ax.plot(crack[0], crack[1], 'o', ms=5, color='lightgrey',
                          zorder=4, alpha=0.8)
        crack_markers.append(marker)

    # Drone body: camera footprint rectangle
    footprint = mpatches.Rectangle(
        (0 - H_STRIP, 0 - H_STRIP), W_STRIP, W_STRIP,
        color='lightblue', alpha=0.3, zorder=3
    )
    ax.add_patch(footprint)

    # Drone dot
    drone_dot, = ax.plot([], [], 'ro', ms=8, zorder=7, label='Drone')

    # Trail
    trail_line, = ax.plot([], [], color='steelblue', lw=0.6, alpha=0.5)

    # Decimate for animation speed
    step = max(1, len(traj) // 80)
    frames = range(0, len(traj), step)
    detected_so_far = np.zeros(N_CRACKS, dtype=bool)

    def init():
        drone_dot.set_data([], [])
        trail_line.set_data([], [])
        footprint.set_xy((0 - H_STRIP, 0 - H_STRIP))
        return [drone_dot, trail_line, footprint] + crack_markers

    def update(fi):
        i  = fi
        y  = traj[i, 0]
        z  = traj[i, 1]
        drone_dot.set_data([y], [z])
        trail_line.set_data(traj[max(0,i-300):i+1, 0], traj[max(0,i-300):i+1, 1])
        footprint.set_xy((y - H_STRIP, z - H_STRIP))

        # Update crack colours as they get detected
        for ci, crack in enumerate(cracks):
            dy = y - crack[0]
            dz = z - crack[1]
            in_frame = (abs(dy) <= H_STRIP) and (abs(dz) <= H_STRIP)
            if in_frame and detected[ci] and not detected_so_far[ci]:
                detected_so_far[ci] = True
            if detected_so_far[ci]:
                crack_markers[ci].set_color('red')
                crack_markers[ci].set_marker('+')
                crack_markers[ci].set_markersize(10)

        return [drone_dot, trail_line, footprint] + crack_markers

    total_frames = len(frames)

    def update_with_progress(fi):
        frame_num = fi // step
        if frame_num % 50 == 0:
            print(f'  Animation frame {frame_num}/{total_frames}')
        return update(fi)

    ani = animation.FuncAnimation(
        fig, update_with_progress, frames=frames,
        init_func=init, interval=30, blit=True
    )
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.mp4')
    plt.rcParams['animation.ffmpeg_path'] = r'C:\Users\user\anaconda3\envs\drones\Library\bin\ffmpeg.exe'
    writer = animation.FFMpegWriter(fps=20, bitrate=1800)
    ani.save(path, writer=writer, dpi=80)
    plt.close()
    print(f'Saved: {path}')


# ── Main ────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Running S044 Wall Crack Inspection simulation...')
    (traj, x_hist, t_hist, cracks, detected,
     coverage, det_rate, path_len,
     pass_coverage, pass_heights) = run_simulation()

    print(f'\n--- Key Results ---')
    print(f'  Wall dimensions        : {WALL_WIDTH} m x {WALL_HEIGHT} m')
    print(f'  Number of passes       : {N_PASS}')
    print(f'  Strip width            : {W_STRIP:.3f} m  (half-width {H_STRIP:.3f} m)')
    print(f'  Vertical step Δz       : {DELTA_Z:.3f} m  (overlap {OVERLAP*100:.0f}%)')
    print(f'  Total flight distance  : {path_len:.1f} m')
    print(f'  Wall coverage fraction : {coverage*100:.2f}%')
    print(f'  Cracks detected        : {detected.sum()} / {N_CRACKS}  ({det_rate*100:.1f}%)')
    print(f'  Simulation steps       : {len(traj)}')
    print(f'  Simulation time        : {t_hist[-1]:.1f} s  ({t_hist[-1]/60:.1f} min)')
    print(f'  Standoff RMS error     : {np.std(x_hist - D_STANDOFF)*100:.2f} cm')

    out_dir = os.path.normpath(OUTPUT_DIR)
    print(f'Output dir: {out_dir}', flush=True)
    try:
        plot_trajectory(traj, cracks, detected, pass_heights, path_len, out_dir)
        plot_coverage_map(traj, cracks, detected, out_dir)
        plot_standoff_error(x_hist, t_hist, out_dir)
        plot_detection_curve(cracks, detected, out_dir)
        plot_coverage_vs_pass(pass_coverage, out_dir)
        plot_detection_by_width_bin(cracks, detected, out_dir)
        save_animation(traj, cracks, detected, pass_heights, out_dir)
        print('\nAll outputs saved.')
    except Exception as e:
        import traceback
        print(f'\nERROR during plotting: {e}', flush=True)
        traceback.print_exc()
