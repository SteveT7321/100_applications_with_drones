"""
S042 Missing Person Localization
=================================
Three drones search a 200x200 m wilderness area for a hiker whose last known
position (LKP) was recorded 2 hours ago. A Gaussian prior (broadened by
Brownian diffusion) represents initial probability of presence. Drones share a
Bayesian belief map updated with binary hit/miss sensor observations; a greedy
max-belief frontier policy navigates them toward the most probable unsearched
cell. Two strategies are compared: greedy and lawnmower sweep. Monte Carlo
trials measure expected detection time distribution.

Usage:
    conda activate drones
    python src/03_environmental_sar/s042_missing_person.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from dataclasses import dataclass

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────
N_DRONES        = 3
AREA_SIZE       = 200.0        # m, square side length
GRID_RES        = 0.5          # m per cell
GRID_N          = int(AREA_SIZE / GRID_RES)   # 400 cells per side

SENSOR_RADIUS   = 2.0          # m
SCAN_HEIGHT     = 10.0         # m (for reference)
DRONE_SPEED     = 5.0          # m/s
TAU_DWELL       = 1.0          # s, sensor dwell at each waypoint

P_DETECT        = 0.90         # P(hit | person in footprint)
P_FALSE_ALARM   = 0.02         # P(hit | person NOT in footprint)

SIGMA_LKP       = 10.0         # m, LKP positional uncertainty
DIFFUSION_D     = 0.02         # m^2/s, Brownian diffusion coefficient
DELTA_T_SEC     = 2.0 * 3600.0  # 7200 s elapsed since LKP

LKP = np.array([100.0, 120.0])  # last known position (m)

MAX_SIM_TIME    = 1800.0       # s, 30-minute hard cutoff
N_MC_TRIALS     = 50           # Monte Carlo trials for box-plot

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's042_missing_person',
)

RNG = np.random.default_rng(0)   # fixed seed for reproducibility

# ── Pre-compute grid cell centres ──────────────
_xs = np.arange(0.5 * GRID_RES, AREA_SIZE, GRID_RES)   # (400,)
_ys = np.arange(0.5 * GRID_RES, AREA_SIZE, GRID_RES)   # (400,)
_XX, _YY = np.meshgrid(_xs, _ys)                        # (400, 400)
CELLS = np.stack([_XX.ravel(), _YY.ravel()], axis=-1)   # (160000, 2)

# ── Helpers ────────────────────────────────────

def build_prior():
    """Gaussian prior centred on LKP, broadened by Brownian diffusion."""
    sigma2 = SIGMA_LKP**2 + 2.0 * DIFFUSION_D * DELTA_T_SEC
    dist2 = np.sum((CELLS - LKP)**2, axis=1)
    log_p = -dist2 / (2.0 * sigma2)
    log_p -= log_p.max()
    p = np.exp(log_p)
    return p / p.sum()   # shape (160000,)


def sensor_footprint(drone_pos):
    """Boolean mask of cells within sensor radius of drone position."""
    dist = np.linalg.norm(CELLS - drone_pos, axis=1)
    return dist <= SENSOR_RADIUS   # (160000,)


def bayes_update(belief, drone_pos, hit):
    """Apply one Bayesian belief update from a binary hit/miss observation."""
    in_fp = sensor_footprint(drone_pos)
    if hit:
        likelihood = np.where(in_fp, P_DETECT, P_FALSE_ALARM)
    else:
        likelihood = np.where(in_fp, 1.0 - P_DETECT, 1.0 - P_FALSE_ALARM)
    posterior = likelihood * belief
    total = posterior.sum()
    if total < 1e-300:
        return belief   # numerical guard
    return posterior / total


def belief_entropy(belief):
    """Shannon entropy of the belief map (nats)."""
    mask = belief > 0
    return -np.sum(belief[mask] * np.log(belief[mask]))


def greedy_next_waypoint(belief, assigned_wps):
    """
    Select the highest-belief cell not redundantly covered.
    Cells within 2*r_s of an already-assigned waypoint are excluded.
    """
    scores = belief.copy()
    for wp in assigned_wps:
        too_close = np.linalg.norm(CELLS - wp, axis=1) < 2.0 * SENSOR_RADIUS
        scores[too_close] = 0.0
    best_idx = np.argmax(scores)
    return CELLS[best_idx].copy()


# ── Lawnmower (boustrophedon) state ────────────

def build_lawnmower_sequence():
    """
    Pre-compute a boustrophedon waypoint sequence that sweeps the 200x200 m
    area with rows spaced 2*r_s apart. Returns array of (N_wp, 2) positions.
    """
    row_spacing = 2.0 * SENSOR_RADIUS   # 4.0 m
    rows = np.arange(SENSOR_RADIUS, AREA_SIZE, row_spacing)
    wps = []
    for i, y in enumerate(rows):
        xs = np.arange(SENSOR_RADIUS, AREA_SIZE, row_spacing)
        if i % 2 == 1:
            xs = xs[::-1]
        for x in xs:
            wps.append([x, y])
    return np.array(wps)


LAWNMOWER_SEQ = build_lawnmower_sequence()


# ── Drone dataclass ────────────────────────────

@dataclass
class Drone:
    idx: int
    pos: np.ndarray
    waypoint: np.ndarray
    t_arrive: float = 0.0   # sim time of arrival at current waypoint
    lm_cursor: int = 0      # lawnmower sequence cursor


# ── Simulation ─────────────────────────────────

def simulate(strategy="greedy", seed=0, record_snapshots=False):
    """
    Run one full search mission.

    Parameters
    ----------
    strategy : "greedy" or "lawnmower"
    seed     : RNG seed (for reproducibility / MC trials)
    record_snapshots : if True, save (time, belief_copy, drone_positions) at
                       t=0, 60, 120, and detection.

    Returns
    -------
    history          : list of (t, entropy) tuples
    detection_time   : float or None
    final_belief     : (160000,) array
    snapshots        : list of (t, belief, [pos, ...]) or [] if not requested
    drone_traj       : dict {drone_idx: [(t, x, y), ...]}
    """
    rng = np.random.default_rng(seed)
    belief = build_prior()

    # Draw true person location from prior
    person_idx = rng.choice(len(CELLS), p=belief)
    person_pos = CELLS[person_idx].copy()

    # Initialise drones spread along the south edge
    start_xs = [40.0, 100.0, 160.0]
    drones = [
        Drone(
            idx=k,
            pos=np.array([start_xs[k], 5.0]),
            waypoint=np.array([start_xs[k], 5.0]),
            lm_cursor=k * (len(LAWNMOWER_SEQ) // N_DRONES),
        )
        for k in range(N_DRONES)
    ]

    sim_time    = 0.0
    history     = []
    snapshots   = []
    detection_time = None
    snap_times  = {0.0, 60.0, 120.0}   # record belief map at these times
    last_snap_t = -1.0

    # Drone trajectories for animation: {idx: [(t, x, y), ...]}
    drone_traj = {k: [(0.0, drones[k].pos[0], drones[k].pos[1])] for k in range(N_DRONES)}

    # Assign initial waypoints
    assigned = []
    for drone in drones:
        if strategy == "greedy":
            wp = greedy_next_waypoint(belief, assigned)
        else:
            wp = LAWNMOWER_SEQ[drone.lm_cursor % len(LAWNMOWER_SEQ)].copy()
            drone.lm_cursor += 1
        dist = np.linalg.norm(wp - drone.pos)
        drone.pos = wp.copy()
        drone.waypoint = wp.copy()
        drone.t_arrive = dist / DRONE_SPEED + TAU_DWELL
        assigned.append(wp)

    # Record initial snapshot
    if record_snapshots:
        snapshots.append((0.0, belief.copy(), [d.pos.copy() for d in drones]))

    while sim_time < MAX_SIM_TIME:
        # Find next drone arrival
        next_t = min(d.t_arrive for d in drones)
        if next_t > MAX_SIM_TIME:
            break
        sim_time = next_t

        # Record snapshots at designated times
        if record_snapshots:
            for st in sorted(snap_times):
                if last_snap_t < st <= sim_time:
                    snapshots.append((st, belief.copy(), [d.pos.copy() for d in drones]))
                    snap_times.discard(st)
            last_snap_t = sim_time

        # Process all drones arriving at this time
        arriving = [d for d in drones if abs(d.t_arrive - sim_time) < 1e-6]

        assigned_wps = [d.waypoint for d in drones if d not in arriving]

        for drone in arriving:
            # Sensor observation
            dist_person = np.linalg.norm(drone.pos - person_pos)
            in_fp = dist_person <= SENSOR_RADIUS
            if in_fp:
                hit = rng.random() < P_DETECT
            else:
                hit = rng.random() < P_FALSE_ALARM

            # Bayesian update
            belief = bayes_update(belief, drone.pos, hit)

            # Termination: confirmed hit inside footprint
            if hit and in_fp:
                detection_time = sim_time
                history.append((sim_time, belief_entropy(belief)))
                if record_snapshots:
                    snapshots.append((sim_time, belief.copy(), [d.pos.copy() for d in drones]))
                return history, detection_time, belief, snapshots, drone_traj

            # Assign next waypoint
            if strategy == "greedy":
                new_wp = greedy_next_waypoint(belief, assigned_wps)
            else:
                new_wp = LAWNMOWER_SEQ[drone.lm_cursor % len(LAWNMOWER_SEQ)].copy()
                drone.lm_cursor += 1

            assigned_wps.append(new_wp)
            dist = np.linalg.norm(new_wp - drone.pos)
            drone.pos = new_wp.copy()
            drone.waypoint = new_wp.copy()
            drone.t_arrive = sim_time + dist / DRONE_SPEED + TAU_DWELL
            drone_traj[drone.idx].append((sim_time, drone.pos[0], drone.pos[1]))

        history.append((sim_time, belief_entropy(belief)))

    return history, detection_time, belief, snapshots, drone_traj


def run_simulation():
    """Run all strategies and Monte Carlo trials."""
    results = {}
    mc_times = {}

    print("Running greedy strategy (seed=0)...")
    hist_g, dt_g, bel_g, snaps_g, traj_g = simulate("greedy", seed=0, record_snapshots=True)
    results["greedy"] = (hist_g, dt_g, bel_g, snaps_g, traj_g)
    status_g = f"{dt_g:.1f} s" if dt_g is not None else "NOT FOUND"
    print(f"  Greedy detection time: {status_g}")

    print("Running lawnmower strategy (seed=0)...")
    hist_l, dt_l, bel_l, snaps_l, traj_l = simulate("lawnmower", seed=0, record_snapshots=True)
    results["lawnmower"] = (hist_l, dt_l, bel_l, snaps_l, traj_l)
    status_l = f"{dt_l:.1f} s" if dt_l is not None else "NOT FOUND"
    print(f"  Lawnmower detection time: {status_l}")

    # Entropy summary
    h0_g = hist_g[0][1] if hist_g else float("nan")
    hf_g = hist_g[-1][1] if hist_g else float("nan")
    h0_l = hist_l[0][1] if hist_l else float("nan")
    hf_l = hist_l[-1][1] if hist_l else float("nan")
    print(f"  Greedy entropy: {h0_g:.3f} -> {hf_g:.3f} nats")
    print(f"  Lawnmower entropy: {h0_l:.3f} -> {hf_l:.3f} nats")

    # Monte Carlo
    print(f"Running {N_MC_TRIALS} Monte Carlo trials per strategy...")
    for strat in ("greedy", "lawnmower"):
        times = []
        for seed in range(N_MC_TRIALS):
            _, dt, _, _, _ = simulate(strat, seed=seed)
            times.append(dt if dt is not None else MAX_SIM_TIME)
        mc_times[strat] = times
        found = sum(1 for t in times if t < MAX_SIM_TIME)
        print(f"  [{strat}] detected {found}/{N_MC_TRIALS}, "
              f"median={np.median(times):.1f} s, "
              f"IQR=[{np.percentile(times, 25):.1f}, {np.percentile(times, 75):.1f}] s")

    return results, mc_times


# ── Plots ──────────────────────────────────────

def plot_prior(out_dir):
    """Prior belief heatmap centred on LKP."""
    prior = build_prior()
    grid = prior.reshape(GRID_N, GRID_N)

    fig, ax = plt.subplots(figsize=(7, 7))
    im = ax.imshow(
        grid, origin="lower",
        extent=[0, AREA_SIZE, 0, AREA_SIZE],
        cmap="hot_r", interpolation="nearest",
    )
    plt.colorbar(im, ax=ax, label="Prior probability P0(x)")
    ax.plot(*LKP, "wx", ms=14, mew=3, label="LKP (100, 120)")
    sigma2 = SIGMA_LKP**2 + 2.0 * DIFFUSION_D * DELTA_T_SEC
    sigma0 = np.sqrt(sigma2)
    circ = plt.Circle(LKP, sigma0, color="white", fill=False, lw=1.5, ls="--",
                      label=f"1-sigma ({sigma0:.1f} m)")
    ax.add_patch(circ)
    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Prior Belief Map: Gaussian + Brownian Diffusion")
    ax.legend(loc="upper right", fontsize=9)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "prior_belief.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_belief_snapshots(results, out_dir):
    """Four-panel belief snapshots at t=0, 60, 120, t_det for greedy strategy."""
    hist, det_time, final_bel, snapshots, _ = results["greedy"]

    # We may have fewer than 4 snapshots if detection happened early
    snap_list = snapshots[:4] if len(snapshots) >= 4 else snapshots
    while len(snap_list) < 4:
        snap_list.append(snap_list[-1])

    fig, axes = plt.subplots(2, 2, figsize=(13, 12))
    titles = [f"t = {s[0]:.0f} s" for s in snap_list]
    titles[-1] = f"t = {snap_list[-1][0]:.0f} s (detection)" if det_time else titles[-1]

    for ax, (t_snap, bel_snap, drone_poses), title in zip(axes.ravel(), snap_list, titles):
        grid = bel_snap.reshape(GRID_N, GRID_N)
        im = ax.imshow(
            grid, origin="lower",
            extent=[0, AREA_SIZE, 0, AREA_SIZE],
            cmap="hot_r", interpolation="nearest",
            vmin=0, vmax=build_prior().max(),
        )
        plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        ax.plot(*LKP, "wx", ms=10, mew=2.5, label="LKP")
        colors = ["cyan", "lime", "yellow"]
        for k, pos in enumerate(drone_poses):
            circ = plt.Circle(pos, SENSOR_RADIUS, color=colors[k], fill=False, lw=1.5)
            ax.add_patch(circ)
            ax.plot(*pos, "^", color=colors[k], ms=8, label=f"D{k}")
        ax.set_xlim(0, AREA_SIZE)
        ax.set_ylim(0, AREA_SIZE)
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")

    fig.suptitle("Belief Map Evolution — Greedy Strategy", fontsize=13, y=1.01)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "belief_snapshots.png")
    plt.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {path}")


def plot_entropy(results, out_dir):
    """Entropy reduction curves for both strategies."""
    colors = {"greedy": "tab:blue", "lawnmower": "tab:orange"}
    fig, ax = plt.subplots(figsize=(9, 4))

    for label, (history, det_time, _, _, _) in results.items():
        ts = [h[0] for h in history]
        hs = [h[1] for h in history]
        ax.plot(ts, hs, color=colors[label], label=label.capitalize(), lw=2)
        if det_time is not None:
            ax.axvline(det_time, color=colors[label], ls="--", lw=1.2,
                       label=f"{label.capitalize()} detection ({det_time:.0f} s)")

    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("Belief entropy H(B) (nats)")
    ax.set_title("Entropy Reduction over Time")
    ax.legend(fontsize=9)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "entropy_reduction.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_mc_boxplot(mc_times, out_dir):
    """Box-plot of detection times from Monte Carlo trials."""
    labels = list(mc_times.keys())
    data = [mc_times[k] for k in labels]

    fig, ax = plt.subplots(figsize=(6, 5))
    bp = ax.boxplot(data, labels=[l.capitalize() for l in labels],
                    patch_artist=True, notch=False)
    colors_box = ["#4c72b0", "#dd8452"]
    for patch, color in zip(bp["boxes"], colors_box):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)

    for i, (label, times) in enumerate(mc_times.items()):
        found = [t for t in times if t < MAX_SIM_TIME]
        median = np.median(times)
        ax.text(i + 1, median + 20,
                f"median={median:.0f}s\n({len(found)}/{len(times)} found)",
                ha="center", va="bottom", fontsize=8)

    ax.axhline(MAX_SIM_TIME, color="red", ls="--", lw=1, label="30-min cutoff")
    ax.set_ylabel("Detection time (s)")
    ax.set_title(f"Detection Time Distribution ({N_MC_TRIALS} MC Trials)")
    ax.legend(fontsize=8)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "mc_boxplot.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def save_animation(results, out_dir):
    """Animate belief map + drone positions for the greedy strategy."""
    import matplotlib.animation as animation

    hist, det_time, _, snapshots, drone_traj = results["greedy"]

    # Build animation frames from drone trajectories
    # Collect all unique times across drones
    all_times = sorted({t for k, tlist in drone_traj.items() for t, _, _ in tlist})
    if det_time is not None:
        all_times = [t for t in all_times if t <= det_time]
    all_times = all_times[:300]   # cap at 300 frames

    # Interpolate belief for each frame using the snapshot nearest in time
    # Build a timeline of belief states: (t, belief)
    belief_timeline = []
    b = build_prior()
    belief_timeline.append((0.0, b.copy()))
    for t_s, bel_s, _ in snapshots:
        if t_s > 0:
            belief_timeline.append((t_s, bel_s.copy()))

    def get_belief_at(t):
        """Return the most recent belief update at or before time t."""
        b_cur = belief_timeline[0][1]
        for tb, bb in belief_timeline:
            if tb <= t:
                b_cur = bb
        return b_cur

    def get_drone_pos_at(t):
        """Return drone positions at time t (last known position before t)."""
        positions = []
        for k in range(N_DRONES):
            tlist = drone_traj[k]
            pos = tlist[0][1:]
            for ti, xi, yi in tlist:
                if ti <= t:
                    pos = (xi, yi)
            positions.append(pos)
        return positions

    fig, ax = plt.subplots(figsize=(7, 7))
    initial_belief = get_belief_at(0.0)
    grid0 = initial_belief.reshape(GRID_N, GRID_N)
    im = ax.imshow(
        grid0, origin="lower",
        extent=[0, AREA_SIZE, 0, AREA_SIZE],
        cmap="hot_r", interpolation="nearest",
        vmin=0, vmax=initial_belief.max(),
        animated=True,
    )
    plt.colorbar(im, ax=ax, label="Belief")
    ax.plot(*LKP, "wx", ms=10, mew=2.5, label="LKP")

    drone_colors = ["cyan", "lime", "yellow"]
    drone_markers = []
    drone_circles = []
    init_pos = get_drone_pos_at(0.0)
    for k in range(N_DRONES):
        mk, = ax.plot(*init_pos[k], "^", color=drone_colors[k], ms=9, zorder=5)
        circ = plt.Circle(init_pos[k], SENSOR_RADIUS,
                          color=drone_colors[k], fill=False, lw=1.5)
        ax.add_patch(circ)
        drone_markers.append(mk)
        drone_circles.append(circ)

    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    title_text = ax.set_title("Greedy Search — t = 0.0 s")

    # Decimation for reasonable GIF size
    step = max(1, len(all_times) // 120)
    frame_times = all_times[::step]

    def update(frame_t):
        bel = get_belief_at(frame_t)
        im.set_data(bel.reshape(GRID_N, GRID_N))
        poses = get_drone_pos_at(frame_t)
        for k in range(N_DRONES):
            drone_markers[k].set_data([poses[k][0]], [poses[k][1]])
            drone_circles[k].set_center(poses[k])
        title_text.set_text(f"Greedy Search — t = {frame_t:.0f} s  |  "
                            f"H = {belief_entropy(bel):.2f} nats")
        return [im, title_text] + drone_markers + drone_circles

    ani = animation.FuncAnimation(
        fig, update, frames=frame_times,
        interval=100, blit=False,
    )

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "animation.gif")
    ani.save(path, writer="pillow", fps=10, dpi=100)
    plt.close()
    print(f"Saved: {path}")


# ── Main ───────────────────────────────────────

if __name__ == "__main__":
    results, mc_times = run_simulation()

    # Print summary metrics
    for strat, (hist, dt, _, _, _) in results.items():
        h0 = hist[0][1] if hist else float("nan")
        hf = hist[-1][1] if hist else float("nan")
        status = f"{dt:.1f} s" if dt is not None else "NOT FOUND"
        print(f"[{strat}] det={status} | H: {h0:.3f}->{hf:.3f} nats")

    for strat, times in mc_times.items():
        found = sum(1 for t in times if t < MAX_SIM_TIME)
        print(f"[MC {strat}] found={found}/{N_MC_TRIALS} "
              f"median={np.median(times):.1f}s "
              f"p25={np.percentile(times, 25):.1f}s "
              f"p75={np.percentile(times, 75):.1f}s")

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_prior(out_dir)
    plot_belief_snapshots(results, out_dir)
    plot_entropy(results, out_dir)
    plot_mc_boxplot(mc_times, out_dir)
    save_animation(results, out_dir)
