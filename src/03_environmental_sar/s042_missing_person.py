"""
S042 Missing Person Localization
=================================
Three drones search a 200x200 m wilderness area for a hiker whose last known
position (LKP) was recorded 2 hours ago. A Gaussian prior (broadened by
Brownian diffusion, sigma0 = sqrt(10^2 + 2*0.02*7200) = 19.7 m) represents the
initial probability of presence. Drones share a Bayesian belief map updated
with binary hit/miss observations (Pd = 0.90, Pfa = 0.02) taken after a 1 s
dwell at each waypoint. Drones fly kinematically (5 m/s straight legs).

Three strategies are compared under identical conditions (same start
positions, speed, sensor, dwell, and the same person positions per trial):
  1. Greedy max-belief   — fly to the highest-belief cell (excluding cells
                            within 2 r_s of other drones' targets).
  2. Lawnmower           — boustrophedon sweep of the LKP +/- 2 sigma0 box,
                            split into three strips, 2.5 m waypoint lattice
                            (<= sqrt(2) r_s, so footprints leave no gaps).
  3. Belief-weighted random walk — next waypoint sampled among 2.5 m blocks
                            within 20 m, with probability ~ (block belief)^(1/T)
                            and temperature T annealed from 1.0 to 0.25.

Usage:
    conda activate drones
    python src/03_environmental_sar/s042_missing_person.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── Parameters ─────────────────────────────────
N_DRONES        = 3
AREA_SIZE       = 200.0        # m, square side length
GRID_RES        = 0.5          # m per cell
GRID_N          = int(AREA_SIZE / GRID_RES)   # 400 cells per side

SENSOR_RADIUS   = 2.0          # m
SCAN_HEIGHT     = 10.0         # m (for reference, 2D simulation)
DRONE_SPEED     = 5.0          # m/s
TAU_DWELL       = 1.0          # s, sensor dwell at each waypoint

P_DETECT        = 0.90         # P(hit | person in footprint)
P_FALSE_ALARM   = 0.02         # P(hit | person NOT in footprint)

SIGMA_LKP       = 10.0         # m, LKP positional uncertainty
DIFFUSION_D     = 0.02         # m^2/s, Brownian diffusion coefficient
DELTA_T_SEC     = 2.0 * 3600.0  # 7200 s elapsed since LKP
SIGMA0          = np.sqrt(SIGMA_LKP ** 2 + 2.0 * DIFFUSION_D * DELTA_T_SEC)  # 19.7 m

LKP = np.array([100.0, 120.0])  # last known position (m)
START_POS = np.array([[50.0, 10.0], [100.0, 10.0], [150.0, 10.0]])  # south edge (card)

MAX_SIM_TIME    = 1800.0       # s, 30-minute hard cutoff
N_MC_TRIALS     = 50           # Monte Carlo trials per strategy

LM_SPACING      = 2.5          # m, lawnmower lattice spacing (<= sqrt(2) r_s = 2.83 m)
LM_HALF_WIDTH   = 2.0 * SIGMA0  # m, lawnmower box = LKP +/- 2 sigma0

BLOCK           = 5            # cells per random-walk block side (2.5 m)
RW_RADIUS       = 20.0         # m, random-walk candidate radius
RW_T0           = 1.0          # initial temperature
RW_T_MIN        = 0.25         # final temperature
RW_TAU          = 600.0        # s, annealing time constant  T = max(T_MIN, T0 exp(-t/tau))

STRATEGIES = ["greedy", "lawnmower", "random_walk"]
LABELS = {"greedy": "Greedy max-belief", "lawnmower": "Lawnmower (2σ box)",
          "random_walk": "Belief-weighted random walk"}
COLORS = {"greedy": "tab:blue", "lawnmower": "tab:orange", "random_walk": "tab:green"}

N_FRAMES        = 150          # GIF frames
ENTROPY_EVERY   = 5.0          # s, entropy sampling interval

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's042_missing_person',
)

CELL_C = (np.arange(GRID_N) + 0.5) * GRID_RES      # cell-centre coordinate
NB = GRID_N // BLOCK                               # 80 blocks per side
BLOCK_C = (np.arange(NB) + 0.5) * BLOCK * GRID_RES  # block-centre coordinate


# ── Belief helpers (B[iy, ix], row = y) ────────

def build_prior():
    """Gaussian prior centred on LKP, broadened by Brownian diffusion."""
    X, Y = np.meshgrid(CELL_C, CELL_C)            # X[iy, ix]
    p = np.exp(-((X - LKP[0]) ** 2 + (Y - LKP[1]) ** 2) / (2.0 * SIGMA0 ** 2))
    return p / p.sum()


def footprint_window(pos):
    """Index window and boolean disc mask of the cells within r_s of pos."""
    ix0 = max(int((pos[0] - SENSOR_RADIUS) / GRID_RES) - 1, 0)
    ix1 = min(int((pos[0] + SENSOR_RADIUS) / GRID_RES) + 2, GRID_N)
    iy0 = max(int((pos[1] - SENSOR_RADIUS) / GRID_RES) - 1, 0)
    iy1 = min(int((pos[1] + SENSOR_RADIUS) / GRID_RES) + 2, GRID_N)
    X, Y = np.meshgrid(CELL_C[ix0:ix1], CELL_C[iy0:iy1])
    mask = np.hypot(X - pos[0], Y - pos[1]) <= SENSOR_RADIUS
    return (slice(iy0, iy1), slice(ix0, ix1)), mask


def bayes_update(B, pos, hit):
    """
    In-place Bayes update with one binary observation. Outside the footprint the
    likelihood is the same constant for every cell, so it only rescales B and is
    dropped; inside the footprint B is multiplied by the likelihood ratio.
    B is kept unnormalised (normalise before use as a probability).
    """
    win, mask = footprint_window(pos)
    ratio = P_DETECT / P_FALSE_ALARM if hit else (1.0 - P_DETECT) / (1.0 - P_FALSE_ALARM)
    sub = B[win]
    sub[mask] *= ratio


def belief_entropy(B):
    """Shannon entropy (nats) of the normalised belief."""
    p = B / B.sum()
    p = p[p > 0]
    return float(-np.sum(p * np.log(p)))


# ── Strategies ─────────────────────────────────

def greedy_next(B, other_targets):
    """Highest-belief cell, excluding cells within 2 r_s of other drones' targets."""
    saved = []
    for wp in other_targets:
        ix0 = max(int((wp[0] - 2 * SENSOR_RADIUS) / GRID_RES), 0)
        ix1 = min(int((wp[0] + 2 * SENSOR_RADIUS) / GRID_RES) + 1, GRID_N)
        iy0 = max(int((wp[1] - 2 * SENSOR_RADIUS) / GRID_RES), 0)
        iy1 = min(int((wp[1] + 2 * SENSOR_RADIUS) / GRID_RES) + 1, GRID_N)
        X, Y = np.meshgrid(CELL_C[ix0:ix1], CELL_C[iy0:iy1])
        m = np.hypot(X - wp[0], Y - wp[1]) < 2 * SENSOR_RADIUS
        win = (slice(iy0, iy1), slice(ix0, ix1))
        saved.append((win, m, B[win][m].copy()))
        B[win][m] = -1.0
    k = int(np.argmax(B))
    for win, m, vals in reversed(saved):
        B[win][m] = vals
    iy, ix = divmod(k, GRID_N)
    return np.array([CELL_C[ix], CELL_C[iy]])


def build_lawnmower_sequences():
    """Per-drone boustrophedon waypoint lists over three vertical strips of the LKP +/- 2 sigma0 box."""
    x_lo, x_hi = LKP[0] - LM_HALF_WIDTH, LKP[0] + LM_HALF_WIDTH
    y_lo, y_hi = LKP[1] - LM_HALF_WIDTH, LKP[1] + LM_HALF_WIDTH
    edges = np.linspace(x_lo, x_hi, N_DRONES + 1)
    ys = np.arange(y_lo + LM_SPACING / 2, y_hi, LM_SPACING)
    seqs = []
    for k in range(N_DRONES):
        xs = np.arange(edges[k] + LM_SPACING / 2, edges[k + 1], LM_SPACING)
        wps = []
        for r, y in enumerate(ys):
            for x in (xs if r % 2 == 0 else xs[::-1]):
                wps.append([x, y])
        seqs.append(np.array(wps))
    return seqs


LAWNMOWER_SEQS = build_lawnmower_sequences()


def random_walk_next(B, pos, other_targets, t, rng):
    """Sample a nearby 2.5 m block with probability ~ (block belief)^(1/T(t))."""
    T = max(RW_T_MIN, RW_T0 * np.exp(-t / RW_TAU))
    mass = B.reshape(NB, BLOCK, NB, BLOCK).sum(axis=(1, 3))     # mass[by, bx]
    BX, BY = np.meshgrid(BLOCK_C, BLOCK_C)
    ok = np.hypot(BX - pos[0], BY - pos[1]) <= RW_RADIUS
    for wp in other_targets:
        ok &= np.hypot(BX - wp[0], BY - wp[1]) > 2 * SENSOR_RADIUS
    cand = np.flatnonzero(ok)
    if len(cand) == 0:
        cand = np.flatnonzero(np.hypot(BX - pos[0], BY - pos[1]) <= RW_RADIUS)
    m = mass.ravel()[cand]
    w = (m / m.max()) ** (1.0 / T)
    k = cand[rng.choice(len(cand), p=w / w.sum())]
    by, bx = divmod(k, NB)
    return np.array([BLOCK_C[bx], BLOCK_C[by]])


# ── Simulation ─────────────────────────────────

class Drone:
    """Kinematic drone: straight leg at DRONE_SPEED, then TAU_DWELL hover, then observe."""

    def __init__(self, idx, pos):
        self.idx = idx
        self.legs = []                 # (t_depart, p_from, t_arrive, p_to)
        self.target = pos.copy()
        self.t_obs = 0.0
        self.lm_cursor = 0
        self.legs.append((0.0, pos.copy(), 0.0, pos.copy()))

    def assign(self, wp, t_now):
        p_from = self.target.copy()
        t_arr = t_now + np.linalg.norm(wp - p_from) / DRONE_SPEED
        self.legs.append((t_now, p_from, t_arr, wp.copy()))
        self.target = wp.copy()
        self.t_obs = t_arr + TAU_DWELL

    def pos_at(self, t):
        for t0, p0, t1, p1 in reversed(self.legs):
            if t >= t0:
                if t >= t1 or t1 <= t0:
                    return p1.copy()
                return p0 + (p1 - p0) * (t - t0) / (t1 - t0)
        return self.legs[0][1].copy()


def next_waypoint(strategy, B, drone, drones, t, rng_w):
    others = [d.target for d in drones if d is not drone]
    if strategy == "greedy":
        return greedy_next(B, others)
    if strategy == "lawnmower":
        seq = LAWNMOWER_SEQS[drone.idx]
        wp = seq[drone.lm_cursor % len(seq)].copy()
        drone.lm_cursor += 1
        return wp
    return random_walk_next(B, drone.target, others, t, rng_w)


def simulate(strategy, seed, record=False, person_absent=False, frame_times=None):
    """
    Run one search mission. Returns dict with detection time (None if not found),
    entropy history (if record), snapshots and animation frames (if requested).
    The person position depends only on the seed, so every strategy searches for
    the same person in trial `seed`.
    """
    rng_p = np.random.default_rng([seed, 0])   # person position
    rng_s = np.random.default_rng([seed, 1])   # sensor outcomes
    rng_w = np.random.default_rng([seed, 2])   # random-walk choices
    prior = build_prior()
    B = prior.copy()
    k = rng_p.choice(GRID_N * GRID_N, p=prior.ravel())
    iy, ix = divmod(int(k), GRID_N)
    person = np.array([CELL_C[ix], CELL_C[iy]])

    drones = [Drone(i, START_POS[i]) for i in range(N_DRONES)]
    for d in drones:
        d.assign(next_waypoint(strategy, B, d, drones, 0.0, rng_w), 0.0)

    Q = np.ones_like(prior)        # prod (1 - Pd) over scans covering each cell
    ent_t = [0.0]
    ent_h = [belief_entropy(B)] if record else [np.nan]
    pdet = [0.0]
    next_ent = ENTROPY_EVERY
    snap_times = [0.0, 60.0, 120.0]
    snaps, frames = [], []
    f_idx = 0
    frame_times = [] if frame_times is None else list(frame_times)
    n_obs = 0
    det_time = None

    def capture(t_cap):
        return (t_cap, B / B.sum(), np.array([d.pos_at(t_cap) for d in drones]))

    while True:
        d = min(drones, key=lambda dd: (dd.t_obs, dd.idx))
        t = d.t_obs
        if t > MAX_SIM_TIME:
            break
        # record states strictly before this observation
        if record:
            while snap_times and snap_times[0] < t:
                snaps.append(capture(snap_times.pop(0)))
            while next_ent < t:
                ent_t.append(next_ent)
                ent_h.append(belief_entropy(B))
                pdet.append(1.0 - float((prior * Q).sum()))
                next_ent += ENTROPY_EVERY
        while f_idx < len(frame_times) and frame_times[f_idx] < t:
            tf = frame_times[f_idx]
            frames.append((tf, (B / B.sum()).reshape(GRID_N // 2, 2, GRID_N // 2, 2).sum(axis=(1, 3)),
                           np.array([dd.pos_at(tf) for dd in drones]), belief_entropy(B)))
            f_idx += 1
        # observation at the end of the dwell
        pos = d.target
        in_fp = (not person_absent) and np.hypot(*(pos - person)) <= SENSOR_RADIUS
        hit = rng_s.random() < (P_DETECT if in_fp else P_FALSE_ALARM)
        bayes_update(B, pos, hit)
        if record:
            win, mask = footprint_window(pos)
            Q[win][mask] *= (1.0 - P_DETECT)
        n_obs += 1
        if hit and in_fp:
            det_time = t
            break
        d.assign(next_waypoint(strategy, B, d, drones, t, rng_w), t)

    t_end = det_time if det_time is not None else MAX_SIM_TIME
    if record:
        ent_t.append(t_end)
        ent_h.append(belief_entropy(B))
        pdet.append(1.0 - float((prior * Q).sum()))
        snaps.append(capture(t_end))
    while f_idx < len(frame_times):     # freeze the final state for remaining frames
        tf = frame_times[f_idx]
        frames.append((min(tf, t_end), (B / B.sum()).reshape(GRID_N // 2, 2, GRID_N // 2, 2).sum(axis=(1, 3)),
                       np.array([dd.pos_at(min(tf, t_end)) for dd in drones]), belief_entropy(B)))
        f_idx += 1
    return dict(det_time=det_time, person=person, n_obs=n_obs,
                ent_t=np.array(ent_t), ent_h=np.array(ent_h, dtype=float), pdet=np.array(pdet),
                snaps=snaps, frames=frames, drones=drones, t_end=t_end)


def empirical_cross(times, level=0.95):
    """Earliest time at which the fraction of trials detected reaches `level` (nan if never)."""
    k = int(np.ceil(level * len(times)))
    f = np.sort(times[np.isfinite(times)])
    return float(f[k - 1]) if len(f) >= k else float('nan')


def model_cross(r, level=0.95):
    idx = np.flatnonzero(r['pdet'] >= level)
    return float(r['ent_t'][idx[0]]) if len(idx) else float('nan')


def run_simulation():
    """Seed-0 demo runs, person-absent entropy runs, and Monte Carlo trials."""
    demo, absent, mc = {}, {}, {}
    print(f"Prior sigma0 = sqrt({SIGMA_LKP:.0f}^2 + 2*{DIFFUSION_D}*{DELTA_T_SEC:.0f}) = {SIGMA0:.2f} m")
    print(f"Prior entropy H0 = {belief_entropy(build_prior()):.3f} nats")
    print(f"Lawnmower box: x {LKP[0] - LM_HALF_WIDTH:.1f}-{LKP[0] + LM_HALF_WIDTH:.1f} m, "
          f"y {LKP[1] - LM_HALF_WIDTH:.1f}-{LKP[1] + LM_HALF_WIDTH:.1f} m, "
          f"{sum(len(s) for s in LAWNMOWER_SEQS)} waypoints")
    X, Y = np.meshgrid(CELL_C, CELL_C)
    inbox = (np.abs(X - LKP[0]) <= LM_HALF_WIDTH) & (np.abs(Y - LKP[1]) <= LM_HALF_WIDTH)
    print(f"Prior mass inside lawnmower box: {build_prior()[inbox].sum():.3f}")

    print("--- Seed-0 demo runs ---")
    for s in STRATEGIES:
        demo[s] = simulate(s, 0, record=True)
    t_last = max(r["t_end"] for r in demo.values())
    frame_times = np.linspace(0.0, t_last, N_FRAMES)
    for s in STRATEGIES:
        demo[s] = simulate(s, 0, record=True, frame_times=frame_times)
        r = demo[s]
        st = f"{r['det_time']:.1f} s" if r['det_time'] is not None else "NOT FOUND"
        print(f"[{s:>11s}] person at ({r['person'][0]:.2f}, {r['person'][1]:.2f}), "
              f"detection {st}, {r['n_obs']} observations, "
              f"H {r['ent_h'][0]:.3f} -> {r['ent_h'][-1]:.3f} nats")

    print("--- Person-absent runs (entropy reduction over the full 1800 s, seed 0) ---")
    for s in STRATEGIES:
        absent[s] = simulate(s, 0, record=True, person_absent=True)
        r = absent[s]
        hs = {tt: np.interp(tt, r['ent_t'], r['ent_h']) for tt in (300, 900, 1800)}
        ps = {tt: np.interp(tt, r['ent_t'], r['pdet']) for tt in (300, 900, 1800)}
        print(f"[{s:>11s}] H(0)={r['ent_h'][0]:.3f}, H(300)={hs[300]:.3f}, "
              f"H(900)={hs[900]:.3f}, H(1800)={hs[1800]:.3f} nats, {r['n_obs']} observations")
        legs = np.array([np.linalg.norm(p1 - p0) for d in r['drones'] for (_, p0, _, p1) in d.legs[2:]])
        print(f"[{s:>11s}] mean leg {legs.mean():.1f} m, transit share of mission time "
              f"{100 * legs.sum() / DRONE_SPEED / (N_DRONES * MAX_SIM_TIME):.0f} %")
        print(f"[{s:>11s}] model-expected P_det: t=300 s {ps[300]:.3f}, t=900 s {ps[900]:.3f}, "
              f"t=1800 s {ps[1800]:.3f}; reaches 0.95 at t = {model_cross(r):.1f} s")

    print(f"--- Monte Carlo: {N_MC_TRIALS} trials per strategy (same person positions) ---")
    for s in STRATEGIES:
        times = []
        for seed in range(N_MC_TRIALS):
            r = simulate(s, seed)
            times.append(r['det_time'] if r['det_time'] is not None else np.nan)
        times = np.array(times)
        mc[s] = times
        found = np.isfinite(times)
        cens = np.where(found, times, MAX_SIM_TIME)
        print(f"[{s:>11s}] found {found.sum()}/{N_MC_TRIALS}, "
              f"median {np.median(cens):.1f} s, IQR [{np.percentile(cens, 25):.1f}, "
              f"{np.percentile(cens, 75):.1f}] s (not-found counted as {MAX_SIM_TIME:.0f} s); "
              f"median of found {np.median(times[found]):.1f} s; "
              f"detected by 60 s: {np.sum(times <= 60) / N_MC_TRIALS:.2f}, by 1800 s: {found.mean():.2f}; "
              f"empirical P_det reaches 0.95 at t = {empirical_cross(times):.1f} s")
    return demo, absent, mc


# ── Plots ──────────────────────────────────────

def plot_prior(out_dir):
    """Prior belief heatmap centred on LKP, with the lawnmower box."""
    fig, ax = plt.subplots(figsize=(7, 7))
    im = ax.imshow(build_prior(), origin="lower", extent=[0, AREA_SIZE, 0, AREA_SIZE],
                   cmap="hot_r", interpolation="nearest")
    plt.colorbar(im, ax=ax, fraction=0.046, label="Prior probability P0(x)")
    ax.plot(*LKP, "x", color="k", ms=12, mew=2.5, label="LKP (100, 120)")
    ax.add_patch(plt.Circle(LKP, SIGMA0, color="k", fill=False, lw=1.2, ls="--",
                            label=f"1σ0 ({SIGMA0:.1f} m)"))
    ax.add_patch(plt.Rectangle(LKP - LM_HALF_WIDTH, 2 * LM_HALF_WIDTH, 2 * LM_HALF_WIDTH,
                               fill=False, ec="tab:orange", lw=1.5, label="Lawnmower box (±2σ0)"))
    ax.plot(START_POS[:, 0], START_POS[:, 1], "^", color="tab:purple", ms=9, label="Drone starts")
    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Prior belief: Gaussian + Brownian diffusion")
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, "prior_belief.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_belief_snapshots(demo, out_dir):
    """Belief at t = 0, 60, 120 s and at the end of the run, one row per strategy (seed 0)."""
    vmax = build_prior().max()
    fig, axes = plt.subplots(3, 4, figsize=(17, 13))
    dcol = ["cyan", "lime", "magenta"]
    for row, s in enumerate(STRATEGIES):
        r = demo[s]
        for col, (t_s, bel, poses) in enumerate(r["snaps"][:4]):
            ax = axes[row, col]
            ax.imshow(bel, origin="lower", extent=[0, AREA_SIZE, 0, AREA_SIZE],
                      cmap="hot_r", interpolation="nearest", vmin=0, vmax=vmax)
            ax.plot(*LKP, "x", color="k", ms=8, mew=2)
            for k, p in enumerate(poses):
                ax.add_patch(plt.Circle(p, SENSOR_RADIUS, color=dcol[k], fill=False, lw=1.2))
                ax.plot(*p, "^", color=dcol[k], ms=7, mec="k", mew=0.5)
            ax.plot(*r["person"], "*", color="green", ms=13, mec="k")
            end = " (detection)" if col == 3 and r["det_time"] is not None else \
                  (" (end, not found)" if col == 3 else "")
            ax.set_title(f"{LABELS[s]} — t = {t_s:.0f} s{end}", fontsize=10)
            ax.set_xlim(40, 160)
            ax.set_ylim(0, 180)
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
        for col in range(len(r["snaps"][:4]), 4):
            axes[row, col].axis("off")
    fig.suptitle("Belief map evolution, seed 0 (green star = true person, revealed for display)",
                 fontsize=13)
    plt.tight_layout()
    path = os.path.join(out_dir, "belief_snapshots.png")
    plt.savefig(path, dpi=110)
    plt.close()
    print(f"Saved: {path}")


def plot_entropy(absent, demo, out_dir):
    """Entropy vs time for the person-absent runs (full 1800 s) and the seed-0 demo runs."""
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for s in STRATEGIES:
        ax.plot(absent[s]["ent_t"], absent[s]["ent_h"], color=COLORS[s], lw=2,
                label=f"{LABELS[s]} (person absent)")
        ax.plot(demo[s]["ent_t"], demo[s]["ent_h"], color=COLORS[s], lw=1, ls=":",
                label=f"{LABELS[s]} (seed 0, until detection)")
        if demo[s]["det_time"] is not None:
            ax.plot(demo[s]["det_time"], demo[s]["ent_h"][-1], "o", color=COLORS[s])
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("Belief entropy H(B) (nats)")
    ax.set_title("Entropy reduction over time")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, "entropy_reduction.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_mc_boxplot(mc, out_dir):
    """Box-plot of detection times (not-found trials counted as the 1800 s cutoff)."""
    fig, ax = plt.subplots(figsize=(8, 5))
    data = [np.where(np.isfinite(mc[s]), mc[s], MAX_SIM_TIME) for s in STRATEGIES]
    bp = ax.boxplot(data, tick_labels=[LABELS[s] for s in STRATEGIES], patch_artist=True)
    for patch, s in zip(bp["boxes"], STRATEGIES):
        patch.set_facecolor(COLORS[s])
        patch.set_alpha(0.6)
    for i, s in enumerate(STRATEGIES):
        d = data[i]
        found = np.isfinite(mc[s]).sum()
        ax.text(i + 1, 1850, f"median {np.median(d):.0f} s\nIQR [{np.percentile(d, 25):.0f}, "
                f"{np.percentile(d, 75):.0f}]\n{found}/{len(d)} found",
                ha="center", va="bottom", fontsize=8)
    ax.axhline(MAX_SIM_TIME, color="red", ls="--", lw=1, label="30-min cutoff")
    ax.set_ylim(0, 2150)
    ax.set_ylabel("Detection time (s)")
    ax.set_title(f"Detection time distribution ({N_MC_TRIALS} MC trials per strategy)")
    ax.legend(fontsize=8, loc="lower right")
    plt.tight_layout()
    path = os.path.join(out_dir, "mc_boxplot.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_detection_cdf(mc, absent, out_dir):
    """Empirical P_det(t) over the MC trials (solid) and model-expected P_det(t)
    = sum_x B0(x) (1 - prod(1 - Pd)) along the person-absent search paths (dashed)."""
    fig, ax = plt.subplots(figsize=(9, 4.5))
    tt = np.linspace(0, MAX_SIM_TIME, 721)
    for s in STRATEGIES:
        times = mc[s]
        cdf = [(times <= t).sum() / len(times) for t in tt]
        ax.plot(tt, cdf, color=COLORS[s], lw=2, label=f"{LABELS[s]} (MC empirical)")
        for tc, mk in ((empirical_cross(times), 'o'), (model_cross(absent[s]), 's')):
            if np.isfinite(tc):
                ax.plot(tc, 0.95, mk, color=COLORS[s], mec='k', ms=8, zorder=6)
                ax.annotate(f"{tc:.0f} s", (tc, 0.95), textcoords='offset points', xytext=(-10, -16),
                            fontsize=8, color=COLORS[s])
        ax.plot(absent[s]["ent_t"], absent[s]["pdet"], color=COLORS[s], lw=1.2, ls="--",
                label=f"{LABELS[s]} (model-expected, seed-0 path)")
    ax.axhline(0.95, color="grey", ls="--", lw=1, label="0.95 target")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Probability of detection")
    ax.set_ylim(0, 1.02)
    ax.set_title(f"P_det(t): empirical over {N_MC_TRIALS} MC trials (solid) vs model-expected (dashed)\n"
                 "circle / square = time the empirical / model curve reaches 0.95", fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7.5, loc="lower right")
    plt.tight_layout()
    path = os.path.join(out_dir, "detection_cdf.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def save_animation(demo, out_dir):
    """Three panels (one per strategy), seed 0, kinematic drone positions, belief updates."""
    import matplotlib.animation as animation
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.6))
    vmax = build_prior().max() * 4
    dcol = ["cyan", "lime", "magenta"]
    ims, marks, circs, titles = [], [], [], []
    for ax, s in zip(axes, STRATEGIES):
        f0 = demo[s]["frames"][0]
        ims.append(ax.imshow(f0[1], origin="lower", extent=[0, AREA_SIZE, 0, AREA_SIZE],
                             cmap="hot_r", interpolation="nearest", vmin=0, vmax=vmax))
        ax.plot(*LKP, "x", color="k", ms=8, mew=2)
        ax.plot(*demo[s]["person"], "*", color="green", ms=12, mec="k")
        mk, cc = [], []
        for k in range(N_DRONES):
            mk.append(ax.plot([], [], "^", color=dcol[k], ms=7, mec="k", mew=0.5)[0])
            c = plt.Circle((0, 0), SENSOR_RADIUS, color=dcol[k], fill=False, lw=1.2)
            ax.add_patch(c)
            cc.append(c)
        marks.append(mk)
        circs.append(cc)
        titles.append(ax.set_title(""))
        ax.set_xlim(40, 160)
        ax.set_ylim(0, 180)
        ax.set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")

    def update(i):
        for j, s in enumerate(STRATEGIES):
            tf, bel, poses, h = demo[s]["frames"][i]
            ims[j].set_data(bel)
            for k in range(N_DRONES):
                marks[j][k].set_data([poses[k][0]], [poses[k][1]])
                circs[j][k].set_center(poses[k])
            done = demo[s]["det_time"] is not None and tf >= demo[s]["det_time"]
            titles[j].set_text(f"{LABELS[s]}\nt = {tf:.0f} s, H = {h:.2f} nats" + ("  FOUND" if done else ""))
        return []

    ani = animation.FuncAnimation(fig, update, frames=N_FRAMES, interval=100, blit=False)
    path = os.path.join(out_dir, "animation.gif")
    ani.save(path, writer="pillow", fps=10, dpi=70)
    plt.close()
    print(f"Saved: {path}  ({N_FRAMES} frames)")


# ── Main ───────────────────────────────────────

if __name__ == "__main__":
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    demo, absent, mc = run_simulation()
    plot_prior(out_dir)
    plot_belief_snapshots(demo, out_dir)
    plot_entropy(absent, demo, out_dir)
    plot_mc_boxplot(mc, out_dir)
    plot_detection_cdf(mc, absent, out_dir)
    save_animation(demo, out_dir)
    print("Done.")
