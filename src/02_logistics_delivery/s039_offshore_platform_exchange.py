"""
S039 Offshore Platform Exchange
================================
Simulates a fleet of 4 drones redistributing cargo among 5 offshore platforms
scattered across a 6x6 km sea area, with a mobile support vessel providing
battery swaps en route. Wind (steady + Dryden gust) forces crab-angle heading
correction; platform pad conflicts trigger holding orbits; three dispatch
strategies (greedy direct, vessel-relay scheduled, rolling-horizon replanning)
are compared by total fleet energy and deadline violations.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s039_offshore_platform_exchange.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyArrowPatch
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────
N_DRONES    = 4
N_PLATFORMS = 5
V_AIRSPEED  = 12.0       # m/s  drone cruise airspeed
V_VESSEL    = 4.0        # m/s  support vessel speed
R_MAX       = 5000.0     # m    max range per full battery (derived from energy)
D_SAFETY    = 0.15       # –    battery reserve fraction
P_CRUISE    = 80.0       # W    nominal cruise power
K_WIND      = 0.8        # W·s²/m²  wind penalty coefficient
W_MEAN      = np.array([4.0, 1.5])  # m/s  mean wind vector
SIGMA_GUST  = 1.2        # m/s  gust intensity
L_GUST      = 200.0      # m    gust length scale
D_DOCK      = 15.0       # m    moving-deck engagement radius
EPS_P       = 1.0        # m    landing position tolerance
EPS_V       = 0.5        # m/s  landing velocity tolerance
KP, KD      = 1.8, 1.2   # –    deck-tracking PD gains
R_HOLD      = 30.0       # m    holding orbit radius
DT          = 0.1        # s    simulation timestep
T_MAX       = 600.0      # s    mission horizon
DT_PLAN     = 30.0       # s    replanning interval

# Compute battery capacity from range and speed
P_WIND_MEAN = K_WIND * float(np.dot(W_MEAN, W_MEAN))
P_TOTAL_NOM = P_CRUISE + P_WIND_MEAN
_, VG_NOM = None, None  # computed in helpers

PLATFORMS = np.array([
    [500.,  500.],
    [2000., 4500.],
    [4000., 1000.],
    [5500., 3500.],
    [3000., 2800.],
])

VESSEL_ROUTE = np.array([
    [1000., 1000.],
    [3000.,  500.],
    [5000., 2000.],
    [4500., 4500.],
    [1500., 3500.],
    [1000., 1000.],
])

# (origin_idx, dest_idx, payload_kg, deadline_s)
REQUESTS = [
    (0, 3, 1.5, 400.0),
    (2, 1, 0.8, 350.0),
    (4, 0, 2.0, 500.0),
    (1, 4, 1.2, 450.0),
]

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's039_offshore_platform_exchange',
)

RNG = np.random.default_rng(0)

# ── Helpers ────────────────────────────────────

def crab_angle_and_groundspeed(target_dir, v_air, wind):
    """Return (crab_angle_rad, groundspeed_scalar) to fly along target_dir in wind."""
    psi_des = np.arctan2(target_dir[1], target_dir[0])
    psi_w   = np.arctan2(wind[1], wind[0])
    w_mag   = np.linalg.norm(wind)
    if v_air < 1e-6:
        return 0.0, max(w_mag, 0.1)
    sin_d = np.clip(w_mag * np.sin(psi_des - psi_w) / v_air, -1.0, 1.0)
    delta = np.arcsin(sin_d)
    Vg    = v_air * np.cos(delta) + w_mag * np.cos(psi_des - psi_w)
    return delta, max(Vg, 0.5)


def vessel_position(t, route=VESSEL_ROUTE, speed=V_VESSEL):
    """2D vessel position at time t along piecewise-linear looping route."""
    legs = np.diff(route, axis=0)
    leg_len = np.linalg.norm(legs, axis=1)
    total_len = leg_len.sum()
    d = (speed * t) % total_len
    cum = np.concatenate([[0.], np.cumsum(leg_len)])
    seg = int(np.searchsorted(cum, d, side='right')) - 1
    seg = min(seg, len(route) - 2)
    frac = (d - cum[seg]) / leg_len[seg]
    return route[seg] + frac * legs[seg]


def vessel_velocity(t, route=VESSEL_ROUTE, speed=V_VESSEL):
    """Vessel velocity vector at time t."""
    legs = np.diff(route, axis=0)
    leg_len = np.linalg.norm(legs, axis=1)
    total_len = leg_len.sum()
    d = (speed * t) % total_len
    cum = np.concatenate([[0.], np.cumsum(leg_len)])
    seg = int(np.searchsorted(cum, d, side='right')) - 1
    seg = min(seg, len(route) - 2)
    return speed * legs[seg] / leg_len[seg]


def dryden_gust_step(xi, dt, L_g=L_GUST, sigma_g=SIGMA_GUST, rng=RNG):
    """First-order Dryden gust update (2D)."""
    tau = L_g / V_AIRSPEED
    a   = np.exp(-dt / tau)
    b   = sigma_g * np.sqrt(2.0 / tau) * np.sqrt(max(1.0 - a**2, 0.0))
    return a * xi + b * rng.standard_normal(2)


def predict_range(energy, wind):
    """Predicted remaining range given current energy and wind."""
    P = P_CRUISE + K_WIND * float(np.dot(wind, wind))
    _, Vg = crab_angle_and_groundspeed(np.array([1., 0.]), V_AIRSPEED, wind)
    if P < 1e-6:
        return R_MAX
    return max(0.0, energy / P * Vg)


def intercept_vessel(p_drone, wind_total, t_now, max_iter=8):
    """Fixed-point iteration for vessel intercept time and position."""
    t_star = t_now
    for _ in range(max_iter):
        pv = vessel_position(t_star)
        diff = pv - p_drone
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            break
        unit = diff / dist
        _, Vg = crab_angle_and_groundspeed(unit, V_AIRSPEED, wind_total)
        t_star = t_now + dist / max(Vg, 0.1)
    return t_star, vessel_position(t_star)


# Battery capacity: energy to fly R_MAX at nominal conditions
def battery_capacity():
    _, Vg = crab_angle_and_groundspeed(np.array([1., 0.]), V_AIRSPEED, W_MEAN)
    P = P_CRUISE + K_WIND * float(np.dot(W_MEAN, W_MEAN))
    return P * R_MAX / max(Vg, 1.0)


BATTERY_FULL = battery_capacity()


# ── Simulation core ────────────────────────────

class DroneState:
    def __init__(self, idx, pos, energy):
        self.idx    = idx
        self.pos    = np.array(pos, dtype=float)
        self.vel    = np.zeros(2)
        self.energy = float(energy)
        self.gust   = np.zeros(2)
        # mission state
        self.state   = "idle"          # idle | flying | approach_vessel | holding | docked
        self.target  = None            # np.array target position
        self.target_type = None        # "platform" | "vessel"
        self.cargo   = None            # request index or None
        self.dest_platform = None      # final platform idx
        self.hold_angle = 0.0          # current holding orbit angle
        self.hold_platform = None      # platform idx holding around
        self.swap_count = 0            # number of battery swaps
        self.dock_timer = 0.0          # time spent docked on vessel
        # tracking
        self.traj   = [pos.copy()]
        self.energy_hist = [self.energy]
        self.swap_events = []          # list of times when swap occurred
        self.pad_events  = []          # list of (t, platform_idx, type)


def run_strategy(strategy="greedy", rng_seed=0):
    """
    Run the full mission with a given strategy.
    strategy: "greedy" | "relay" | "rolling"
    Returns a dict of simulation records.
    """
    rng = np.random.default_rng(rng_seed)

    drones = [
        DroneState(0, PLATFORMS[0].copy(), BATTERY_FULL),
        DroneState(1, PLATFORMS[2].copy(), BATTERY_FULL),
        DroneState(2, PLATFORMS[4].copy(), BATTERY_FULL),
        DroneState(3, PLATFORMS[1].copy(), BATTERY_FULL),
    ]

    # Requests state: pending, assigned drone idx, delivered time
    req_status   = ["pending"] * len(REQUESTS)   # pending | assigned | delivered
    req_drone    = [None] * len(REQUESTS)
    req_deliver  = [None] * len(REQUESTS)
    req_pickup   = [False] * len(REQUESTS)        # True once cargo picked up

    # Platform pad occupancy: pad_occ[m] = drone idx or None
    pad_occ   = [None] * N_PLATFORMS
    vessel_occ = None  # drone idx or None

    t = 0.0
    n_steps = int(T_MAX / DT)
    last_plan_t = -DT_PLAN  # force planning on step 0

    vessel_traj = []
    time_arr    = []

    # ── helper: assign requests to idle drones ──
    def assign_requests():
        for r_idx, (orig, dest, wt, dl) in enumerate(REQUESTS):
            if req_status[r_idx] != "pending":
                continue
            # find idle drone nearest origin
            best_d = 1e18
            best_k = None
            for k, dr in enumerate(drones):
                if dr.state not in ("idle", "flying") or dr.cargo is not None:
                    continue
                if dr.state == "flying" and dr.target_type == "platform":
                    # Only re-assign if idle
                    continue
                if dr.state == "idle":
                    dist = np.linalg.norm(dr.pos - PLATFORMS[orig])
                    if dist < best_d:
                        best_d = dist
                        best_k = k
            if best_k is not None:
                req_status[r_idx] = "assigned"
                req_drone[r_idx]  = best_k
                dr = drones[best_k]
                dr.cargo = r_idx
                dr.dest_platform = dest
                # Direct to origin platform first
                dr.target = PLATFORMS[orig].copy()
                dr.target_type = "platform"
                dr.state = "flying"

    # ── helper: check battery and divert to vessel ──
    def check_battery_divert(dr, wind_total):
        if dr.state not in ("flying",):
            return
        if dr.target is None:
            return
        d_to_dest = np.linalg.norm(dr.pos - dr.target)
        d_rem = predict_range(dr.energy, wind_total)
        d_safety_m = D_SAFETY * R_MAX
        if d_rem < d_to_dest + d_safety_m:
            # divert to vessel
            t_int, p_int = intercept_vessel(dr.pos, wind_total, t)
            dr.target = p_int.copy()
            dr.target_type = "vessel"
            dr.state = "approach_vessel"

    def maybe_use_vessel_proactively(dr, wind_total, dest_platform):
        """For relay strategy: always route via vessel if range is marginal."""
        if dr.state != "idle" or dr.cargo is None:
            return
        orig = REQUESTS[dr.cargo][0]
        d_orig_dest = np.linalg.norm(PLATFORMS[orig] - PLATFORMS[dest_platform])
        if d_orig_dest > 0.5 * R_MAX:
            t_int, p_int = intercept_vessel(dr.pos, wind_total, t)
            dr.target = p_int.copy()
            dr.target_type = "vessel"
            dr.state = "approach_vessel"

    for step in range(n_steps):
        t = step * DT
        time_arr.append(t)

        # Replanning
        if strategy in ("rolling",) and t - last_plan_t >= DT_PLAN:
            last_plan_t = t
            assign_requests()
        elif t == 0.0:
            assign_requests()

        vp = vessel_position(t)
        vessel_traj.append(vp.copy())

        for dr in drones:
            dr.traj.append(dr.pos.copy())
            dr.energy_hist.append(dr.energy)

            # Gust update
            dr.gust = dryden_gust_step(dr.gust, DT, rng=rng)
            wind_total = W_MEAN + dr.gust

            # ── State machine ──

            if dr.state == "idle":
                if strategy == "greedy":
                    assign_requests()
                elif strategy == "relay":
                    assign_requests()
                continue

            if dr.state == "docked":
                # Swap battery after short delay
                dr.dock_timer += DT
                dr.pos = vessel_position(t).copy()
                if dr.dock_timer >= 5.0:
                    dr.energy = BATTERY_FULL
                    dr.swap_count += 1
                    dr.swap_events.append(t)
                    dr.dock_timer = 0.0
                    # Resume mission
                    if dr.cargo is not None:
                        r_idx = dr.cargo
                        orig, dest, wt, dl = REQUESTS[r_idx]
                        if not req_pickup[r_idx]:
                            dr.target = PLATFORMS[orig].copy()
                        else:
                            dr.target = PLATFORMS[dest].copy()
                        dr.target_type = "platform"
                        dr.state = "flying"
                    else:
                        dr.state = "idle"
                    # Release vessel pad
                    vessel_occ = None
                continue

            if dr.state == "holding":
                # Circular orbit around hold_platform
                dr.hold_angle += V_AIRSPEED / R_HOLD * DT
                hc = PLATFORMS[dr.hold_platform]
                dr.pos = hc + R_HOLD * np.array([np.cos(dr.hold_angle), np.sin(dr.hold_angle)])
                P_total = P_CRUISE + K_WIND * float(np.dot(wind_total, wind_total))
                dr.energy = max(0.0, dr.energy - P_total * DT)
                # Check if pad is free
                if pad_occ[dr.hold_platform] is None:
                    pad_occ[dr.hold_platform] = dr.idx
                    dr.state = "landing"
                    dr.pad_events.append((t, dr.hold_platform, "land_from_hold"))
                continue

            if dr.state == "landing":
                # Immediately arrive at platform
                pm = dr.hold_platform if dr.hold_platform is not None else None
                if pm is not None:
                    dr.pos = PLATFORMS[pm].copy()
                    r_idx = dr.cargo
                    if r_idx is not None:
                        orig, dest, wt, dl = REQUESTS[r_idx]
                        if not req_pickup[r_idx] and pm == orig:
                            req_pickup[r_idx] = True
                            dr.target = PLATFORMS[dest].copy()
                            dr.target_type = "platform"
                            dr.state = "flying"
                            pad_occ[pm] = None
                        elif req_pickup[r_idx] and pm == dest:
                            req_status[r_idx] = "delivered"
                            req_deliver[r_idx] = t
                            dr.cargo = None
                            dr.dest_platform = None
                            dr.state = "idle"
                            pad_occ[pm] = None
                            dr.pad_events.append((t, pm, "delivered"))
                        else:
                            dr.state = "flying"
                            pad_occ[pm] = None
                    dr.hold_platform = None
                continue

            if dr.state in ("flying", "approach_vessel"):
                # Battery check (greedy diverts on emergency; relay/rolling proactive)
                if strategy == "greedy":
                    check_battery_divert(dr, wind_total)
                elif strategy in ("relay", "rolling"):
                    if dr.state == "flying":
                        check_battery_divert(dr, wind_total)

                target = dr.target
                if target is None:
                    dr.state = "idle"
                    continue

                direction = target - dr.pos
                dist = np.linalg.norm(direction)

                if dr.state == "approach_vessel" or dr.target_type == "vessel":
                    # Update intercept point each step
                    if dist > D_DOCK:
                        t_int, p_int = intercept_vessel(dr.pos, wind_total, t)
                        dr.target = p_int.copy()
                        direction = dr.target - dr.pos
                        dist = np.linalg.norm(direction)

                if dist < 2.0:
                    # Arrived
                    if dr.target_type == "vessel":
                        # Moving deck landing
                        vv = vessel_velocity(t)
                        if vessel_occ is None or vessel_occ == dr.idx:
                            vessel_occ = dr.idx
                            dr.pos = vessel_position(t).copy()
                            dr.vel = vv.copy()
                            dr.state = "docked"
                            dr.pad_events.append((t, -1, "vessel_swap"))
                        else:
                            # Vessel occupied — circle and retry
                            t_int, p_int = intercept_vessel(dr.pos, wind_total, t + 10.0)
                            dr.target = p_int.copy()
                    else:
                        # Arriving at a platform
                        pm_idx = None
                        for m in range(N_PLATFORMS):
                            if np.linalg.norm(dr.target - PLATFORMS[m]) < 5.0:
                                pm_idx = m
                                break
                        if pm_idx is not None:
                            if pad_occ[pm_idx] is None or pad_occ[pm_idx] == dr.idx:
                                pad_occ[pm_idx] = dr.idx
                                dr.pos = PLATFORMS[pm_idx].copy()
                                dr.hold_platform = pm_idx
                                dr.state = "landing"
                                dr.pad_events.append((t, pm_idx, "land"))
                            else:
                                # Enter holding orbit
                                dr.hold_platform = pm_idx
                                dr.hold_angle = np.arctan2(
                                    dr.pos[1] - PLATFORMS[pm_idx][1],
                                    dr.pos[0] - PLATFORMS[pm_idx][0])
                                dr.state = "holding"
                                dr.pad_events.append((t, pm_idx, "hold_start"))
                        else:
                            dr.state = "idle"
                else:
                    # Move toward target
                    unit = direction / dist
                    _, Vg = crab_angle_and_groundspeed(unit, V_AIRSPEED, wind_total)
                    move = unit * Vg * DT
                    if np.linalg.norm(move) > dist:
                        dr.pos = target.copy()
                    else:
                        dr.pos += move
                    P_total = P_CRUISE + K_WIND * float(np.dot(wind_total, wind_total))
                    dr.energy = max(0.0, dr.energy - P_total * DT)

        # Check for unassigned requests after each step (greedy)
        if strategy == "greedy":
            for r_idx in range(len(REQUESTS)):
                if req_status[r_idx] == "pending":
                    assign_requests()
                    break

    # Compute metrics
    total_energy = sum(BATTERY_FULL - dr.energy for dr in drones)
    deadline_violations = sum(
        1 for i, (_, _, _, dl) in enumerate(REQUESTS)
        if req_deliver[i] is not None and req_deliver[i] > dl
    )
    undelivered = sum(1 for s in req_status if s != "delivered")

    return {
        "drones": drones,
        "vessel_traj": np.array(vessel_traj),
        "time": np.array(time_arr),
        "req_status": req_status,
        "req_deliver": req_deliver,
        "total_energy": total_energy,
        "deadline_violations": deadline_violations,
        "undelivered": undelivered,
        "strategy": strategy,
    }


def run_simulation():
    """Run all three strategies and return consolidated data."""
    data_greedy  = run_strategy("greedy",  rng_seed=0)
    data_relay   = run_strategy("relay",   rng_seed=0)
    data_rolling = run_strategy("rolling", rng_seed=0)
    return data_greedy, data_relay, data_rolling


# ── Plots ──────────────────────────────────────

def plot_sea_area(data, out_dir):
    """2D sea area map: platforms, vessel route, drone trajectories, swap events."""
    fig, ax = plt.subplots(figsize=(10, 10))

    # Vessel route
    vr = VESSEL_ROUTE
    ax.plot(vr[:, 0], vr[:, 1], '--', color='gray', lw=1.5, label='Vessel route', zorder=1)
    ax.scatter(vr[:-1, 0], vr[:-1, 1], s=60, color='gray', zorder=2, marker='D')

    # Vessel trajectory (sampled)
    vt = data["vessel_traj"]
    step = max(1, len(vt) // 200)
    ax.plot(vt[::step, 0], vt[::step, 1], '-', color='steelblue', lw=1, alpha=0.4, label='Vessel path')

    # Platforms
    colors_p = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    for m, (px, py) in enumerate(PLATFORMS):
        ax.add_patch(plt.Rectangle((px - 100, py - 100), 200, 200,
                                   color=colors_p[m], alpha=0.7, zorder=3))
        ax.text(px, py + 180, f'P{m+1}', ha='center', fontsize=9, fontweight='bold')

    # Drone trajectories
    drone_colors = ['red', 'green', 'purple', 'darkorange']
    for dr in data["drones"]:
        traj = np.array(dr.traj)
        step_t = max(1, len(traj) // 500)
        ax.plot(traj[::step_t, 0], traj[::step_t, 1],
                color=drone_colors[dr.idx], lw=1.2, alpha=0.7,
                label=f'Drone {dr.idx+1}')
        # Swap events
        for sw_t in dr.swap_events:
            sw_step = int(sw_t / DT)
            if sw_step < len(traj):
                ax.scatter(traj[sw_step, 0], traj[sw_step, 1],
                           s=120, marker='*', color=drone_colors[dr.idx],
                           edgecolors='black', linewidths=0.5, zorder=5)

    # Holding orbits
    for dr in data["drones"]:
        for (ev_t, pm_idx, ev_type) in dr.pad_events:
            if ev_type == "hold_start" and pm_idx >= 0:
                circ = plt.Circle(PLATFORMS[pm_idx], R_HOLD,
                                  fill=False, linestyle=':', color=drone_colors[dr.idx],
                                  linewidth=1.0, alpha=0.6)
                ax.add_patch(circ)

    # Wind arrow
    wx, wy = W_MEAN
    ax.annotate('', xy=(5700 + wx * 80, 300 + wy * 80), xytext=(5700, 300),
                arrowprops=dict(arrowstyle='->', color='navy', lw=2))
    ax.text(5700, 200, f'Wind\n({wx},{wy}) m/s', ha='center', fontsize=7, color='navy')

    ax.set_xlim(-200, 6500)
    ax.set_ylim(-200, 6500)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title(f'S039 Offshore Platform Exchange — Sea Area Map\n'
                 f'Strategy: {data["strategy"].title()}', fontsize=12)
    ax.legend(loc='upper left', fontsize=8)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    star_patch = mpatches.Patch(color='none', label='★ = battery swap')
    ax.legend(handles=ax.get_legend_handles_labels()[0] + [star_patch],
              labels=ax.get_legend_handles_labels()[1] + ['★ = battery swap'],
              loc='upper left', fontsize=7)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'sea_area_map.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_battery_soc(data, out_dir):
    """Battery SoC vs time for all drones."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    axes = axes.flatten()
    drone_colors = ['red', 'green', 'purple', 'darkorange']
    time = data["time"]

    # energy_hist has one extra entry (initial) vs traj — align
    for dr in data["drones"]:
        ax = axes[dr.idx]
        eh = np.array(dr.energy_hist[:len(time)])
        soc = eh / BATTERY_FULL * 100.0
        ax.plot(time, soc, color=drone_colors[dr.idx], lw=1.5)
        ax.axhline(D_SAFETY * 100, color='red', linestyle='--', lw=1, label='15% reserve')
        ax.fill_between(time, 0, np.minimum(soc, D_SAFETY * 100),
                        alpha=0.25, color='red')
        for sw_t in dr.swap_events:
            ax.axvline(sw_t, color='gold', linestyle=':', lw=1.5, label='Swap')
        ax.set_ylim(0, 105)
        ax.set_title(f'Drone {dr.idx+1}', fontsize=10)
        ax.set_ylabel('SoC (%)')
        ax.grid(True, alpha=0.3)
        if dr.idx >= 2:
            ax.set_xlabel('Time (s)')

    fig.suptitle(f'S039 Battery State-of-Charge — Strategy: {data["strategy"].title()}',
                 fontsize=12)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'battery_soc.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_platform_gantt(data, out_dir):
    """Gantt-style chart of platform pad occupancy."""
    fig, ax = plt.subplots(figsize=(12, 5))
    drone_colors = ['red', 'green', 'purple', 'darkorange']
    time = data["time"]

    # Build occupancy timeline by replaying traj
    # We approximate: for each drone, find stretches near each platform
    for dr in data["drones"]:
        traj = np.array(dr.traj[:len(time)])
        for pm in range(N_PLATFORMS):
            dists = np.linalg.norm(traj - PLATFORMS[pm], axis=1)
            on_pad = dists < 5.0
            # Find contiguous segments
            in_seg = False
            seg_start = 0
            for i, flag in enumerate(on_pad):
                if flag and not in_seg:
                    in_seg = True
                    seg_start = i
                elif not flag and in_seg:
                    in_seg = False
                    dur = (i - seg_start) * DT
                    if dur > 0.5:
                        y = pm + dr.idx * 0.18
                        ax.barh(y, dur, left=time[seg_start], height=0.16,
                                color=drone_colors[dr.idx], alpha=0.7)
            if in_seg:
                dur = (len(on_pad) - seg_start) * DT
                if dur > 0.5:
                    y = pm + dr.idx * 0.18
                    ax.barh(y, dur, left=time[seg_start], height=0.16,
                            color=drone_colors[dr.idx], alpha=0.7)

    # Holding events
    for dr in data["drones"]:
        traj = np.array(dr.traj[:len(time)])
        for pm in range(N_PLATFORMS):
            orbit_center = PLATFORMS[pm]
            dists = np.linalg.norm(traj - orbit_center, axis=1)
            holding = (dists > R_HOLD * 0.7) & (dists < R_HOLD * 1.3)
            in_seg = False
            seg_start = 0
            for i, flag in enumerate(holding):
                if flag and not in_seg:
                    in_seg = True; seg_start = i
                elif not flag and in_seg:
                    in_seg = False
                    dur = (i - seg_start) * DT
                    if dur > 1.0:
                        y = pm + dr.idx * 0.18
                        ax.barh(y, dur, left=time[seg_start], height=0.16,
                                color=drone_colors[dr.idx], alpha=0.4, hatch='///')

    ax.set_yticks(range(N_PLATFORMS))
    ax.set_yticklabels([f'Platform {m+1}' for m in range(N_PLATFORMS)])
    ax.set_xlabel('Time (s)')
    ax.set_title(f'S039 Platform Pad Occupancy — Strategy: {data["strategy"].title()}')
    ax.set_xlim(0, T_MAX)
    # Legend
    legend_handles = [
        mpatches.Patch(color=drone_colors[i], label=f'Drone {i+1}') for i in range(N_DRONES)
    ]
    legend_handles.append(mpatches.Patch(facecolor='gray', hatch='///', alpha=0.4, label='Holding orbit'))
    ax.legend(handles=legend_handles, loc='upper right', fontsize=8)
    ax.grid(True, axis='x', alpha=0.3)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'platform_gantt.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_strategy_comparison(data_g, data_r, data_ro, out_dir):
    """Bar chart comparing strategies by total energy and deadline violations."""
    strategies   = ['Greedy\nDirect', 'Vessel-Relay\nScheduled', 'Rolling-Horizon\nReplanning']
    energies     = [data_g["total_energy"]/1000,
                    data_r["total_energy"]/1000,
                    data_ro["total_energy"]/1000]
    violations   = [data_g["deadline_violations"],
                    data_r["deadline_violations"],
                    data_ro["deadline_violations"]]
    undelivered  = [data_g["undelivered"],
                    data_r["undelivered"],
                    data_ro["undelivered"]]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    x = np.arange(len(strategies))
    bar_colors = ['#e74c3c', '#3498db', '#2ecc71']
    bars = ax1.bar(x, energies, color=bar_colors, alpha=0.8, edgecolor='black')
    ax1.set_xticks(x); ax1.set_xticklabels(strategies, fontsize=9)
    ax1.set_ylabel('Total Fleet Energy (kJ)')
    ax1.set_title('Total Fleet Energy Consumption')
    for bar, val in zip(bars, energies):
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                 f'{val:.1f} kJ', ha='center', va='bottom', fontsize=9)
    ax1.grid(True, axis='y', alpha=0.3)

    # Stacked: violations + undelivered
    w = 0.35
    ax2.bar(x - w/2, violations, w, label='Deadline violations', color='#e74c3c', alpha=0.8)
    ax2.bar(x + w/2, undelivered, w, label='Undelivered requests', color='#f39c12', alpha=0.8)
    ax2.set_xticks(x); ax2.set_xticklabels(strategies, fontsize=9)
    ax2.set_ylabel('Count')
    ax2.set_title('Deadline Violations & Undelivered Requests')
    ax2.legend(fontsize=9)
    ax2.grid(True, axis='y', alpha=0.3)
    ax2.set_ylim(0, max(max(violations), max(undelivered)) + 2)

    fig.suptitle('S039 Strategy Comparison: Greedy vs Relay vs Rolling-Horizon', fontsize=12)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'strategy_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    """Animated GIF of drone fleet and vessel movement."""
    import matplotlib.animation as animation

    fig, ax = plt.subplots(figsize=(8, 8))

    # Static elements
    vr = VESSEL_ROUTE
    ax.plot(vr[:, 0], vr[:, 1], '--', color='gray', lw=1, alpha=0.5)
    colors_p = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    for m, (px, py) in enumerate(PLATFORMS):
        ax.add_patch(plt.Rectangle((px - 120, py - 120), 240, 240,
                                   color=colors_p[m], alpha=0.5))
        ax.text(px, py + 220, f'P{m+1}', ha='center', fontsize=8, fontweight='bold')

    ax.set_xlim(-200, 6500)
    ax.set_ylim(-200, 6500)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.grid(True, alpha=0.3)

    # Dynamic elements
    drone_colors = ['red', 'green', 'purple', 'darkorange']
    n_drones = len(data["drones"])
    drone_markers = [ax.plot([], [], 'o', color=drone_colors[i], ms=8, zorder=5)[0]
                     for i in range(n_drones)]
    drone_trails  = [ax.plot([], [], '-', color=drone_colors[i], lw=0.8, alpha=0.5)[0]
                     for i in range(n_drones)]
    vessel_marker = ax.plot([], [], 's', color='steelblue', ms=12, zorder=4)[0]

    traj_arrays = [np.array(dr.traj) for dr in data["drones"]]
    vessel_traj = data["vessel_traj"]
    time_arr    = data["time"]
    n_total = min(len(time_arr), len(vessel_traj))

    # Subsample for animation speed
    frame_step = max(1, n_total // 150)
    frames = range(0, n_total, frame_step)

    title = ax.set_title('')

    def init():
        for dm in drone_markers: dm.set_data([], [])
        for dt in drone_trails:  dt.set_data([], [])
        vessel_marker.set_data([], [])
        return drone_markers + drone_trails + [vessel_marker, title]

    def update(frame):
        i = frame
        title.set_text(f'S039 Offshore Exchange (t={time_arr[i]:.0f}s) — {data["strategy"].title()}')
        vp = vessel_traj[i]
        vessel_marker.set_data([vp[0]], [vp[1]])
        trail_len = 80 // frame_step
        for k, dr in enumerate(data["drones"]):
            traj = traj_arrays[k]
            if i < len(traj):
                drone_markers[k].set_data([traj[i, 0]], [traj[i, 1]])
                start = max(0, i - trail_len * frame_step)
                trail_pts = traj[start:i:frame_step]
                if len(trail_pts) > 0:
                    drone_trails[k].set_data(trail_pts[:, 0], trail_pts[:, 1])
        return drone_markers + drone_trails + [vessel_marker, title]

    ani = animation.FuncAnimation(fig, update, frames=frames,
                                  init_func=init, blit=True, interval=50)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    print('Running S039 Offshore Platform Exchange...')
    data_greedy, data_relay, data_rolling = run_simulation()

    # ── Print key metrics ──
    print('\n=== Strategy Comparison ===')
    for data in [data_greedy, data_relay, data_rolling]:
        strat = data['strategy'].upper()
        E_kJ  = data['total_energy'] / 1000.0
        viol  = data['deadline_violations']
        undel = data['undelivered']
        swaps = sum(dr.swap_count for dr in data['drones'])
        print(f'  [{strat:8s}] Energy: {E_kJ:.2f} kJ  |  Deadline violations: {viol}'
              f'  |  Undelivered: {undel}  |  Battery swaps: {swaps}')

    print('\n=== Greedy — Per-drone ===')
    for dr in data_greedy['drones']:
        E_used = (BATTERY_FULL - dr.energy) / 1000
        print(f'  Drone {dr.idx+1}: Energy used {E_used:.3f} kJ, '
              f'Swaps: {dr.swap_count}, Final SoC: {dr.energy/BATTERY_FULL*100:.1f}%')

    print('\n=== Cargo Deliveries (Greedy) ===')
    for i, (orig, dest, wt, dl) in enumerate(REQUESTS):
        st   = data_greedy['req_status'][i]
        t_dl = data_greedy['req_deliver'][i]
        late = ''
        if t_dl is not None and t_dl > dl:
            late = f' *** LATE by {t_dl-dl:.0f}s ***'
        elif t_dl is None:
            late = ' *** NOT DELIVERED ***'
        print(f'  R{i+1}: P{orig+1}→P{dest+1} ({wt}kg, dl={dl}s) '
              f'delivered={t_dl:.1f}s{late}' if t_dl else
              f'  R{i+1}: P{orig+1}→P{dest+1} ({wt}kg, dl={dl}s) {late}')

    out_dir = os.path.normpath(OUTPUT_DIR)
    print(f'\nSaving outputs to: {out_dir}')

    # Use rolling-horizon data as primary for sea area, battery, gantt, animation
    plot_sea_area(data_rolling, out_dir)
    plot_battery_soc(data_rolling, out_dir)
    plot_platform_gantt(data_rolling, out_dir)
    plot_strategy_comparison(data_greedy, data_relay, data_rolling, out_dir)
    save_animation(data_rolling, out_dir)

    print('\nDone.')
