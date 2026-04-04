"""
S035 UTM Simulation
====================
Simulates a centralised UAS Traffic Management (UTM) service managing 12
delivery drones simultaneously operating in a 1000 x 1000 x 120 m urban
airspace divided into 4 altitude-layered corridor sets. The UTM registers
4D trajectory intents, detects pairwise conflicts, and resolves them using
three strategies: First-Come First-Served (FCFS), Priority-Based, and
Centralised LP Optimisation. During execution, separation assurance
monitoring issues Resolution Advisories (RAs) when predicted separation
drops below the safety warning distance.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s035_utm_simulation.py
"""

import sys
import os
import copy
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import linprog
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────────
N_DRONES        = 12
N_CORRIDORS     = 16       # 4 per layer × 4 layers
V_CRUISE        = 10.0     # m/s cruise speed
V_CLOSE         = 7.0      # m/s closing speed for delta_t_sep calculation
D_SEP_H         = 30.0     # m horizontal separation minimum
D_SEP_V         = 10.0     # m vertical separation minimum
D_WARN_H        = 45.0     # m warning distance (1.5 × D_SEP_H)
T_LOOKAHEAD     = 15.0     # s separation assurance lookahead
T_BUFFER        = 2.0      # s extra temporal buffer
RHO_MAX         = 3.0      # drones/km corridor max density
DT              = 0.5      # s simulation timestep (0.1 too slow for animation)
DT_CHECK        = 0.5      # s conflict scan resolution
ARENA_X         = 1000.0   # m
ARENA_Y         = 1000.0   # m
ARENA_Z         = 120.0    # m

# Altitude layers: [z_lo, z_hi, nominal_heading_deg]
# heading: 90=East, 0=North, 270=West, 180=South
LAYERS = [
    (20.0,  40.0,  90.0),   # L1 Eastbound
    (40.0,  60.0,   0.0),   # L2 Northbound
    (60.0,  80.0, 270.0),   # L3 Westbound
    (80.0, 100.0, 180.0),   # L4 Southbound
]

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's035_utm_simulation',
)

RNG = np.random.default_rng(0)

# ── Data Classes ───────────────────────────────────────────────────────────────

@dataclass
class Corridor:
    id: int
    start: np.ndarray    # (x, y)
    end: np.ndarray      # (x, y)
    z_lo: float
    z_hi: float
    width: float = 50.0  # m lane half-width

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.end - self.start))

    @property
    def unit_vec(self) -> np.ndarray:
        diff = self.end - self.start
        n = np.linalg.norm(diff)
        return diff / n if n > 0 else diff

    @property
    def z_mid(self) -> float:
        return (self.z_lo + self.z_hi) / 2.0

    def contains_xy(self, p: np.ndarray) -> bool:
        r = p[:2] - self.start
        along = float(np.dot(r, self.unit_vec))
        perp  = float(np.linalg.norm(r - along * self.unit_vec))
        return 0 <= along <= self.length and perp <= self.width


@dataclass
class DroneIntent:
    id: int
    waypoints: List[np.ndarray]   # 3D positions
    timestamps: List[float]       # arrival time at each waypoint
    priority: float = 1.0
    corridor_ids: List[int] = field(default_factory=list)

    def position_at(self, t: float) -> np.ndarray:
        if t <= self.timestamps[0]:
            return self.waypoints[0].copy()
        if t >= self.timestamps[-1]:
            return self.waypoints[-1].copy()
        for i in range(len(self.timestamps) - 1):
            if self.timestamps[i] <= t <= self.timestamps[i + 1]:
                alpha = (t - self.timestamps[i]) / (
                    self.timestamps[i + 1] - self.timestamps[i])
                return (1 - alpha) * self.waypoints[i] + alpha * self.waypoints[i + 1]
        return self.waypoints[-1].copy()


# ── Airspace & Intent Setup ───────────────────────────────────────────────────

def build_corridors() -> List[Corridor]:
    """Build 16 corridors: 4 per altitude layer."""
    corridors = []
    cid = 0
    for layer_idx, (z_lo, z_hi, heading_deg) in enumerate(LAYERS):
        heading_rad = np.radians(heading_deg)
        dx = np.cos(heading_rad)
        dy = np.sin(heading_rad)
        # 4 parallel corridors per layer at y-offsets (or x-offsets for N/S)
        offsets = [150.0, 350.0, 650.0, 850.0]
        for off in offsets:
            # Build corridor along primary direction, offset perpendicular
            if abs(dx) > abs(dy):  # East or West: offset in Y
                if dx > 0:  # Eastbound
                    start = np.array([0.0,   off])
                    end   = np.array([1000.0, off])
                else:       # Westbound
                    start = np.array([1000.0, off])
                    end   = np.array([0.0,    off])
            else:  # North or South: offset in X
                if dy > 0:  # Northbound
                    start = np.array([off, 0.0])
                    end   = np.array([off, 1000.0])
                else:       # Southbound
                    start = np.array([off, 1000.0])
                    end   = np.array([off, 0.0])
            corridors.append(Corridor(
                id=cid, start=start, end=end,
                z_lo=z_lo, z_hi=z_hi, width=60.0,
            ))
            cid += 1
    return corridors


def build_drone_intents(corridors: List[Corridor]) -> List[DroneIntent]:
    """
    Generate 12 drone intents. Each drone picks a random depot on one edge,
    a delivery target on the opposite edge, routes through two corridors
    (one for each axis), and builds a piecewise-linear 4D trajectory.
    """
    intents = []
    depots = [
        np.array([RNG.uniform(50, 950), 0.0,    30.0]),   # S edge
        np.array([RNG.uniform(50, 950), 1000.0, 30.0]),   # N edge
        np.array([0.0,    RNG.uniform(50, 950), 30.0]),   # W edge
        np.array([1000.0, RNG.uniform(50, 950), 30.0]),   # E edge
    ]
    targets_base = [
        np.array([RNG.uniform(50, 950), 1000.0, 30.0]),
        np.array([RNG.uniform(50, 950), 0.0,    30.0]),
        np.array([1000.0, RNG.uniform(50, 950), 30.0]),
        np.array([0.0,    RNG.uniform(50, 950), 30.0]),
    ]

    priority_vals = RNG.uniform(0.5, 2.5, N_DRONES)

    for k in range(N_DRONES):
        # Choose departure edge and target edge (opposite)
        edge = k % 4
        depot  = depots[edge].copy()
        depot[:2] += RNG.uniform(-30, 30, 2)
        depot[:2]  = np.clip(depot[:2], 20, 980)

        target = targets_base[edge].copy()
        target[:2] += RNG.uniform(-30, 30, 2)
        target[:2]  = np.clip(target[:2], 20, 980)

        # Pick altitude layer based on drone index
        layer_idx = k % 4
        z_lo, z_hi, _ = LAYERS[layer_idx]
        z_fly = (z_lo + z_hi) / 2.0

        # Climb waypoint
        wp0 = depot.copy()
        wp1 = depot.copy(); wp1[2] = z_fly                  # climb
        wp2 = np.array([target[0], depot[1], z_fly])         # fly X
        wp3 = target.copy(); wp3[2] = z_fly                  # fly Y
        wp4 = target.copy()                                   # descend

        waypoints = [wp0, wp1, wp2, wp3, wp4]

        # Timestamps: depart at t=k*5 (staggered), cruise at V_CRUISE
        t_depart = float(k) * 5.0
        t = t_depart
        timestamps = [t]
        for i in range(1, len(waypoints)):
            dist = np.linalg.norm(waypoints[i] - waypoints[i - 1])
            dt_seg = dist / V_CRUISE
            t += max(dt_seg, 1.0)
            timestamps.append(t)

        # Assign corridor IDs (best matching)
        used_corridors = []
        for c in corridors:
            if abs(c.z_mid - z_fly) < 15.0:
                used_corridors.append(c.id)
            if len(used_corridors) >= 2:
                break

        intents.append(DroneIntent(
            id=k,
            waypoints=waypoints,
            timestamps=timestamps,
            priority=float(priority_vals[k]),
            corridor_ids=used_corridors,
        ))
    return intents


# ── Conflict Detection ─────────────────────────────────────────────────────────

def detect_conflicts(
    intents: List[DroneIntent],
    d_sep_h: float = D_SEP_H,
    d_sep_v: float = D_SEP_V,
    dt_check: float = DT_CHECK,
) -> List[Tuple[int, int, float, float]]:
    """
    Scan all N*(N-1)/2 pairs at dt_check resolution.
    Return list of (id_k, id_j, t_first_conflict, min_separation).
    """
    conflicts = []
    t_start = min(i.timestamps[0] for i in intents)
    t_end   = max(i.timestamps[-1] for i in intents)
    t_grid  = np.arange(t_start, t_end + dt_check, dt_check)

    for a in range(len(intents)):
        for b in range(a + 1, len(intents)):
            ka, kb = intents[a], intents[b]
            in_conflict = False
            t_first = None
            min_sep = np.inf
            for t in t_grid:
                pa = ka.position_at(t)
                pb = kb.position_at(t)
                dh = float(np.linalg.norm(pa[:2] - pb[:2]))
                dv = abs(float(pa[2]) - float(pb[2]))
                sep = np.sqrt(dh**2 + dv**2)
                if dh < d_sep_h and dv < d_sep_v:
                    if not in_conflict:
                        t_first = float(t)
                        in_conflict = True
                    min_sep = min(min_sep, sep)
            if in_conflict:
                conflicts.append((ka.id, kb.id, t_first, float(min_sep)))
    return conflicts


# ── Resolution Strategies ──────────────────────────────────────────────────────

def resolve_fcfs(
    intents: List[DroneIntent],
    conflicts: List[Tuple[int, int, float, float]],
    v_close: float = V_CLOSE,
    d_sep_h: float = D_SEP_H,
    t_buffer: float = T_BUFFER,
) -> Dict[int, float]:
    """FCFS: lower-id drone has priority; higher-id drone gets delay."""
    delays = {i.id: 0.0 for i in intents}
    dt_sep = d_sep_h / v_close + t_buffer
    for id_k, id_j, _t_conf, _ in sorted(conflicts, key=lambda x: x[2]):
        current_gap = delays[id_k] - delays[id_j]
        if current_gap < dt_sep:
            delays[id_j] += dt_sep - current_gap
    return delays


def resolve_priority(
    intents: List[DroneIntent],
    conflicts: List[Tuple[int, int, float, float]],
    alpha_u: float = 0.5,
    beta_u:  float = 1.0,
    gamma_u: float = 0.3,
    v_close: float = V_CLOSE,
    d_sep_h: float = D_SEP_H,
    t_buffer: float = T_BUFFER,
) -> Dict[int, float]:
    """
    Priority-based: urgency score u_k = alpha * 1/(deadline-t_now)
    + beta * IsEmergency + gamma * payload_mass.
    Higher u_k holds trajectory; lower u_k gets delay.
    """
    t_now = min(i.timestamps[0] for i in intents)
    # Compute urgency scores
    scores: Dict[int, float] = {}
    for drone in intents:
        deadline = drone.timestamps[-1]
        urgency_time  = alpha_u / max(deadline - t_now, 1.0)
        is_emergency  = float(drone.priority > 2.0)
        payload_mass  = drone.priority  # use priority as proxy
        scores[drone.id] = urgency_time + beta_u * is_emergency + gamma_u * payload_mass

    delays = {i.id: 0.0 for i in intents}
    dt_sep = d_sep_h / v_close + t_buffer
    for id_k, id_j, _t_conf, _ in sorted(conflicts, key=lambda x: x[2]):
        # Whichever has lower score gets the delay
        if scores[id_k] >= scores[id_j]:
            # id_j is lower priority → delay id_j
            current_gap = delays[id_k] - delays[id_j]
            if current_gap < dt_sep:
                delays[id_j] += dt_sep - current_gap
        else:
            # id_k is lower priority → delay id_k
            current_gap = delays[id_j] - delays[id_k]
            if current_gap < dt_sep:
                delays[id_k] += dt_sep - current_gap
    return delays


def resolve_lp(
    intents: List[DroneIntent],
    conflicts: List[Tuple[int, int, float, float]],
    v_close: float = V_CLOSE,
    d_sep_h: float = D_SEP_H,
    t_buffer: float = T_BUFFER,
) -> Dict[int, float]:
    """
    LP: minimise sum(delta_k) subject to delta_k - delta_j >= dt_sep
    for each FCFS-directed conflict pair.
    """
    n = len(intents)
    id_to_idx = {i.id: idx for idx, i in enumerate(intents)}
    dt_sep = d_sep_h / v_close + t_buffer

    c_obj = np.ones(n)
    A_ub, b_ub = [], []

    for id_k, id_j, *_ in conflicts:
        # Require delta_j - delta_k >= dt_sep → -delta_j + delta_k <= -dt_sep
        row = np.zeros(n)
        row[id_to_idx[id_k]] =  1.0
        row[id_to_idx[id_j]] = -1.0
        A_ub.append(row)
        b_ub.append(-dt_sep)

    A_ub_arr = np.array(A_ub) if A_ub else np.empty((0, n))
    b_ub_arr = np.array(b_ub) if b_ub else np.empty(0)
    bounds = [(0.0, None)] * n

    result = linprog(c_obj, A_ub=A_ub_arr, b_ub=b_ub_arr,
                     bounds=bounds, method='highs')
    if result.success:
        return {i.id: float(result.x[idx]) for idx, i in enumerate(intents)}
    else:
        # Fallback to FCFS
        return resolve_fcfs(intents, conflicts, v_close, d_sep_h, t_buffer)


# ── Execution Simulation ───────────────────────────────────────────────────────

def simulate_utm(
    intents_in: List[DroneIntent],
    delays: Dict[int, float],
    dt: float = DT,
    t_lookahead: float = T_LOOKAHEAD,
    d_warn_h: float = D_WARN_H,
    d_sep_v: float = D_SEP_V,
) -> Dict:
    """
    Execute all drone trajectories with applied delays.
    Monitor separation in real time and issue RAs when predicted separation
    drops below d_warn_h. Return trajectory history, RA events, and delivery times.
    """
    # Deep-copy intents so delays don't accumulate across strategies
    intents = copy.deepcopy(intents_in)
    for drone in intents:
        drone.timestamps = [t + delays[drone.id] for t in drone.timestamps]

    t_end  = max(i.timestamps[-1] for i in intents) + 10.0
    t_grid = np.arange(0.0, t_end + dt, dt)

    history: Dict[int, List] = {i.id: [] for i in intents}
    ra_events: List[Dict]    = []
    delivery_times: Dict[int, float] = {}

    for t in t_grid:
        positions = {i.id: i.position_at(t) for i in intents}

        # Separation assurance lookahead
        for a_idx in range(len(intents)):
            for b_idx in range(a_idx + 1, len(intents)):
                ka, kb = intents[a_idx], intents[b_idx]
                pa_pred = ka.position_at(t + t_lookahead)
                pb_pred = kb.position_at(t + t_lookahead)
                dh_pred = float(np.linalg.norm(pa_pred[:2] - pb_pred[:2]))
                dv_pred = abs(float(pa_pred[2]) - float(pb_pred[2]))
                if dh_pred < d_warn_h and dv_pred < d_sep_v:
                    ra_events.append({
                        't': float(t),
                        'drone_a': ka.id,
                        'drone_b': kb.id,
                        'dh_pred': dh_pred,
                        'dv_pred': dv_pred,
                    })

        for drone in intents:
            pos = positions[drone.id]
            history[drone.id].append((float(t), pos.copy()))
            if drone.id not in delivery_times:
                if np.linalg.norm(pos[:2] - drone.waypoints[-1][:2]) < 8.0:
                    delivery_times[drone.id] = float(t)

    return {
        'history': history,
        'ra_events': ra_events,
        'delivery_times': delivery_times,
        'intents': intents,
    }


# ── Metrics ────────────────────────────────────────────────────────────────────

def compute_metrics(
    intents_orig: List[DroneIntent],
    delays: Dict[int, float],
    result: Dict,
    conflicts: List,
) -> Dict:
    """Compute total delay, mean delay, throughput, conflict rate."""
    n = len(intents_orig)
    delivery_times = result['delivery_times']

    # Planned arrival = timestamps[-1] without delay
    total_delay = sum(delays[d.id] for d in intents_orig)
    mean_delay  = total_delay / n

    # Mission duration = time from first drone start to last delivery
    t_start   = min(d.timestamps[0] for d in intents_orig)
    t_mission = max(delivery_times.values()) - t_start if delivery_times else 1.0
    throughput = n / t_mission * 60.0  # deliveries/min

    n_pairs = n * (n - 1) / 2
    conflict_rate = len(conflicts) / (n_pairs * t_mission / 60.0)

    return {
        'n_conflicts':   len(conflicts),
        'total_delay':   total_delay,
        'mean_delay':    mean_delay,
        'throughput':    throughput,
        'conflict_rate': conflict_rate,
        't_mission':     t_mission,
        'n_ra':          len(result['ra_events']),
    }


# ── Main Simulation Runner ────────────────────────────────────────────────────

def run_simulation():
    corridors = build_corridors()
    intents_orig = build_drone_intents(corridors)

    print("\n=== Pre-resolution conflict scan ===")
    conflicts_raw = detect_conflicts(intents_orig)
    print(f"  Raw conflicts detected: {len(conflicts_raw)}")
    for c in conflicts_raw[:6]:
        print(f"    Drones {c[0]:2d} & {c[1]:2d}  t={c[2]:.1f}s  min_sep={c[3]:.1f}m")

    # Three strategies
    delays_fcfs     = resolve_fcfs(intents_orig, conflicts_raw)
    delays_priority = resolve_priority(intents_orig, conflicts_raw)
    delays_lp       = resolve_lp(intents_orig, conflicts_raw)

    print("\n=== Running UTM simulations (3 strategies) ===")
    result_fcfs     = simulate_utm(intents_orig, delays_fcfs)
    result_priority = simulate_utm(intents_orig, delays_priority)
    result_lp       = simulate_utm(intents_orig, delays_lp)

    # Post-resolution conflict checks
    conflicts_fcfs     = detect_conflicts(result_fcfs['intents'])
    conflicts_priority = detect_conflicts(result_priority['intents'])
    conflicts_lp       = detect_conflicts(result_lp['intents'])

    metrics_fcfs     = compute_metrics(intents_orig, delays_fcfs,     result_fcfs,     conflicts_fcfs)
    metrics_priority = compute_metrics(intents_orig, delays_priority, result_priority, conflicts_priority)
    metrics_lp       = compute_metrics(intents_orig, delays_lp,       result_lp,       conflicts_lp)

    print("\n=== Results ===")
    for name, m in [('FCFS', metrics_fcfs), ('Priority', metrics_priority), ('LP', metrics_lp)]:
        print(f"  {name:10s}  conflicts={m['n_conflicts']:2d}  "
              f"total_delay={m['total_delay']:6.1f}s  "
              f"mean_delay={m['mean_delay']:5.1f}s  "
              f"throughput={m['throughput']:.3f} del/min  "
              f"RA_events={m['n_ra']:3d}")

    return (corridors, intents_orig,
            result_fcfs, result_priority, result_lp,
            delays_fcfs, delays_priority, delays_lp,
            conflicts_raw, conflicts_fcfs, conflicts_priority, conflicts_lp,
            metrics_fcfs, metrics_priority, metrics_lp)


# ── Plot Functions ────────────────────────────────────────────────────────────

def plot_3d_airspace(corridors, intents_orig, result_fcfs, conflicts_raw, out_dir):
    """3D view of all drone trajectories, corridor centrelines, conflict points."""
    fig = plt.figure(figsize=(13, 9))
    ax  = fig.add_subplot(111, projection='3d')

    # Draw corridor centrelines
    layer_colors = ['#4FC3F7', '#81C784', '#FFB74D', '#CE93D8']
    for c in corridors:
        col = layer_colors[c.id // 4 % 4]
        z_mid = c.z_mid
        ax.plot([c.start[0], c.end[0]],
                [c.start[1], c.end[1]],
                [z_mid, z_mid],
                color=col, alpha=0.25, linewidth=1.5)

    # Draw drone trajectories
    cmap = plt.cm.get_cmap('tab20', N_DRONES)
    intents_after = result_fcfs['intents']
    for drone in intents_after:
        history = result_fcfs['history'][drone.id]
        if not history:
            continue
        xs = [p[1][0] for p in history]
        ys = [p[1][1] for p in history]
        zs = [p[1][2] for p in history]
        col = cmap(drone.id)
        ax.plot(xs, ys, zs, color=col, linewidth=1.2, alpha=0.85)
        ax.scatter(xs[0], ys[0], zs[0], color=col, marker='o', s=30, zorder=5)
        ax.scatter(xs[-1], ys[-1], zs[-1], color=col, marker='*', s=60, zorder=5)

    # Mark conflict points
    for cfl in conflicts_raw:
        id_k, id_j, t_c, _ = cfl
        drone_k = next(d for d in intents_orig if d.id == id_k)
        p = drone_k.position_at(t_c)
        ax.scatter(p[0], p[1], p[2], color='red', marker='o', s=80, zorder=10, alpha=0.8)

    # Shade altitude layers
    for layer_idx, (z_lo, z_hi, _) in enumerate(LAYERS):
        col = layer_colors[layer_idx]
        for xi in [0, 1000]:
            ax.plot([xi, xi], [0, 1000], [z_lo, z_lo], color=col, alpha=0.08)

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_xlim(0, 1000); ax.set_ylim(0, 1000); ax.set_zlim(0, 120)
    ax.set_title('S035 UTM Simulation — 3D Airspace (FCFS strategy)', fontsize=13)

    handles = [
        mpatches.Patch(color='red',    label=f'Conflict points ({len(conflicts_raw)})'),
        mpatches.Patch(color='#4FC3F7', label='L1 20–40 m (Eastbound)'),
        mpatches.Patch(color='#81C784', label='L2 40–60 m (Northbound)'),
        mpatches.Patch(color='#FFB74D', label='L3 60–80 m (Westbound)'),
        mpatches.Patch(color='#CE93D8', label='L4 80–100 m (Southbound)'),
    ]
    ax.legend(handles=handles, loc='upper left', fontsize=7)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, '3d_airspace.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_conflict_timeline(intents_orig, result_fcfs, conflicts_raw, out_dir):
    """Horizontal bar chart: drone mission duration with conflict intervals."""
    fig, ax = plt.subplots(figsize=(12, 6))
    intents_after = result_fcfs['intents']

    for drone in intents_after:
        t0 = drone.timestamps[0]
        t1 = drone.timestamps[-1]
        ax.barh(drone.id, t1 - t0, left=t0, color='steelblue', alpha=0.6, height=0.5)

    # Overlay conflicts
    for cfl in conflicts_raw:
        id_k, id_j, t_conf, _ = cfl
        ax.scatter(t_conf, id_k, color='red',    marker='|', s=120, zorder=5)
        ax.scatter(t_conf, id_j, color='red',    marker='|', s=120, zorder=5)

    # RA events
    for ev in result_fcfs['ra_events'][:200]:  # limit markers
        ax.scatter(ev['t'], ev['drone_a'], color='orange', marker='D', s=15, alpha=0.4)

    ax.set_xlabel('Simulation time (s)')
    ax.set_ylabel('Drone ID')
    ax.set_yticks(range(N_DRONES))
    ax.set_title('Conflict Timeline — FCFS Strategy\n'
                 '(blue=mission, red=conflict, orange=RA)', fontsize=12)
    ax.grid(axis='x', alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'conflict_timeline.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_separation_histogram(result_fcfs, result_priority, result_lp, out_dir):
    """Histogram of minimum pairwise horizontal separation under each strategy."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), sharey=True)
    strategy_data = [
        ('FCFS',      result_fcfs,     '#4FC3F7'),
        ('Priority',  result_priority, '#81C784'),
        ('LP Optim.', result_lp,       '#FFB74D'),
    ]

    for ax, (name, result, color) in zip(axes, strategy_data):
        intents = result['intents']
        history = result['history']
        # Sample every 5 time steps
        t_vals = [h[0] for h in history[0]][::5]
        min_seps_h = []
        for t in t_vals:
            positions = {}
            for drone in intents:
                idx = min(int(t / DT), len(history[drone.id]) - 1)
                _, pos = history[drone.id][idx]
                positions[drone.id] = pos
            for a in range(len(intents)):
                for b in range(a + 1, len(intents)):
                    pa = positions[intents[a].id]
                    pb = positions[intents[b].id]
                    dh = float(np.linalg.norm(pa[:2] - pb[:2]))
                    if dh < 500:  # exclude far-apart pairs
                        min_seps_h.append(dh)

        ax.hist(min_seps_h, bins=40, color=color, alpha=0.75, edgecolor='white')
        ax.axvline(D_SEP_H, color='red',    linestyle='--', label=f'd_sep_H={D_SEP_H}m')
        ax.axvline(D_WARN_H, color='orange', linestyle='--', label=f'd_warn_H={D_WARN_H}m')
        ax.set_xlabel('Horizontal separation (m)')
        ax.set_title(name)
        ax.legend(fontsize=7)

    axes[0].set_ylabel('Count')
    fig.suptitle('Pairwise Horizontal Separation Distribution (each strategy)', fontsize=12)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'separation_histogram.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_delay_comparison(metrics_fcfs, metrics_priority, metrics_lp, out_dir):
    """Bar chart comparing total and mean delay across strategies."""
    strategies = ['FCFS', 'Priority-Based', 'LP Optimisation']
    total_delays = [metrics_fcfs['total_delay'],
                    metrics_priority['total_delay'],
                    metrics_lp['total_delay']]
    mean_delays  = [metrics_fcfs['mean_delay'],
                    metrics_priority['mean_delay'],
                    metrics_lp['mean_delay']]

    x = np.arange(3)
    width = 0.35
    fig, ax = plt.subplots(figsize=(9, 5))
    bars1 = ax.bar(x - width/2, total_delays, width, label='Total fleet delay (s)',
                   color='#4FC3F7', edgecolor='white')
    bars2 = ax.bar(x + width/2, mean_delays,  width, label='Mean per-drone delay (s)',
                   color='#FF8A65', edgecolor='white')

    for bar in bars1 + bars2:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2, h + 0.5, f'{h:.1f}',
                ha='center', va='bottom', fontsize=9)

    ax.set_xticks(x); ax.set_xticklabels(strategies)
    ax.set_ylabel('Delay (s)')
    ax.set_title('Fleet Delay Comparison — FCFS vs Priority vs LP', fontsize=12)
    ax.legend()
    ax.grid(axis='y', alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'delay_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_corridor_utilisation(corridors, result_fcfs, out_dir):
    """Heat-map of drone density per corridor vs time."""
    intents = result_fcfs['intents']
    history = result_fcfs['history']

    t_grid  = np.array([h[0] for h in history[0]])
    step    = max(1, len(t_grid) // 200)
    t_sub   = t_grid[::step]
    n_c     = len(corridors)
    density = np.zeros((n_c, len(t_sub)))

    for ti, t in enumerate(t_sub):
        t_idx = min(int(round(t / DT)), len(t_grid) - 1)
        for drone in intents:
            _, pos = history[drone.id][t_idx]
            for c in corridors:
                if c.contains_xy(pos) and c.z_lo <= pos[2] <= c.z_hi:
                    l_km = max(c.length / 1000.0, 1e-6)
                    density[c.id, ti] += 1.0 / l_km

    fig, ax = plt.subplots(figsize=(14, 6))
    im = ax.imshow(density, aspect='auto', origin='lower',
                   extent=[t_sub[0], t_sub[-1], -0.5, n_c - 0.5],
                   cmap='YlOrRd', vmin=0, vmax=RHO_MAX * 1.2)
    ax.axhline(RHO_MAX, color='white', linestyle='--', alpha=0)  # legend proxy
    plt.colorbar(im, ax=ax, label='Drone density (drones/km)')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Corridor ID')
    ax.set_title('Corridor Utilisation Heat-Map (FCFS)', fontsize=12)
    # Mark layer boundaries
    for i in range(1, 4):
        ax.axhline(i * 4 - 0.5, color='white', linewidth=1.0, linestyle='-', alpha=0.7)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'corridor_utilisation.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(corridors, result_fcfs, conflicts_raw, out_dir):
    """Top-down 2D animation: drone dots, corridors, separation violation zones."""
    import matplotlib.animation as animation

    intents = result_fcfs['intents']
    history = result_fcfs['history']
    ra_events = result_fcfs['ra_events']

    t_grid = np.array([h[0] for h in history[0]])
    step   = max(1, len(t_grid) // 150)  # ≤150 frames
    frames = range(0, len(t_grid), step)

    # Build RA time set for quick lookup
    ra_times = {ev['t'] for ev in ra_events}

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, 1000); ax.set_ylim(0, 1000)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('S035 UTM Simulation — Top-Down View (FCFS)', fontsize=11)
    ax.set_facecolor('#1a1a2e')

    # Draw corridor centrelines
    layer_colors = ['#4FC3F7', '#81C784', '#FFB74D', '#CE93D8']
    for c in corridors:
        col = layer_colors[c.id // 4 % 4]
        ax.plot([c.start[0], c.end[0]], [c.start[1], c.end[1]],
                color=col, alpha=0.20, linewidth=1.5)

    cmap = plt.cm.get_cmap('tab20', N_DRONES)
    dots = [ax.plot([], [], 'o', color=cmap(i), markersize=7)[0]
            for i in range(N_DRONES)]
    trails = [ax.plot([], [], '-', color=cmap(i), alpha=0.35, linewidth=1.2)[0]
              for i in range(N_DRONES)]
    time_text = ax.text(10, 975, '', color='white', fontsize=9)

    # Pre-build conflict circle patches (initially invisible)
    conflict_circles = []
    for cfl in conflicts_raw:
        drone = intents[cfl[0]] if cfl[0] < len(intents) else None
        if drone:
            p = drone.position_at(cfl[2])
            circ = plt.Circle((p[0], p[1]), D_SEP_H, color='red', fill=False,
                              linewidth=1.5, alpha=0.0)
            ax.add_patch(circ)
            conflict_circles.append((cfl[2], circ))

    trail_len = 20  # frames

    def init():
        for d_dot, d_trail in zip(dots, trails):
            d_dot.set_data([], [])
            d_trail.set_data([], [])
        time_text.set_text('')
        return dots + trails + [time_text]

    def update(frame_idx):
        t_idx = list(frames)[frame_idx]
        t     = t_grid[t_idx]

        for i, drone in enumerate(intents):
            _, pos = history[drone.id][t_idx]
            dots[i].set_data([pos[0]], [pos[1]])
            # Trail
            t0_idx = max(0, t_idx - trail_len * step)
            xs = [history[drone.id][j][1][0] for j in range(t0_idx, t_idx + 1, step)]
            ys = [history[drone.id][j][1][1] for j in range(t0_idx, t_idx + 1, step)]
            trails[i].set_data(xs, ys)

        # Pulse conflict circles near current time
        for t_c, circ in conflict_circles:
            alpha = max(0.0, 0.7 - abs(t - t_c) / 20.0)
            circ.set_alpha(alpha)

        time_text.set_text(f't = {t:.1f} s')
        return dots + trails + [time_text] + [c for _, c in conflict_circles]

    frames_list = list(frames)
    ani = animation.FuncAnimation(
        fig, update, frames=len(frames_list),
        init_func=init, blit=True, interval=60,
    )

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Entry Point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    data = run_simulation()
    (corridors, intents_orig,
     result_fcfs, result_priority, result_lp,
     delays_fcfs, delays_priority, delays_lp,
     conflicts_raw, conflicts_fcfs, conflicts_priority, conflicts_lp,
     metrics_fcfs, metrics_priority, metrics_lp) = data

    print("\n=== Key Metrics Summary ===")
    print(f"  Raw conflicts (before resolution): {len(conflicts_raw)}")
    for name, m in [('FCFS',     metrics_fcfs),
                    ('Priority', metrics_priority),
                    ('LP',       metrics_lp)]:
        print(f"  [{name:10s}]  post-conflicts={m['n_conflicts']}  "
              f"total_delay={m['total_delay']:.1f}s  "
              f"mean_delay={m['mean_delay']:.1f}s  "
              f"throughput={m['throughput']:.3f} del/min  "
              f"conflict_rate={m['conflict_rate']:.4f}/pair/min  "
              f"RA_events={m['n_ra']}")

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_3d_airspace(corridors, intents_orig, result_fcfs, conflicts_raw, out_dir)
    plot_conflict_timeline(intents_orig, result_fcfs, conflicts_raw, out_dir)
    plot_separation_histogram(result_fcfs, result_priority, result_lp, out_dir)
    plot_delay_comparison(metrics_fcfs, metrics_priority, metrics_lp, out_dir)
    plot_corridor_utilisation(corridors, result_fcfs, out_dir)
    save_animation(corridors, result_fcfs, conflicts_raw, out_dir)
