"""
S019 3D Upgrade — Dynamic Target Reassignment
Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s019_3d_dynamic_reassignment.py
"""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.optimize import linear_sum_assignment
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'outputs', '01_pursuit_evasion', '3d', 's019_3d_dynamic_reassignment')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ─────────────────────────────────
M           = 3       # pursuers
N           = 2       # evaders (start with 2; one disappears, one new appears)
V_PURSUER   = 5.0     # m/s pursuer speed
V_EVADER    = 3.5     # m/s evader speed
CAPTURE_R   = 0.30    # m
DT          = 0.05    # s
T_MAX       = 20.0    # s
T_DISAPPEAR = 5.0     # s: evader 1 disappears
T_APPEAR    = 8.0     # s: new evader appears

RNG = np.random.default_rng(42)

# Initial state (from spec)
PURSUER_STARTS = np.array([[-5., -3., 1.0],
                             [-5.,  0., 3.0],
                             [-5.,  3., 5.0]], dtype=float)
EVADER_STARTS  = np.array([[20.,  6., 2.0],
                             [20., -6., 4.0]], dtype=float)

# New evader random position (fixed seed for reproducibility)
_rng_ev = np.random.default_rng(7)
NEW_EVADER_POS = np.array([
    _rng_ev.uniform(10, 20),
    _rng_ev.uniform(-5, 5),
    _rng_ev.uniform(0.5, 6.0)
])

PURSUER_COLORS = ['crimson', 'tomato', 'firebrick']
EVADER_COLORS  = ['royalblue', 'steelblue', 'navy', 'deepskyblue']
STRATEGY_COLORS = {'static': 'darkorange', 'hungarian_reassign': 'seagreen', 'altitude_aware': 'mediumpurple'}


# ── Cost matrix helpers ─────────────────────────────────
def cost_matrix_3d(pos_p, pos_e, alt_weight=2.0):
    """Altitude-aware cost matrix (used only for altitude_aware strategy)."""
    C = np.zeros((len(pos_p), len(pos_e)))
    for i, pp in enumerate(pos_p):
        for j, pe in enumerate(pos_e):
            C[i, j] = np.linalg.norm(pp - pe) + alt_weight * abs(pp[2] - pe[2])
    return C


def cost_matrix_euclidean(pos_p, pos_e):
    """Standard 3D Euclidean cost matrix."""
    C = np.zeros((len(pos_p), len(pos_e)))
    for i, pp in enumerate(pos_p):
        for j, pe in enumerate(pos_e):
            C[i, j] = np.linalg.norm(pp - pe)
    return C


def hungarian(C):
    """Return full assignment array of length nrows (mapped to available cols)."""
    rows, cols = linear_sum_assignment(C)
    return dict(zip(rows.tolist(), cols.tolist()))


# ── Simulation ─────────────────────────────────
def run_simulation(strategy):
    """
    strategy: 'static' | 'hungarian_reassign' | 'altitude_aware'
    Returns dict with trajectories, events, assignments.
    """
    pursuers = PURSUER_STARTS.copy()
    # Start with 2 evaders; evader indices: 0,1 initially; after disappear/appear index map changes
    # We maintain a dynamic list of active evader positions
    evader_list = list(EVADER_STARTS.copy())  # each entry: np.array shape (3,)
    evader_active = [True, True]              # whether each initial evader is active
    # Extra slot for new evader (index 2)
    new_evader_pos = NEW_EVADER_POS.copy()
    new_evader_active = False
    new_evader_appeared = False

    # Assignment: pursuer_i -> evader_slot_j (0,1 initially; 2 = new evader)
    # Build initial cost matrix over active evaders
    active_idx = [j for j in range(len(evader_list)) if evader_active[j]]
    pos_e_active = np.array([evader_list[j] for j in active_idx])

    if strategy == 'altitude_aware':
        C0 = cost_matrix_3d(pursuers, pos_e_active)
    else:
        C0 = cost_matrix_euclidean(pursuers, pos_e_active)

    raw_assign = hungarian(C0)  # pursuer_i -> index into active list (only assigned pursuers)
    # Convert to global evader slot index; unassigned pursuers get nearest evader
    assignment = {}
    for i in range(M):
        if i in raw_assign:
            assignment[i] = active_idx[raw_assign[i]]
        else:
            # Assign to nearest active evader
            assignment[i] = active_idx[int(np.argmin([np.linalg.norm(pursuers[i] - pos_e_active[k]) for k in range(len(active_idx))]))]

    n_reassignments = 0
    disappear_event_time = None
    appear_event_time = None

    # Evader escape directions (initialized away from nearest pursuer)
    ev_dirs = []
    for j in range(len(evader_list)):
        d = evader_list[j] - pursuers[np.argmin([np.linalg.norm(pursuers[i] - evader_list[j]) for i in range(M)])]
        n = np.linalg.norm(d) + 1e-8
        ev_dirs.append(d / n)
    ev_dirs.append(np.array([1., 0., 0.]))  # placeholder for new evader

    # Capture tracking
    captured_slots = set()
    capture_times = {}

    # Trajectory storage
    p_traj = [pursuers.copy()]
    e_traj_slots = [[evader_list[j].copy()] for j in range(len(evader_list))]
    e_traj_slots.append([new_evader_pos.copy()])  # slot 2 (not active yet)
    assign_hist = [assignment.copy()]
    event_times = []  # (time, event_type)
    distance_hist = []  # per-step: [dist p0-assigned, p1-assigned, p2-assigned]

    max_steps = int(T_MAX / DT)

    def get_all_active_evader_positions():
        """Return dict slot_idx -> pos for all active evaders."""
        pos = {}
        for j in range(len(evader_list)):
            if evader_active[j] and j not in captured_slots:
                pos[j] = evader_list[j].copy()
        if new_evader_active and 2 not in captured_slots:
            pos[2] = new_evader_pos.copy()
        return pos

    def reassign(current_assignment, pursuers, strategy_name):
        """Compute new Hungarian assignment over currently active evaders."""
        active_pos_dict = get_all_active_evader_positions()
        if not active_pos_dict:
            return current_assignment
        slots = sorted(active_pos_dict.keys())
        pos_e = np.array([active_pos_dict[s] for s in slots])

        if strategy_name == 'altitude_aware':
            C = cost_matrix_3d(pursuers, pos_e)
        else:
            C = cost_matrix_euclidean(pursuers, pos_e)

        raw = hungarian(C)
        new_assign = current_assignment.copy()
        for i in range(M):
            if i in raw:
                new_assign[i] = slots[raw[i]]
            else:
                # Assign to nearest active evader
                new_assign[i] = slots[int(np.argmin([np.linalg.norm(pursuers[i] - active_pos_dict[s]) for s in slots]))]
        return new_assign

    for step in range(max_steps):
        t = step * DT

        # ── Evader 1 disappears at T_DISAPPEAR
        if not new_evader_appeared and t >= T_DISAPPEAR and evader_active[1] and 1 not in captured_slots:
            evader_active[1] = False
            disappear_event_time = t
            event_times.append((t, 'disappear'))
            # For reassign strategies: pursuer assigned to slot 1 holds position
            # (no action needed — pursuer simply won't move toward captured/inactive target)
            if strategy in ('hungarian_reassign', 'altitude_aware'):
                assignment = reassign(assignment, pursuers, strategy)
                n_reassignments += 1

        # ── New evader appears at T_APPEAR
        if not new_evader_appeared and t >= T_APPEAR:
            new_evader_active = True
            new_evader_appeared = True
            appear_event_time = t
            event_times.append((t, 'appear'))
            # Direction: away from nearest pursuer
            nearest_p = pursuers[np.argmin([np.linalg.norm(pursuers[i] - new_evader_pos) for i in range(M)])]
            d = new_evader_pos - nearest_p
            n = np.linalg.norm(d) + 1e-8
            ev_dirs[2] = d / n
            if strategy in ('hungarian_reassign', 'altitude_aware'):
                assignment = reassign(assignment, pursuers, strategy)
                n_reassignments += 1

        # ── Move evaders
        active_pos_dict = get_all_active_evader_positions()
        for slot, pos in active_pos_dict.items():
            # Find pursuer assigned to this slot
            assigned_pursuers = [i for i in range(M) if assignment.get(i) == slot]
            if assigned_pursuers:
                nearest_p_pos = pursuers[assigned_pursuers[0]]
            else:
                # No pursuer assigned: escape from nearest pursuer
                nearest_p_pos = pursuers[np.argmin([np.linalg.norm(pursuers[i] - pos) for i in range(M)])]
            away = pos - nearest_p_pos
            n = np.linalg.norm(away) + 1e-8
            ev_dirs[slot] = away / n
            new_pos = pos + ev_dirs[slot] * V_EVADER * DT
            new_pos[2] = np.clip(new_pos[2], 0.3, 7.0)
            if slot < len(evader_list):
                evader_list[slot] = new_pos
            else:
                new_evader_pos[:] = new_pos

        # ── Move pursuers
        for i in range(M):
            j = assignment.get(i, -1)
            # Check if assigned target is active
            target_active = (
                (j < len(evader_list) and evader_active[j] and j not in captured_slots) or
                (j == 2 and new_evader_active and 2 not in captured_slots)
            )
            if not target_active:
                # For static: just hold; for reassign: find new target
                if strategy == 'static':
                    continue  # hold position
                else:
                    active_pos_d = get_all_active_evader_positions()
                    if active_pos_d:
                        j_new = min(active_pos_d.keys(), key=lambda jj: np.linalg.norm(pursuers[i] - active_pos_d[jj]))
                        assignment[i] = j_new
                        j = j_new
                    else:
                        continue

            if j < len(evader_list):
                target_pos = evader_list[j].copy()
            else:
                target_pos = new_evader_pos.copy()

            diff = target_pos - pursuers[i]
            dist = np.linalg.norm(diff) + 1e-8
            step_size = min(V_PURSUER * DT, dist)
            pursuers[i] = pursuers[i] + (diff / dist) * step_size

        # ── Capture check
        for i in range(M):
            j = assignment.get(i, -1)
            if j in captured_slots:
                continue
            target_active = (
                (j < len(evader_list) and evader_active[j]) or
                (j == 2 and new_evader_active)
            )
            if not target_active:
                continue
            if j < len(evader_list):
                target_pos = evader_list[j].copy()
            else:
                target_pos = new_evader_pos.copy()
            if np.linalg.norm(pursuers[i] - target_pos) <= CAPTURE_R:
                captured_slots.add(j)
                capture_times[j] = t

        # ── Record distances
        dists = []
        for i in range(M):
            j = assignment.get(i, -1)
            target_active = (
                (j >= 0 and j < len(evader_list) and evader_active[j] and j not in captured_slots) or
                (j == 2 and new_evader_active and 2 not in captured_slots)
            )
            if target_active:
                if j < len(evader_list):
                    tp = evader_list[j].copy()
                else:
                    tp = new_evader_pos.copy()
                dists.append(np.linalg.norm(pursuers[i] - tp))
            else:
                dists.append(np.nan)
        distance_hist.append(dists)

        # ── Record trajectories
        p_traj.append(pursuers.copy())
        for j in range(len(evader_list)):
            e_traj_slots[j].append(evader_list[j].copy())
        e_traj_slots[2].append(new_evader_pos.copy())
        assign_hist.append(assignment.copy())

    return {
        'p_traj': np.array(p_traj),
        'e_traj_slots': [np.array(sl) for sl in e_traj_slots],
        'assign_hist': assign_hist,
        'distance_hist': np.array(distance_hist),
        'event_times': event_times,
        'n_reassignments': n_reassignments,
        'capture_times': capture_times,
        'T_DISAPPEAR': disappear_event_time,
        'T_APPEAR': appear_event_time,
        'new_evader_pos': NEW_EVADER_POS.copy(),
    }


# ── Plotting ─────────────────────────────────

def plot_trajectories_3d(data_dict):
    fig = plt.figure(figsize=(18, 6))
    strategies = ['static', 'hungarian_reassign', 'altitude_aware']
    titles = ['Static\n(Fixed at t=0)', 'Hungarian Reassign\n(Re-run on event)', 'Altitude-Aware\n(Hungarian + alt penalty)']

    for col, (strat, title) in enumerate(zip(strategies, titles)):
        d = data_dict[strat]
        ax = fig.add_subplot(1, 3, col + 1, projection='3d')

        p_traj = d['p_traj']
        e_traj_slots = d['e_traj_slots']
        n_steps = len(p_traj)
        t_dis = d['T_DISAPPEAR']
        t_app = d['T_APPEAR']
        t_dis_idx = int(t_dis / DT) if t_dis else n_steps
        t_app_idx = int(t_app / DT) if t_app else n_steps

        # Pursuer trajectories
        for i in range(M):
            ax.plot(p_traj[:, i, 0], p_traj[:, i, 1], p_traj[:, i, 2],
                    color=PURSUER_COLORS[i], lw=1.5, alpha=0.9, label=f'P{i}' if col == 0 else '')
            ax.scatter(*p_traj[0, i], color=PURSUER_COLORS[i], s=60, marker='D', zorder=5)
            ax.scatter(*p_traj[-1, i], color=PURSUER_COLORS[i], s=60, marker='s', zorder=5)

        # Evader 0 (always active)
        ax.plot(e_traj_slots[0][:, 0], e_traj_slots[0][:, 1], e_traj_slots[0][:, 2],
                color=EVADER_COLORS[0], lw=1.5, linestyle='--', alpha=0.8)
        ax.scatter(*e_traj_slots[0][0], color=EVADER_COLORS[0], s=60, marker='^', zorder=5)

        # Evader 1 (disappears at T_DISAPPEAR)
        ax.plot(e_traj_slots[1][:t_dis_idx, 0], e_traj_slots[1][:t_dis_idx, 1], e_traj_slots[1][:t_dis_idx, 2],
                color=EVADER_COLORS[1], lw=1.5, linestyle='--', alpha=0.8)
        ax.scatter(*e_traj_slots[1][0], color=EVADER_COLORS[1], s=60, marker='^', zorder=5)
        # Mark disappearance
        ax.scatter(*e_traj_slots[1][min(t_dis_idx, len(e_traj_slots[1]) - 1)],
                   color='red', s=120, marker='x', zorder=7)

        # New evader (slot 2, appears at T_APPEAR)
        ax.plot(e_traj_slots[2][t_app_idx:, 0], e_traj_slots[2][t_app_idx:, 1], e_traj_slots[2][t_app_idx:, 2],
                color=EVADER_COLORS[3], lw=1.5, linestyle='-.', alpha=0.9)
        ax.scatter(*e_traj_slots[2][t_app_idx], color=EVADER_COLORS[3], s=120, marker='*', zorder=7)

        ax.set_title(f'{title}\nReassigns: {d["n_reassignments"]}', fontsize=8)
        ax.set_xlabel('X (m)', fontsize=7)
        ax.set_ylabel('Y (m)', fontsize=7)
        ax.set_zlabel('Z (m)', fontsize=7)
        ax.tick_params(labelsize=6)

    # Legend handles
    from matplotlib.lines import Line2D
    handles = [
        Line2D([0], [0], color='gray', lw=1.5, label='Pursuer'),
        Line2D([0], [0], color=EVADER_COLORS[0], lw=1.5, linestyle='--', label='Evader 0 (active)'),
        Line2D([0], [0], color=EVADER_COLORS[1], lw=1.5, linestyle='--', label='Evader 1 (disappears)'),
        Line2D([0], [0], color=EVADER_COLORS[3], lw=1.5, linestyle='-.', label='New Evader (appears)'),
        Line2D([0], [0], marker='x', color='red', lw=0, markersize=8, label='Disappear event'),
        Line2D([0], [0], marker='*', color=EVADER_COLORS[3], lw=0, markersize=10, label='Appear event'),
    ]
    fig.legend(handles=handles, loc='lower center', ncol=6, fontsize=7, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle('S019 3D Dynamic Reassignment — Trajectories', fontsize=12, y=1.02)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_assignment_heatmap(data_dict):
    """For each strategy: which pursuer tracks which evader over time."""
    strategies = ['static', 'hungarian_reassign', 'altitude_aware']
    titles = ['Static', 'Hungarian Reassign', 'Altitude-Aware']
    fig, axes = plt.subplots(3, 1, figsize=(13, 9))

    for ax, strat, title in zip(axes, strategies, titles):
        d = data_dict[strat]
        assign_hist = d['assign_hist']
        n_steps = len(assign_hist)
        # Build matrix: rows=3 pursuers, cols=time, value=evader slot
        A = np.zeros((M, n_steps))
        for t_idx, asgn in enumerate(assign_hist):
            for i in range(M):
                A[i, t_idx] = asgn.get(i, -1)

        times = np.linspace(0, n_steps * DT, n_steps)
        im = ax.imshow(A, aspect='auto', cmap='tab10', vmin=-1, vmax=3,
                       extent=[0, n_steps * DT, M - 0.5, -0.5])

        # Mark events
        if d['T_DISAPPEAR']:
            ax.axvline(d['T_DISAPPEAR'], color='red', lw=2, linestyle='--', alpha=0.9, label=f'Disappear t={d["T_DISAPPEAR"]:.1f}s')
        if d['T_APPEAR']:
            ax.axvline(d['T_APPEAR'], color='lime', lw=2, linestyle='--', alpha=0.9, label=f'Appear t={d["T_APPEAR"]:.1f}s')

        ax.set_title(f'{title} — Reassignments: {d["n_reassignments"]}', fontsize=9)
        ax.set_xlabel('Time (s)', fontsize=8)
        ax.set_ylabel('Pursuer', fontsize=8)
        ax.set_yticks([0, 1, 2])
        ax.set_yticklabels(['P0', 'P1', 'P2'], fontsize=8)
        ax.legend(loc='upper right', fontsize=7)
        cbar = fig.colorbar(im, ax=ax, ticks=[-1, 0, 1, 2])
        cbar.ax.set_yticklabels(['None', 'E0', 'E1', 'E2(new)'], fontsize=7)

    fig.suptitle('S019 3D — Assignment Heatmap over Time', fontsize=11)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'assignment_heatmap.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_distance_time(data_dict):
    """Pursuer-to-assigned-evader distance for all 3 strategies."""
    strategies = ['static', 'hungarian_reassign', 'altitude_aware']
    labels = ['Static', 'Hungarian Reassign', 'Altitude-Aware']
    colors = [STRATEGY_COLORS[s] for s in strategies]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    for ax, strat, label, color in zip(axes, strategies, labels, colors):
        d = data_dict[strat]
        dist_hist = d['distance_hist']  # shape (n_steps, 3)
        n_steps = len(dist_hist)
        times = np.arange(n_steps) * DT

        for i in range(M):
            ax.plot(times, dist_hist[:, i], color=PURSUER_COLORS[i],
                    lw=1.5, alpha=0.85, label=f'P{i}')

        if d['T_DISAPPEAR']:
            ax.axvline(d['T_DISAPPEAR'], color='red', lw=1.5, linestyle='--', alpha=0.8, label='Disappear')
        if d['T_APPEAR']:
            ax.axvline(d['T_APPEAR'], color='lime', lw=1.5, linestyle='--', alpha=0.8, label='Appear')
        ax.axhline(CAPTURE_R, color='gold', lw=1.0, linestyle=':', label=f'Capture r={CAPTURE_R}m')

        ax.set_title(f'{label}\nReassigns: {d["n_reassignments"]}', fontsize=9)
        ax.set_xlabel('Time (s)', fontsize=8)
        ax.set_ylabel('Distance to Assigned Evader (m)', fontsize=8)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    fig.suptitle('S019 3D — Pursuer-to-Assigned-Evader Distance over Time', fontsize=11)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'distance_time.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(data_dict):
    """Animate altitude_aware strategy in 3D."""
    d = data_dict['altitude_aware']
    p_traj = d['p_traj']
    e_traj_slots = d['e_traj_slots']
    n_steps = len(p_traj)
    t_app_idx = int(d['T_APPEAR'] / DT) if d['T_APPEAR'] else n_steps
    t_dis_idx = int(d['T_DISAPPEAR'] / DT) if d['T_DISAPPEAR'] else n_steps

    step_dec = max(1, n_steps // 80)

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')

    all_pts = np.concatenate([p_traj.reshape(-1, 3)] + [sl.reshape(-1, 3) for sl in e_traj_slots], axis=0)
    margin = 2.0
    ax.set_xlim(all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin)
    ax.set_ylim(all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin)
    ax.set_zlim(max(0, all_pts[:, 2].min() - margin), all_pts[:, 2].max() + margin)
    ax.set_xlabel('X (m)', fontsize=7)
    ax.set_ylabel('Y (m)', fontsize=7)
    ax.set_zlabel('Z (m)', fontsize=7)

    p_lines = [ax.plot([], [], [], color=PURSUER_COLORS[i], lw=1.5)[0] for i in range(M)]
    p_dots  = [ax.plot([], [], [], 'o', color=PURSUER_COLORS[i], ms=7)[0] for i in range(M)]
    e0_line, = ax.plot([], [], [], color=EVADER_COLORS[0], lw=1.5, linestyle='--')
    e0_dot,  = ax.plot([], [], [], '^', color=EVADER_COLORS[0], ms=7)
    e1_line, = ax.plot([], [], [], color=EVADER_COLORS[1], lw=1.5, linestyle='--')
    e1_dot,  = ax.plot([], [], [], '^', color=EVADER_COLORS[1], ms=7)
    e2_line, = ax.plot([], [], [], color=EVADER_COLORS[3], lw=1.5, linestyle='-.')
    e2_dot,  = ax.plot([], [], [], '*', color=EVADER_COLORS[3], ms=9)
    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=8)

    all_artists = p_lines + p_dots + [e0_line, e0_dot, e1_line, e1_dot, e2_line, e2_dot, time_text]

    sub_p  = p_traj[::step_dec]
    sub_e0 = e_traj_slots[0][::step_dec]
    sub_e1 = e_traj_slots[1][::step_dec]
    sub_e2 = e_traj_slots[2][::step_dec]
    n_frames = len(sub_p)
    t_dis_sub = t_dis_idx // step_dec
    t_app_sub = t_app_idx // step_dec

    def init():
        for art in all_artists[:-1]:
            art.set_data([], [])
            art.set_3d_properties([])
        time_text.set_text('')
        return all_artists

    def update(frame):
        fp = min(frame, n_frames - 1)
        t_cur = fp * step_dec * DT
        for i in range(M):
            p_lines[i].set_data(sub_p[:fp+1, i, 0], sub_p[:fp+1, i, 1])
            p_lines[i].set_3d_properties(sub_p[:fp+1, i, 2])
            p_dots[i].set_data([sub_p[fp, i, 0]], [sub_p[fp, i, 1]])
            p_dots[i].set_3d_properties([sub_p[fp, i, 2]])

        e0_line.set_data(sub_e0[:fp+1, 0], sub_e0[:fp+1, 1])
        e0_line.set_3d_properties(sub_e0[:fp+1, 2])
        e0_dot.set_data([sub_e0[fp, 0]], [sub_e0[fp, 1]])
        e0_dot.set_3d_properties([sub_e0[fp, 2]])

        # E1 only shown up to disappear
        end1 = min(fp+1, t_dis_sub)
        if end1 > 0:
            e1_line.set_data(sub_e1[:end1, 0], sub_e1[:end1, 1])
            e1_line.set_3d_properties(sub_e1[:end1, 2])
            e1_dot.set_data([sub_e1[min(fp, t_dis_sub-1), 0]], [sub_e1[min(fp, t_dis_sub-1), 1]])
            e1_dot.set_3d_properties([sub_e1[min(fp, t_dis_sub-1), 2]])
        else:
            e1_line.set_data([], [])
            e1_line.set_3d_properties([])
            e1_dot.set_data([], [])
            e1_dot.set_3d_properties([])

        # E2 only shown after appear
        if fp >= t_app_sub:
            start2 = t_app_sub
            e2_line.set_data(sub_e2[start2:fp+1, 0], sub_e2[start2:fp+1, 1])
            e2_line.set_3d_properties(sub_e2[start2:fp+1, 2])
            e2_dot.set_data([sub_e2[fp, 0]], [sub_e2[fp, 1]])
            e2_dot.set_3d_properties([sub_e2[fp, 2]])
        else:
            e2_line.set_data([], [])
            e2_line.set_3d_properties([])
            e2_dot.set_data([], [])
            e2_dot.set_3d_properties([])

        time_text.set_text(f't = {t_cur:.1f}s')
        ax.set_title(f'S019 3D Altitude-Aware (t={t_cur:.1f}s)', fontsize=9)
        return all_artists

    ani = FuncAnimation(fig, update, frames=n_frames, init_func=init, blit=False, interval=80)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=12), dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────

if __name__ == '__main__':
    print('Running S019 3D Dynamic Reassignment simulations...')
    data = {}
    for strat in ['static', 'hungarian_reassign', 'altitude_aware']:
        print(f'  Strategy: {strat}')
        data[strat] = run_simulation(strat)
        d = data[strat]
        print(f'    Reassignments: {d["n_reassignments"]}  |  Captures: {d["capture_times"]}')

    print('\n' + '='*60)
    print('S019 3D Dynamic Reassignment — Summary')
    print('='*60)
    for strat in ['static', 'hungarian_reassign', 'altitude_aware']:
        d = data[strat]
        print(f'{strat:20s}  reassigns={d["n_reassignments"]:2d}  captures={d["capture_times"]}')
    print('='*60)

    print('\nGenerating plots...')
    plot_trajectories_3d(data)
    plot_assignment_heatmap(data)
    plot_distance_time(data)
    save_animation(data)
    print('All outputs saved to:', OUTPUT_DIR)
