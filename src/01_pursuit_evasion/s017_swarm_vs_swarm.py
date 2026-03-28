"""
S017 Swarm vs Swarm
===================
M pursuers compete against N evaders in an open 2D arena. Two cases are studied:
3v3 (symmetric) and 5v3 (pursuer advantage). The Hungarian algorithm is used for
optimal assignment, and results are compared against greedy nearest-neighbour
assignment. When an evader is captured, its pursuer dynamically reassigns to the
nearest remaining evader. Evaders scatter away from the centroid of all pursuers.
This scenario explores how assignment strategy and pursuer count affect total
mission time.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s017_swarm_vs_swarm.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.optimize import linear_sum_assignment

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ──────────────────────────────────────────────────────────────
V_PURSUER = 5.0    # m/s — pursuer speed
V_EVADER  = 3.5    # m/s — evader speed
CAPTURE_R = 0.15   # m   — capture radius
DT        = 0.02   # s   — timestep
T_MAX     = 30.0   # s   — max simulation time

# Case A: 3 pursuers vs 3 evaders
PURSUERS_3 = np.array([[-5.0, -2.0], [-5.0,  0.0], [-5.0,  2.0]], dtype=float)
EVADERS_3  = np.array([[ 0.0, -2.0], [ 0.0,  0.0], [ 0.0,  2.0]], dtype=float)

# Case B: 5 pursuers vs 3 evaders (same 3 evaders)
PURSUERS_5 = np.array([[-5.0, -3.0], [-5.0, -1.5], [-5.0,  0.0],
                        [-5.0,  1.5], [-5.0,  3.0]], dtype=float)

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's017_swarm_vs_swarm',
)

RNG = np.random.default_rng(0)   # fixed seed for reproducibility

# ── Helpers ─────────────────────────────────────────────────────────────────

def build_cost_matrix(pursuers, evaders, captured):
    """Build Euclidean cost matrix M×N (only uncaptured evaders in active columns)."""
    M = len(pursuers)
    N = len(evaders)
    C = np.full((M, N), 1e9)
    for i in range(M):
        for j in range(N):
            if not captured[j]:
                C[i, j] = np.linalg.norm(pursuers[i] - evaders[j])
    return C


def hungarian_assignment(pursuers, evaders, captured):
    """
    Assign each pursuer to an evader using the Hungarian algorithm.
    For M > N or partially captured evaders, dummy (infinite-cost) columns
    ensure a valid M-sized row index is returned.
    Returns a dict: pursuer_idx -> evader_idx.
    """
    active_j = [j for j in range(len(evaders)) if not captured[j]]
    if not active_j:
        return {}
    M = len(pursuers)
    N_act = len(active_j)

    # Build sub-cost matrix over active evaders, padded to be at least M×M
    pad = max(M, N_act)
    C_sub = np.full((M, pad), 1e9)
    for col_idx, j in enumerate(active_j):
        for i in range(M):
            C_sub[i, col_idx] = np.linalg.norm(pursuers[i] - evaders[j])

    row_ind, col_ind = linear_sum_assignment(C_sub)
    assignment = {}
    for r, c in zip(row_ind, col_ind):
        if c < N_act:          # valid real evader column
            assignment[r] = active_j[c]
        # if c >= N_act it is a dummy column — pursuer has no assignment yet
    return assignment


def greedy_assignment(pursuers, evaders, captured):
    """
    Greedy nearest-neighbour: each pursuer in order takes the nearest
    unclaimed, uncaptured evader.  Surplus pursuers are assigned to the
    nearest remaining evader (shared assignment allowed).
    """
    active_j = [j for j in range(len(evaders)) if not captured[j]]
    if not active_j:
        return {}
    M = len(pursuers)
    assignment = {}
    taken = []
    for i in range(M):
        candidates = [j for j in active_j if j not in taken]
        if not candidates:
            # All evaders claimed — assign to nearest (shared)
            dists = [np.linalg.norm(pursuers[i] - evaders[j]) for j in active_j]
            assignment[i] = active_j[int(np.argmin(dists))]
        else:
            dists = [np.linalg.norm(pursuers[i] - evaders[j]) for j in candidates]
            best = candidates[int(np.argmin(dists))]
            assignment[i] = best
            taken.append(best)
    return assignment


def nearest_reassign(pursuer_pos, evaders, captured):
    """Return index of nearest uncaptured evader, or None if all captured."""
    remaining = [j for j in range(len(evaders)) if not captured[j]]
    if not remaining:
        return None
    dists = [np.linalg.norm(pursuer_pos - evaders[j]) for j in remaining]
    return remaining[int(np.argmin(dists))]


# ── Simulation ───────────────────────────────────────────────────────────────

def run_simulation_case(pursuers_init, evaders_init, method='hungarian'):
    """
    Run one swarm-vs-swarm engagement.
    Returns:
        capture_times : list[float|None]  — time each evader was captured
        mission_time  : float             — time of last capture (T_mission)
        traj_p        : list[ndarray]     — trajectory per pursuer  (T×2)
        traj_e        : list[ndarray]     — trajectory per evader   (T×2)
        assignment_hist : list[dict]      — assignment at each step (for plotting)
        capture_steps : list[int|None]    — step index when each evader captured
    """
    pursuers = pursuers_init.copy()
    evaders  = evaders_init.copy()
    M = len(pursuers)
    N = len(evaders)

    captured     = [False] * N
    capture_times = [None] * N
    capture_steps = [None] * N

    # Trajectory storage
    traj_p = [[] for _ in range(M)]
    traj_e = [[] for _ in range(N)]
    for i in range(M):
        traj_p[i].append(pursuers[i].copy())
    for j in range(N):
        traj_e[j].append(evaders[j].copy())

    assignment_hist = []

    # Initial assignment
    if method == 'hungarian':
        assignment = hungarian_assignment(pursuers, evaders, captured)
    else:
        assignment = greedy_assignment(pursuers, evaders, captured)

    n_steps = int(T_MAX / DT)
    for step in range(n_steps):
        t = (step + 1) * DT

        # ── Move evaders: scatter from centroid of all pursuers ──────────────
        centroid = pursuers.mean(axis=0)
        for j in range(N):
            if not captured[j]:
                direction = evaders[j] - centroid
                norm = np.linalg.norm(direction)
                if norm < 1e-8:
                    # Edge case: evader is exactly at centroid — move at random
                    angle = RNG.uniform(0, 2 * np.pi)
                    direction = np.array([np.cos(angle), np.sin(angle)])
                    norm = 1.0
                evaders[j] = evaders[j] + (direction / norm) * V_EVADER * DT

        # ── Move pursuers toward assigned evader ──────────────────────────────
        for i in range(M):
            target_j = assignment.get(i)

            # Reassign if target was captured or no assignment
            if target_j is None or captured[target_j]:
                target_j = nearest_reassign(pursuers[i], evaders, captured)
                if target_j is None:
                    break  # all captured
                assignment[i] = target_j

            direction = evaders[target_j] - pursuers[i]
            norm = np.linalg.norm(direction)
            if norm > 1e-8:
                pursuers[i] = pursuers[i] + (direction / norm) * V_PURSUER * DT

            # Check capture
            if np.linalg.norm(pursuers[i] - evaders[target_j]) < CAPTURE_R:
                if not captured[target_j]:
                    captured[target_j] = True
                    capture_times[target_j] = t
                    capture_steps[target_j] = step
                    # Reassign this pursuer immediately
                    new_j = nearest_reassign(pursuers[i], evaders, captured)
                    assignment[i] = new_j  # may be None if all done

        assignment_hist.append(dict(assignment))

        # Record trajectories
        for i in range(M):
            traj_p[i].append(pursuers[i].copy())
        for j in range(N):
            traj_e[j].append(evaders[j].copy())

        if all(captured):
            break

    # Convert to arrays
    traj_p = [np.array(t) for t in traj_p]
    traj_e = [np.array(t) for t in traj_e]

    valid_times = [t for t in capture_times if t is not None]
    mission_time = max(valid_times) if valid_times else T_MAX

    return capture_times, mission_time, traj_p, traj_e, assignment_hist, capture_steps


def run_simulation():
    """Run all 4 cases: (3v3, 5v3) × (Hungarian, Greedy)."""
    cases = {
        '3v3_hungarian': (PURSUERS_3, EVADERS_3, 'hungarian'),
        '3v3_greedy':    (PURSUERS_3, EVADERS_3, 'greedy'),
        '5v3_hungarian': (PURSUERS_5, EVADERS_3, 'hungarian'),
        '5v3_greedy':    (PURSUERS_5, EVADERS_3, 'greedy'),
    }
    results = {}
    for key, (p_init, e_init, method) in cases.items():
        cap_times, mission_t, traj_p, traj_e, assign_hist, cap_steps = \
            run_simulation_case(p_init, e_init, method)
        results[key] = {
            'capture_times': cap_times,
            'mission_time':  mission_t,
            'traj_p':        traj_p,
            'traj_e':        traj_e,
            'assign_hist':   assign_hist,
            'capture_steps': cap_steps,
            'n_pursuers':    len(p_init),
            'method':        method,
        }
        print(f"  {key:20s}: capture_times={[f'{t:.2f}s' if t else 'None' for t in cap_times]}"
              f"  mission_time={mission_t:.3f}s")
    return results


# ── Plots ────────────────────────────────────────────────────────────────────

PURSUER_COLORS_3 = ['#e63946', '#f4a261', '#2a9d8f']
PURSUER_COLORS_5 = ['#e63946', '#f4a261', '#2a9d8f', '#8338ec', '#fb8500']
EVADER_COLORS    = ['#023e8a', '#0077b6', '#00b4d8']


def plot_trajectories(results, out_dir):
    """2×2 panel: top-down 2D trajectories for all 4 cases."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    case_keys = ['3v3_hungarian', '3v3_greedy', '5v3_hungarian', '5v3_greedy']
    titles = ['3v3 — Hungarian', '3v3 — Greedy', '5v3 — Hungarian', '5v3 — Greedy']

    for ax, key, title in zip(axes.flat, case_keys, titles):
        res = results[key]
        n_p = res['n_pursuers']
        p_colors = PURSUER_COLORS_3 if n_p == 3 else PURSUER_COLORS_5

        # Collect all trajectory points for axis limits
        all_pts = []
        for tp in res['traj_p']:
            all_pts.append(tp)
        for te in res['traj_e']:
            all_pts.append(te)
        all_pts = np.concatenate(all_pts, axis=0)
        margin = 1.5
        x_min, x_max = all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin
        y_min, y_max = all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin

        # Plot pursuer trajectories
        for i, tp in enumerate(res['traj_p']):
            color = p_colors[i % len(p_colors)]
            ax.plot(tp[:, 0], tp[:, 1], color=color, linewidth=1.4, alpha=0.85,
                    label=f'P{i+1}' if i < n_p else None)
            ax.scatter(tp[0, 0], tp[0, 1], color=color, s=80, marker='o',
                       edgecolors='black', linewidths=0.8, zorder=6)
            ax.scatter(tp[-1, 0], tp[-1, 1], color=color, s=60, marker='s',
                       edgecolors='black', linewidths=0.8, zorder=6)

        # Plot evader trajectories
        for j, te in enumerate(res['traj_e']):
            color = EVADER_COLORS[j]
            cap_step = res['capture_steps'][j]
            end_idx = cap_step + 1 if cap_step is not None else len(te)
            ax.plot(te[:end_idx, 0], te[:end_idx, 1], color=color,
                    linewidth=1.4, linestyle='--', alpha=0.85, label=f'E{j+1}')
            ax.scatter(te[0, 0], te[0, 1], color=color, s=80, marker='^',
                       edgecolors='black', linewidths=0.8, zorder=6)
            if cap_step is not None:
                ax.scatter(te[cap_step, 0], te[cap_step, 1], color=color,
                           s=150, marker='x', linewidths=2.5, zorder=7)

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.25)
        ax.set_xlabel('X (m)', fontsize=8)
        ax.set_ylabel('Y (m)', fontsize=8)
        ax.set_title(f'S017 {title}\nMission time = {res["mission_time"]:.2f}s', fontsize=9)
        ax.legend(fontsize=7, loc='lower right', ncol=2)

        # Label start positions
        if n_p == 3:
            p_init = PURSUERS_3
        else:
            p_init = PURSUERS_5
        for i, pos in enumerate(p_init):
            ax.annotate(f'P{i+1}', xy=(pos[0], pos[1]),
                        xytext=(pos[0] - 0.3, pos[1] + 0.3), fontsize=7,
                        color=p_colors[i % len(p_colors)])
        for j, pos in enumerate(EVADERS_3):
            ax.annotate(f'E{j+1}', xy=(pos[0], pos[1]),
                        xytext=(pos[0] + 0.2, pos[1] + 0.3), fontsize=7,
                        color=EVADER_COLORS[j])

    plt.suptitle('S017 Swarm vs Swarm — Top-Down Trajectories\n'
                 'Circles=pursuer start, Triangles=evader start, X=capture, Squares=final',
                 fontsize=10, fontweight='bold')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_gantt(results, out_dir):
    """Gantt chart: per-evader capture time bars for all 4 cases."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    case_pairs = [
        ('3v3_hungarian', '3v3_greedy', '3v3'),
        ('5v3_hungarian', '5v3_greedy', '5v3'),
    ]

    for ax, (key_h, key_g, label) in zip(axes, case_pairs):
        n_evaders = len(EVADERS_3)
        bar_height = 0.35
        y_h = np.arange(n_evaders) + bar_height / 2 + 0.05
        y_g = np.arange(n_evaders) - bar_height / 2 - 0.05

        res_h = results[key_h]
        res_g = results[key_g]

        for j in range(n_evaders):
            t_h = res_h['capture_times'][j]
            t_g = res_g['capture_times'][j]
            color = EVADER_COLORS[j]

            if t_h is not None:
                ax.barh(y_h[j], t_h, height=bar_height, color=color, alpha=0.85,
                        edgecolor='black', linewidth=0.5)
                ax.text(t_h + 0.1, y_h[j], f'{t_h:.2f}s', va='center', fontsize=8)
            if t_g is not None:
                ax.barh(y_g[j], t_g, height=bar_height, color=color, alpha=0.55,
                        edgecolor='black', linewidth=0.5, hatch='//')
                ax.text(t_g + 0.1, y_g[j], f'{t_g:.2f}s', va='center', fontsize=8)

        # Mission time vertical lines
        ax.axvline(res_h['mission_time'], color='navy', linestyle='-', linewidth=1.6,
                   label=f'Hungarian T_mission={res_h["mission_time"]:.2f}s')
        ax.axvline(res_g['mission_time'], color='darkred', linestyle='--', linewidth=1.6,
                   label=f'Greedy T_mission={res_g["mission_time"]:.2f}s')

        ax.set_yticks(np.arange(n_evaders))
        ax.set_yticklabels([f'E{j+1}' for j in range(n_evaders)], fontsize=9)
        ax.set_xlabel('Capture Time (s)', fontsize=9)
        ax.set_title(f'{label} — Capture Gantt Chart\n'
                     'Solid=Hungarian, Hatched=Greedy', fontsize=9)
        ax.legend(fontsize=7, loc='lower right')
        ax.grid(True, alpha=0.25, axis='x')

    plt.suptitle('S017 Swarm vs Swarm — Evader Capture Times (Gantt Chart)',
                 fontsize=10, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'gantt_chart.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_mission_time_comparison(results, out_dir):
    """Bar chart comparing total mission time across all 4 cases."""
    labels = ['3v3\nGreedy', '3v3\nHungarian', '5v3\nGreedy', '5v3\nHungarian']
    keys   = ['3v3_greedy', '3v3_hungarian', '5v3_greedy', '5v3_hungarian']
    times  = [results[k]['mission_time'] for k in keys]
    colors = ['#f4a261', '#2a9d8f', '#e63946', '#8338ec']

    fig, ax = plt.subplots(figsize=(9, 6))
    bars = ax.bar(labels, times, color=colors, width=0.5, alpha=0.87,
                  edgecolor='black', linewidth=0.8)

    for bar, t in zip(bars, times):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.2,
                f'{t:.2f}s', ha='center', va='bottom', fontsize=10, fontweight='bold')

    ax.set_ylabel('Total Mission Time (s)', fontsize=10)
    ax.set_title('S017 Swarm vs Swarm — Total Mission Time Comparison\n'
                 '(Lower is better for pursuers)', fontsize=10)
    ax.set_ylim(0, max(times) * 1.2)
    ax.grid(True, alpha=0.3, axis='y')

    # Annotate advantage
    for i, (lbl, t) in enumerate(zip(labels, times)):
        pass  # bars already annotated

    plt.tight_layout()
    path = os.path.join(out_dir, 'mission_time_comparison.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_assignment_bipartite(results, out_dir):
    """
    Visualize the initial assignment bipartite graph at t=0 for all 4 cases.
    Shows how pursuers are linked to evaders by the two assignment methods.
    """
    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    case_keys = ['3v3_hungarian', '3v3_greedy', '5v3_hungarian', '5v3_greedy']
    titles    = ['3v3 — Hungarian', '3v3 — Greedy', '5v3 — Hungarian', '5v3 — Greedy']

    for ax, key, title in zip(axes.flat, case_keys, titles):
        res = results[key]
        n_p = res['n_pursuers']
        p_colors = PURSUER_COLORS_3 if n_p == 3 else PURSUER_COLORS_5
        p_init = PURSUERS_3 if n_p == 3 else PURSUERS_5

        # First-step assignment
        first_assign = res['assign_hist'][0] if res['assign_hist'] else {}

        # Layout: pursuers on left (x=-1), evaders on right (x=1)
        p_y = np.linspace(1, -1, n_p)
        e_y = np.linspace(0.8, -0.8, 3)

        # Draw assignment edges
        for i, j in first_assign.items():
            if j is not None:
                color = p_colors[i % len(p_colors)]
                ax.plot([-1, 1], [p_y[i], e_y[j]], color=color, linewidth=2.0,
                        alpha=0.75, zorder=2)
                # Cost label (Euclidean distance)
                dist = np.linalg.norm(p_init[i] - EVADERS_3[j])
                mid_x, mid_y = 0, (p_y[i] + e_y[j]) / 2
                ax.text(mid_x, mid_y + 0.04, f'{dist:.1f}m', fontsize=7,
                        ha='center', color=color, alpha=0.8)

        # Draw pursuer nodes
        for i in range(n_p):
            color = p_colors[i % len(p_colors)]
            ax.scatter(-1, p_y[i], s=220, color=color, edgecolors='black',
                       linewidths=1.2, zorder=5)
            ax.text(-1.25, p_y[i], f'P{i+1}', ha='right', va='center',
                    fontsize=9, color=color, fontweight='bold')

        # Draw evader nodes
        for j in range(3):
            color = EVADER_COLORS[j]
            ax.scatter(1, e_y[j], s=220, color=color, marker='^',
                       edgecolors='black', linewidths=1.2, zorder=5)
            ax.text(1.25, e_y[j], f'E{j+1}', ha='left', va='center',
                    fontsize=9, color=color, fontweight='bold')

        ax.set_xlim(-1.8, 1.8)
        ax.set_ylim(-1.4, 1.4)
        ax.axis('off')
        ax.set_title(f'S017 {title}\n'
                     f'T_mission={res["mission_time"]:.2f}s', fontsize=9)

    plt.suptitle('S017 Swarm vs Swarm — Initial Assignment Bipartite Graph (t=0)\n'
                 'Edge = pursuer→evader assignment, label = initial Euclidean distance',
                 fontsize=10, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'assignment_bipartite.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    """
    Animate the 3v3 Hungarian case (clean, representative engagement).
    Shows pursuer and evader positions over time with trajectory tails.
    """
    key = '3v3_hungarian'
    res = results[key]
    traj_p = res['traj_p']
    traj_e = res['traj_e']
    capture_steps = res['capture_steps']

    n_p = len(traj_p)
    max_len = max(len(tp) for tp in traj_p)

    # All points for axis limits
    all_pts = np.concatenate([tp for tp in traj_p] + [te for te in traj_e], axis=0)
    margin = 2.0
    x_min, x_max = all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin
    y_min, y_max = all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin

    step = 2   # frame decimation
    n_frames = max_len // step

    fig, ax = plt.subplots(figsize=(9, 8))
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.2)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    title = ax.set_title('')

    # Trail lines
    p_lines = []
    e_lines = []
    p_dots  = []
    e_dots  = []

    for i in range(n_p):
        color = PURSUER_COLORS_3[i]
        pl, = ax.plot([], [], color=color, linewidth=1.3, alpha=0.8)
        pd, = ax.plot([], [], color=color, marker='o', markersize=9, linestyle='',
                      markeredgecolor='black', markeredgewidth=0.8, zorder=6)
        p_lines.append(pl); p_dots.append(pd)

    for j in range(len(traj_e)):
        color = EVADER_COLORS[j]
        el, = ax.plot([], [], color=color, linewidth=1.3, linestyle='--', alpha=0.8)
        ed, = ax.plot([], [], color=color, marker='^', markersize=9, linestyle='',
                      markeredgecolor='black', markeredgewidth=0.8, zorder=6)
        e_lines.append(el); e_dots.append(ed)

    # Capture markers (drawn statically when triggered)
    cap_markers = [None] * len(traj_e)

    def update(frame):
        si = min(frame * step, max_len - 1)
        t_cur = si * DT

        for i in range(n_p):
            tp = traj_p[i]
            n = min(si + 1, len(tp))
            p_lines[i].set_data(tp[:n, 0], tp[:n, 1])
            p_dots[i].set_data([tp[n-1, 0]], [tp[n-1, 1]])

        for j in range(len(traj_e)):
            te = traj_e[j]
            cap = capture_steps[j]
            end = min(si + 1, len(te)) if cap is None else min(si + 1, cap + 2, len(te))
            e_lines[j].set_data(te[:end, 0], te[:end, 1])
            if cap is not None and si >= cap:
                e_dots[j].set_data([te[cap, 0]], [te[cap, 1]])
                e_dots[j].set_marker('x')
                e_dots[j].set_markersize(12)
            else:
                n = min(si + 1, len(te))
                e_dots[j].set_data([te[n-1, 0]], [te[n-1, 1]])
                e_dots[j].set_marker('^')
                e_dots[j].set_markersize(9)

        title.set_text(f'S017 Swarm vs Swarm — 3v3 Hungarian   t={t_cur:.2f}s')
        return p_lines + e_lines + p_dots + e_dots + [title]

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    print('S017 Swarm vs Swarm — Simulation Results')
    print('=' * 55)
    results = run_simulation()

    print('\nMission time summary:')
    for key, res in results.items():
        cap_ts = [f'{t:.2f}s' if t is not None else 'None' for t in res['capture_times']]
        print(f'  {key:20s}: T_mission={res["mission_time"]:.3f}s  captures={cap_ts}')

    # Hungarian vs Greedy improvement
    for scenario in ['3v3', '5v3']:
        t_h = results[f'{scenario}_hungarian']['mission_time']
        t_g = results[f'{scenario}_greedy']['mission_time']
        saving = t_g - t_h
        print(f'\n  {scenario} Hungarian advantage over Greedy: {saving:+.3f}s '
              f'({saving/t_g*100:+.1f}%)')

    print(f'\n  5v3 vs 3v3 mission time speedup (Hungarian): '
          f'{results["3v3_hungarian"]["mission_time"] - results["5v3_hungarian"]["mission_time"]:.3f}s')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories(results, out_dir)
    plot_gantt(results, out_dir)
    plot_mission_time_comparison(results, out_dir)
    plot_assignment_bipartite(results, out_dir)
    save_animation(results, out_dir)
