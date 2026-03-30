"""
S013 3D Upgrade — Pincer Movement
===================================
Three pursuers form a tilted equilateral triangle around the evader's predicted
position (trident formation).  Triangle centroid tracks evader's predicted
position (lead 0.5 s ahead); formation normal tilts to face evader velocity.
Evader escapes via solid-angle maximisation in 3D.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s013_3d_pincer_movement.py
"""
import sys, os, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'outputs',
    '01_pursuit_evasion', '3d', 's013_3d_pincer_movement',
)
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ────────────────────────────────────────────────
N_PINCER   = 3
R0         = 3.0      # initial offset radius
R_MIN      = 0.3
V_SHRINK   = 0.3      # m/s — radius shrink rate
V_PURSUER  = 5.0
V_EVADER   = 3.5
PSI_INIT   = np.radians(45.0)
K_TILT     = 0.5      # rad/s — tilt gain
Z_MIN      = 0.5
Z_MAX      = 6.0
DT         = 1 / 48
MAX_TIME   = 20.0
CAPTURE_R  = 0.15
LEAD_TIME  = 0.5      # s — prediction lead

INIT_E     = np.array([ 2.0,  0.0,  2.0])
INIT_P     = np.array([
    [-4.0, -2.0, 4.0],   # P0: above
    [-4.0,  3.0, 2.0],   # P1: lateral
    [-4.0, -2.0, 0.8],   # P2: below
], dtype=float)

P_COLORS   = ['firebrick', 'darkorange', 'crimson']

# ── Geometry helpers ──────────────────────────────────────────

def rotation_x(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def pincer_targets_3d(pos_e: np.ndarray, vel_e: np.ndarray,
                       R: float, psi_tilt: float) -> np.ndarray:
    """Equilateral triangle in plane perpendicular to vel_e, tilted by psi_tilt."""
    # Lead position
    lead = pos_e + LEAD_TIME * vel_e
    # Build tilted equilateral triangle around lead
    R_tilt = rotation_x(psi_tilt)
    targets = []
    for i in range(N_PINCER):
        angle = 2.0 * np.pi * i / N_PINCER
        d_local = np.array([np.cos(angle), np.sin(angle), 0.0])
        d_world = R_tilt @ d_local
        targets.append(lead + R * d_world)
    return np.array(targets)


def blocking_plane_normal(pts: np.ndarray) -> np.ndarray:
    v1 = pts[1] - pts[0]
    v2 = pts[2] - pts[0]
    n  = np.cross(v1, v2)
    nn = np.linalg.norm(n)
    return n / nn if nn > 1e-8 else np.array([0., 0., 1.])


def evader_escape_3d(pos_e: np.ndarray, pos_p: np.ndarray,
                      v_max: float, K: int = 200) -> np.ndarray:
    """Maximise minimum angular distance to any pursuer."""
    dirs_to_p = pos_p - pos_e[np.newaxis, :]
    norms_ = np.linalg.norm(dirs_to_p, axis=1, keepdims=True) + 1e-8
    dirs_to_p = dirs_to_p / norms_
    # Sample candidates on sphere
    theta = np.arccos(np.clip(1 - 2 * np.random.rand(K), -1.0, 1.0))
    phi   = 2.0 * np.pi * np.random.rand(K)
    cands = np.column_stack([
        np.sin(theta) * np.cos(phi),
        np.sin(theta) * np.sin(phi),
        np.cos(theta),
    ])
    dots = np.clip(cands @ dirs_to_p.T, -1.0, 1.0)
    gaps = np.arccos(dots).min(axis=1)
    best = int(np.argmax(gaps))
    return v_max * cands[best]


# ── Simulation ────────────────────────────────────────────────

def run_simulation():
    np.random.seed(42)

    pos_p = INIT_P.copy()
    pos_e = INIT_E.copy()
    vel_e = np.array([0.5, 0.0, 0.1])   # initial guess; will be updated

    psi_tilt = PSI_INIT

    traj_e = [pos_e.copy()]
    traj_p = [pos_p.copy()]
    times  = [0.0]
    solid_angles = []
    dists_log    = []
    alt_log_e    = [pos_e[2]]
    alt_log_p    = [[pos_p[i, 2] for i in range(N_PINCER)]]
    capture_time = None

    for step in range(1, int(MAX_TIME / DT) + 1):
        t = step * DT
        R = max(R0 - V_SHRINK * t, R_MIN)

        # Update formation tilt toward evader altitude
        centroid = pos_p.mean(axis=0)
        dz_e  = pos_e[2] - centroid[2]
        dxy_e = np.linalg.norm(pos_e[:2] - centroid[:2]) + 1e-8
        psi_cmd = np.arctan2(dz_e, dxy_e)
        psi_tilt += K_TILT * (psi_cmd - psi_tilt) * DT
        psi_tilt  = np.clip(psi_tilt, -np.pi / 3, np.pi / 3)

        # Pursuer targets
        targets = pincer_targets_3d(pos_e, vel_e, R, psi_tilt)
        targets[:, 2] = np.clip(targets[:, 2], Z_MIN, Z_MAX)

        for i in range(N_PINCER):
            d = targets[i] - pos_p[i]
            nn = np.linalg.norm(d)
            step_v = V_PURSUER * d / nn if nn > 1e-8 else np.zeros(3)
            pos_p[i] += step_v * DT
            pos_p[i, 2] = np.clip(pos_p[i, 2], Z_MIN, Z_MAX)

        # Evader escape
        prev_e = pos_e.copy()
        v_e    = evader_escape_3d(pos_e, pos_p, V_EVADER)
        pos_e  = pos_e + v_e * DT
        pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)
        vel_e  = (pos_e - prev_e) / DT

        # Metrics
        dists = np.linalg.norm(pos_p - pos_e[np.newaxis, :], axis=1)
        omega_esc = max(4.0 * np.pi - 3.0 * np.pi * (CAPTURE_R / R) ** 2, 0.0)
        solid_angles.append(omega_esc)
        dists_log.append(dists.copy())
        traj_e.append(pos_e.copy())
        traj_p.append(pos_p.copy())
        times.append(t)
        alt_log_e.append(pos_e[2])
        alt_log_p.append([pos_p[i, 2] for i in range(N_PINCER)])

        if dists.min() < CAPTURE_R:
            capture_time = t
            print(f'Captured at t={t:.2f}s, tilt={np.degrees(psi_tilt):.1f} deg')
            break

    if capture_time is None:
        print('Timeout — evader not captured.')

    return (
        np.array(traj_e), np.array(traj_p),
        np.array(times), np.array(dists_log),
        np.array(solid_angles), np.array(alt_log_e),
        np.array(alt_log_p), capture_time,
    )


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories_3d(traj_e, traj_p, times, capture_time, out_dir):
    fig = plt.figure(figsize=(12, 9))
    ax  = fig.add_subplot(111, projection='3d')

    # Draw trajectories
    ax.plot(traj_e[:, 0], traj_e[:, 1], traj_e[:, 2],
            color='royalblue', linewidth=2.0, label='Evader', zorder=4)
    for i in range(N_PINCER):
        ax.plot(traj_p[:, i, 0], traj_p[:, i, 1], traj_p[:, i, 2],
                color=P_COLORS[i], linewidth=1.6,
                label=f'Pursuer {i}', zorder=3)

    # Draw triangle formations at t=0, 5, 10 s
    snap_times = [0.0, 5.0, 10.0]
    snap_colors = ['gray', 'peru', 'olive']
    for snap_t, sc in zip(snap_times, snap_colors):
        idx = min(int(snap_t / DT), len(traj_p) - 1)
        pts = traj_p[idx]  # (3, 3)
        verts = [[tuple(pts[0]), tuple(pts[1]), tuple(pts[2])]]
        tri = Poly3DCollection(verts, alpha=0.20, facecolor=sc, edgecolor=sc, linewidth=1.2)
        ax.add_collection3d(tri)
        ax.text(*pts.mean(axis=0), f't={snap_t:.0f}s', fontsize=7, color=sc)

    # Start/end markers
    ax.scatter(*traj_e[0], color='blue', s=80, marker='o', zorder=6, label='E start')
    for i in range(N_PINCER):
        ax.scatter(*traj_p[0, i], color=P_COLORS[i], s=60, marker='^', zorder=6)

    if capture_time is not None:
        ax.scatter(*traj_e[-1], color='black', s=150, marker='X',
                   zorder=8, label=f'Captured t={capture_time:.2f}s')

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('S013 3D Pincer — Trident Formation Trajectories')
    ax.legend(fontsize=8, loc='upper left')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def plot_altitude_time(times, alt_log_e, alt_log_p, capture_time, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(times, alt_log_e, color='royalblue', linewidth=2.0, label='Evader z')
    for i in range(N_PINCER):
        ax.plot(times, alt_log_p[:, i], color=P_COLORS[i],
                linewidth=1.6, label=f'Pursuer {i} z')
    ax.axhline(Z_MIN, color='gray', linestyle=':', alpha=0.7, label='z bounds')
    ax.axhline(Z_MAX, color='gray', linestyle=':', alpha=0.7)
    if capture_time is not None:
        ax.axvline(capture_time, color='black', linestyle='--', linewidth=1.2,
                   label=f'Capture t={capture_time:.2f}s')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude z (m)')
    ax.set_title('S013 3D Pincer — Altitude vs Time')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_distance_time(times, dists_log, capture_time, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))
    for i in range(N_PINCER):
        ax.plot(times[1:], dists_log[:, i], color=P_COLORS[i],
                linewidth=1.6, label=f'Pursuer {i}')
    ax.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1.0,
               label=f'Capture r={CAPTURE_R}m')
    if capture_time is not None:
        ax.axvline(capture_time, color='black', linestyle='--', linewidth=1.2,
                   label=f'Capture t={capture_time:.2f}s')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Distance to Evader (m)')
    ax.set_title('S013 3D Pincer — Pursuer Distances to Evader')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'distance_time.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def save_animation(traj_e, traj_p, times, capture_time, out_dir):
    step = 4
    n_frames = max(1, len(times) // step)

    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')

    all_pts = np.vstack([traj_e, traj_p.reshape(-1, 3)])
    margin  = 0.5
    lo = all_pts.min(axis=0) - margin
    hi = all_pts.max(axis=0) + margin
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')

    lines_p  = [ax.plot([], [], [], color=P_COLORS[i], linewidth=1.4)[0] for i in range(N_PINCER)]
    dots_p   = [ax.plot([], [], [], '^', color=P_COLORS[i], markersize=9)[0] for i in range(N_PINCER)]
    line_e,  = ax.plot([], [], [], color='royalblue', linewidth=2.0, label='Evader')
    dot_e,   = ax.plot([], [], [], 'bs', markersize=10)
    tri_lines = [ax.plot([], [], [], '-', color='gray', linewidth=1.2, alpha=0.7)[0] for _ in range(3)]
    title    = ax.set_title('')
    ax.legend(fontsize=8, loc='upper left')

    def update(fi):
        si = min(fi * step, len(times) - 1)
        t  = times[si]

        line_e.set_data(traj_e[:si+1, 0], traj_e[:si+1, 1])
        line_e.set_3d_properties(traj_e[:si+1, 2])
        dot_e.set_data([traj_e[si, 0]], [traj_e[si, 1]])
        dot_e.set_3d_properties([traj_e[si, 2]])

        for i in range(N_PINCER):
            lines_p[i].set_data(traj_p[:si+1, i, 0], traj_p[:si+1, i, 1])
            lines_p[i].set_3d_properties(traj_p[:si+1, i, 2])
            dots_p[i].set_data([traj_p[si, i, 0]], [traj_p[si, i, 1]])
            dots_p[i].set_3d_properties([traj_p[si, i, 2]])

        # Draw triangle edges
        pts = traj_p[si]  # (3, 3)
        tri_idx = [(0, 1), (1, 2), (2, 0)]
        for k, (a, b) in enumerate(tri_idx):
            tri_lines[k].set_data([pts[a, 0], pts[b, 0]], [pts[a, 1], pts[b, 1]])
            tri_lines[k].set_3d_properties([pts[a, 2], pts[b, 2]])

        title.set_text(f'S013 3D Pincer  t={t:.1f}s')
        return [line_e, dot_e] + lines_p + dots_p + tri_lines + [title]

    ani = FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=90)
    plt.close()
    print(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────
if __name__ == '__main__':
    traj_e, traj_p, times, dists_log, solid_angles, alt_log_e, alt_log_p, capture_time = (
        run_simulation()
    )

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(traj_e, traj_p, times, capture_time, out_dir)
    plot_altitude_time(times, alt_log_e, alt_log_p, capture_time, out_dir)
    plot_distance_time(times, dists_log, capture_time, out_dir)
    save_animation(traj_e, traj_p, times, capture_time, out_dir)
    print('S013 3D complete.')
