"""
S012 3D Upgrade — Relay Pursuit
Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s012_3d_relay_pursuit.py
"""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from matplotlib.animation import FuncAnimation, PillowWriter
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'outputs', '01_pursuit_evasion', '3d', 's012_3d_relay_pursuit')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ───────────────────────────────────────────────
N_PURSUERS    = 3
E0            = 50.0    # J each (per card spec)
P_HOVER       = 10.0    # W
K_H           = 0.4     # W·s²/m² horizontal
K_V           = 1.2     # W·s²/m² climb
# Note: at v=5 m/s active pursuer: P = 10 + 0.4*25 = 20W → 50J → 2s to handoff @ 20%
# Three drones relay covers 3 × 2s ≈ 6s total; initial gap ~7.5m / 1.5m/s → ~5s to close
HANDOFF_FRAC  = 0.20    # trigger at 20% battery
V_ACTIVE      = 5.0     # m/s
V_STANDBY     = 4.0     # m/s
V_EVADER      = 3.5     # m/s
Z_TIERS       = [1.0, 2.5, 4.5]   # home altitudes
Z_MIN, Z_MAX  = 0.5, 6.0
K_HOLD        = 2.0     # altitude hold gain
CAPTURE_R     = 0.15    # m
DT            = 0.02    # s
MAX_TIME      = 40.0    # s
W_ENERGY      = 0.6
W_DIST        = 0.4
D_MAX         = 20.0
HANDOFF_COOLDOWN = 2.0  # seconds before next handoff allowed

P_COLORS = ['crimson', 'steelblue', 'seagreen']


# ── Power model ───────────────────────────────────────────────

def power_3d(vel):
    vxy = np.sqrt(vel[0]**2 + vel[1]**2)
    vz  = vel[2]
    return P_HOVER + K_H * vxy**2 + K_V * max(vz, 0.0)**2


def handoff_score(energy_k, pos_k, pos_intercept):
    d = np.linalg.norm(pos_intercept - pos_k)
    return W_ENERGY * (energy_k / E0) - W_DIST * (d / D_MAX)


# ── Evader motion: straight + altitude cycles ────────────────

def evader_velocity(pos_e, t):
    """Straight escape in xy + alternating altitude cycles."""
    base = np.array([1.0, 0.3, 0.0])
    # Altitude strategy: climb to z~4 then dive to z~1, period ~10s
    if (t % 20) < 10:
        az = 0.3   # climb phase
    else:
        az = -0.3  # dive phase
    v = np.array([base[0], base[1], az])
    v = V_EVADER * v / (np.linalg.norm(v) + 1e-8)
    # Clamp z
    if pos_e[2] >= Z_MAX and v[2] > 0:
        v[2] = 0.0
    if pos_e[2] <= Z_MIN and v[2] < 0:
        v[2] = 0.0
    return v


# ── Simulation ────────────────────────────────────────────────

def run_simulation(seed=42):
    rng = np.random.default_rng(seed)

    # Initial positions: each at home altitude tier
    # D0 behind evader; D1/D2 flanking at different tiers
    pos_p = np.array([
        [ 0.0,  0.0, Z_TIERS[0]],   # D0: low tier, behind evader
        [ 2.0,  5.0, Z_TIERS[1]],   # D1: mid tier, flanking
        [ 4.0, -4.0, Z_TIERS[2]],   # D2: high tier, flanking ahead
    ], dtype=float)
    pos_e = np.array([6.0, 1.0, 2.5])

    energy  = np.full(N_PURSUERS, E0)
    active  = 0    # P0 starts active (nearest evader initially)
    last_handoff_t = -HANDOFF_COOLDOWN  # allow immediate first handoff

    traj_e  = [pos_e.copy()]
    traj_p  = [pos_p.copy()]
    en_log  = [[E0] for _ in range(N_PURSUERS)]
    act_log = [active]
    t_hist  = [0.0]
    handoff_events = []    # list of (t, from_drone, to_drone)

    captured = False
    cap_time = None
    max_steps = int(MAX_TIME / DT)

    for step in range(1, max_steps + 1):
        t = step * DT

        # Estimate time to handoff for standby intercept
        e_act = energy[active]
        T_h = (e_act - HANDOFF_FRAC * E0) / (power_3d(V_ACTIVE * np.ones(3)) + 1e-8)
        T_h = max(T_h, 0.0)
        vel_e_pred = evader_velocity(pos_e, t)
        p_intercept = pos_e + vel_e_pred * T_h

        # Handoff check (with cooldown to prevent ping-pong)
        if e_act / E0 < HANDOFF_FRAC and (t - last_handoff_t) >= HANDOFF_COOLDOWN:
            scores = []
            for k in range(N_PURSUERS):
                if k == active or energy[k] <= 0:
                    scores.append(-np.inf)
                else:
                    scores.append(handoff_score(energy[k], pos_p[k], p_intercept))
            new_active = int(np.argmax(scores))
            if new_active != active and scores[new_active] > -np.inf:
                handoff_events.append((t, active, new_active))
                active = new_active
                last_handoff_t = t

        # Active pursuer: pure pursuit in 3D
        d_act = pos_e - pos_p[active]
        dn    = np.linalg.norm(d_act)
        vel_a = V_ACTIVE * d_act / dn if dn > 1e-8 else np.zeros(3)
        dE_a  = power_3d(vel_a) * DT
        energy[active] = max(energy[active] - dE_a, 0.0)
        if energy[active] > 0:
            pos_p[active] += vel_a * DT
            pos_p[active, 2] = np.clip(pos_p[active, 2], Z_MIN, Z_MAX)

        # Standby pursuers: fly toward intercept or hold tier altitude
        for k in range(N_PURSUERS):
            if k == active:
                continue
            d_int  = p_intercept - pos_p[k]
            dist_int = np.linalg.norm(d_int)

            if dist_int > 1.0:
                vel_k = V_STANDBY * d_int / (dist_int + 1e-8)
            else:
                # Altitude hold toward home tier
                dz = Z_TIERS[k] - pos_p[k, 2]
                vel_k = np.array([0.0, 0.0, K_HOLD * dz])

            dE_k = power_3d(vel_k) * DT * 0.4   # standby: reduced power draw
            energy[k] = max(energy[k] - dE_k, 0.0)
            if energy[k] > 0:
                pos_p[k] += vel_k * DT
                pos_p[k, 2] = np.clip(pos_p[k, 2], Z_MIN, Z_MAX)

        # Evader motion
        vel_e = evader_velocity(pos_e, t)
        pos_e = pos_e + vel_e * DT
        pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)

        traj_e.append(pos_e.copy())
        traj_p.append(pos_p.copy())
        for k in range(N_PURSUERS):
            en_log[k].append(energy[k])
        act_log.append(active)
        t_hist.append(t)

        # Capture check
        if np.linalg.norm(pos_p[active] - pos_e) < CAPTURE_R and energy[active] > 0:
            captured = True
            cap_time = t
            break

    return (np.array(traj_e), np.array(traj_p),
            [np.array(e) for e in en_log],
            np.array(t_hist), np.array(act_log),
            handoff_events, captured, cap_time)


# ── Plot 1: 3D trajectories with handoff markers ──────────────

def plot_trajectories_3d(traj_e, traj_p, act_log, handoff_events, captured, cap_time, out_dir):
    fig = plt.figure(figsize=(11, 8))
    ax  = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=25, azim=-55)

    # Draw each pursuer trajectory; mark handoff points with stars
    for k in range(N_PURSUERS):
        ax.plot(traj_p[:, k, 0], traj_p[:, k, 1], traj_p[:, k, 2],
                color=P_COLORS[k], linewidth=1.4, alpha=0.8, label=f'Drone {k}')
        ax.scatter(*traj_p[0, k], color=P_COLORS[k], s=50, marker='o')

    # Handoff events
    for t_hf, from_k, to_k in handoff_events:
        step_idx = int(t_hf / DT)
        step_idx = min(step_idx, len(traj_p) - 1)
        ax.scatter(*traj_p[step_idx, to_k], color=P_COLORS[to_k],
                   s=150, marker='*', zorder=8)

    ax.plot(traj_e[:, 0], traj_e[:, 1], traj_e[:, 2],
            color='royalblue', linewidth=2.0, label='Evader', zorder=5)
    ax.scatter(*traj_e[0], color='blue', s=60, marker='o', zorder=6)

    if captured:
        ax.scatter(*traj_e[-1], color='black', s=140, marker='X', zorder=9,
                   label=f'Captured @ {cap_time:.2f}s')

    # Stars legend entry
    if handoff_events:
        ax.scatter([], [], [], color='grey', s=150, marker='*', label='Handoff event')

    status = f'Captured @ {cap_time:.2f} s' if captured else 'Timeout — evader escaped'
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title(f'S012 3D Relay Pursuit — 3D Trajectories\n{status}', fontsize=10)
    ax.legend(fontsize=8, loc='upper left')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close('all')
    print(f'Saved: {path}')


# ── Plot 2: Battery vs time with active shading ────────────────

def plot_battery_time(en_log, t_hist, act_log, handoff_events, captured, cap_time, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))

    # Shade active periods for each drone
    n_t = len(t_hist)
    for k in range(N_PURSUERS):
        # Find contiguous active segments
        in_seg = False
        seg_start = None
        for idx in range(n_t):
            is_active = (act_log[idx] == k)
            if is_active and not in_seg:
                seg_start = t_hist[idx]
                in_seg = True
            elif not is_active and in_seg:
                ax.axvspan(seg_start, t_hist[idx], alpha=0.12, color=P_COLORS[k])
                in_seg = False
        if in_seg:
            ax.axvspan(seg_start, t_hist[-1], alpha=0.12, color=P_COLORS[k])

    # Battery curves
    for k in range(N_PURSUERS):
        e = en_log[k]
        ax.plot(t_hist[:len(e)], e, color=P_COLORS[k], linewidth=2.0,
                label=f'Drone {k} (tier z={Z_TIERS[k]}m)')

    ax.axhline(E0 * HANDOFF_FRAC, color='grey', linestyle='--', linewidth=1.2,
               label=f'Handoff threshold ({HANDOFF_FRAC*100:.0f}%)')

    for t_hf, from_k, to_k in handoff_events:
        ax.axvline(t_hf, color='orange', linestyle=':', linewidth=1.8, alpha=0.9)
    if handoff_events:
        ax.axvline(handoff_events[0][0], color='orange', linestyle=':', linewidth=1.8,
                   alpha=0.9, label='Handoff event')

    if captured:
        ax.axvline(cap_time, color='black', linestyle='--', linewidth=1.4,
                   label=f'Captured @ {cap_time:.2f}s')

    ax.set_xlabel('Time (s)'); ax.set_ylabel('Battery Energy (J)')
    ax.set_title('S012 3D Relay Pursuit — Battery Level vs Time\n'
                 '(shaded regions = active periods per drone)', fontsize=10)
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'battery_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close('all')
    print(f'Saved: {path}')


# ── Plot 3: Altitude vs time ──────────────────────────────────

def plot_altitude_time(traj_e, traj_p, t_hist, handoff_events, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))

    for k in range(N_PURSUERS):
        ax.plot(t_hist, traj_p[:, k, 2], color=P_COLORS[k], linewidth=1.6,
                label=f'Drone {k} (home z={Z_TIERS[k]}m)')
        ax.axhline(Z_TIERS[k], color=P_COLORS[k], linestyle=':', linewidth=0.8, alpha=0.5)

    ax.plot(t_hist, traj_e[:, 2], color='royalblue', linewidth=2.0,
            label='Evader', zorder=5)
    ax.axhline(Z_MIN, color='grey', linestyle='--', linewidth=0.8, alpha=0.6)
    ax.axhline(Z_MAX, color='grey', linestyle='--', linewidth=0.8, alpha=0.6,
               label='z bounds')

    for t_hf, from_k, to_k in handoff_events:
        ax.axvline(t_hf, color='orange', linestyle=':', linewidth=1.6, alpha=0.8)

    ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude z (m)')
    ax.set_title('S012 3D Relay Pursuit — Altitude vs Time\n'
                 '(dotted lines = tier home altitudes)', fontsize=10)
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close('all')
    print(f'Saved: {path}')


# ── Animation ─────────────────────────────────────────────────

def save_animation(traj_e, traj_p, en_log, t_hist, act_log,
                   handoff_events, captured, cap_time, out_dir):
    T = len(t_hist)
    step_size = max(1, T // 200)

    fig = plt.figure(figsize=(12, 7))
    ax3d = fig.add_axes([0.05, 0.15, 0.60, 0.80], projection='3d')
    ax3d.view_init(elev=25, azim=-55)

    # Battery bar axes (right side)
    ax_bat = fig.add_axes([0.70, 0.15, 0.28, 0.70])

    all_pts = np.vstack([traj_e] + [traj_p[:, k] for k in range(N_PURSUERS)])
    lo = all_pts.min(axis=0) - 0.5
    hi = all_pts.max(axis=0) + 0.5
    ax3d.set_xlim(lo[0], hi[0]); ax3d.set_ylim(lo[1], hi[1]); ax3d.set_zlim(lo[2], hi[2])
    ax3d.set_xlabel('X'); ax3d.set_ylabel('Y'); ax3d.set_zlabel('Z')

    lines_p = [ax3d.plot([], [], [], color=P_COLORS[k], linewidth=1.4, alpha=0.8)[0]
               for k in range(N_PURSUERS)]
    dots_p  = [ax3d.plot([], [], [], '^', color=P_COLORS[k], markersize=9, zorder=6)[0]
               for k in range(N_PURSUERS)]
    line_e, = ax3d.plot([], [], [], color='royalblue', linewidth=1.8)
    dot_e,  = ax3d.plot([], [], [], 'bs', markersize=10, zorder=7)

    # Battery bars
    ax_bat.set_xlim(-0.5, N_PURSUERS - 0.5)
    ax_bat.set_ylim(0, E0)
    ax_bat.set_xticks(range(N_PURSUERS))
    ax_bat.set_xticklabels([f'D{k}' for k in range(N_PURSUERS)])
    ax_bat.set_ylabel('Battery (J)')
    ax_bat.set_title('Battery', fontsize=9)
    ax_bat.axhline(E0 * HANDOFF_FRAC, color='grey', linestyle='--', linewidth=0.8)
    bars = ax_bat.bar(range(N_PURSUERS), [E0]*N_PURSUERS, color=P_COLORS, alpha=0.7)

    title = fig.suptitle('', fontsize=10)

    frames = list(range(0, T, step_size))
    if frames[-1] != T - 1:
        frames.append(T - 1)

    def update(frame_idx):
        si = frames[frame_idx]
        t  = t_hist[si]
        ai = act_log[si]

        for k in range(N_PURSUERS):
            lines_p[k].set_data(traj_p[:si+1, k, 0], traj_p[:si+1, k, 1])
            lines_p[k].set_3d_properties(traj_p[:si+1, k, 2])
            dots_p[k].set_data([traj_p[si, k, 0]], [traj_p[si, k, 1]])
            dots_p[k].set_3d_properties([traj_p[si, k, 2]])
            dots_p[k].set_markersize(14 if k == ai else 7)

            en_val = en_log[k][min(si, len(en_log[k])-1)]
            bars[k].set_height(en_val)
            bars[k].set_edgecolor('black' if k == ai else 'none')
            bars[k].set_linewidth(2.0 if k == ai else 0)

        line_e.set_data(traj_e[:si+1, 0], traj_e[:si+1, 1])
        line_e.set_3d_properties(traj_e[:si+1, 2])
        dot_e.set_data([traj_e[si, 0]], [traj_e[si, 1]])
        dot_e.set_3d_properties([traj_e[si, 2]])

        if frame_idx == len(frames) - 1 and captured:
            title.set_text(f'S012 3D Relay Pursuit  t={t:.2f}s  Captured! [DONE]')
        else:
            title.set_text(f'S012 3D Relay Pursuit  t={t:.2f}s  Active: D{ai}')

        return lines_p + dots_p + [line_e, dot_e, title] + list(bars)

    ani = FuncAnimation(fig, update, frames=len(frames), interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=15))
    plt.close('all')
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Running S012 3D Relay Pursuit...')
    traj_e, traj_p, en_log, t_hist, act_log, handoff_events, captured, cap_time = run_simulation()

    status = f'Captured @ {cap_time:.2f}s' if captured else 'Timeout — evader escaped'
    print(f'Result: {status}')
    print(f'Handoff events: {[(f"t={t:.2f} D{f}->D{to}" ) for t, f, to in handoff_events]}')

    plot_trajectories_3d(traj_e, traj_p, act_log, handoff_events, captured, cap_time, OUTPUT_DIR)
    plot_battery_time(en_log, t_hist, act_log, handoff_events, captured, cap_time, OUTPUT_DIR)
    plot_altitude_time(traj_e, traj_p, t_hist, handoff_events, OUTPUT_DIR)

    print('Saving animation...')
    save_animation(traj_e, traj_p, en_log, t_hist, act_log,
                   handoff_events, captured, cap_time, OUTPUT_DIR)

    print('All outputs saved to:', OUTPUT_DIR)
