"""
S027 3D Aerial Refueling Relay
================================
A delivery drone (Receiver) flies a 3-D waypoint mission at cruise altitude 10 m.
Its battery is insufficient to complete the round trip, so a Tanker drone is dispatched
to a rendezvous point at altitude 15 m, 5 m above the Receiver's cruise altitude.
The Receiver must simultaneously close horizontal distance AND match the Tanker's altitude
before executing a full 3-D soft-dock maneuver. After energy transfer the Receiver
resumes its 3-D mission; the Tanker returns to base.

Outputs: trajectory_3d.png, altitude_profile.png, energy_levels.png,
         docking_detail.png, animation.gif

Usage:
    conda activate drones
    "C:\\Users\\user\\anaconda3\\envs\\drones\\python.exe" -u src/02_logistics_delivery/3d/s027_3d_aerial_refueling.py
"""

import matplotlib
matplotlib.use('Agg')

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import matplotlib.animation as animation
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

# ── Output directory ────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's027_3d_aerial_refueling',
))

# ── Parameters ──────────────────────────────────────────────────────────────
DT          = 0.05      # s  time step
T_MAX       = 120.0     # s  max simulation time

V_CRUISE    = 5.0       # m/s  cruise speed (both drones, nominal)
V_T_ORB     = 2.0       # m/s  tanker loiter speed
R_ORB       = 4.0       # m    loiter radius
Z_R_CRUISE  = 10.0      # m    receiver cruise altitude
Z_T_LOITER  = 15.0      # m    tanker loiter altitude

K_ALT       = 1.2       # altitude-match gain (1/s)
V_Z_MAX     = 2.0       # m/s  max vertical speed for altitude match
DELTA_Z_OK  = 0.5       # m    altitude-match tolerance

K_FLIGHT    = 0.3       # W·s²/m²  flight power coeff
K_HOVER     = 0.5       # W        hover drain

E0_R        = 40.0      # J   Receiver initial battery
E0_T        = 80.0      # J   Tanker initial battery
E_XFER      = 30.0      # J   energy transferred at dock
ETA_LOSS    = 0.05      # transfer loss fraction

D_CAP       = 12.0      # m   rendezvous capture radius (enter docking mode)
D_DOCK      = 0.5       # m   docking engagement radius (soft-dock gains)
EPS_P       = 0.20      # m   position dock tolerance
EPS_V       = 0.08      # m/s velocity dock tolerance
KP, KD      = 2.0, 1.5  # approach PD gains
KP2, KD2    = 4.0, 2.5  # soft-dock PD gains

MARGIN      = 5.0       # m   range safety margin
ALPHA_RVZ   = 0.50      # rendezvous fraction along base-to-target

# Geometry
BASE    = np.array([0.0,  0.0,  0.0])
TARGET  = np.array([60.0, 0.0,  0.0])
DIST_BT = np.linalg.norm(TARGET[:2] - BASE[:2])

# Rendezvous point: midpoint along route, at tanker altitude
W_RVZ = np.array([
    (1 - ALPHA_RVZ) * BASE[0] + ALPHA_RVZ * TARGET[0],
    (1 - ALPHA_RVZ) * BASE[1] + ALPHA_RVZ * TARGET[1],
    Z_T_LOITER,
])

# Receiver waypoints (3-D mission route)
# Base -> climb -> cruise segment 1 -> RVZ altitude (auto) -> target -> return
WAYPOINTS_R = np.array([
    [0.0,   0.0,  0.0],    # WP0: base (launch)
    [5.0,   0.0,  Z_R_CRUISE],  # WP1: climb-out
    [55.0,  0.0,  Z_R_CRUISE],  # WP2: near target
    [60.0,  0.0,  5.0],    # WP3: target (descend slightly for delivery)
    [55.0,  0.0,  Z_R_CRUISE],  # WP4: return climb
    [5.0,   0.0,  Z_R_CRUISE],  # WP5: homeward
    [0.0,   0.0,  0.0],    # WP6: base (land)
])

MAX_STEPS = int(T_MAX / DT)
RNG = np.random.default_rng(0)


# ── Physics helpers ──────────────────────────────────────────────────────────

def battery_drain(speed, dt):
    """Energy consumed in one timestep at given speed."""
    return (K_FLIGHT * speed ** 2 + K_HOVER) * dt


def range_remaining(energy, speed):
    """Estimated remaining range (m) given current energy and speed."""
    power = K_FLIGHT * speed ** 2 + K_HOVER
    if power <= 0 or speed <= 0:
        return 0.0
    return (energy / power) * speed


def clamp_speed(vel, v_max):
    """Scale velocity vector so its norm does not exceed v_max."""
    n = np.linalg.norm(vel)
    if n > v_max:
        return vel * (v_max / n)
    return vel


def loiter_pos(t, center, omega, phi0, radius):
    """Tanker loiter position at time t (horizontal circle, constant altitude)."""
    angle = omega * t + phi0
    return center + np.array([radius * np.cos(angle), radius * np.sin(angle), 0.0])


def loiter_vel(t, omega, phi0, radius):
    """Tanker loiter velocity (analytic derivative)."""
    angle = omega * t + phi0
    v_tang = omega * radius
    return np.array([-v_tang * np.sin(angle), v_tang * np.cos(angle), 0.0])


# ── Simulation ───────────────────────────────────────────────────────────────

def run_simulation():
    """Run the full 3-D aerial refueling simulation.

    Returns
    -------
    dict with trajectory arrays, energy histories, event times, etc.
    """
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # ---- Initial conditions -------------------------------------------------
    pos_R = BASE.copy().astype(float)
    vel_R = np.zeros(3)
    pos_T = BASE.copy().astype(float)
    vel_T = np.zeros(3)

    E_R = E0_R
    E_T = E0_T

    # Receiver FSM states:
    # fly_mission -> alt_match -> docking -> deliver_resume -> return_home
    # Tanker FSM states: fly_to_rvz -> loiter -> return_home
    phase_R = 'fly_mission'
    phase_T = 'fly_to_rvz'

    wp_idx = 1          # next waypoint index for receiver mission
    docked = False
    dock_success = False
    refuel_triggered = False
    phi0_loiter = 0.0   # loiter start angle
    omega_orb = V_T_ORB / R_ORB
    loiter_start_t = None

    # History buffers
    times       = []
    pos_R_hist  = []
    pos_T_hist  = []
    vel_R_hist  = []
    vel_T_hist  = []
    E_R_hist    = []
    E_T_hist    = []
    phase_R_hist = []

    # Event timestamps
    events = {
        'refuel_trigger': None,
        'tanker_arrive_rvz': None,
        'alt_match_start': None,
        'dock_start': None,
        'dock_success': None,
        'mission_resume': None,
    }

    t = 0.0
    docking_active = False

    for step in range(MAX_STEPS):
        t = step * DT

        # ── Record state ───────────────────────────────────────────────────
        times.append(t)
        pos_R_hist.append(pos_R.copy())
        pos_T_hist.append(pos_T.copy())
        vel_R_hist.append(vel_R.copy())
        vel_T_hist.append(vel_T.copy())
        E_R_hist.append(E_R)
        E_T_hist.append(E_T)
        phase_R_hist.append(phase_R)

        # ── Receiver FSM ───────────────────────────────────────────────────

        if phase_R == 'fly_mission':
            # Check if refueling is needed
            if not refuel_triggered:
                d_to_target = np.linalg.norm(TARGET - pos_R)
                d_to_base_via_target = d_to_target + np.linalg.norm(BASE - TARGET)
                d_rem = range_remaining(E_R, V_CRUISE)
                if d_rem < d_to_base_via_target + MARGIN:
                    refuel_triggered = True
                    events['refuel_trigger'] = t
                    phase_R = 'fly_to_rvz'
                    # Override waypoint navigation toward rvz horizontal approach point
                else:
                    # Normal waypoint following
                    target_wp = WAYPOINTS_R[wp_idx]
                    dir_r = target_wp - pos_R
                    dist_wp = np.linalg.norm(dir_r)
                    if dist_wp < 1.0 and wp_idx < len(WAYPOINTS_R) - 1:
                        wp_idx += 1
                        target_wp = WAYPOINTS_R[wp_idx]
                        dir_r = target_wp - pos_R
                        dist_wp = np.linalg.norm(dir_r)
                    if dist_wp > 1e-3:
                        vel_R = V_CRUISE * dir_r / dist_wp
                    else:
                        vel_R = np.zeros(3)

        if phase_R == 'fly_to_rvz':
            # Fly horizontally toward rendezvous XY, maintaining current cruise altitude
            rvz_approach = np.array([W_RVZ[0], W_RVZ[1], Z_R_CRUISE])
            dir_r = rvz_approach - pos_R
            dist_h = np.linalg.norm(dir_r[:2])  # horizontal distance
            dist_3d = np.linalg.norm(dir_r)

            # Maintain Z_R_CRUISE while approaching horizontally
            if dist_3d > 1e-3:
                vel_R = V_CRUISE * dir_r / dist_3d
            else:
                vel_R = np.zeros(3)

            # Transition to altitude match when close enough horizontally
            if dist_h < D_CAP:
                phase_R = 'alt_match'
                events['alt_match_start'] = t

        elif phase_R == 'alt_match':
            # Horizontal approach continues, plus vertical correction to match tanker altitude
            hor_dir = np.array([W_RVZ[0] - pos_R[0], W_RVZ[1] - pos_R[1], 0.0])
            hor_dist = np.linalg.norm(hor_dir)

            # Horizontal velocity component
            if hor_dist > 1e-3:
                vel_R_hor = V_CRUISE * 0.7 * hor_dir / hor_dist
            else:
                vel_R_hor = np.zeros(3)

            # Vertical velocity component: climb to tanker altitude
            z_err = Z_T_LOITER - pos_R[2]
            vz = np.clip(K_ALT * z_err, -V_Z_MAX, V_Z_MAX)
            vel_R = vel_R_hor + np.array([0.0, 0.0, vz])
            vel_R = clamp_speed(vel_R, V_CRUISE)

            # Transition to docking when altitude matched and within capture radius
            alt_ok = abs(pos_R[2] - Z_T_LOITER) < DELTA_Z_OK
            close_enough = np.linalg.norm(pos_R[:2] - W_RVZ[:2]) < D_CAP
            if alt_ok and close_enough:
                phase_R = 'docking'
                docking_active = True
                events['dock_start'] = t

        elif phase_R == 'docking':
            # Full 3-D PD docking controller
            dr = pos_R - pos_T
            dv = vel_R - vel_T
            dist_dock = np.linalg.norm(dr)

            # Tanker acceleration (finite difference of loiter velocity)
            if loiter_start_t is not None:
                vel_T_next = loiter_vel(t + DT, omega_orb, phi0_loiter, R_ORB)
                acc_T = (vel_T_next - vel_T) / DT
            else:
                acc_T = np.zeros(3)

            if dist_dock > D_DOCK:
                # Phase 1: approach
                u_R = -KP * dr - KD * dv + acc_T
            else:
                # Phase 2: soft dock
                u_R = -KP2 * dr - KD2 * dv

            vel_R = vel_R + u_R * DT
            vel_R = clamp_speed(vel_R, V_CRUISE * 1.2)  # allow slight overspeed for catch-up

            # Check docking success
            if np.linalg.norm(dr) < EPS_P and np.linalg.norm(dv) < EPS_V:
                if not dock_success:
                    dock_success = True
                    events['dock_success'] = t
                    # Energy transfer
                    E_R = min(E_R + E_XFER, E0_R + E_XFER)
                    E_T -= E_XFER * (1 + ETA_LOSS)
                    phase_R = 'deliver_resume'
                    phase_T = 'return_home'
                    events['mission_resume'] = t
                    docking_active = False

        elif phase_R == 'deliver_resume':
            # Resume mission from TARGET waypoint
            dir_r = TARGET - pos_R
            dist = np.linalg.norm(dir_r)
            if dist < 2.0:
                phase_R = 'return_home'
                wp_idx = 4  # waypoints for return
            elif dist > 1e-3:
                vel_R = V_CRUISE * dir_r / dist
            else:
                vel_R = np.zeros(3)

        elif phase_R == 'return_home':
            # Fly back to base, descend on approach
            approach = np.array([BASE[0], BASE[1], Z_R_CRUISE if np.linalg.norm(pos_R - BASE) > 10 else 0.0])
            dir_r = approach - pos_R
            dist = np.linalg.norm(dir_r)
            if np.linalg.norm(pos_R - BASE) < 2.0:
                vel_R = np.zeros(3)  # landed
            elif dist > 1e-3:
                vel_R = V_CRUISE * dir_r / dist
            else:
                vel_R = np.zeros(3)

        # ── Tanker FSM ─────────────────────────────────────────────────────

        if phase_T == 'fly_to_rvz':
            # Fly directly to rendezvous point (including altitude)
            dir_t = W_RVZ - pos_T
            dist_t = np.linalg.norm(dir_t)
            if dist_t > 2.0:
                vel_T = V_CRUISE * dir_t / dist_t
            else:
                # Close to rendezvous — switch to loiter
                phase_T = 'loiter'
                loiter_start_t = t
                phi0_loiter = np.arctan2(pos_T[1] - W_RVZ[1], pos_T[0] - W_RVZ[0])
                events['tanker_arrive_rvz'] = t
                pos_T = loiter_pos(0.0, W_RVZ, omega_orb, phi0_loiter, R_ORB)
                vel_T = loiter_vel(0.0, omega_orb, phi0_loiter, R_ORB)

        elif phase_T == 'loiter':
            elapsed = t - loiter_start_t
            pos_T = loiter_pos(elapsed, W_RVZ, omega_orb, phi0_loiter, R_ORB)
            vel_T = loiter_vel(elapsed, omega_orb, phi0_loiter, R_ORB)

        elif phase_T == 'return_home':
            dir_t = BASE - pos_T
            dist_t = np.linalg.norm(dir_t)
            if dist_t > 2.0:
                vel_T = V_CRUISE * dir_t / dist_t
            else:
                vel_T = np.zeros(3)

        # ── Integrate positions ────────────────────────────────────────────
        # For loiter phase, pos_T is set analytically above; skip integration
        if phase_T != 'loiter':
            pos_T = pos_T + vel_T * DT

        if phase_R != 'docking':
            pos_R = pos_R + vel_R * DT
        else:
            pos_R = pos_R + vel_R * DT

        # ── Battery drain ──────────────────────────────────────────────────
        E_R = max(0.0, E_R - battery_drain(np.linalg.norm(vel_R), DT))
        E_T = max(0.0, E_T - battery_drain(np.linalg.norm(vel_T), DT))

        # ── Termination check ──────────────────────────────────────────────
        if dock_success and phase_R == 'return_home' and np.linalg.norm(pos_R - BASE) < 2.0:
            # Mission complete
            t_end = t
            times.append(t_end + DT)
            pos_R_hist.append(pos_R.copy())
            pos_T_hist.append(pos_T.copy())
            vel_R_hist.append(vel_R.copy())
            vel_T_hist.append(vel_T.copy())
            E_R_hist.append(E_R)
            E_T_hist.append(E_T)
            phase_R_hist.append(phase_R)
            break

    # Convert to arrays
    times      = np.array(times)
    pos_R_hist = np.array(pos_R_hist)
    pos_T_hist = np.array(pos_T_hist)
    vel_R_hist = np.array(vel_R_hist)
    vel_T_hist = np.array(vel_T_hist)
    E_R_hist   = np.array(E_R_hist)
    E_T_hist   = np.array(E_T_hist)

    # Print key metrics
    print(f"Simulation duration:   {times[-1]:.1f} s")
    print(f"Dock success:          {dock_success}")
    if events['refuel_trigger']:
        print(f"Refuel trigger time:   {events['refuel_trigger']:.1f} s")
    if events['tanker_arrive_rvz']:
        print(f"Tanker at RVZ time:    {events['tanker_arrive_rvz']:.1f} s")
    if events['alt_match_start']:
        print(f"Altitude match start:  {events['alt_match_start']:.1f} s")
    if events['dock_start']:
        print(f"Docking start time:    {events['dock_start']:.1f} s")
    if events['dock_success']:
        print(f"Dock success time:     {events['dock_success']:.1f} s")
        # Time from dock start to success
        if events['dock_start']:
            print(f"Docking duration:      {events['dock_success'] - events['dock_start']:.1f} s")
    print(f"Receiver final energy: {E_R_hist[-1]:.2f} J")
    print(f"Tanker final energy:   {E_T_hist[-1]:.2f} J")
    alt_diff_at_rvz = Z_T_LOITER - Z_R_CRUISE
    print(f"Altitude gap at RVZ:   {alt_diff_at_rvz:.1f} m  ({Z_R_CRUISE:.0f} m -> {Z_T_LOITER:.0f} m)")

    return {
        'times': times,
        'pos_R': pos_R_hist,
        'pos_T': pos_T_hist,
        'vel_R': vel_R_hist,
        'vel_T': vel_T_hist,
        'E_R':   E_R_hist,
        'E_T':   E_T_hist,
        'phases': phase_R_hist,
        'events': events,
        'dock_success': dock_success,
        'W_RVZ': W_RVZ,
    }


# ── Plot functions ───────────────────────────────────────────────────────────

def plot_trajectory_3d(data, out_dir):
    """3-D trajectory of both drones with event markers."""
    pos_R = data['pos_R']
    pos_T = data['pos_T']
    events = data['events']
    times  = data['times']
    W_RVZ  = data['W_RVZ']

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Full trajectories
    ax.plot(pos_R[:, 0], pos_R[:, 1], pos_R[:, 2],
            color='red',  lw=1.5, label='Receiver (red)', alpha=0.85)
    ax.plot(pos_T[:, 0], pos_T[:, 1], pos_T[:, 2],
            color='blue', lw=1.5, label='Tanker (blue)',  alpha=0.85)

    # Highlight docking segment in yellow
    if events['dock_start'] and events['dock_success']:
        i0 = np.searchsorted(times, events['dock_start'])
        i1 = np.searchsorted(times, events['dock_success'])
        ax.plot(pos_R[i0:i1+1, 0], pos_R[i0:i1+1, 1], pos_R[i0:i1+1, 2],
                color='gold', lw=3.0, label='Docking approach', zorder=5)

    # Key markers
    ax.scatter(*BASE,   color='black', s=80, marker='^', zorder=6, label='Base')
    ax.scatter(*TARGET[:3], color='purple', s=80, marker='s', zorder=6, label='Target')
    ax.scatter(*W_RVZ,  color='green', s=120, marker='*', zorder=6, label='Refueling point')

    # Start positions
    ax.scatter(*pos_R[0], color='red',  s=60, marker='o', zorder=5)
    ax.scatter(*pos_T[0], color='blue', s=60, marker='o', zorder=5)

    # Altitude reference lines (dashed)
    ax.plot([0, 65], [0, 0], [Z_R_CRUISE, Z_R_CRUISE],
            'r--', lw=0.6, alpha=0.4, label=f'Receiver cruise z={Z_R_CRUISE}m')
    ax.plot([0, 65], [0, 0], [Z_T_LOITER, Z_T_LOITER],
            'b--', lw=0.6, alpha=0.4, label=f'Tanker loiter z={Z_T_LOITER}m')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z / altitude (m)')
    ax.set_title('S027 3D Aerial Refueling — Trajectory')
    ax.legend(fontsize=8, loc='upper left')

    # Set sensible axis limits
    all_x = np.concatenate([pos_R[:, 0], pos_T[:, 0]])
    all_y = np.concatenate([pos_R[:, 1], pos_T[:, 1]])
    all_z = np.concatenate([pos_R[:, 2], pos_T[:, 2]])
    pad = 3.0
    ax.set_xlim(all_x.min() - pad, all_x.max() + pad)
    ax.set_ylim(all_y.min() - pad*3, all_y.max() + pad*3)
    ax.set_zlim(max(0, all_z.min() - pad), all_z.max() + pad)

    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_profile(data, out_dir):
    """Altitude z(t) for both drones with event markers."""
    times  = data['times']
    pos_R  = data['pos_R']
    pos_T  = data['pos_T']
    events = data['events']

    fig, ax = plt.subplots(figsize=(11, 4))

    ax.plot(times, pos_R[:, 2], color='red',  lw=1.8, label='Receiver altitude')
    ax.plot(times, pos_T[:, 2], color='blue', lw=1.8, label='Tanker altitude')

    # Cruise altitude references
    ax.axhline(Z_R_CRUISE, color='red',  ls='--', lw=0.8, alpha=0.5, label=f'Receiver cruise ({Z_R_CRUISE} m)')
    ax.axhline(Z_T_LOITER, color='blue', ls='--', lw=0.8, alpha=0.5, label=f'Tanker loiter ({Z_T_LOITER} m)')

    colors_ev = {
        'refuel_trigger':   ('orange',  'Refuel trigger'),
        'tanker_arrive_rvz':('cyan',    'Tanker at RVZ'),
        'alt_match_start':  ('magenta', 'Alt-match start'),
        'dock_start':       ('lime',    'Dock start'),
        'dock_success':     ('green',   'Dock success'),
    }
    for key, (col, label) in colors_ev.items():
        t_ev = events.get(key)
        if t_ev is not None:
            ax.axvline(t_ev, color=col, ls=':', lw=1.4, label=label)

    # Shade the altitude gap region
    ax.fill_between([0, times[-1]], Z_R_CRUISE, Z_T_LOITER,
                    alpha=0.07, color='gray', label='Altitude gap')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('S027 3D — Altitude Profile: Receiver climbs 5 m to match Tanker')
    ax.legend(fontsize=8, ncol=2)
    ax.set_ylim(-1, Z_T_LOITER + 5)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_profile.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_energy_levels(data, out_dir):
    """Battery energy E(t) for both drones with event annotations."""
    times  = data['times']
    E_R    = data['E_R']
    E_T    = data['E_T']
    events = data['events']

    fig, ax = plt.subplots(figsize=(11, 4))

    ax.plot(times, E_R, color='red',  lw=1.8, label='Receiver energy')
    ax.plot(times, E_T, color='blue', lw=1.8, label='Tanker energy')

    colors_ev = {
        'refuel_trigger':   ('orange',  'Refuel trigger'),
        'tanker_arrive_rvz':('cyan',    'Tanker at RVZ'),
        'alt_match_start':  ('magenta', 'Alt-match start'),
        'dock_start':       ('lime',    'Dock start'),
        'dock_success':     ('green',   f'Dock success\n(+{E_XFER:.0f} J to Receiver)'),
    }
    for key, (col, label) in colors_ev.items():
        t_ev = events.get(key)
        if t_ev is not None:
            ax.axvline(t_ev, color=col, ls=':', lw=1.4, label=label)

    # Annotate the energy jump
    t_ds = events.get('dock_success')
    if t_ds is not None:
        idx = np.searchsorted(times, t_ds)
        if idx < len(E_R) - 1:
            ax.annotate(
                f'+{E_XFER:.0f} J',
                xy=(times[idx], E_R[idx]),
                xytext=(times[idx] + 3, E_R[idx] + 3),
                arrowprops=dict(arrowstyle='->', color='red'),
                color='red', fontsize=9,
            )

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Energy (J)')
    ax.set_title('S027 3D — Battery Energy Levels During Aerial Refueling')
    ax.legend(fontsize=8, ncol=2)
    ax.set_ylim(-2, max(E0_T, E0_R + E_XFER) + 5)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'energy_levels.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_docking_detail(data, out_dir):
    """Three-panel plot of dx, dy, dz relative motion during docking approach."""
    times  = data['times']
    pos_R  = data['pos_R']
    pos_T  = data['pos_T']
    events = data['events']

    t_start = events.get('alt_match_start') or events.get('dock_start')
    t_end   = events.get('dock_success')

    if t_start is None or t_end is None:
        print("Docking detail: insufficient event data, skipping.")
        return

    # Add some context before and after
    i0 = max(0, np.searchsorted(times, t_start) - 20)
    i1 = min(len(times) - 1, np.searchsorted(times, t_end) + 40)

    t_seg  = times[i0:i1]
    dr     = pos_R[i0:i1] - pos_T[i0:i1]
    dr_mag = np.linalg.norm(dr, axis=1)

    labels = ['Δx (m)', 'Δy (m)', 'Δz (m)']
    colors = ['tomato', 'steelblue', 'seagreen']

    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)

    for i, (ax, lab, col) in enumerate(zip(axes[:3], labels, colors)):
        ax.plot(t_seg, dr[:, i], color=col, lw=1.5)
        ax.axhline(0, color='k', lw=0.6, ls='--')
        ax.axhline( EPS_P, color='gray', lw=0.8, ls=':', alpha=0.7)
        ax.axhline(-EPS_P, color='gray', lw=0.8, ls=':', alpha=0.7)
        ax.set_ylabel(lab)
        ax.grid(True, alpha=0.3)
        if events.get('dock_start'):
            ax.axvline(events['dock_start'], color='lime',  lw=1.2, ls='--', label='Dock start')
        if events.get('dock_success'):
            ax.axvline(events['dock_success'], color='green', lw=1.2, ls='--', label='Dock success')
        ax.legend(fontsize=7)

    axes[3].plot(t_seg, dr_mag, color='purple', lw=1.8, label='||Δr|| (m)')
    axes[3].axhline(EPS_P,  color='gray',  lw=0.8, ls=':', label=f'ε_p={EPS_P}m')
    axes[3].axhline(D_DOCK, color='orange',lw=0.8, ls=':', label=f'd_dock={D_DOCK}m')
    if events.get('dock_start'):
        axes[3].axvline(events['dock_start'],   color='lime',  lw=1.2, ls='--', label='Dock start')
    if events.get('dock_success'):
        axes[3].axvline(events['dock_success'], color='green', lw=1.2, ls='--', label='Dock success')
    axes[3].set_ylabel('||Δr|| (m)')
    axes[3].set_xlabel('Time (s)')
    axes[3].legend(fontsize=7)
    axes[3].grid(True, alpha=0.3)

    fig.suptitle('S027 3D — Docking Approach: 3-D Relative Position Convergence', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'docking_detail.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    """3-D animated view of both drones with energy bars."""
    times  = data['times']
    pos_R  = data['pos_R']
    pos_T  = data['pos_T']
    E_R    = data['E_R']
    E_T    = data['E_T']
    events = data['events']
    W_RVZ  = data['W_RVZ']

    # Downsample for animation speed
    step = 4
    idx  = np.arange(0, len(times), step)
    t_an = times[idx]
    pR   = pos_R[idx]
    pT   = pos_T[idx]
    eR   = E_R[idx]
    eT   = E_T[idx]

    trail_len = 40  # frames

    fig = plt.figure(figsize=(12, 7))
    ax3d = fig.add_axes([0.0, 0.15, 0.75, 0.82], projection='3d')
    ax_eR = fig.add_axes([0.78, 0.55, 0.06, 0.35])
    ax_eT = fig.add_axes([0.88, 0.55, 0.06, 0.35])

    # Static elements
    ax3d.scatter(*BASE,   color='black',  s=80, marker='^', zorder=6)
    ax3d.scatter(*TARGET[:3], color='purple', s=80, marker='s', zorder=6)
    ax3d.scatter(*W_RVZ,  color='green',  s=150, marker='*', zorder=6)
    ax3d.text(TARGET[0], TARGET[1], TARGET[2]+1, 'Target', fontsize=7, color='purple')
    ax3d.text(W_RVZ[0],  W_RVZ[1],  W_RVZ[2]+1, 'RVZ',    fontsize=7, color='green')

    # Altitude reference planes (faint)
    xs = np.array([0, 65, 65, 0])
    ys = np.array([-6, -6, 6, 6])
    ax3d.plot_surface(
        xs.reshape(2, 2), ys.reshape(2, 2),
        np.full((2, 2), Z_R_CRUISE),
        alpha=0.05, color='red'
    )
    ax3d.plot_surface(
        xs.reshape(2, 2), ys.reshape(2, 2),
        np.full((2, 2), Z_T_LOITER),
        alpha=0.05, color='blue'
    )

    # Axis limits
    all_x = np.concatenate([pR[:, 0], pT[:, 0]])
    all_y = np.concatenate([pR[:, 1], pT[:, 1]])
    all_z = np.concatenate([pR[:, 2], pT[:, 2]])
    pad = 3.0
    ax3d.set_xlim(all_x.min() - pad, all_x.max() + pad)
    ax3d.set_ylim(all_y.min() - pad*3, all_y.max() + pad*3)
    ax3d.set_zlim(max(0, all_z.min() - 1), all_z.max() + pad)

    ax3d.set_xlabel('X (m)', fontsize=8)
    ax3d.set_ylabel('Y (m)', fontsize=8)
    ax3d.set_zlabel('Z (m)', fontsize=8)

    # Drone markers
    dot_R, = ax3d.plot([], [], [], 'o', color='red',  ms=8, zorder=10)
    dot_T, = ax3d.plot([], [], [], 'o', color='blue', ms=8, zorder=10)
    trail_R, = ax3d.plot([], [], [], '-', color='red',  lw=1.2, alpha=0.6)
    trail_T, = ax3d.plot([], [], [], '-', color='blue', lw=1.2, alpha=0.6)

    title_text = ax3d.set_title('', fontsize=10)

    # Energy bar axes
    for ax_e, label, color in [(ax_eR, 'Recv\nEnergy', 'red'), (ax_eT, 'Tanker\nEnergy', 'blue')]:
        ax_e.set_xlim(0, 1)
        ax_e.set_ylim(0, max(E0_T, E0_R + E_XFER))
        ax_e.set_xticks([])
        ax_e.set_title(label, fontsize=8, color=color)
        ax_e.set_ylabel('J', fontsize=7)
        ax_e.grid(True, axis='y', alpha=0.3)

    bar_R = ax_eR.bar([0.5], [E0_R], width=0.6, color='red',  alpha=0.7)
    bar_T = ax_eT.bar([0.5], [E0_T], width=0.6, color='blue', alpha=0.7)

    # Find dock_success frame index
    t_ds = events.get('dock_success')
    frame_dock = np.searchsorted(t_an, t_ds) if t_ds else None

    def update(frame):
        i = frame
        # Trail
        i0 = max(0, i - trail_len)
        trail_R.set_data(pR[i0:i+1, 0], pR[i0:i+1, 1])
        trail_R.set_3d_properties(pR[i0:i+1, 2])
        trail_T.set_data(pT[i0:i+1, 0], pT[i0:i+1, 1])
        trail_T.set_3d_properties(pT[i0:i+1, 2])
        # Dots
        dot_R.set_data([pR[i, 0]], [pR[i, 1]])
        dot_R.set_3d_properties([pR[i, 2]])
        dot_T.set_data([pT[i, 0]], [pT[i, 1]])
        dot_T.set_3d_properties([pT[i, 2]])
        # Title
        phase_label = ''
        if t_ds and t_an[i] >= t_ds:
            phase_label = ' [REFUELED]'
        title_text.set_text(f't = {t_an[i]:.1f} s{phase_label}')
        # Energy bars
        bar_R[0].set_height(max(0, eR[i]))
        bar_T[0].set_height(max(0, eT[i]))
        # Color the receiver bar after dock success
        if frame_dock and i >= frame_dock:
            bar_R[0].set_color('limegreen')
        return dot_R, dot_T, trail_R, trail_T, title_text, bar_R[0], bar_T[0]

    legend_elements = [
        Line2D([0], [0], color='red',    lw=2, label='Receiver'),
        Line2D([0], [0], color='blue',   lw=2, label='Tanker'),
        Line2D([0], [0], marker='*', color='green',  ls='None', ms=10, label='Refuel point'),
        Line2D([0], [0], marker='^', color='black',  ls='None', ms=8,  label='Base'),
        Line2D([0], [0], marker='s', color='purple', ls='None', ms=8,  label='Target'),
    ]
    ax3d.legend(handles=legend_elements, fontsize=8, loc='upper left')

    ani = animation.FuncAnimation(
        fig, update, frames=len(t_an), interval=50, blit=False
    )
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    data = run_simulation()
    out_dir = OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    plot_trajectory_3d(data, out_dir)
    plot_altitude_profile(data, out_dir)
    plot_energy_levels(data, out_dir)
    plot_docking_detail(data, out_dir)
    save_animation(data, out_dir)
    print("All outputs saved.")
