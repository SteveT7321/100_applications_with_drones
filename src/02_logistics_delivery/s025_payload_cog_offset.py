"""
S025 Payload CoG Offset
=======================
A delivery drone carries a payload whose centre-of-gravity (CoG) is not at
the geometric centre of the airframe.  The offset produces a constant roll /
pitch torque disturbance.  Three control strategies are compared:

  1. No compensation  — drone flies with raw PD attitude controller, offset
                        causes sustained tilt and lateral drift.
  2. Static feed-forward — pilot manually trims the reference attitude to
                           counteract the known offset (perfect knowledge).
  3. Integral (I-term) compensation — a PID attitude controller winds up an
                                      integrator that cancels the offset torque
                                      automatically without prior knowledge.

The drone is modelled as a 6-DoF rigid body (position + Euler angles) with
linearised dynamics around hover.  The payload CoG offset is (dx, dy) = (+0.12, +0.08) m
relative to the airframe centre, producing roll and pitch disturbance torques.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s025_payload_cog_offset.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Physical constants ────────────────────────────────────────────────────────
G = 9.81          # m/s²

# ── Drone parameters ──────────────────────────────────────────────────────────
DRONE_MASS     = 1.5        # kg  (airframe + motors)
PAYLOAD_MASS   = 0.8        # kg
TOTAL_MASS     = DRONE_MASS + PAYLOAD_MASS   # 2.3 kg

# Moments of inertia (kg·m²) — approximate quadrotor values
Ixx = 0.015
Iyy = 0.015
Izz = 0.025

# Payload CoG offset relative to drone geometric centre
COG_DX = +0.12   # m  (positive → right / +Y axis on drone body)
COG_DY = +0.08   # m  (positive → forward / +X axis on drone body)

# Disturbance torques caused by CoG offset at hover (τ = m_payload × g × offset)
TAU_DIST_ROLL  =  PAYLOAD_MASS * G * COG_DX   # roll  (about x)  [N·m]
TAU_DIST_PITCH = -PAYLOAD_MASS * G * COG_DY   # pitch (about y)  [N·m]

# ── Control parameters ────────────────────────────────────────────────────────
DT       = 1 / 200          # 200 Hz control loop (smaller dt for stability)
MAX_TIME = 20.0             # seconds
MAX_TILT = np.deg2rad(25)   # safety limit

# PD position controller gains
KP_POS = 1.5
KD_POS = 1.8

# PID attitude controller gains
KP_ATT = 8.0
KD_ATT = 3.0
KI_ATT = 2.0    # only active in PID mode

# Integral anti-windup limit (rad·s)
I_LIMIT = 0.5

# ── Scenario: waypoint mission ────────────────────────────────────────────────
WAYPOINTS = np.array([
    [0.0, 0.0, 2.0],
    [4.0, 0.0, 2.0],
    [4.0, 4.0, 2.0],
    [4.0, 4.0, 0.3],   # descend (delivery)
])
WAYPOINT_REACH_R = 0.15   # m

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's025_payload_cog_offset'
)


# ── 6-DoF drone model ─────────────────────────────────────────────────────────

class DroneRigidBody:
    """
    Simplified 6-DoF rigid body model with linearised rotational dynamics.

    State: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    (position, velocity, Euler angles, body angular rates)
    """

    def __init__(self, init_pos=(0, 0, 0)):
        self.pos   = np.array(init_pos, dtype=float)
        self.vel   = np.zeros(3)
        self.euler = np.zeros(3)   # roll (phi), pitch (theta), yaw (psi)
        self.omega = np.zeros(3)   # body angular rates p, q, r
        self.integral_att = np.zeros(3)

        # history
        self.pos_hist   = [self.pos.copy()]
        self.euler_hist = [self.euler.copy()]
        self.time_hist  = [0.0]
        self._t = 0.0

    def step(self, tau_cmd, thrust, dt, tau_disturbance=np.zeros(3)):
        """
        Advance the state by one time step.

        Parameters
        ----------
        tau_cmd        : (3,) commanded torques [N·m] roll, pitch, yaw
        thrust         : scalar total thrust [N]
        dt             : time step [s]
        tau_disturbance: (3,) external disturbance torques [N·m]
        """
        phi, theta, psi = self.euler

        # ── Rotational dynamics (Euler's equation, simplified) ────────────
        tau_total = tau_cmd + tau_disturbance
        alpha = np.array([tau_total[0] / Ixx,
                          tau_total[1] / Iyy,
                          tau_total[2] / Izz])
        self.omega += alpha * dt
        # Clamp angular rate (physical limit ~10 rad/s)
        self.omega = np.clip(self.omega, -10.0, 10.0)
        # Euler-angle kinematics (small-angle approximation good near hover)
        self.euler += self.omega * dt
        # clamp tilt for numerical stability
        self.euler[0] = np.clip(self.euler[0], -MAX_TILT, MAX_TILT)
        self.euler[1] = np.clip(self.euler[1], -MAX_TILT, MAX_TILT)

        # ── Translational dynamics ────────────────────────────────────────
        phi_c, theta_c = self.euler[0], self.euler[1]
        # Thrust vector in world frame (small angle approx)
        # F_world = R_body2world * [0, 0, thrust]
        # For small angles: Fx ≈ -sin(theta)*T, Fy ≈ sin(phi)*T, Fz ≈ cos(phi)cos(theta)*T
        Fx = -np.sin(theta_c) * thrust
        Fy =  np.sin(phi_c)   * thrust
        Fz =  np.cos(phi_c) * np.cos(theta_c) * thrust

        ax = Fx / TOTAL_MASS
        ay = Fy / TOTAL_MASS
        az = Fz / TOTAL_MASS - G

        self.vel += np.array([ax, ay, az]) * dt
        # Clamp velocity (physical speed limit ~8 m/s)
        spd = np.linalg.norm(self.vel)
        if spd > 8.0:
            self.vel = self.vel / spd * 8.0
        self.pos += self.vel * dt

        self._t += dt
        self.pos_hist.append(self.pos.copy())
        self.euler_hist.append(self.euler.copy())
        self.time_hist.append(self._t)

    def get_trajectories(self):
        return (np.array(self.pos_hist),
                np.array(self.euler_hist),
                np.array(self.time_hist))


# ── Controller ─────────────────────────────────────────────────────────────────

def compute_attitude_command(drone, euler_ref, dt, use_integral=False):
    """PD (or PID) attitude controller → returns torque command."""
    err = euler_ref - drone.euler
    if use_integral:
        drone.integral_att += err * dt
        # Anti-windup: clamp integral
        drone.integral_att = np.clip(drone.integral_att, -I_LIMIT, I_LIMIT)
        tau = (KP_ATT * err
               + KD_ATT * (-drone.omega)
               + KI_ATT * drone.integral_att)
    else:
        tau = KP_ATT * err + KD_ATT * (-drone.omega)
    # Clamp torque to reasonable range
    tau = np.clip(tau, -5.0, 5.0)
    return tau


def run_simulation(mode='no_comp'):
    """
    Run the delivery mission under one of three compensation modes:
      'no_comp'   – PD attitude, no knowledge of CoG offset
      'feedfwd'   – PD attitude, reference angle trimmed to cancel offset
      'pid'       – PID attitude, integrator winds up automatically
    """
    drone = DroneRigidBody(init_pos=WAYPOINTS[0])
    tau_dist = np.array([TAU_DIST_ROLL, TAU_DIST_PITCH, 0.0])

    # Pre-compute static trim angle (used in feedfwd mode)
    # At equilibrium with PD only: KP_ATT*(phi_ref - phi_eq) + tau_dist = 0
    #   => phi_eq = phi_ref + tau_dist/(Ixx*KP_ATT)  [for zero ref: phi_eq = tau_dist/(Ixx*KP_ATT)]
    # Feed-forward offsets the ref by the negative of the equilibrium offset:
    trim_roll  = -(TAU_DIST_ROLL  / Ixx) / KP_ATT
    trim_pitch = -(TAU_DIST_PITCH / Iyy) / KP_ATT

    wpt_idx = 0
    max_steps = int(MAX_TIME / DT)
    use_integral = (mode == 'pid')
    waypoint_times = []

    for step in range(max_steps):
        target = WAYPOINTS[wpt_idx]

        # ── Position controller → desired thrust magnitude and lean direction ──
        pos_err = target - drone.pos
        vel_cmd = KP_POS * pos_err + KD_POS * (-drone.vel)

        # Desired z-axis force (includes gravity compensation)
        thrust = TOTAL_MASS * (G + vel_cmd[2])
        thrust = max(thrust, 0.1)   # keep positive

        # Desired roll/pitch to achieve xy acceleration
        # ax_desired = -sin(theta)*T/m   → theta_ref ≈ -ax*m/T
        # ay_desired =  sin(phi)*T/m     → phi_ref   ≈  ay*m/T
        ax_d = vel_cmd[0]
        ay_d = vel_cmd[1]
        theta_ref = -ax_d * TOTAL_MASS / thrust
        phi_ref   =  ay_d * TOTAL_MASS / thrust
        theta_ref = np.clip(theta_ref, -MAX_TILT, MAX_TILT)
        phi_ref   = np.clip(phi_ref,   -MAX_TILT, MAX_TILT)

        # Apply trim in feed-forward mode
        if mode == 'feedfwd':
            phi_ref   += trim_roll
            theta_ref += trim_pitch

        euler_ref = np.array([phi_ref, theta_ref, 0.0])

        # ── Attitude controller ────────────────────────────────────────────
        tau_cmd = compute_attitude_command(drone, euler_ref, DT,
                                          use_integral=use_integral)

        # ── Plant step ────────────────────────────────────────────────────
        drone.step(tau_cmd, thrust, DT, tau_disturbance=tau_dist)

        # ── Waypoint advance ──────────────────────────────────────────────
        if np.linalg.norm(drone.pos - target) < WAYPOINT_REACH_R:
            waypoint_times.append(step * DT)
            wpt_idx += 1
            if wpt_idx >= len(WAYPOINTS):
                break

    pos_hist, euler_hist, time_hist = drone.get_trajectories()
    return pos_hist, euler_hist, time_hist, waypoint_times


# ── Plotting ──────────────────────────────────────────────────────────────────

MODE_STYLE = {
    'no_comp': dict(color='tomato',      label='No compensation',       ls='-'),
    'feedfwd': dict(color='dodgerblue',  label='Static feed-forward',   ls='--'),
    'pid':     dict(color='limegreen',   label='PID (integral)',         ls='-.'),
}


def plot_trajectories_3d(results, out_dir):
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')

    for mode, (pos, euler, time, wt) in results.items():
        st = MODE_STYLE[mode]
        ax.plot(pos[:, 0], pos[:, 1], pos[:, 2],
                color=st['color'], linewidth=1.6, linestyle=st['ls'],
                label=st['label'])

    # Waypoints
    for i, wp in enumerate(WAYPOINTS):
        ax.scatter(*wp, color='gold', s=120, marker='*', zorder=6,
                   label='Waypoint' if i == 0 else '')
        ax.text(wp[0]+0.1, wp[1]+0.1, wp[2]+0.1, f'WP{i+1}', fontsize=8)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('S025 Payload CoG Offset — 3D Trajectories\n'
                 f'CoG offset: dx={COG_DX:+.2f} m, dy={COG_DY:+.2f} m  '
                 f'| τ_roll={TAU_DIST_ROLL:.3f} N·m, τ_pitch={TAU_DIST_PITCH:.3f} N·m')
    ax.legend(loc='upper left', fontsize=9)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_attitude_and_position_error(results, out_dir):
    fig, axes = plt.subplots(3, 2, figsize=(13, 10))
    fig.suptitle('S025 Payload CoG Offset — Attitude & Position Error Comparison', fontsize=12)

    ax_roll, ax_pitch  = axes[0]
    ax_xe,   ax_ye     = axes[1]
    ax_ze,   ax_dist   = axes[2]

    ideal_traj = np.array([[0, 0, 2], [4, 0, 2], [4, 4, 2], [4, 4, 0.3]])

    for mode, (pos, euler, time, wt) in results.items():
        st   = MODE_STYLE[mode]
        roll_deg  = np.rad2deg(euler[:, 0])
        pitch_deg = np.rad2deg(euler[:, 1])
        n = len(time)

        ax_roll.plot(time, roll_deg,  color=st['color'], lw=1.4,
                     ls=st['ls'], label=st['label'])
        ax_pitch.plot(time, pitch_deg, color=st['color'], lw=1.4,
                      ls=st['ls'])
        ax_xe.plot(time, pos[:, 0], color=st['color'], lw=1.4, ls=st['ls'])
        ax_ye.plot(time, pos[:, 1], color=st['color'], lw=1.4, ls=st['ls'])
        ax_ze.plot(time, pos[:, 2], color=st['color'], lw=1.4, ls=st['ls'])

        # distance from ideal straight line between waypoints (simplified: dist from wpts)
        lateral_err = np.sqrt((pos[:, 0] - pos[:, 0])**2 +
                              (pos[:, 1] - pos[:, 1])**2)  # placeholder
        # Actual lateral deviation: compute distance from x=0 plane on leg 1
        # For simplicity: show total speed (magnitude of vel would need vel_hist)
        # Instead show total deviation from final waypoint
        final_wp = WAYPOINTS[-1]
        dist_to_final = np.linalg.norm(pos - final_wp, axis=1)
        ax_dist.plot(time, dist_to_final, color=st['color'], lw=1.4, ls=st['ls'])

    # Disturbance equilibrium angles
    eq_roll  = np.rad2deg(-TAU_DIST_ROLL  / (Ixx * KP_ATT))
    eq_pitch = np.rad2deg(-TAU_DIST_PITCH / (Iyy * KP_ATT))
    ax_roll.axhline(eq_roll, color='gray', ls=':', lw=1,
                    label=f'Equilibrium {eq_roll:.1f}°')
    ax_pitch.axhline(eq_pitch, color='gray', ls=':', lw=1,
                     label=f'Equilibrium {eq_pitch:.1f}°')

    ax_roll.set_ylabel('Roll φ (deg)')
    ax_roll.legend(fontsize=8)
    ax_roll.grid(True, alpha=0.3)
    ax_roll.set_title('Roll angle over time')

    ax_pitch.set_ylabel('Pitch θ (deg)')
    ax_pitch.grid(True, alpha=0.3)
    ax_pitch.set_title('Pitch angle over time')

    ax_xe.set_ylabel('X (m)')
    ax_xe.grid(True, alpha=0.3)
    ax_xe.set_title('X position')

    ax_ye.set_ylabel('Y (m)')
    ax_ye.grid(True, alpha=0.3)
    ax_ye.set_title('Y position')

    ax_ze.set_ylabel('Z (m)')
    ax_ze.grid(True, alpha=0.3)
    ax_ze.set_title('Altitude Z')

    ax_dist.set_ylabel('Distance to final WP (m)')
    ax_dist.grid(True, alpha=0.3)
    ax_dist.set_title('Distance to final waypoint')

    for ax in axes.flat:
        ax.set_xlabel('Time (s)')

    plt.tight_layout()
    path = os.path.join(out_dir, 'attitude_position.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_roll_pitch_comparison(results, out_dir):
    """Separate detailed roll/pitch plot with equilibrium annotations."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle('S025 — Roll & Pitch Angle Comparison\n'
                 f'CoG offset  dx={COG_DX:+.3f} m  dy={COG_DY:+.3f} m  '
                 f'(τ_roll={TAU_DIST_ROLL:.3f} N·m, τ_pitch={TAU_DIST_PITCH:.3f} N·m)',
                 fontsize=11)

    for mode, (pos, euler, time, wt) in results.items():
        st = MODE_STYLE[mode]
        ax1.plot(time, np.rad2deg(euler[:, 0]),
                 color=st['color'], lw=1.8, ls=st['ls'], label=st['label'])
        ax2.plot(time, np.rad2deg(euler[:, 1]),
                 color=st['color'], lw=1.8, ls=st['ls'], label=st['label'])

    # Steady-state tilt for no-compensation case
    ss_roll  = np.rad2deg(TAU_DIST_ROLL  / (KP_ATT * Ixx))
    ss_pitch = np.rad2deg(TAU_DIST_PITCH / (KP_ATT * Iyy))

    ax1.axhline(ss_roll, color='tomato', ls=':', lw=1.2,
                label=f'SS tilt (no comp): {ss_roll:.1f}°')
    ax1.axhline(0, color='k', lw=0.8, alpha=0.4)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Roll φ (deg)')
    ax1.set_title('Roll angle φ')
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    ax2.axhline(ss_pitch, color='tomato', ls=':', lw=1.2,
                label=f'SS tilt (no comp): {ss_pitch:.1f}°')
    ax2.axhline(0, color='k', lw=0.8, alpha=0.4)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Pitch θ (deg)')
    ax2.set_title('Pitch angle θ')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'roll_pitch_comparison.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    """Top-down (XY) view animation showing trajectory deviation per mode."""
    STEP  = 5    # sample every 5 control steps (~20 fps effective)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect('equal')

    # Ideal path
    ideal = WAYPOINTS[:, :2]
    ax.plot(ideal[:, 0], ideal[:, 1], 'k--', lw=1.2, alpha=0.4, label='Ideal path')
    for i, wp in enumerate(WAYPOINTS):
        ax.scatter(wp[0], wp[1], s=100, color='gold', marker='*', zorder=5,
                   label='Waypoints' if i == 0 else '')
        ax.text(wp[0]+0.12, wp[1]+0.12, f'WP{i+1}', fontsize=8)

    ax.set_xlim(-1, 5.5)
    ax.set_ylim(-1, 5.5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('S025 Payload CoG Offset — Top-Down Animation')
    ax.grid(True, alpha=0.25)

    lines, dots, labels_done = {}, {}, set()
    for mode, (pos, euler, time, wt) in results.items():
        st = MODE_STYLE[mode]
        ln, = ax.plot([], [], color=st['color'], lw=1.6,
                      ls=st['ls'], label=st['label'])
        dt2, = ax.plot([], [], 'o', color=st['color'], ms=7, zorder=6)
        lines[mode] = (ln, pos, time)
        dots[mode]  = (dt2, pos)

    ax.legend(loc='upper left', fontsize=9)

    # Find the maximum number of frames across all modes
    max_frames = max(
        len(range(0, len(pos), STEP))
        for _, (pos, euler, time, wt) in results.items()
    )

    def update(fi):
        artists = []
        for mode, (ln, pos, time) in lines.items():
            idx = min(fi * STEP, len(pos) - 1)
            ln.set_data(pos[:idx+1, 0], pos[:idx+1, 1])
            dots[mode][0].set_data([pos[idx, 0]], [pos[idx, 1]])
            artists.extend([ln, dots[mode][0]])
        t_val = fi * STEP * DT
        ax.set_title(f'S025 Payload CoG Offset — Top-Down   t={t_val:.2f} s')
        return artists

    ani = animation.FuncAnimation(fig, update, frames=max_frames,
                                  interval=50, blit=False)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20)
    plt.close()
    print(f'Saved: {path}')


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('=== S025 Payload CoG Offset Simulation ===')
    print(f'Total mass        : {TOTAL_MASS:.2f} kg')
    print(f'CoG offset        : dx={COG_DX:+.3f} m, dy={COG_DY:+.3f} m')
    print(f'Disturbance torque: τ_roll={TAU_DIST_ROLL:.3f} N·m, '
          f'τ_pitch={TAU_DIST_PITCH:.3f} N·m')
    print()

    results = {}
    for mode in ('no_comp', 'feedfwd', 'pid'):
        pos, euler, time, wt = run_simulation(mode)
        results[mode] = (pos, euler, time, wt)
        final_pos = pos[-1]
        final_target = WAYPOINTS[min(len(wt), len(WAYPOINTS)-1)]
        ss_roll  = np.rad2deg(euler[-1, 0])
        ss_pitch = np.rad2deg(euler[-1, 1])
        print(f'[{mode:8s}]  Waypoints reached: {len(wt)}/{len(WAYPOINTS)}  '
              f'Final pos: ({final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f})  '
              f'Final roll={ss_roll:.2f}° pitch={ss_pitch:.2f}°')

    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)

    plot_trajectories_3d(results, out_dir)
    plot_attitude_and_position_error(results, out_dir)
    plot_roll_pitch_comparison(results, out_dir)
    save_animation(results, out_dir)
    print('\nAll outputs saved.')
