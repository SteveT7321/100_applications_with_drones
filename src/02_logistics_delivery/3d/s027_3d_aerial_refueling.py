"""
S027 3D Aerial Refueling Relay
================================
A delivery drone (Receiver) flies a 3-D waypoint mission (climb-out, 10 m cruise,
descent to a 5 m delivery point, return).  Its battery cannot cover the whole
route, which the card's range trigger detects; the trigger launches a Tanker
that flies to a loiter orbit at 15 m, 5 m above the Receiver's cruise altitude.
The Receiver captures the rendezvous horizontally, climbs to the Tanker's
altitude (altitude-match phase), soft-docks in all three axes with a PD law
that includes the Tanker's acceleration as feed-forward, receives energy and
resumes its waypoint mission; the Tanker returns to base.  A second run
without refuelling shows the forced landing the trigger predicts.

A drone whose battery reaches 0 J stops (forced landing).  The dock condition
is checked on the recorded state of both drones before either is advanced.

Outputs: trajectory_3d.png, altitude_profile.png, energy_levels.png,
         docking_detail.png, animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s027_3d_aerial_refueling.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import matplotlib.animation as animation
from matplotlib.lines import Line2D

# ── Output directory ────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's027_3d_aerial_refueling',
))

# ── Parameters (card values unless noted) ──────────────────────────────────
DT          = 0.05      # s
T_MAX       = 120.0     # s

V_CRUISE    = 5.0       # m/s  cruise speed and speed limit (both drones)
V_T_ORB     = 2.0       # m/s  tanker loiter speed
R_ORB       = 4.0       # m    loiter radius
Z_R_CRUISE  = 10.0      # m    receiver cruise altitude
Z_T_LOITER  = 15.0      # m    tanker loiter altitude

K_ALT       = 1.2       # 1/s  altitude-match gain
V_Z_MAX     = 2.0       # m/s  max vertical speed in altitude match
DELTA_Z_OK  = 1.0       # m    altitude-match tolerance (card delta_z)
V_H_ALT     = 3.5       # m/s  horizontal speed during altitude match (0.7 V)

K_FLIGHT    = 0.3       # W s^2/m^2
K_HOVER     = 0.5       # W

E_SCALE     = 4.0       # card energies x4 (card 40 J = 25 m of range, see README)
E0_R        = 40.0 * E_SCALE   # J  receiver initial battery (160 J)
E0_T        = 80.0 * E_SCALE   # J  tanker initial battery   (320 J)
E_XFER      = 30.0 * E_SCALE   # J  energy transferred       (120 J)
ETA_LOSS    = 0.05

D_CAP       = 12.0      # m    horizontal capture radius around the RVZ
D_DOCK      = 0.5       # m    soft-dock engagement radius
EPS_P       = 0.20      # m    dock position tolerance
EPS_V       = 0.08      # m/s  dock velocity tolerance
KP, KD      = 2.0, 1.5  # approach PD gains
KP2, KD2    = 4.0, 2.5  # soft-dock PD gains

MARGIN      = 5.0       # m    range safety margin
ALPHA_RVZ   = 0.50      # rendezvous fraction along base -> target
ARRIVE_R    = 0.5       # m    waypoint arrival radius

BASE        = np.array([0.0, 0.0, 0.0])
TARGET      = np.array([60.0, 0.0, 5.0])     # delivery point (descend to 5 m)
TANKER_PAD  = np.array([0.0, 2.0, 0.0])      # tanker launch pad at the base
W_RVZ = np.array([(1 - ALPHA_RVZ) * BASE[0] + ALPHA_RVZ * TARGET[0],
                  (1 - ALPHA_RVZ) * BASE[1] + ALPHA_RVZ * TARGET[1],
                  Z_T_LOITER])
OMEGA_ORB   = V_T_ORB / R_ORB
PHI0        = np.pi     # orbit entry point = side of the circle facing the base

WAYPOINTS_R = np.array([
    [0.0,  0.0, 0.0],          # WP0 base
    [5.0,  0.0, Z_R_CRUISE],   # WP1 climb-out
    [55.0, 0.0, Z_R_CRUISE],   # WP2 cruise
    [60.0, 0.0, 5.0],          # WP3 delivery point (TARGET)
    [55.0, 0.0, Z_R_CRUISE],   # WP4 return climb
    [5.0,  0.0, Z_R_CRUISE],   # WP5 homeward
    [0.0,  0.0, 0.0],          # WP6 base
])
WP_TARGET = 3

CRUISE_POWER = K_FLIGHT * V_CRUISE ** 2 + K_HOVER      # 8.0 W
ENERGY_PER_M = CRUISE_POWER / V_CRUISE                 # 1.6 J/m


# ── Helpers ─────────────────────────────────────────────────────────────────

def battery_drain(speed, dt):
    return (K_FLIGHT * speed ** 2 + K_HOVER) * dt


def range_remaining(energy, speed=V_CRUISE):
    """Card model: d_rem = E / (k_f v^2 + k_h) * v."""
    return energy / (K_FLIGHT * speed ** 2 + K_HOVER) * speed


def route_remaining(pos, wp_idx):
    """Length of the rest of the waypoint route from pos via WAYPOINTS_R[wp_idx:]."""
    if wp_idx >= len(WAYPOINTS_R):
        return 0.0
    d = np.linalg.norm(WAYPOINTS_R[wp_idx] - pos)
    legs = np.linalg.norm(np.diff(WAYPOINTS_R[wp_idx:], axis=0), axis=1).sum()
    return float(d + legs)


ROUTE_LEN = route_remaining(BASE, 1)


def orbit_state(tl):
    """Tanker loiter position, velocity and acceleration at loiter time tl."""
    a = OMEGA_ORB * tl + PHI0
    c, s = np.cos(a), np.sin(a)
    p = W_RVZ + R_ORB * np.array([c, s, 0.0])
    v = V_T_ORB * np.array([-s, c, 0.0])
    acc = -R_ORB * OMEGA_ORB ** 2 * np.array([c, s, 0.0])
    return p, v, acc


def clamp_speed(vel, v_max=V_CRUISE):
    n = np.linalg.norm(vel)
    return vel * (v_max / n) if n > v_max else vel


def toward(pos, goal, speed=V_CRUISE):
    d = goal - pos
    n = np.linalg.norm(d)
    if n < 1e-9:
        return np.zeros(3)
    return d / n * min(speed, n / DT)


# ── Simulation ──────────────────────────────────────────────────────────────

def simulate(refuel=True, soft_dock_ff=True):
    pR, vR, ER = BASE.copy(), np.zeros(3), E0_R
    pT, vT, ET = TANKER_PAD.copy(), np.zeros(3), E0_T
    aT = np.zeros(3)
    fsm_R, fsm_T = 'mission', 'at_base'
    wp_idx = 1
    t_loiter = 0.0
    ev = {}
    log = []
    max_step_T = 0.0
    for k in range(int(round(T_MAX / DT)) + 1):
        t = k * DT
        dr, dv = pR - pT, vR - vT
        need = route_remaining(pR, wp_idx) + MARGIN if fsm_R != 'home' else 0.0
        d_rem = range_remaining(ER)
        log.append((t, *pR, *pT, *vR, *vT, ER, ET, d_rem, need, fsm_R, fsm_T))

        # ---- termination ----------------------------------------------------
        if fsm_R == 'home' and fsm_T in ('home', 'at_base'):
            ev['complete'] = t
            break
        if fsm_R == 'landed' and fsm_T in ('home', 'at_base', 'landed'):
            break

        # ---- events on the recorded state ------------------------------------
        if refuel and 'trigger' not in ev and fsm_R == 'mission' and d_rem < need:
            ev['trigger'] = (t, d_rem, need)
            fsm_T = 'to_orbit'
        if (fsm_R == 'docking' and np.linalg.norm(dr) < EPS_P
                and np.linalg.norm(dv) < EPS_V):
            ev['dock'] = (t, float(np.linalg.norm(dr)), float(np.linalg.norm(dv)), k,
                          ER, ET)
            ER += E_XFER
            ET -= E_XFER * (1 + ETA_LOSS)
            ev['after_xfer'] = (ER, ET)
            fsm_R, fsm_T = 'mission', 'return'

        # ---- Tanker guidance --------------------------------------------------
        if fsm_T == 'to_orbit':
            entry, _, _ = orbit_state(0.0)
            if np.linalg.norm(entry - pT) > 1e-6:
                vT, aT = toward(pT, entry), np.zeros(3)
            else:
                fsm_T, t_loiter = 'loiter', 0.0
                ev['loiter'] = t
        if fsm_T == 'loiter':
            _, vT, aT = orbit_state(t_loiter)
        elif fsm_T == 'return':
            aT = np.zeros(3)
            if np.linalg.norm(TANKER_PAD - pT) > 1e-6:
                vT = toward(pT, TANKER_PAD)
            else:
                vT, fsm_T = np.zeros(3), 'home'
                ev['tanker_home'] = t
        elif fsm_T == 'landed':
            vT, aT = np.zeros(3), np.zeros(3)

        # ---- Receiver guidance ------------------------------------------------
        if fsm_R == 'mission':
            h_to_rvz = np.linalg.norm(pR[:2] - W_RVZ[:2])
            if (refuel and 'trigger' in ev and 'dock' not in ev
                    and h_to_rvz < D_CAP and wp_idx <= 2):
                fsm_R = 'alt_match'
                ev['alt_match'] = t
            else:
                if np.linalg.norm(WAYPOINTS_R[wp_idx] - pR) < ARRIVE_R:
                    if wp_idx == WP_TARGET:
                        ev['delivered'] = t
                    if wp_idx == len(WAYPOINTS_R) - 1:
                        fsm_R = 'home'
                        ev['receiver_home'] = t
                    else:
                        wp_idx += 1
                vR = toward(pR, WAYPOINTS_R[wp_idx]) if fsm_R == 'mission' else np.zeros(3)
        if fsm_R == 'alt_match':
            hor = np.array([W_RVZ[0] - pR[0], W_RVZ[1] - pR[1], 0.0])
            hn = np.linalg.norm(hor)
            v_h = hor / hn * min(V_H_ALT, hn / DT) if hn > 1e-9 else np.zeros(3)
            vz = float(np.clip(K_ALT * (Z_T_LOITER - pR[2]), -V_Z_MAX, V_Z_MAX))
            vR = clamp_speed(v_h + np.array([0.0, 0.0, vz]))
            if abs(pR[2] - Z_T_LOITER) < DELTA_Z_OK:
                if fsm_T == 'loiter':
                    fsm_R = 'docking'
                    ev['dock_start'] = t
                else:
                    fsm_R = 'hold'
                    ev['hold'] = t
        if fsm_R == 'hold':
            vR = np.zeros(3)
            if fsm_T == 'loiter':
                fsm_R = 'docking'
                ev['dock_start'] = t
        if fsm_R == 'docking':
            if np.linalg.norm(dr) > D_DOCK:
                acc = -KP * dr - KD * dv + aT
            else:
                acc = -KP2 * dr - KD2 * dv + (aT if soft_dock_ff else 0.0)
            vR = clamp_speed(vR + acc * DT)
        if fsm_R in ('home', 'landed'):
            vR = np.zeros(3)

        # ---- integrate ------------------------------------------------------
        pR = pR + vR * DT
        pT_old = pT.copy()
        if fsm_T == 'loiter':
            t_loiter += DT
            pT, _, _ = orbit_state(t_loiter)
        else:
            pT = pT + vT * DT
        max_step_T = max(max_step_T, float(np.linalg.norm(pT - pT_old)))

        # ---- battery: an empty battery is a forced landing --------------------
        if fsm_R not in ('home', 'landed'):
            ER -= battery_drain(np.linalg.norm(vR), DT)
            if ER <= 0.0:
                ER = 0.0
                fsm_R = 'landed'
                ev['forced_landing_R'] = (t + DT, pR.copy())
        if fsm_T not in ('at_base', 'home', 'landed'):
            ET -= battery_drain(np.linalg.norm(vT), DT)
            if ET <= 0.0:
                ET = 0.0
                fsm_T = 'landed'
                ev['forced_landing_T'] = (t + DT, pT.copy())

    cols = list(zip(*log))
    h = {
        'times': np.array(cols[0]),
        'pos_R': np.array(cols[1:4]).T, 'pos_T': np.array(cols[4:7]).T,
        'vel_R': np.array(cols[7:10]).T, 'vel_T': np.array(cols[10:13]).T,
        'E_R': np.array(cols[13]), 'E_T': np.array(cols[14]),
        'd_rem': np.array(cols[15]), 'need': np.array(cols[16]),
        'phase_R': np.array(cols[17]), 'phase_T': np.array(cols[18]),
        'events': ev, 'max_step_T': max_step_T, 'W_RVZ': W_RVZ,
    }
    h['phases'] = h['phase_R']
    h['path_len_R'] = float(np.sum(np.linalg.norm(np.diff(h['pos_R'], axis=0), axis=1)))
    return h


def run_simulation():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print('S027 3D Aerial Refueling Relay')
    print(f'  Route length base->target->base : {ROUTE_LEN:.2f} m '
          f'({ROUTE_LEN * ENERGY_PER_M:.1f} J at {ENERGY_PER_M:.1f} J/m)')
    print(f'  Receiver battery {E0_R:.0f} J -> range {range_remaining(E0_R):.1f} m; '
          f'tanker {E0_T:.0f} J; transfer {E_XFER:.0f} J (+{ETA_LOSS:.0%} loss)')

    data = simulate(refuel=True)
    ev = data['events']
    print('\nRefuelled run:')
    if 'trigger' in ev:
        tt, dr_, nd = ev['trigger']
        print(f'  Refuel trigger      : t = {tt:.2f} s  (d_rem {dr_:.1f} m < need {nd:.1f} m)')
    for key, lab in [('loiter', 'Tanker on orbit'), ('alt_match', 'Altitude match start'),
                     ('hold', 'Receiver holds'), ('dock_start', 'Docking start')]:
        if key in ev:
            print(f'  {lab:<20s}: t = {ev[key]:.2f} s')
    if 'dock' in ev:
        td, drm, dvm, k, er_b, et_b = ev['dock']
        print(f'  Dock success        : t = {td:.2f} s  (docking {td - ev["dock_start"]:.2f} s), '
              f'|dr| {drm:.4f} m, |dv| {dvm:.4f} m/s (tol {EPS_P} m / {EPS_V} m/s)')
        print(f'  Energy before xfer  : Receiver {er_b:.2f} J, Tanker {et_b:.2f} J')
        print(f'  Energy after xfer   : Receiver {ev["after_xfer"][0]:.2f} J, '
              f'Tanker {ev["after_xfer"][1]:.2f} J')
        i0 = int(round(ev['dock_start'] / DT))
        alt_i = int(round(ev['alt_match'] / DT))
        print(f'  Altitude at alt-match start: Receiver {data["pos_R"][alt_i, 2]:.2f} m, '
              f'Tanker {data["pos_T"][alt_i, 2]:.2f} m')
        print(f'  Relative state at dock start: |dr| {np.linalg.norm(data["pos_R"][i0] - data["pos_T"][i0]):.3f} m '
              f'(dz {data["pos_R"][i0, 2] - data["pos_T"][i0, 2]:+.3f} m), '
              f'|dv| {np.linalg.norm(data["vel_R"][i0] - data["vel_T"][i0]):.3f} m/s')
    else:
        print('  Dock success        : NO')
    for key, lab in [('delivered', 'Package delivered'), ('tanker_home', 'Tanker home'),
                     ('receiver_home', 'Receiver home'), ('complete', 'Mission complete')]:
        if key in ev:
            print(f'  {lab:<20s}: t = {ev[key]:.2f} s')
    for key, lab in [('forced_landing_R', 'Receiver'), ('forced_landing_T', 'Tanker')]:
        if key in ev:
            print(f'  {lab} FORCED LANDING at t = {ev[key][0]:.2f} s, pos {np.round(ev[key][1], 2)}')
    print(f'  Final energy        : Receiver {data["E_R"][-1]:.2f} J, Tanker {data["E_T"][-1]:.2f} J')
    print(f'  Receiver path flown : {data["path_len_R"]:.2f} m')
    print(f'  Max tanker step     : {data["max_step_T"]:.3f} m per {DT} s '
          f'(speed limit x DT = {V_CRUISE * DT:.3f} m; no teleport)')
    if 'dock' in ev:
        k = ev['dock'][3]
        margin = data['d_rem'][k + 1:] - data['need'][k + 1:]
        home = data['phase_R'][k + 1:] != 'home'
        print(f'  Min range margin after refuel (d_rem - need, need incl. {MARGIN:.0f} m): '
              f'{margin[home].min():.2f} m')

    base = simulate(refuel=False)
    bev = base['events']
    print('\nNo-refuel run:')
    if 'forced_landing_R' in bev:
        tl, pl = bev['forced_landing_R']
        print(f'  Receiver FORCED LANDING at t = {tl:.2f} s, pos ({pl[0]:.2f}, {pl[1]:.2f}, {pl[2]:.2f}) m, '
              f'after {base["path_len_R"]:.2f} m of {ROUTE_LEN:.2f} m'
              f'{"; package delivered" if "delivered" in bev else "; package NOT delivered"}')
    else:
        print('  Receiver completed without refuel')
    data['no_refuel'] = base

    chk = simulate(refuel=True, soft_dock_ff=False)
    cev = chk['events']
    ds = int(round(cev['dock_start'] / DT)) if 'dock_start' in cev else None
    print('\nCheck: card soft-dock law without the a_T feed-forward:')
    if 'dock' in cev:
        print(f'  docks at t = {cev["dock"][0]:.2f} s')
    elif ds is not None:
        dk = np.nonzero(chk['phase_R'] == 'docking')[0]
        rel = np.linalg.norm(chk['pos_R'][dk] - chk['pos_T'][dk], axis=1)
        tail = rel[-int(round(20.0 / DT)):]
        fl = cev.get('forced_landing_R')
        print(f'  never docks: docking from t = {cev["dock_start"]:.2f} s until '
              f'{"the battery is empty at t = %.2f s" % fl[0] if fl else "T_MAX"}')
        print(f'  |dr| over the last 20 s of docking: {tail.min():.3f}-{tail.max():.3f} m '
              f'(eps_p {EPS_P} m), expected lag a_T / sqrt((K_p2 - w^2)^2 + (K_d2 w)^2) = '
              f'{R_ORB * OMEGA_ORB ** 2 / np.hypot(KP2 - OMEGA_ORB ** 2, KD2 * OMEGA_ORB):.3f} m')
    return data


# ── Plot helpers ────────────────────────────────────────────────────────────

EV_STYLE = [
    ('loiter', 'cyan', 'Tanker on orbit'),
    ('alt_match', 'magenta', 'Alt-match start'),
    ('dock_start', 'orange', 'Dock start'),
    ('dock', 'green', 'Dock success / transfer'),
    ('delivered', 'purple', 'Package delivered'),
]


def _ev_time(ev, key):
    v = ev.get(key)
    if v is None:
        return None
    return v[0] if isinstance(v, tuple) else v


def _mark_events(ax, ev):
    for key, col, lab in EV_STYLE:
        tv = _ev_time(ev, key)
        if tv is not None:
            ax.axvline(tv, color=col, ls=':', lw=1.4, label=lab)


# ── Plots ───────────────────────────────────────────────────────────────────

def plot_trajectory_3d(data, out_dir):
    pR, pT, ev, times = data['pos_R'], data['pos_T'], data['events'], data['times']
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(*pR.T, color='red', lw=1.5, label='Receiver', alpha=0.85)
    ax.plot(*pT.T, color='blue', lw=1.5, label='Tanker', alpha=0.85)
    nr = data['no_refuel']
    ax.plot(*nr['pos_R'].T, color='red', lw=1.0, ls='--', alpha=0.5,
            label='Receiver without refuel')
    if 'forced_landing_R' in nr['events']:
        ax.scatter(*nr['events']['forced_landing_R'][1], color='red', marker='x', s=90,
                   label='Forced landing (no refuel)')
    if 'dock' in ev:
        i0 = int(round(ev['dock_start'] / DT))
        i1 = ev['dock'][3]
        ax.plot(*pR[i0:i1 + 1].T, color='gold', lw=3.0, label='Docking approach')
    ax.scatter(*BASE, color='black', s=80, marker='^', label='Base')
    ax.scatter(*TARGET, color='purple', s=80, marker='s', label='Delivery point')
    ax.scatter(*W_RVZ, color='green', s=150, marker='*', label='Refueling point (RVZ)')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z / altitude (m)')
    ax.set_title('S027 3D Aerial Refueling — Trajectories')
    ax.legend(fontsize=8, loc='upper left')
    ax.set_xlim(-3, 63); ax.set_ylim(-12, 12); ax.set_zlim(0, 18)
    ax.view_init(elev=22, azim=-70)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_profile(data, out_dir):
    times, pR, pT, ev = data['times'], data['pos_R'], data['pos_T'], data['events']
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(times, pR[:, 2], color='red', lw=1.8, label='Receiver altitude')
    ax.plot(times, pT[:, 2], color='blue', lw=1.8, label='Tanker altitude')
    ax.axhline(Z_R_CRUISE, color='red', ls='--', lw=0.8, alpha=0.5,
               label=f'Receiver cruise ({Z_R_CRUISE:g} m)')
    ax.axhline(Z_T_LOITER, color='blue', ls='--', lw=0.8, alpha=0.5,
               label=f'Tanker loiter ({Z_T_LOITER:g} m)')
    _mark_events(ax, ev)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude (m)')
    ax.set_title('S027 3D — Altitude profile: the Receiver climbs 5 m to the Tanker before docking')
    ax.legend(fontsize=8, ncol=4, loc='upper center', bbox_to_anchor=(0.5, -0.13))
    ax.set_ylim(-1, Z_T_LOITER + 3); ax.grid(True, alpha=0.3)
    path = os.path.join(out_dir, 'altitude_profile.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_energy_levels(data, out_dir):
    times, ev = data['times'], data['events']
    nr = data['no_refuel']
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(times, data['E_R'], color='red', lw=1.8, label='Receiver energy')
    ax.plot(times, data['E_T'], color='blue', lw=1.8, label='Tanker energy')
    ax.plot(nr['times'], nr['E_R'], color='red', lw=1.2, ls='--', alpha=0.6,
            label='Receiver without refuel')
    if 'forced_landing_R' in nr['events']:
        tl = nr['events']['forced_landing_R'][0]
        ax.scatter([tl], [0.0], color='red', marker='x', s=80, zorder=5,
                   label=f'Forced landing (no refuel), t = {tl:.1f} s')
    _mark_events(ax, ev)
    if 'dock' in ev:
        k = ev['dock'][3]
        ax.annotate(f'+{E_XFER:.0f} J', xy=(times[k + 1], data['E_R'][k + 1]),
                    xytext=(times[k + 1] + 3, data['E_R'][k + 1] + 15),
                    arrowprops=dict(arrowstyle='->', color='red'), color='red', fontsize=9)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Energy (J)')
    ax.set_title('S027 3D — Battery energy (refuelled run, and Receiver without refuel)')
    ax.legend(fontsize=8, ncol=2, loc='upper right')
    ax.set_ylim(-5, E0_T + 10); ax.grid(True, alpha=0.3)
    path = os.path.join(out_dir, 'energy_levels.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_docking_detail(data, out_dir):
    ev = data['events']
    if 'dock' not in ev:
        print('Docking detail: no dock, skipped')
        return
    times = data['times']
    i0 = max(0, int(round(ev['alt_match'] / DT)) - 10)
    i1 = min(len(times), ev['dock'][3] + 30)
    t = times[i0:i1]
    dr = data['pos_R'][i0:i1] - data['pos_T'][i0:i1]
    dv = data['vel_R'][i0:i1] - data['vel_T'][i0:i1]
    fig, axes = plt.subplots(5, 1, figsize=(10, 11), sharex=True)
    for i, (lab, col) in enumerate(zip(['Δx (m)', 'Δy (m)', 'Δz (m)'],
                                       ['tomato', 'steelblue', 'seagreen'])):
        ax = axes[i]
        ax.plot(t, dr[:, i], color=col, lw=1.5)
        ax.axhline(0, color='k', lw=0.6, ls='--')
        ax.set_ylabel(lab); ax.grid(True, alpha=0.3)
    ax = axes[3]
    ax.semilogy(t, np.maximum(np.linalg.norm(dr, axis=1), 1e-4), color='purple', lw=1.8,
                label='|Δr|')
    ax.axhline(EPS_P, color='red', lw=0.9, ls=':', label=f'ε_p = {EPS_P} m')
    ax.axhline(D_DOCK, color='orange', lw=0.9, ls='--', label=f'd_dock = {D_DOCK} m')
    ax.set_ylabel('|Δr| (m)'); ax.grid(True, alpha=0.3, which='both')
    ax.legend(fontsize=8, loc='upper right')
    ax = axes[4]
    ax.semilogy(t, np.maximum(np.linalg.norm(dv, axis=1), 1e-4), color='brown', lw=1.8,
                label='|Δv|')
    ax.axhline(EPS_V, color='red', lw=0.9, ls=':', label=f'ε_v = {EPS_V} m/s')
    ax.set_ylabel('|Δv| (m/s)'); ax.set_xlabel('Time (s)  (after the dock the Receiver leaves)')
    ax.grid(True, alpha=0.3, which='both'); ax.legend(fontsize=8, loc='upper right')
    for ax in axes:
        for key, col, lab in EV_STYLE[1:4]:
            tv = _ev_time(ev, key)
            if tv is not None:
                ax.axvline(tv, color=col, ls=':', lw=1.3)
    handles = [Line2D([0], [0], color=c, ls=':', lw=1.3, label=l) for _, c, l in EV_STYLE[1:4]]
    axes[0].legend(handles=handles, fontsize=8, loc='upper right')
    fig.suptitle('S027 3D — Altitude match and docking: relative position / velocity (Receiver − Tanker)',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'docking_detail.png')
    plt.savefig(path, dpi=140)
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    times, pR, pT = data['times'], data['pos_R'], data['pos_T']
    E_R, E_T, ev = data['E_R'], data['E_T'], data['events']
    step = 6
    idx = list(range(0, len(times), step))
    if idx[-1] != len(times) - 1:
        idx.append(len(times) - 1)
    idx = np.array(idx)
    trail = 40
    fig = plt.figure(figsize=(11, 6.5))
    ax3d = fig.add_axes([0.0, 0.05, 0.75, 0.9], projection='3d')
    ax_eR = fig.add_axes([0.80, 0.55, 0.06, 0.33])
    ax_eT = fig.add_axes([0.90, 0.55, 0.06, 0.33])
    ax3d.scatter(*BASE, color='black', s=60, marker='^')
    ax3d.scatter(*TARGET, color='purple', s=60, marker='s')
    ax3d.scatter(*W_RVZ, color='green', s=120, marker='*')
    th = np.linspace(0, 2 * np.pi, 100)
    ax3d.plot(W_RVZ[0] + R_ORB * np.cos(th), W_RVZ[1] + R_ORB * np.sin(th),
              np.full_like(th, Z_T_LOITER), 'b:', lw=0.8, alpha=0.6)
    ax3d.set_xlim(-3, 63); ax3d.set_ylim(-12, 12); ax3d.set_zlim(0, 18)
    ax3d.set_xlabel('X (m)', fontsize=8); ax3d.set_ylabel('Y (m)', fontsize=8)
    ax3d.set_zlabel('Z (m)', fontsize=8)
    ax3d.view_init(elev=22, azim=-70)
    dot_R, = ax3d.plot([], [], [], 'o', color='red', ms=7)
    dot_T, = ax3d.plot([], [], [], 'o', color='blue', ms=7)
    tr_R, = ax3d.plot([], [], [], '-', color='red', lw=1.2, alpha=0.6)
    tr_T, = ax3d.plot([], [], [], '-', color='blue', lw=1.2, alpha=0.6)
    title = ax3d.set_title('', fontsize=10)
    for ax_e, lab, col in [(ax_eR, 'Receiver\nenergy', 'red'), (ax_eT, 'Tanker\nenergy', 'blue')]:
        ax_e.set_xlim(0, 1); ax_e.set_ylim(0, E0_T); ax_e.set_xticks([])
        ax_e.set_title(lab, fontsize=8, color=col); ax_e.set_ylabel('J', fontsize=7)
        ax_e.grid(True, axis='y', alpha=0.3)
    bar_R = ax_eR.bar([0.5], [E0_R], width=0.6, color='red', alpha=0.7)
    bar_T = ax_eT.bar([0.5], [E0_T], width=0.6, color='blue', alpha=0.7)
    ax3d.legend(handles=[Line2D([0], [0], color='red', lw=2, label='Receiver'),
                         Line2D([0], [0], color='blue', lw=2, label='Tanker'),
                         Line2D([0], [0], marker='*', color='green', ls='None', ms=10, label='RVZ'),
                         Line2D([0], [0], marker='^', color='black', ls='None', ms=8, label='Base'),
                         Line2D([0], [0], marker='s', color='purple', ls='None', ms=8,
                                label='Delivery point')],
                fontsize=8, loc='upper left')

    def update(f):
        i = idx[f]
        j0 = idx[max(0, f - trail)]
        tr_R.set_data(pR[j0:i + 1, 0], pR[j0:i + 1, 1]); tr_R.set_3d_properties(pR[j0:i + 1, 2])
        tr_T.set_data(pT[j0:i + 1, 0], pT[j0:i + 1, 1]); tr_T.set_3d_properties(pT[j0:i + 1, 2])
        dot_R.set_data([pR[i, 0]], [pR[i, 1]]); dot_R.set_3d_properties([pR[i, 2]])
        dot_T.set_data([pT[i, 0]], [pT[i, 1]]); dot_T.set_3d_properties([pT[i, 2]])
        title.set_text(f't = {times[i]:5.1f} s   Receiver phase: {data["phase_R"][i]}')
        bar_R[0].set_height(max(0.0, E_R[i])); bar_T[0].set_height(max(0.0, E_T[i]))
        return dot_R, dot_T, tr_R, tr_T, title

    ani = animation.FuncAnimation(fig, update, frames=len(idx), interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=15, dpi=80)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    data = run_simulation()
    out_dir = OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    print()
    plot_trajectory_3d(data, out_dir)
    plot_altitude_profile(data, out_dir)
    plot_energy_levels(data, out_dir)
    plot_docking_detail(data, out_dir)
    save_animation(data, out_dir)
    print('All outputs saved.')
