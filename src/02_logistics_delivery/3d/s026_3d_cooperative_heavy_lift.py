"""
S026 3D Cooperative Heavy Lift
==============================
Four quadrotors carry a 0.8 kg rigid box on four 0.5 m cables: lift from
(0, 0, 0.1) to z = 4 m (0-5 s), cruise to (5, 3, 4) m past two cylindrical
poles (5-20 s), descend to (5, 3, 0.1) m (20-25 s).

What the 3D card adds over the 2D S026, and how it is modelled here
-------------------------------------------------------------------
* 6-DOF load: the box rotates (quaternion + Euler's equations, uniform-cuboid
  inertia).  Cables attach at the top-face edge midpoints.
* Cable weight (catenary, parabolic approximation): each cable's weight
  mu g l0 is split half to each end; the allocator uses the card's sagged
  bottom-end direction c_bot = normalise(chord - mu g l0 / (2 f) z).
* Moment-aware allocation: bounded least squares on the card's 6-row
  equilibrium matrix (force + moment rows), f_i >= f_min.  The 2D allocator
  (force rows only) is kept as a comparison.
* Swing damping: a tilt-dependent altitude offset per drone (gain K_eta) fed
  into the PD position AND velocity setpoints, so K_d turns it into damping.
* Obstacle negotiation with per-drone altitude changes:
    flat  -- 2D behaviour: the cross stays flat, no avoidance
    climb -- the card's rule: a drone whose slot comes within r_obs + d_clear
             of a pole climbs dz_step = 0.8 m (at 0.3 m/s) above its slot
    tuck1 -- feasible magnitude: the triggered drone climbs dz_tuck = 0.08 m
             along its cable sphere (moving inward so the cable stays at
             l0), which narrows the cross on that side
    pair  -- as tuck1, but the opposite drone of the same pair tucks too,
             so each pair stays symmetric (no net moment on the load)
  A single-drone tuck is also flown at hover (mission='hover': drone 3 tucks
  at t = 5 s and holds), to separate the static allocation question from the
  cruise dynamics, and in cruise with a slower 0.05 m/s tuck rate.

Physics (1 kHz, semi-implicit Euler): drones are point masses with vector
thrust |u| <= T_MAX; cables are stiff unilateral spring-dampers, as in the 2D
script.  Controller at 100 Hz.  Everything is deterministic (no random
inputs), so there is no Monte Carlo; robustness to the obstacle placement is
checked with a sweep of the poles' lateral offset.

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s026_3d_cooperative_heavy_lift.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.optimize import lsq_linear

# ── Output directory ──────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, 'outputs', '02_logistics_delivery',
                          '3d', 's026_3d_cooperative_heavy_lift')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Physical constants ────────────────────────────────────────────────────────
G           = 9.81
N_DRONES    = 4
M_LOAD      = 0.8                    # kg
HALF        = np.array([0.15, 0.15, 0.05])   # (a, b, c) half-lengths, m
I_LOAD      = M_LOAD / 3.0 * np.array([HALF[1]**2 + HALF[2]**2,
                                       HALF[0]**2 + HALF[2]**2,
                                       HALF[0]**2 + HALF[1]**2])
M_DRONE     = 0.5                    # kg (2D fix: 27 g cannot pull 2.4 N)
T_MAX       = 15.0                   # N  per drone (2D value)
L0          = 0.5                    # m  cable rest length
MU          = 0.005                  # kg/m cable linear density
K_CABLE     = 1000.0                 # N/m (2D value)
C_CABLE     = 5.0                    # N s/m (2D value)
H_C         = 0.40                   # m  nominal vertical cable projection
ATT_R       = HALF[0]                # attachment radius = a = 0.15 m
S_FORM      = ATT_R + np.sqrt(L0**2 - H_C**2)   # 0.45 m (card: 0.6)
F_MIN       = 0.05                   # N
K_ETA       = 0.3                    # m/rad swing-damping gain
KP, KD      = 4.0, 2.0
DT          = 0.01                   # control period
N_SUB       = 10                     # physics sub-steps (1 kHz)
DT_PHYS     = DT / N_SUB
T_TOTAL     = 25.0
W_CABLE_END = 0.5 * MU * G * L0      # half a cable's weight, N

# Obstacles (poles on the ground)
R_OBS       = 0.4
D_CLEAR     = 0.5
H_OBS       = 4.6                    # m pole height (card: unspecified)
E_LAT       = 0.8                    # m lateral offset of each pole from path
DZ_STEP     = 0.8                    # m card's climb step
DZ_TUCK     = 0.08                   # m feasible climb along the cable sphere
DZ_RATE     = 0.3                    # m/s climb / return rate
R_DRONE     = 0.10                   # m drone body radius (collision check)
E_SWEEP     = [0.7, 0.8, 0.9, 1.0]
DZ_RATE_SLOW = 0.05                  # m/s slower tuck rate (cruise check)
T_HOVER_TUCK = 5.0                   # s  hover mission: drone 3 tucks here

# Mission
Q0      = np.array([0.0, 0.0, 0.1])
Q_UP    = np.array([0.0, 0.0, 4.0])
Q_GOAL  = np.array([5.0, 3.0, 4.0])
Q_DOWN  = np.array([5.0, 3.0, 0.1])
WAYPOINTS = [(0.0, 5.0, Q0, Q_UP), (5.0, 20.0, Q_UP, Q_GOAL),
             (20.0, 25.0, Q_GOAL, Q_DOWN)]
U_PATH  = (Q_GOAL - Q_UP)[:2] / np.linalg.norm((Q_GOAL - Q_UP)[:2])
N_PATH  = np.array([-U_PATH[1], U_PATH[0]])            # left normal
L_PATH  = np.linalg.norm((Q_GOAL - Q_UP)[:2])
ZHAT    = np.array([0.0, 0.0, 1.0])

# Body-frame attachment points (top-face edge midpoints) and arm directions
DIRS = np.array([[1.0, 0, 0], [-1.0, 0, 0], [0, 1.0, 0], [0, -1.0, 0]])
PARTNER = np.array([1, 0, 3, 2])             # opposite drone of each arm
B_ATT = DIRS * ATT_R + np.array([0, 0, HALF[2]])
DRONE_COLORS = ['red', 'orange', 'magenta', 'gold']
DRONE_LABELS = [f'Drone {i+1}' for i in range(N_DRONES)]

# Box corners (body frame) for clearance / ground checks
CORNERS = np.array([[sx * HALF[0], sy * HALF[1], sz * HALF[2]]
                    for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
CABLE_S = np.linspace(0.0, 1.0, 11)

# Runs: (key, label, avoid, alloc, damp_sign, cable_model, extra kwargs)
RUNS = [
    ('2d',      '2D controller, flat',       'flat',  'force',  0, 'straight', {}),
    ('flat',    '3D controller, flat',       'flat',  'wrench', 1, 'catenary', {}),
    ('climb',   'Card rule: climb 0.8 m',    'climb', 'wrench', 1, 'catenary', {}),
    ('tuck1',   'Single-drone tuck',         'tuck1', 'wrench', 1, 'catenary', {}),
    ('pair',    'Pair tuck (main)',          'pair',  'wrench', 1, 'catenary', {}),
    ('tuck1_f', 'Single tuck, force-only',   'tuck1', 'force',  1, 'catenary', {}),
    ('tuck1_s', 'Single tuck, 0.05 m/s',     'tuck1', 'wrench', 1, 'catenary',
     {'dz_rate': DZ_RATE_SLOW}),
    ('pair_f',  'Pair tuck, force-only',     'pair',  'force',  1, 'catenary', {}),
    ('pair_nd', 'Pair tuck, no damping',     'pair',  'wrench', 0, 'catenary', {}),
    ('pair_rd', 'Pair tuck, reversed damp.', 'pair',  'wrench', -1, 'catenary', {}),
    ('pair_st', 'Pair tuck, straight cable', 'pair',  'wrench', 1, 'straight', {}),
]
# Hover runs: the load hovers at Q_UP for the whole 25 s; drone 3 tucks at
# T_HOVER_TUCK and stays tucked (no poles, no cruise acceleration).
HOVER_RUNS = [
    ('hov1',   'Hover single tuck',             'tuck1', 'wrench', 1, 'catenary',
     {'mission': 'hover'}),
    ('hov1_f', 'Hover single tuck, force-only', 'tuck1', 'force',  1, 'catenary',
     {'mission': 'hover'}),
]


def obstacles(e_lat):
    """Two poles: left of the path at 35 %, right of it at 70 %."""
    o1 = Q_UP[:2] + 0.35 * L_PATH * U_PATH + e_lat * N_PATH
    o2 = Q_UP[:2] + 0.70 * L_PATH * U_PATH - e_lat * N_PATH
    return np.array([o1, o2])


def load_ref(t):
    for (t0, t1, q0, q1) in WAYPOINTS:
        if t <= t1:
            tau = np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
            dur = t1 - t0
            s = 10*tau**3 - 15*tau**4 + 6*tau**5
            ds = (30*tau**2 - 60*tau**3 + 30*tau**4) / dur
            dds = (60*tau - 180*tau**2 + 120*tau**3) / dur**2
            dq = q1 - q0
            return q0 + s*dq, ds*dq, dds*dq
    return Q_DOWN.copy(), np.zeros(3), np.zeros(3)


# ── Rotation helpers ──────────────────────────────────────────────────────────

def quat_to_R(qa):
    w, x, y, z = qa
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])


def quat_step(qa, omega_b, dt):
    """q <- q (x) exp(omega dt / 2), body-frame omega."""
    wn = np.linalg.norm(omega_b)
    if wn < 1e-12:
        return qa
    half = 0.5 * wn * dt
    dw = np.cos(half)
    dv = np.sin(half) * omega_b / wn
    w, x, y, z = qa
    out = np.array([w*dw - x*dv[0] - y*dv[1] - z*dv[2],
                    w*dv[0] + x*dw + y*dv[2] - z*dv[1],
                    w*dv[1] - x*dv[2] + y*dw + z*dv[0],
                    w*dv[2] + x*dv[1] - y*dv[0] + z*dw])
    return out / np.linalg.norm(out)


def tilt_deg(R):
    return np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))


# ── Cable model ───────────────────────────────────────────────────────────────

def cable_state(p, v, q, vq, R, omega_w):
    """Chord unit vectors, lengths, spring tensions, attachment points."""
    rb = B_ATT @ R.T                               # world lever arms (4,3)
    a = q + rb
    va = vq + np.cross(omega_w, rb)
    d = p - a
    L = np.linalg.norm(d, axis=1)
    c = d / L[:, None]
    Ldot = np.sum((v - va) * c, axis=1)
    T = np.where(L > L0, K_CABLE * (L - L0) + C_CABLE * Ldot, 0.0)
    return c, L, np.maximum(T, 0.0), a, rb


def sag_perp(T, L, c):
    """Mid-span sag perpendicular to the chord (parabolic): mu g cos(b) L^2/(8T)."""
    cos_b = np.sqrt(np.maximum(0.0, 1.0 - c[:, 2]**2))   # chord vs horizontal
    return MU * G * cos_b * L**2 / (8.0 * np.maximum(T, 1e-9))


# ── Allocation ────────────────────────────────────────────────────────────────

def allocate(c_alloc, rb, W, mode):
    """Bounded least squares, f >= F_MIN.
    force : min ||A3 f - W||^2 + eps ||f||^2            (2D method)
    wrench: min ||A3 f - W||^2 + ||A_M f / r||^2 + eps ||f||^2
            (card's 6-row matrix; moments about the centroid, scaled by the
             attachment radius so both blocks are in newtons)"""
    eps = 1e-4
    A3 = c_alloc.T
    rows = [A3]
    rhs = [W]
    if mode == 'wrench':
        AM = np.cross(rb, c_alloc).T / ATT_R
        rows.append(AM)
        rhs.append(np.zeros(3))
    rows.append(np.sqrt(eps) * np.eye(N_DRONES))
    rhs.append(np.zeros(N_DRONES))
    A = np.vstack(rows)
    b = np.concatenate(rhs)
    return lsq_linear(A, b, bounds=(F_MIN, np.inf), method='bvls').x


# ── Clearance to the poles ────────────────────────────────────────────────────

def clearance(points, obs):
    """Signed distance of points (n,3) to the solid poles (min over poles);
    negative = inside."""
    out = np.full(len(points), np.inf)
    for o in obs:
        rho = np.linalg.norm(points[:, :2] - o, axis=1)
        dr = rho - R_OBS
        dz = points[:, 2] - H_OBS
        outside = np.sqrt(np.maximum(dr, 0)**2 + np.maximum(dz, 0)**2)
        inside = np.maximum(dr, dz)            # both < 0 when inside
        dist = np.where((dr < 0) & (dz < 0), inside, outside)
        out = np.minimum(out, dist)
    return out


# ── Simulation ────────────────────────────────────────────────────────────────

def hover_ref(t):
    return Q_UP.copy(), np.zeros(3), np.zeros(3)


def run_simulation(avoid='pair', alloc='wrench', damp=1, cable='catenary',
                   e_lat=E_LAT, label='', mission='route', dz_rate=DZ_RATE):
    """Simulate one mission; returns a dict of time histories.
    mission='route': lift, cruise past the poles, descend (the main mission).
    mission='hover': hold Q_UP for T_TOTAL; drone 3's tuck is triggered from
    T_HOVER_TUCK on (no poles)."""
    hover = mission == 'hover'
    ref = hover_ref if hover else load_ref
    obs = np.zeros((0, 2)) if hover else obstacles(e_lat)
    steps = int(round(T_TOTAL / DT)) + 1
    time = np.arange(steps) * DT
    H = {k: np.zeros((steps, N_DRONES)) for k in
         ('f', 'T', 'L', 'thrust', 'dz', 'sag', 'clr_drone', 'clr_cable')}
    H.update(p=np.zeros((steps, N_DRONES, 3)), q=np.zeros((steps, 3)),
             q_ref=np.zeros((steps, 3)), R=np.zeros((steps, 3, 3)),
             omega=np.zeros((steps, 3)), clr_load=np.zeros(steps),
             trig=np.zeros((steps, N_DRONES), bool),
             corner_zmin=np.zeros(steps))
    n_sat = 0

    # Initial state: load suspended at Q0, level, cables at static tension
    q = (Q_UP if hover else Q0).copy(); vq = np.zeros(3)
    qa = np.array([1.0, 0, 0, 0]); omega_b = np.zeros(3)
    R = np.eye(3)
    f0 = (M_LOAD * G + N_DRONES * W_CABLE_END) / (N_DRONES * H_C / L0)
    chord0 = np.column_stack([DIRS[:, :2] * (S_FORM - ATT_R),
                              np.full(N_DRONES, H_C)])
    p = q + B_ATT + chord0 * (1.0 + f0 / (K_CABLE * L0))
    v = np.zeros((N_DRONES, 3))
    f_prev = np.full(N_DRONES, f0)
    dz = np.zeros(N_DRONES)
    dz_tgt_val = {'flat': 0.0, 'climb': DZ_STEP, 'tuck1': DZ_TUCK,
                  'pair': DZ_TUCK}[avoid]
    fail_t, fail_why = None, ''

    for k in range(steps):
        t = time[k]
        q_r, v_r, a_r = ref(t)
        omega_w = R @ omega_b
        c, L, T, a_pts, rb = cable_state(p, v, q, vq, R, omega_w)

        # ── Avoidance trigger on the nominal (flat) slot positions ──
        slot_xy = q_r[:2] + DIRS[:, :2] * S_FORM
        trig = np.zeros(N_DRONES, bool)
        for o in obs:
            trig |= np.linalg.norm(slot_xy - o, axis=1) < R_OBS + D_CLEAR
        if hover:
            trig = np.array([False, False, t >= T_HOVER_TUCK, False])
        if avoid == 'flat':
            trig[:] = False
        elif avoid == 'pair':                  # opposite partner tucks too
            trig = trig | trig[PARTNER]
        target = np.where(trig, dz_tgt_val, 0.0)
        dz_new = dz + np.clip(target - dz, -dz_rate * DT, dz_rate * DT)
        dz_dot = (dz_new - dz) / DT
        dz = dz_new

        # Slot offsets (centroid -> drone) and their rates
        h_i = H_C + dz
        if avoid in ('tuck1', 'pair'):
            horiz = np.sqrt(L0**2 - h_i**2)
            r_i = ATT_R + horiz
            r_dot = -h_i / horiz * dz_dot
        else:
            r_i = np.full(N_DRONES, S_FORM)
            r_dot = np.zeros(N_DRONES)
        d_off = np.column_stack([DIRS[:, :2] * r_i[:, None], HALF[2] + h_i])
        d_dot = np.column_stack([DIRS[:, :2] * r_dot[:, None], dz_dot])

        # ── Swing damping: altitude offset from attachment tilt drop ──
        drop = rb[:, 2] - B_ATT[:, 2]                 # world z change, m
        drop_rate = np.cross(omega_w, rb)[:, 2]
        z_damp = -damp * K_ETA * drop / ATT_R
        vz_damp = -damp * K_ETA * drop_rate / ATT_R

        # ── Allocation ──
        if cable == 'catenary':
            cb = c - (MU * G * L0 / (2.0 * f_prev))[:, None] * ZHAT
            c_alloc = cb / np.linalg.norm(cb, axis=1)[:, None]
        else:
            c_alloc = c
        W = M_LOAD * (G * ZHAT + a_r)
        f = allocate(c_alloc, rb, W, alloc)
        f_prev = f

        # ── Drone PD + tension feed-forward ──
        p_des = q_r + d_off + z_damp[:, None] * ZHAT
        v_des = v_r + d_dot + vz_damp[:, None] * ZHAT
        a_cmd = a_r + KP * (p_des - p) + KD * (v_des - v)
        ff = f[:, None] * c
        if cable == 'catenary':
            ff = ff + W_CABLE_END * ZHAT
        u = M_DRONE * (a_cmd + G * ZHAT) + ff
        un = np.linalg.norm(u, axis=1)
        sat = un > T_MAX
        n_sat += int(sat.sum())
        u[sat] *= (T_MAX / un[sat])[:, None]

        # ── Record ──
        corners_w = q + CORNERS @ R.T
        cab_pts = (a_pts[:, None, :] + CABLE_S[None, :, None]
                   * (p - a_pts)[:, None, :])
        H['q'][k] = q; H['q_ref'][k] = q_r; H['p'][k] = p; H['R'][k] = R
        H['omega'][k] = omega_w; H['f'][k] = f; H['T'][k] = T; H['L'][k] = L
        H['thrust'][k] = un; H['dz'][k] = dz      # demanded (pre-clip) thrust
        H['trig'][k] = trig
        H['sag'][k] = sag_perp(T, L, c)
        H['clr_drone'][k] = clearance(p, obs)
        H['clr_cable'][k] = clearance(cab_pts.reshape(-1, 3),
                                      obs).reshape(N_DRONES, -1).min(axis=1)
        H['clr_load'][k] = clearance(corners_w, obs).min()
        H['corner_zmin'][k] = corners_w[:, 2].min()

        # Physical invariants (simulation must never violate these)
        assert np.all(np.isfinite(p)) and np.all(np.isfinite(q))
        assert np.all(T >= 0.0) and np.all(f >= F_MIN - 1e-9)
        assert abs(np.linalg.norm(qa) - 1.0) < 1e-9
        # Mission failures: stop the run (no ground contact is modelled)
        if corners_w[:, 2].min() <= 0.0:
            fail_t, fail_why = t, 'load hit the ground'
        elif np.any(p[:, 2] <= a_pts[:, 2]):
            fail_t, fail_why = t, 'drone fell below its attachment point'
        if fail_t is not None or k == steps - 1:
            break

        # ── Physics sub-steps ──
        for _ in range(N_SUB):
            omega_w = R @ omega_b
            c, L, T, a_pts, rb = cable_state(p, v, q, vq, R, omega_w)
            F_cab = T[:, None] * c
            F_on_load = F_cab - W_CABLE_END * ZHAT
            F_on_drone = -F_cab - W_CABLE_END * ZHAT
            acc_d = (u + F_on_drone) / M_DRONE - G * ZHAT
            F_load = F_on_load.sum(axis=0) - M_LOAD * G * ZHAT
            tau_w = np.cross(rb, F_on_load).sum(axis=0)
            tau_b = R.T @ tau_w
            omega_dot = (tau_b - np.cross(omega_b, I_LOAD * omega_b)) / I_LOAD
            v = v + acc_d * DT_PHYS
            p = p + v * DT_PHYS
            vq = vq + F_load / M_LOAD * DT_PHYS
            q = q + vq * DT_PHYS
            omega_b = omega_b + omega_dot * DT_PHYS
            qa = quat_step(qa, omega_b, DT_PHYS)
            R = quat_to_R(qa)

    # Keep only valid samples: a failing sample (box touching the ground or
    # an inverted drone) is dropped, so nothing underground is reported.
    n = k if fail_t is not None else k + 1
    for key in list(H):
        H[key] = H[key][:n]
    time = time[:n]
    tilt = np.degrees(np.arccos(np.clip(H['R'][:, 2, 2], -1, 1)))
    H.update(fail_t=fail_t, fail_why=fail_why, time=time, tilt=tilt,
             n_sat=n_sat, obs=obs, label=label, avoid=avoid, alloc=alloc,
             damp=damp, cable=cable, e_lat=e_lat, mission=mission,
             pos_err=np.linalg.norm(H['q'] - H['q_ref'], axis=1))
    return H


def summarize(d):
    tm = d['time']
    cruise = (tm >= 5.0) & (tm <= 20.0)
    gap = d['p'][:, :, 2] - (d['q'][:, None, 2]
                             + np.einsum('kij,nj->kni', d['R'], B_ATT)[:, :, 2])
    trig_any = d['trig'].any(axis=1)
    return {
        'tilt_max': d['tilt'].max(),
        't_tilt_max': tm[np.argmax(d['tilt'])],
        'tilt_rms_cruise': np.sqrt(np.mean(d['tilt'][cruise]**2)),
        'tilt_over5': int(np.sum(d['tilt'] > 5.0)),
        'omega_rms': np.degrees(np.sqrt(np.mean(np.sum(d['omega'][cruise]**2,
                                                       axis=1)))),
        'T_min': d['T'].min(), 'T_max': d['T'].max(),
        'n_slack': int(np.sum(d['T'].min(axis=1) <= 0.0)),
        'n_below_fmin': int(np.sum(d['T'].min(axis=1) < F_MIN)),
        'f_min': d['f'].min(), 'f_max': d['f'].max(),
        'thrust_max': d['thrust'].max(), 'n_sat': d['n_sat'],
        'err_rms': np.sqrt(np.mean(d['pos_err']**2)),
        'err_max': d['pos_err'].max(),
        'clr_drone': d['clr_drone'].min(),
        'clr_cable': d['clr_cable'].min(), 'clr_load': d['clr_load'].min(),
        'dz_cmd_max': d['dz'].max(),
        'gap_max': gap.max(), 'gap_min': gap.min(),
        'rise_max': gap.max() - H_C,
        'trig_time': trig_any.sum() * DT,
        'sag_max_mm': 1e3 * d['sag'][d['T'] > 0.5].max(),
        'sag_med_mm': 1e3 * np.median(d['sag'][d['T'] > 0.5]),
        'corner_zmin': d['corner_zmin'].min(),
        'q_final': d['q'][-1],
    }


def abreast_time(frac):
    """Time at which the load reference passes `frac` of the cruise leg."""
    ts = np.arange(5.0, 20.0 + 1e-9, DT)
    proj = np.array([(load_ref(t)[0][:2] - Q_UP[:2]) @ U_PATH for t in ts])
    return ts[np.argmin(np.abs(proj - frac * L_PATH))]


T_A, T_B = abreast_time(0.35), abreast_time(0.70)
K_HOVER = int(round(5.0 / DT))


def _windows(d):
    """Contiguous intervals in which any drone's avoidance is triggered."""
    trig = d['trig'].any(axis=1)
    out, start = [], None
    for k, on in enumerate(trig):
        if on and start is None:
            start = d['time'][k]
        if not on and start is not None:
            out.append((start, d['time'][k]))
            start = None
    if start is not None:
        out.append((start, d['time'][-1]))
    return out


def _shade(ax, d, label=True):
    for j, (a, b) in enumerate(_windows(d)):
        ax.axvspan(a, b, color='grey', alpha=0.15,
                   label='avoidance triggered' if (label and j == 0) else None)


def _save(fig, name):
    fp = os.path.join(OUTPUT_DIR, name)
    fig.savefig(fp, dpi=110)
    plt.close(fig)
    print(f'  Saved: {fp}')


def _cylinder(ax, o, color='grey', alpha=0.35):
    th = np.linspace(0, 2 * np.pi, 30)
    TH, ZZ = np.meshgrid(th, np.array([0.0, H_OBS]))
    ax.plot_surface(o[0] + R_OBS * np.cos(TH), o[1] + R_OBS * np.sin(TH), ZZ,
                    color=color, alpha=alpha, linewidth=0)


def _box_faces(q, R):
    cw = q + CORNERS @ R.T
    idx = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6],
           [0, 2, 6, 4], [1, 3, 7, 5]]
    return [cw[i] for i in idx]


def _top_face(q, R):
    top = (q + CORNERS @ R.T)[CORNERS[:, 2] > 0]
    order = np.argsort(np.arctan2(top[:, 1] - q[1], top[:, 0] - q[0]))
    return top[order, :2]


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_trajectory_3d(d):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    for o in d['obs']:
        _cylinder(ax, o)
    ax.plot(*d['q_ref'].T, 'k:', lw=1.0, label='Load reference')
    ax.plot(*d['q'].T, color='blue', lw=2.0, label='Load centroid')
    for i in range(N_DRONES):
        ax.plot(*d['p'][:, i].T, color=DRONE_COLORS[i], lw=1.0,
                label=DRONE_LABELS[i])
    for tk in (5.0, T_A, T_B, 20.0):
        k = int(round(tk / DT))
        R, q, p = d['R'][k], d['q'][k], d['p'][k]
        ax.add_collection3d(Poly3DCollection(_box_faces(q, R), color='blue',
                                             alpha=0.5))
        a = q + B_ATT @ R.T
        for i in range(N_DRONES):
            ax.plot([a[i, 0], p[i, 0]], [a[i, 1], p[i, 1]],
                    [a[i, 2], p[i, 2]], color='k', lw=1.0)
    ax.set_xlim(-1, 6); ax.set_ylim(-1.5, 4.5); ax.set_zlim(0, 5.5)
    ax.set_box_aspect((7, 6, 5.5))
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title(f'S026 3D Cooperative Heavy Lift — {d["label"]}\n'
                 f'grey: poles r = {R_OBS} m, h = {H_OBS} m; cables + box '
                 f'drawn at t = 5.00, {T_A:.2f}, {T_B:.2f}, 20.00 s')
    ax.view_init(elev=24, azim=-60)
    ax.legend(loc='upper left', fontsize=8)
    _save(fig, 'trajectory_3d.png')


def plot_load_tilt(runs, hov):
    fig, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
    allr = dict(runs, **hov)
    groups = [(['2d', 'flat', 'pair', 'pair_nd', 'pair_f', 'pair_st'],
               'Route runs that stay below 5° (grey: pair-tuck trigger '
               'windows)', 'pair'),
              (['climb', 'tuck1', 'tuck1_f', 'tuck1_s', 'pair_rd'],
               'Route runs that exceed 5° (grey: single-tuck trigger windows; '
               'log scale)', 'tuck1'),
              (['hov1_f', 'hov1'],
               f'Hover: drone 3 tucks at t = {T_HOVER_TUCK:.0f} s and holds '
               '(grey: tuck triggered; log scale)', 'hov1')]
    cols = ['grey', 'black', 'tab:green', 'tab:orange', 'tab:purple',
            'tab:cyan']
    for ax, (keys, title, shade_key) in zip(axes, groups):
        _shade(ax, allr[shade_key])
        for key, col in zip(keys, cols):
            d = allr[key]
            ls = '--' if key in ('pair_f', 'pair_st', 'tuck1_f',
                                 'hov1_f') else '-'
            extra = (f' (stopped t = {d["fail_t"]:.2f} s)'
                     if d['fail_t'] is not None else '')
            ax.plot(d['time'], d['tilt'], color=col, lw=1.2, ls=ls,
                    label=d['label'] + extra)
        ax.axhline(5.0, color='red', ls='--', lw=1, label='5° limit')
        ax.set_ylabel('Load tilt Θ_L (deg)')
        ax.set_title(title, fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, ncol=2)
    axes[0].set_ylim(0, 6)
    for ax in axes[1:]:
        ax.set_yscale('log')
        ax.set_ylim(0.01, 100)
    axes[2].set_xlabel('Time (s)')
    fig.suptitle('S026 3D — load tilt Θ_L(t)')
    plt.tight_layout()
    _save(fig, 'load_tilt.png')


def plot_cable_tensions(runs):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    for ax, key in zip(axes, ['pair', 'tuck1', 'climb']):
        d = runs[key]
        _shade(ax, d)
        for i in range(N_DRONES):
            ax.plot(d['time'], d['T'][:, i], color=DRONE_COLORS[i], lw=1.1,
                    label=f'{DRONE_LABELS[i]} actual T')
            ax.plot(d['time'], d['f'][:, i], color=DRONE_COLORS[i], lw=0.8,
                    ls=':')
        ax.axhline(F_MIN, color='k', ls='--', lw=1, label=f'f_min = {F_MIN} N')
        ax.set_ylabel('Tension (N)')
        ax.grid(True, alpha=0.3)
        ax.set_title(f'{d["label"]} — solid: actual spring tension, '
                     f'dotted: allocated f_i', fontsize=10)
        ax.set_ylim(-0.3, 8)
    axes[0].legend(fontsize=8, ncol=3)
    axes[2].set_xlabel('Time (s)')
    plt.tight_layout()
    _save(fig, 'cable_tensions.png')


def plot_altitude_profiles(runs):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    d = runs['pair']
    ax = axes[0]
    for i in range(N_DRONES):
        ax.plot(d['time'], d['p'][:, i, 2], color=DRONE_COLORS[i], lw=1.1,
                label=DRONE_LABELS[i])
    ax.plot(d['time'], d['q'][:, 2], color='blue', lw=1.5, label='Load')
    ax.set_ylabel('z (m)')
    ax.set_title('Pair tuck — absolute altitudes', fontsize=10)
    ax.legend(fontsize=8, ncol=5)
    ax.grid(True, alpha=0.3)
    for ax, key in zip(axes[1:], ['pair', 'climb']):
        d = runs[key]
        a_z = d['q'][:, None, 2] + np.einsum('kij,nj->kni', d['R'],
                                             B_ATT)[:, :, 2]
        _shade(ax, d)
        for i in range(N_DRONES):
            ax.plot(d['time'], d['p'][:, i, 2] - a_z[:, i],
                    color=DRONE_COLORS[i], lw=1.2, label=DRONE_LABELS[i])
            ax.plot(d['time'], H_C + d['dz'][:, i], color=DRONE_COLORS[i],
                    lw=0.8, ls=':')
        ax.axhline(L0, color='k', ls='--', lw=1,
                   label=f'cable length l0 = {L0} m (max possible)')
        ax.set_ylabel('drone z − attachment z (m)')
        ax.set_title(f'{d["label"]} — height of each drone above its '
                     f'attachment; solid: actual, dotted: commanded',
                     fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, ncol=3)
    axes[1].set_ylim(0.3, 0.55)
    axes[2].set_ylim(0.0, 1.3)
    for ax in axes:
        ax.set_xlim(0, T_TOTAL)
    axes[2].set_xlabel('Time (s)')
    plt.tight_layout()
    _save(fig, 'altitude_profiles.png')


def plot_cable_sag(d, exag=100.0):
    k_t = int(round(T_A / DT))
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))
    for ax, k, title in [(axes[0], K_HOVER, 'nominal hover, t = 5.00 s'),
                         (axes[1], k_t, f'pair tuck at pole 1, '
                                        f't = {T_A:.2f} s')]:
        q, R, p, T = d['q'][k], d['R'][k], d['p'][k], d['T'][k]
        a = q + B_ATT @ R.T
        for i in range(N_DRONES):
            dvec = p[i] - a[i]
            hdir = dvec[:2] / np.linalg.norm(dvec[:2])
            x0 = (a[i, :2] - q[:2]) @ hdir      # radial position of attach
            xs = np.array([x0, x0 + dvec[:2] @ hdir])
            zs = np.array([a[i, 2], p[i, 2]]) - q[2]
            Lc = np.linalg.norm(dvec)
            cb = np.sqrt(max(0.0, 1 - (dvec[2] / Lc)**2))
            sag = MU * G * cb * Lc**2 / (8 * T[i])
            s_ = np.linspace(0, 1, 41)
            chord = np.column_stack([xs[0] + s_ * (xs[1] - xs[0]),
                                     zs[0] + s_ * (zs[1] - zs[0])])
            nrm = np.array([zs[1] - zs[0], -(xs[1] - xs[0])])
            nrm = nrm / np.linalg.norm(nrm)
            if nrm[1] > 0:
                nrm = -nrm                        # sag points downward
            arc = chord + (exag * 4 * sag * s_ * (1 - s_))[:, None] * nrm
            off = 0.0 if i < 2 else 0.9          # x-pair left, y-pair right
            ax.plot(chord[:, 0] + off, chord[:, 1], color=DRONE_COLORS[i],
                    ls='--', lw=1)
            ax.plot(arc[:, 0] + off, arc[:, 1], color=DRONE_COLORS[i], lw=2,
                    label=f'{DRONE_LABELS[i]}: T = {T[i]:.2f} N, '
                          f'sag = {sag*1e3:.2f} mm, end angle '
                          f'{np.degrees(MU*G*L0*cb/(2*T[i])):.2f}°')
            ax.scatter([xs[1] + off], [zs[1]], color=DRONE_COLORS[i], s=40)
        for off in (0.0, 0.9):
            ax.plot([off - ATT_R, off + ATT_R], [HALF[2], HALF[2]], 'b-',
                    lw=4)
        ax.set_title(title + f'\n(dashed: chord, solid: parabolic arc, sag '
                     f'exaggerated ×{exag:.0f})', fontsize=10)
        ax.set_xlabel('radial position in each cable plane (m); left: '
                      'drones 1-2, right: drones 3-4 (shifted +0.9 m)')
        ax.legend(fontsize=7, loc='lower center')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.set_xlim(-0.2, 1.45)
        ax.set_ylim(-0.3, 0.6)
        ax.set_ylabel('height above load centroid (m)')
    plt.tight_layout()
    _save(fig, 'cable_sag.png')


def plot_formation_topdown(runs):
    keys = ['flat', 'tuck1', 'pair']
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    for row, tk in enumerate((T_A, T_B)):
        k = int(round(tk / DT))
        for col, key in enumerate(keys):
            ax = axes[row, col]
            d = runs[key]
            q, R, p = d['q'][k], d['R'][k], d['p'][k]
            for o in d['obs']:
                ax.add_patch(plt.Circle(o, R_OBS, color='grey', alpha=0.5))
                ax.add_patch(plt.Circle(o, R_OBS + D_CLEAR, color='grey',
                                        fill=False, ls=':'))
            top = _top_face(q, R)
            ax.fill(top[:, 0], top[:, 1], color='blue', alpha=0.6)
            a = q + B_ATT @ R.T
            for i in range(N_DRONES):
                ax.plot([a[i, 0], p[i, 0]], [a[i, 1], p[i, 1]],
                        color=DRONE_COLORS[i], lw=1.5)
                ax.add_patch(plt.Circle(p[i, :2], R_DRONE,
                                        color=DRONE_COLORS[i], alpha=0.7))
            clr = d['clr_drone'][k].min()
            ax.set_title(f'{d["label"]}, t = {tk:.2f} s\n'
                         f'min drone-centre clearance {clr:.3f} m, '
                         f'tilt {d["tilt"][k]:.2f}°', fontsize=10)
            ax.set_xlim(q[0] - 1.2, q[0] + 1.2)
            ax.set_ylim(q[1] - 1.2, q[1] + 1.2)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    fig.suptitle('S026 3D — top-down formation abreast of each pole '
                 '(grey disc: pole, dotted: r_obs + d_clear trigger ring, '
                 f'coloured discs: drones r = {R_DRONE} m)')
    plt.tight_layout()
    _save(fig, 'formation_topdown.png')


def plot_obstacle_sweep(sweep):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))
    styles = {'flat': ('black', 'o'), 'tuck1': ('tab:red', 's'),
              'pair': ('tab:green', '^')}
    for key, rows in sweep.items():
        col, mk = styles[key]
        es = [r['e'] for r in rows]
        for ax, met in zip(axes, ('clr_drone', 'clr_load', 'tilt_max')):
            ax.plot(es, [r[met] for r in rows], color=col, marker=mk,
                    label=rows[0]['label'])
    axes[0].axhline(R_DRONE, color='red', ls='--', lw=1,
                    label=f'drone body radius {R_DRONE} m')
    axes[0].set_ylabel('min drone-centre clearance (m)')
    axes[1].set_ylabel('min load-corner clearance (m)')
    axes[2].axhline(5.0, color='red', ls='--', lw=1, label='5° limit')
    axes[2].set_ylabel('max load tilt (deg)')
    axes[2].set_yscale('log')
    for ax in axes:
        ax.set_xlabel('pole lateral offset from the path e (m)')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle('S026 3D — sweep of the pole placement (same mission, '
                 '3D controller)')
    plt.tight_layout()
    _save(fig, 'obstacle_sweep.png')


# ── Animation ────────────────────────────────────────────────────────────────

def save_animation(d_pair, d_flat, step=25, fps=10):
    """3D overview of the pair-tuck run plus top-down zooms of the flat
    formation and the pair tuck."""
    n = len(d_pair['time'])
    n_frames = int(np.ceil((n - 1) / step)) + 1
    fig = plt.figure(figsize=(15, 5.5))
    ax3 = fig.add_subplot(1, 3, 1, projection='3d')
    for o in d_pair['obs']:
        _cylinder(ax3, o, alpha=0.25)
    ax3.plot(*d_pair['q_ref'].T, 'k:', lw=0.8)
    ax3.set_xlim(-1, 6); ax3.set_ylim(-1.5, 4.5); ax3.set_zlim(0, 5.5)
    ax3.set_xlabel('X'); ax3.set_ylabel('Y'); ax3.set_zlabel('Z')
    ax3.set_title('Pair tuck (3D overview)', fontsize=10)
    load3 = ax3.plot([], [], [], 's', color='blue', ms=6)[0]
    cab3 = [ax3.plot([], [], [], color=DRONE_COLORS[i], lw=1)[0]
            for i in range(N_DRONES)]
    panels = []
    for slot, d in ((2, d_flat), (3, d_pair)):
        ax = fig.add_subplot(1, 3, slot)
        for o in d['obs']:
            ax.add_patch(plt.Circle(o, R_OBS, color='grey', alpha=0.5))
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f'{d["label"]} (top-down zoom)', fontsize=10)
        poly = ax.fill([0, 0, 0], [0, 0, 0], color='blue', alpha=0.6)[0]
        cabs = [ax.plot([], [], color=DRONE_COLORS[i], lw=1.5)[0]
                for i in range(N_DRONES)]
        dots = [ax.plot([], [], 'o', color=DRONE_COLORS[i], ms=8)[0]
                for i in range(N_DRONES)]
        txt = ax.text(0.02, 0.97, '', transform=ax.transAxes, fontsize=8,
                      va='top')
        panels.append((ax, d, poly, cabs, dots, txt))

    def update(j):
        k = min(j * step, n - 1)
        q, R, p = d_pair['q'][k], d_pair['R'][k], d_pair['p'][k]
        a = q + B_ATT @ R.T
        load3.set_data([q[0]], [q[1]])
        load3.set_3d_properties([q[2]])
        for i in range(N_DRONES):
            cab3[i].set_data([a[i, 0], p[i, 0]], [a[i, 1], p[i, 1]])
            cab3[i].set_3d_properties([a[i, 2], p[i, 2]])
        for ax, d, poly, cabs, dots, txt in panels:
            kk = min(k, len(d['time']) - 1)
            q, R, p = d['q'][kk], d['R'][kk], d['p'][kk]
            poly.set_xy(_top_face(q, R))
            a = q + B_ATT @ R.T
            for i in range(N_DRONES):
                cabs[i].set_data([a[i, 0], p[i, 0]], [a[i, 1], p[i, 1]])
                dots[i].set_data([p[i, 0]], [p[i, 1]])
            ax.set_xlim(q[0] - 1.2, q[0] + 1.2)
            ax.set_ylim(q[1] - 1.2, q[1] + 1.2)
            txt.set_text(f't = {d["time"][kk]:.2f} s  z = {q[2]:.2f} m\n'
                         f'tilt {d["tilt"][kk]:.2f}°  min drone clearance '
                         f'{d["clr_drone"][kk].min():.2f} m')
        return []

    anim = FuncAnimation(fig, update, frames=n_frames, blit=False)
    fp = os.path.join(OUTPUT_DIR, 'animation.gif')
    anim.save(fp, writer=PillowWriter(fps=fps), dpi=80)
    plt.close(fig)
    print(f'  Saved: {fp}  ({n_frames} frames, {fps} fps)')


# ── Reporting ─────────────────────────────────────────────────────────────────

def print_setup():
    print('=' * 72)
    print('S026 3D COOPERATIVE HEAVY LIFT')
    print('=' * 72)
    print(f'  Load inertia (uniform cuboid) Ixx, Iyy, Izz = '
          f'{I_LOAD[0]:.5f}, {I_LOAD[1]:.5f}, {I_LOAD[2]:.5f} kg m^2')
    print(f'  Formation radius s = {S_FORM:.3f} m, nominal cable angle '
          f'{np.degrees(np.arccos(H_C / L0)):.2f} deg')
    f0 = (M_LOAD * G + N_DRONES * W_CABLE_END) / (N_DRONES * H_C / L0)
    print(f'  Static tension share (incl. half cable weight) = {f0:.4f} N')
    cb0 = np.sqrt(1 - (H_C / L0)**2)
    print(f'  Hover sag (parabolic, perpendicular to chord) = '
          f'{MU*G*cb0*L0**2/(8*f0)*1e3:.3f} mm; card formula '
          f'mu g d^2/(8f) = {MU*G*L0**2/(8*f0)*1e3:.3f} mm')
    print(f'  Bottom-end angle deviation mu g l0 cos(b)/(2f) = '
          f'{np.degrees(MU*G*L0*cb0/(2*f0)):.3f} deg')
    print(f'  Max climb above the slot with a level load and taut cable: '
          f'l0 - h = {L0 - H_C:.2f} m (card dz_step = {DZ_STEP} m)')
    h_t = H_C + DZ_TUCK
    r_t = ATT_R + np.sqrt(L0**2 - h_t**2)
    a_t = np.degrees(np.arcsin(np.sqrt(L0**2 - h_t**2) / L0))
    print(f'  Tuck: climb {DZ_TUCK} m -> radius {r_t:.3f} m '
          f'(from {S_FORM:.3f}), cable angle {a_t:.2f} deg')
    c_n, c_t = H_C / L0, h_t / L0
    f_n = M_LOAD * G / (2 * (c_n**2 + c_t**2) / c_n)
    print(f'  Pair-tuck static min-norm split (one pair tucked, load weight '
          f'only): {f_n:.3f} N (flat pair) / {f_n * c_t / c_n:.3f} N '
          f'(tucked pair)')
    # Static allocation with one asymmetric pair: level load, drone 3 tucked
    h_s = np.array([H_C, H_C, h_t, H_C])
    c_s = np.column_stack([DIRS[:, :2] * np.sqrt(L0**2 - h_s**2)[:, None],
                           h_s]) / L0
    W_s = M_LOAD * G * ZHAT
    print('  Static allocation, level load, drone 3 tucked (one asymmetric '
          'pair 3-4):')
    for mode in ('wrench', 'force'):
        f_s = allocate(c_s, B_ATT, W_s, mode)
        res_f = c_s.T @ f_s - W_s
        res_m = np.cross(B_ATT, c_s * f_s[:, None]).sum(axis=0)
        print(f'    {mode:6s}: f = ' + ', '.join(f'{x:.3f}' for x in f_s)
              + f' N; force residual |.| = {np.linalg.norm(res_f):.4f} N; '
              f'moment residual (x, y, z) = ({res_m[0]:.4f}, {res_m[1]:.4f}, '
              f'{res_m[2]:.4f}) N m')
    f12 = M_LOAD * G / (2 * c_s[0, 2])
    f_x = np.array([f12, f12, 0.0, 0.0])
    res_x = np.cross(B_ATT, c_s * f_x[:, None]).sum(axis=0)
    print(f'    exact solution f3 = f4 = 0: f1 = f2 = {f12:.3f} N; force '
          f'residual {np.linalg.norm(c_s.T @ f_x - W_s):.1e} N, moment '
          f'residual {np.linalg.norm(res_x):.1e} N m')
    print(f'  Poles: r = {R_OBS} m, h = {H_OBS} m, lateral offset '
          f'{E_LAT} m; load reference abreast at t = {T_A:.2f} s and '
          f'{T_B:.2f} s')


def print_run_table(runs):
    print(f'\n--- Runs (pole offset e = {E_LAT:.1f} m) ---')
    hdr = ('run', 'tilt max', 't@max', 'tilt rms', '|w| rms', 't>5deg',
           'T min', 'T max', 'slack', 'err max', 'clr drone', 'clr cable',
           'clr load', 'rise max', 'gap min', 'corner zmin', 'thr max',
           'sat', 'status')
    print(' | '.join(hdr))
    for key, d in runs.items():
        m = summarize(d)
        status = (f'stopped t={d["fail_t"]:.2f} s: {d["fail_why"]}'
                  if d['fail_t'] is not None else 'completed')
        print(f'{d["label"]} | {m["tilt_max"]:.2f} | {m["t_tilt_max"]:.2f} | '
              f'{m["tilt_rms_cruise"]:.3f} | {m["omega_rms"]:.2f} | '
              f'{m["tilt_over5"] * DT:.2f} | {m["T_min"]:.3f} | '
              f'{m["T_max"]:.2f} | {m["n_slack"]} | {m["err_max"]:.3f} | '
              f'{m["clr_drone"]:.3f} | {m["clr_cable"]:.3f} | '
              f'{m["clr_load"]:.3f} | {m["rise_max"]:.3f} | '
              f'{m["gap_min"]:.3f} | {m["corner_zmin"]:.3f} | '
              f'{m["thrust_max"]:.2f} | {m["n_sat"]} | {status}')
    print('  (tilt in deg, |w| rms in deg/s over the cruise, t>5deg in s, '
          'tensions in N, slack = samples with a cable at T = 0, '
          'clearances in m, rise = max drone height above its attachment '
          f'minus h = {H_C} m, gap min = min drone height above its '
          'attachment, corner zmin = lowest box corner, thr max = max '
          f'demanded thrust, sat = samples clipped at T_max = {T_MAX} N)')
    for key in ('2d', 'flat', 'pair', 'pair_st'):
        m = summarize(runs[key])
        print(f'  {runs[key]["label"]}: load error RMS {m["err_rms"]:.4f} m, '
              f'final load ({m["q_final"][0]:.3f}, {m["q_final"][1]:.3f}, '
              f'{m["q_final"][2]:.3f}) m, lowest box corner '
              f'{m["corner_zmin"]:.3f} m, sag median/max '
              f'{m["sag_med_mm"]:.3f}/{m["sag_max_mm"]:.3f} mm '
              f'(cables with T > 0.5 N)')
    d = runs['pair']
    tucked = d['dz'] > DZ_TUCK - 1e-9
    flat = d['dz'] < 1e-9
    one_pair = ((tucked[:, 0] & tucked[:, 1] & flat[:, 2] & flat[:, 3])
                | (flat[:, 0] & flat[:, 1] & tucked[:, 2] & tucked[:, 3]))
    idx = np.flatnonzero(one_pair)
    print(f'  Pair tuck: exactly one pair fully tucked for '
          f'{len(idx) * DT:.2f} s of the mission')
    if len(idx):
        k = idx[len(idx) // 2]
        print(f'    e.g. t = {d["time"][k]:.2f} s: dz = '
              + ', '.join(f'{x:.3f}' for x in d['dz'][k]) + ' m; actual T = '
              + ', '.join(f'{x:.3f}' for x in d['T'][k]) + ' N; allocated f = '
              + ', '.join(f'{x:.3f}' for x in d['f'][k]) + ' N')
    for tk in (5.0, T_A, T_B):
        k = int(round(tk / DT))
        print(f'  Pair tuck t = {tk:.2f} s: dz = '
              + ', '.join(f'{x:.3f}' for x in d['dz'][k])
              + ' m; actual T = '
              + ', '.join(f'{x:.3f}' for x in d['T'][k]) + ' N; sag = '
              + ', '.join(f'{1e3 * x:.2f}' for x in d['sag'][k]) + ' mm')


def asym_pairs(dz):
    """Number of opposite pairs (1-2, 3-4) whose two drones are at different
    tuck heights, per sample."""
    return ((np.abs(dz[:, 0] - dz[:, 1]) > 1e-9).astype(int)
            + (np.abs(dz[:, 2] - dz[:, 3]) > 1e-9).astype(int))


def print_single_tuck_analysis(runs, hov):
    print('\n--- Single-drone tuck: cruise vs hover ---')
    for key in ('tuck1', 'tuck1_f', 'tuck1_s'):
        d = runs[key]
        na = asym_pairs(d['dz'])
        k = int(np.argmax(d['tilt']))
        print(f'  {d["label"]}: one pair asymmetric {np.sum(na == 1) * DT:.2f} s, '
              f'both pairs asymmetric {np.sum(na == 2) * DT:.2f} s; max tilt '
              f'{d["tilt"][k]:.2f} deg at t = {d["time"][k]:.2f} s with dz = '
              + ', '.join(f'{x:.3f}' for x in d['dz'][k]) + ' m, T = '
              + ', '.join(f'{x:.2f}' for x in d['T'][k]) + ' N; max tilt '
              f'while one pair asymmetric {d["tilt"][na == 1].max():.2f} deg, '
              f'while both {d["tilt"][na == 2].max():.2f} deg')
    print(f'  Hover runs (load held at ({Q_UP[0]:.0f}, {Q_UP[1]:.0f}, '
          f'{Q_UP[2]:.0f}) m, drone 3 tucks {DZ_TUCK} m at {DZ_RATE} m/s from '
          f't = {T_HOVER_TUCK:.1f} s, held to t = {T_TOTAL:.0f} s):')
    for key, d in hov.items():
        m = summarize(d)
        late = d['time'] >= T_TOTAL - 5.0
        status = ('completed' if d['fail_t'] is None
                  else f'stopped t={d["fail_t"]:.2f} s')
        print(f'    {d["label"]}: tilt max {m["tilt_max"]:.2f} deg at t = '
              f'{m["t_tilt_max"]:.2f} s; last 5 s tilt '
              f'{d["tilt"][late].min():.2f}-{d["tilt"][late].max():.2f} deg '
              f'(mean {d["tilt"][late].mean():.2f}); T min {m["T_min"]:.3f} N, '
              f'slack {m["n_slack"]}, sat {m["n_sat"]}; final T = '
              + ', '.join(f'{x:.3f}' for x in d['T'][-1])
              + ' N, final f = ' + ', '.join(f'{x:.3f}' for x in d['f'][-1])
              + f' N; {status}')
        for tk in (T_HOVER_TUCK + 0.5, T_HOVER_TUCK + 1.0):
            k = int(round(tk / DT))
            print(f'      t = {d["time"][k]:.2f} s: tilt {d["tilt"][k]:.2f} '
                  'deg, T = ' + ', '.join(f'{x:.3f}' for x in d['T'][k])
                  + ' N, f = ' + ', '.join(f'{x:.3f}' for x in d['f'][k])
                  + ' N')


def print_sweep(sweep):
    print('\n--- Pole-offset sweep (3D controller) ---')
    print('e (m) | run | clr drone | clr cable | clr load | tilt max | status')
    for key, rows in sweep.items():
        for r in rows:
            print(f'{r["e"]:.1f} | {r["label"]} | {r["clr_drone"]:.3f} | '
                  f'{r["clr_cable"]:.3f} | {r["clr_load"]:.3f} | '
                  f'{r["tilt_max"]:.2f} | {r["status"]}')


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print_setup()
    runs = {}
    for key, label, avoid, alloc, damp, cable, kw in RUNS:
        runs[key] = run_simulation(avoid, alloc, damp, cable, label=label, **kw)
    print_run_table(runs)
    hov = {}
    for key, label, avoid, alloc, damp, cable, kw in HOVER_RUNS:
        hov[key] = run_simulation(avoid, alloc, damp, cable, label=label, **kw)
    print_single_tuck_analysis(runs, hov)

    sweep = {'flat': [], 'tuck1': [], 'pair': []}
    for e in E_SWEEP:
        for key in sweep:
            if e == E_LAT:
                d = runs[key]
            else:
                spec = next(r for r in RUNS if r[0] == key)
                d = run_simulation(*spec[2:6], e_lat=e, label=spec[1],
                                   **spec[6])
            m = summarize(d)
            m.update(e=e, label=d['label'],
                     status='completed' if d['fail_t'] is None
                     else f'stopped t={d["fail_t"]:.2f} s')
            sweep[key].append(m)
    print_sweep(sweep)

    print('\nGenerating plots...')
    plot_trajectory_3d(runs['pair'])
    plot_load_tilt(runs, hov)
    plot_cable_tensions(runs)
    plot_altitude_profiles(runs)
    plot_cable_sag(runs['pair'])
    plot_formation_topdown(runs)
    plot_obstacle_sweep(sweep)
    print('\nGenerating animation...')
    save_animation(runs['pair'], runs['flat'])


