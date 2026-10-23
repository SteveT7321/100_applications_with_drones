"""
S039 3D Upgrade — Offshore Platform Exchange
=============================================
The fixed 2-D S039 mission (4 drones, 5 platforms, a support vessel offering
battery swaps, mean wind + Dryden gust, three dispatch strategies) flown in
true 3-D:

  * drones cruise at 30 m, ramp down to the 3 m platform pads over the last
    100 m and climb out over the first 100 m of every leg; busy pads are
    circled at 13 m;
  * the vessel deck heaves, pitches and rolls (A_h = 0.8 m, 3 deg, 4 deg,
    T_w = 9 s); the drone descends an 8 deg glideslope referenced to the
    deck, converges with the card's 3-D moving-deck PD law inside 15 m, holds
    0.5 m above the pad and touches down only when |dr| < eps_p,
    |dv| < eps_v and the combined deck tilt < 5 deg; in the missions a deck
    strike or a touch-down at >= eps_v triggers a go-around (climb back to
    the hover, retry the gates), so only safe touch-downs dock;
  * a vertical Dryden gust (sigma 0.4 m/s, L 100 m) acts on every flying
    drone, and the power model adds k_vert v_z^2.

Experiments
  1. Deck-approach study (the 3-D question): the card law (deck height and
     heave fed forward) against a "2-D deck" law that assumes a level deck
     at its mean height, on the same 20 seeded approaches per sea state,
     with the card sea state scaled by 1 ... 3 (card extension 1).
  2. Mission: greedy / vessel-relay / rolling-horizon on the card and the
     extended instance, 5 gust seeds, with the 3-D planner; compared with the
     2-D script on the same horizontal gust seeds and with a 2-D planner
     flown in the 3-D world (ablation).

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s039_3d_offshore_platform.py
"""

import matplotlib
matplotlib.use('Agg')

import importlib.util
import math
import os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.lines import Line2D

_HERE = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '02_logistics_delivery', '3d',
    's039_3d_offshore_platform'))


def _load_2d():
    """Import the fixed 2-D script (planner, wind, vessel route, baseline)."""
    path = os.path.join(_HERE, '..', 's039_offshore_platform_exchange.py')
    spec = importlib.util.spec_from_file_location('s039_2d', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


S2 = _load_2d()

# ── 2-D base parameters (unchanged, taken from the 2-D script) ──────────────
N_DRONES, N_PLATFORMS = S2.N_DRONES, S2.N_PLATFORMS
V_AIRSPEED, V_VESSEL = S2.V_AIRSPEED, S2.V_VESSEL
P_CRUISE, K_WIND, W_MEAN = S2.P_CRUISE, S2.K_WIND, S2.W_MEAN
SIGMA_GUST, L_GUST = S2.SIGMA_GUST, S2.L_GUST
D_DOCK, R_HOLD, T_SWAP, T_HANDLE = S2.D_DOCK, S2.R_HOLD, S2.T_SWAP, S2.T_HANDLE
DT, T_MAX, DT_PLAN = S2.DT, S2.T_MAX, S2.DT_PLAN
WAIT_GRID, N_SEEDS, SHOW_SEED = S2.WAIT_GRID, S2.N_SEEDS, S2.SHOW_SEED
PLATFORMS, VESSEL_ROUTE = S2.PLATFORMS, S2.VESSEL_ROUTE
REQUESTS, REQUESTS_EXT, INSTANCES = S2.REQUESTS, S2.REQUESTS_EXT, S2.INSTANCES
DRONE_START, STRATEGIES, STRAT_LABEL = S2.DRONE_START, S2.STRATEGIES, S2.STRAT_LABEL
DRONE_COLORS, PLAT_COLORS = S2.DRONE_COLORS, S2.PLAT_COLORS
BATTERY_FULL, E_RES, P_PLAN, D_SAFETY = S2.BATTERY_FULL, S2.E_RES, S2.P_PLAN, S2.D_SAFETY
SHOW_INSTANCE = S2.SHOW_INSTANCE
power = S2.power


def groundspeed(u, wind):
    """S2 crab-angle groundspeed (same formula), written with math for speed."""
    psi_des = math.atan2(u[1], u[0])
    psi_w = math.atan2(wind[1], wind[0])
    w_mag = math.hypot(wind[0], wind[1])
    sin_d = min(1.0, max(-1.0, w_mag * math.sin(psi_des - psi_w) / V_AIRSPEED))
    vg = V_AIRSPEED * math.cos(math.asin(sin_d)) + w_mag * math.cos(psi_des - psi_w)
    return max(vg, 0.5)


def fly_cost(p, q):
    """(time, planned energy) of the 2-D mean-wind leg p -> q (S2 model)."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return 0.0, 0.0
    t = d / groundspeed((dx / d, dy / d), W_MEAN)
    return t, P_PLAN * t


def intercept(p, t0, n_iter=20):
    """S2 fixed-point vessel intercept, using the fast leg cost."""
    ts = t0
    for _ in range(n_iter):
        q = S2.vessel_position(ts)
        ts = t0 + fly_cost(p, q)[0]
    return ts, S2.vessel_position(ts)

# ── 3-D card parameters ─────────────────────────────────────────────────────
Z_CRUISE    = 30.0                    # m   cruise altitude
Z_PLATFORM  = 3.0                     # m   platform pad height
Z_REF       = 5.0                     # m   mean vessel deck height
A_H         = 0.8                     # m   heave amplitude
T_W         = 9.0                     # s   wave period
OMEGA_W     = 2 * np.pi / T_W
THETA_P     = np.radians(3.0)         # pitch amplitude
PHI_R       = np.radians(4.0)         # roll amplitude
PHI_MAX     = np.radians(5.0)         # touch-down tilt gate
GAMMA_GS    = np.radians(8.0)         # glideslope angle
TAN_GS      = float(np.tan(GAMMA_GS))
EPS_GS_MAX  = 0.03                    # glideslope tolerance (fraction of range)
D_RAMP      = 100.0                   # m   platform descent / climb-out ramp
Z_HOLD      = Z_PLATFORM + 10.0       # m   platform holding orbit altitude
SIGMA_GZ    = 0.4                     # m/s vertical gust intensity
L_GZ        = 100.0                   # m   vertical gust length scale
K_VERT      = 5.0                     # W s^2/m^2 vertical power coefficient
PAD_HALF    = 1.0                     # m   pad half-dimension (l_x = l_y)
EPS_P       = 1.0                     # m   touch-down position tolerance (2-D card)
EPS_V       = 0.5                     # m/s touch-down velocity tolerance (2-D card)
KP, KD      = 1.8, 1.2                # moving-deck PD gains (2-D card)

# ── Added parameters (not on the card) ─────────────────────────────────────
PH_H, PH_P, PH_R = 0.0, np.pi / 2, np.pi / 4   # wave phases of heave/pitch/roll
V_Z_MAX     = 5.0                     # m/s vertical speed limit
K_Z         = 1.5                     # 1/s altitude-tracking gain (outside the PD phase)
A_MAX       = 4.0                     # m/s^2 PD acceleration limit
H_HOVER     = 0.5                     # m   hover height above the pad before touch-down
V_TD        = 0.3                     # m/s commanded touch-down descent rate
V_DOCK      = 3.0                     # m/s closing speed relative to the vessel at d_dock
V_GA        = 2.0                     # m/s go-around climb rate relative to the deck (missions)
A_DEC       = 1.0                     # m/s^2 braking deceleration before d_dock
DECK_HALF   = (10.0, 5.0)             # m   vessel deck half-length / half-beam
D_GS        = (Z_CRUISE - Z_REF) / TAN_GS   # glideslope intercept range (177.9 m)
D_EPS_MIN   = H_HOVER / TAN_GS             # glideslope ends at the hover floor (3.56 m)
SEA_SCALES  = [1.0, 1.5, 2.0, 2.5, 3.0]    # sea-state multipliers (card extension 1)
N_TRIALS    = 20                      # seeded approaches per law and sea state
TRIAL_R0    = 400.0                   # m   approach start range (at cruise altitude)
TRIAL_TMAX  = 150.0                   # s   approach time-out
LAWS        = ['card', 'level']
LAW_LABEL   = {'card': 'Card law (deck height + heave fed forward)',
               'level': '2-D deck law (level deck at mean height)'}
GIF_STEP    = 40                      # simulation steps per GIF frame


# ── Vessel and sea state ────────────────────────────────────────────────────
_V_LEGS = np.diff(VESSEL_ROUTE, axis=0)
_V_LEN = np.linalg.norm(_V_LEGS, axis=1)
_V_CUM = np.concatenate([[0.0], np.cumsum(_V_LEN)])
_V_HEAD = np.arctan2(_V_LEGS[:, 1], _V_LEGS[:, 0])


def vessel_heading(t):
    d = (V_VESSEL * t) % _V_CUM[-1]
    s = min(int(np.searchsorted(_V_CUM, d, side='right')) - 1, len(VESSEL_ROUTE) - 2)
    return float(_V_HEAD[s])


def _rot(psi, th, ph):
    """Deck orientation R_ws = Rz(psi) Ry(theta) Rx(phi) (ship body -> world)."""
    cps, sps = math.cos(psi), math.sin(psi)
    cth, sth = math.cos(th), math.sin(th)
    cph, sph = math.cos(ph), math.sin(ph)
    rz = np.array([[cps, -sps, 0.0], [sps, cps, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cth, 0.0, sth], [0.0, 1.0, 0.0], [-sth, 0.0, cth]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cph, -sph], [0.0, sph, cph]])
    return rz @ ry @ rx


class SeaState:
    def __init__(self, scale=1.0):
        self.scale = scale
        self.a_h, self.th, self.ph = A_H * scale, THETA_P * scale, PHI_R * scale

    def motion(self, t):
        a = OMEGA_W * t
        return (self.a_h * math.sin(a + PH_H), self.th * math.sin(a + PH_P),
                self.ph * math.sin(a + PH_R))

    def tilt(self, t):
        _, th, ph = self.motion(t)
        return math.sqrt(th * th + ph * ph)

    def deck(self, t, law='card'):
        """Deck reference point as the guidance law sees it."""
        xy = S2.vessel_position(t)
        if law == 'level':
            return np.array([xy[0], xy[1], Z_REF])
        return np.array([xy[0], xy[1], Z_REF + self.motion(t)[0]])

    def frame(self, t):
        heave, th, ph = self.motion(t)
        return _rot(vessel_heading(t), th, ph)

    def surface_z(self, t, xy):
        """True deck-plane height at horizontal point xy, or None if off the deck."""
        p = self.deck(t)
        R = self.frame(t)
        n = R[:, 2]
        dx, dy = xy[0] - p[0], xy[1] - p[1]
        zs = p[2] - (n[0] * dx + n[1] * dy) / n[2]
        q = R.T @ np.array([dx, dy, zs - p[2]])
        if abs(q[0]) > DECK_HALF[0] or abs(q[1]) > DECK_HALF[1]:
            return None
        return zs


SEA_CARD = SeaState(1.0)


# ── Gusts ───────────────────────────────────────────────────────────────────
class Gust3D:
    """Horizontal gust identical to the 2-D script (same stream), plus a
    first-order Dryden vertical gust on its own stream."""

    def __init__(self, seed, k, sigma_h=SIGMA_GUST, sigma_z=SIGMA_GZ):
        self.h = S2.GustModel(np.random.default_rng([seed, k]), sigma_h)
        self.rz = np.random.default_rng([seed, k, 1])
        self.az = math.exp(-DT * V_AIRSPEED / L_GZ)
        self.bz = sigma_z * math.sqrt(1.0 - self.az ** 2)
        self.xz = 0.0

    def step(self):
        xi = self.h.step().copy()
        self.xz = self.az * self.xz + self.bz * float(self.rz.standard_normal())
        return xi, self.xz


# ── Drone state and the shared 3-D motion primitives ────────────────────────
class Drone3D:
    def __init__(self, k, pos3):
        self.k = k
        self.pos = np.array(pos3, float)
        self.vel = np.zeros(3)          # ground velocity of the last step
        self.v_ctrl = np.zeros(3)       # PD-phase commanded velocity (mean-wind frame)
        self.energy = BATTERY_FULL
        self.e_h = 0.0                  # horizontal power integral (P_cruise + k_wind |w|^2)
        self.e_v = 0.0                  # vertical power integral (k_vert v_z^2)
        self.queue, self.assigned = [], []
        self.mode = 'idle'
        self.timer = 0.0
        self.cargo = None
        self.swaps = self.swaps_task = 0
        self.min_soc = 1.0
        self.hold_angle = 0.0
        self.dep_xy, self.dep_z = self.pos[:2].copy(), self.pos[2]
        self.zc_prev = None
        self.ref = None
        self.app = None                 # current vessel approach record
        self.approaches = []
        self.log_pos, self.log_soc, self.log_mode = [], [], []


def track_z(dr, z_cmd):
    """Altitude tracking outside the PD phase: feed-forward of the command rate
    plus a P term, limited to V_Z_MAX. Returns the commanded (air) vertical speed."""
    ff = 0.0 if dr.zc_prev is None else (z_cmd - dr.zc_prev) / DT
    dr.zc_prev = z_cmd
    return min(V_Z_MAX, max(-V_Z_MAX, ff + K_Z * (z_cmd - dr.pos[2])))


def climb_profile(dr, xy):
    s_from = float(np.hypot(*(xy - dr.dep_xy)))
    return dr.dep_z + (Z_CRUISE - dr.dep_z) * min(1.0, s_from / D_RAMP)


def new_approach(t):
    return dict(t_start=t, t_gs=None, t_dock=None, t_gate=None, t_contact=None,
                tilt_hold=0.0, gs_n=0, gs_viol=0, gs_max=0.0, min_clear=np.inf,
                contact='', v_impact=None, samples=[], go_arounds=0, failed=[])


def gs_sample(dr, t, sea):
    """Glideslope error against the TRUE heaving deck (evaluation only)."""
    if dr.app is None:
        return
    p = sea.deck(t)
    d = float(np.hypot(*(dr.pos[:2] - p[:2])))
    h = dr.pos[2] - p[2]
    if D_EPS_MIN <= d <= D_GS:
        if dr.app['t_gs'] is None:
            dr.app['t_gs'] = t
        eps = (h - d * TAN_GS) / d
        dr.app['gs_n'] += 1
        dr.app['gs_viol'] += abs(eps) > EPS_GS_MAX
        dr.app['gs_max'] = max(dr.app['gs_max'], abs(eps))
    return d, h


def step_vessel_pursuit(dr, t, wind, xi_z, sea, law):
    """Horizontal pursuit of the vessel (as 2-D) + glideslope altitude.
    Returns the vertical air speed used for the power model."""
    vp = S2.vessel_position(t)
    _, pint = intercept(dr.pos[:2], t, n_iter=6)
    u = (pint - dr.pos[:2]) / max(float(np.hypot(*(pint - dr.pos[:2]))), 1e-6)
    dist = float(np.hypot(*(vp - dr.pos[:2])))
    v_purs = u * groundspeed(u, wind)
    # braking: limit the closing speed relative to the vessel so the PD phase
    # starts at about V_DOCK (the 2-D model docks at full closing speed)
    v_ves = (S2.vessel_position(t + DT) - vp) / DT
    r_hat = (vp - dr.pos[:2]) / max(dist, 1e-6)
    cap = math.sqrt(V_DOCK ** 2 + 2 * A_DEC * max(dist - D_DOCK, 0.0))
    if float((v_purs - v_ves) @ r_hat) > cap:
        v_purs = v_ves + r_hat * cap
    new_xy = dr.pos[:2] + v_purs * DT
    pe = sea.deck(t + DT, law)
    d_new = float(np.hypot(*(new_xy - pe[:2])))
    z_cmd = min(Z_CRUISE, climb_profile(dr, new_xy), pe[2] + d_new * TAN_GS)
    vz = track_z(dr, z_cmd)
    new = np.array([new_xy[0], new_xy[1], dr.pos[2] + (vz + xi_z) * DT])
    return new, vz


def deck_kinematics(sea, t, law):
    """Deck point, finite-difference velocity and acceleration (card: FD)."""
    p0, p1, p2 = sea.deck(t, law), sea.deck(t - DT, law), sea.deck(t - 2 * DT, law)
    return p0, (p0 - p1) / DT, (p0 - 2 * p1 + p2) / DT ** 2


def enter_final(dr, t, sea, law, xi_h, xi_z):
    """Switch to the PD phase at d <= d_dock. The glide reference starts at the
    drone's offset from the (estimated) deck and its relative velocity and
    reaches the hover point with zero relative velocity after T = 2 d0 / v0
    (cubic Hermite, i.e. constant-deceleration-like convergence)."""
    pe, ve, _ = deck_kinematics(sea, t, law)
    o0 = dr.pos[:2] - pe[:2]
    w0 = dr.vel[:2] - ve[:2]
    d0 = float(np.hypot(*o0))
    v_close = -float(w0 @ o0) / max(d0, 1e-6)
    dr.app['v_entry'] = v_close
    v0 = max(v_close, 1.0)
    dr.ref = dict(t0=t, o0=o0, w0=w0, T=float(np.clip(2 * d0 / v0, 3.0, 30.0)))
    dr.v_ctrl = dr.vel - np.array([xi_h[0], xi_h[1], xi_z])
    dr.mode = 'final'
    dr.app['t_dock'] = t


def ref_point(dr, t, sea, law):
    """Reference point at time t: deck estimate + glide offset (+ touch-down)."""
    pe = sea.deck(t, law)
    if law == 'direct':
        # card PD applied straight to the deck point (ablation)
        d = float(np.hypot(*(dr.pos[:2] - pe[:2])))
        oz = max(d * TAN_GS, H_HOVER)
        o = np.zeros(2)
    else:
        r = dr.ref
        tau = min((t - r['t0']) / r['T'], 1.0)
        h00 = 2 * tau ** 3 - 3 * tau ** 2 + 1
        h10 = tau ** 3 - 2 * tau ** 2 + tau
        o = h00 * r['o0'] + h10 * r['T'] * r['w0']
        oz = max(float(np.hypot(*o)) * TAN_GS, H_HOVER)
    tg = dr.app['t_gate']
    if tg is not None and t > tg:
        oz = H_HOVER - V_TD * (t - tg)
    return pe + np.array([o[0], o[1], oz])


def step_pd(dr, t, xi_h, xi_z, sea, law):
    """Card 3-D moving-deck PD law  u = -Kp dr - Kd dv + a_ref, with the
    reference state from finite differences (card). The deck state is the
    law's estimate; returns (new position, ground velocity, vertical air speed)."""
    r0, r1, r2 = (ref_point(dr, t, sea, law), ref_point(dr, t - DT, sea, law),
                  ref_point(dr, t - 2 * DT, sea, law))
    v_ref = (r0 - r1) / DT
    a_ref = (r0 - 2 * r1 + r2) / DT ** 2
    u = -KP * (dr.pos - r0) - KD * (dr.vel - v_ref) + a_ref
    un = float(np.linalg.norm(u))
    if un > A_MAX:
        u *= A_MAX / un
    dr.v_ctrl = dr.v_ctrl + u * DT
    air_xy = dr.v_ctrl[:2] - W_MEAN
    an = float(np.hypot(*air_xy))
    if an > V_AIRSPEED:
        dr.v_ctrl[:2] = W_MEAN + air_xy * (V_AIRSPEED / an)
    dr.v_ctrl[2] = min(V_Z_MAX, max(-V_Z_MAX, float(dr.v_ctrl[2])))
    vg = dr.v_ctrl + np.array([xi_h[0], xi_h[1], xi_z])
    return dr.pos + vg * DT, vg, float(dr.v_ctrl[2])


def deck_gates(dr, t, sea):
    """Touch-down gates against the TRUE deck (position, velocity, tilt, pad)."""
    p, v, _ = deck_kinematics(sea, t, 'card')
    R = sea.frame(t)
    q = R.T @ (dr.pos - p)
    dr_n = float(np.linalg.norm(dr.pos - p))
    dv_n = float(np.linalg.norm(dr.vel - v))
    tilt = sea.tilt(t)
    pad = abs(q[0]) < PAD_HALF and abs(q[1]) < PAD_HALF
    return dr_n, dv_n, tilt, pad, q


def approach_update(dr, t, xi_h, xi_z, sea, law, wind):
    """One step of the final approach (modes 'final', 'touch' and, in missions,
    'goaround'). Returns (vertical air speed, contact-kind or None)."""
    new, vg, vz = step_pd(dr, t, xi_h, xi_z, sea, law)
    if dr.mode == 'goaround':
        # climb back to the hover point: the PD law tracks the hover reference
        # horizontally, the climb is forced to at least V_GA relative to the deck
        _, ve, _ = deck_kinematics(sea, t + DT, 'card')
        zs0 = sea.surface_z(t + DT, new[:2])
        z_floor = sea.deck(t + DT)[2] if zs0 is None else zs0
        if new[2] - z_floor < H_HOVER:
            dr.v_ctrl[2] = min(V_Z_MAX, max(float(dr.v_ctrl[2]), float(ve[2]) + V_GA))
            vg[2] = dr.v_ctrl[2] + xi_z
            new[2] = dr.pos[2] + vg[2] * DT
            vz = float(dr.v_ctrl[2])
        else:
            dr.mode = 'final'
    zs = sea.surface_z(t + DT, new[:2])
    contact = None
    if zs is not None and new[2] <= zs:
        # deck contact: the drone ends on the deck surface
        p1, v1, _ = deck_kinematics(sea, t + DT, 'card')
        v_rel = float(vg[2] - v1[2])
        new[2] = zs
        contact = 'touchdown' if dr.mode == 'touch' else 'strike'
        dr.app['t_contact'] = t + DT
        dr.app['contact'] = contact
        dr.app['v_impact'] = abs(v_rel)
    dr.pos, dr.vel = new, vg
    if contact is None:
        dr_n, dv_n, tilt, pad, q = deck_gates(dr, t + DT, sea)
        zs2 = sea.surface_z(t + DT, dr.pos[:2])
        if zs2 is not None and dr.mode == 'final':     # clearance before the touch-down command
            dr.app['min_clear'] = min(dr.app['min_clear'], dr.pos[2] - zs2)
        dr.app['samples'].append((t + DT, dr_n, dv_n, tilt))
        if dr.mode == 'final':
            pos_ok, vel_ok = dr_n < EPS_P, dv_n < EPS_V
            if pos_ok and vel_ok and pad and tilt >= PHI_MAX:
                dr.app['tilt_hold'] += DT
            if pos_ok and vel_ok and pad and tilt < PHI_MAX:
                dr.mode = 'touch'
                dr.app['t_gate'] = t + DT
    return vz, contact


def burn(dr, wind, vz):
    """Integrate the 3-D power model for one step; returns False if the battery
    is exhausted (the drone ditches)."""
    assert dr.energy > 0.0, 'flying on an empty battery'
    eh = power(wind) * DT
    ev = K_VERT * vz * vz * DT
    e = eh + ev
    if e >= dr.energy:
        f = dr.energy / e
        dr.e_h += eh * f
        dr.e_v += ev * f
        dr.energy = 0.0
        return False
    dr.energy -= e
    dr.e_h += eh
    dr.e_v += ev
    return True


# ── Deck-approach study (experiment 1) ──────────────────────────────────────
def approach_trial(law, scale, seed, gusty=True):
    """One seeded approach: the drone starts TRIAL_R0 from the vessel at cruise
    altitude on a random bearing; the vessel follows its route from a random
    time t0. Same seed -> same t0, bearing and gusts for both laws."""
    rng = np.random.default_rng([39, seed])
    t0 = float(rng.uniform(0.0, 600.0))
    t0 = round(t0 / DT) * DT
    brg = float(rng.uniform(-np.pi, np.pi))
    sea = SeaState(scale)
    vp = S2.vessel_position(t0)
    dr = Drone3D(0, [vp[0] + TRIAL_R0 * math.cos(brg), vp[1] + TRIAL_R0 * math.sin(brg),
                     Z_CRUISE])
    dr.dep_xy, dr.dep_z = dr.pos[:2].copy(), Z_CRUISE
    dr.mode = 'vessel'
    dr.app = new_approach(t0)
    gust = (Gust3D(1000 + seed, 0) if gusty else Gust3D(1000 + seed, 0, 0.0, 0.0))
    t_pred = intercept(dr.pos[:2], t0)[0] - t0
    track = []
    n = int(round(TRIAL_TMAX / DT))
    for i in range(n):
        t = t0 + i * DT
        xi_h, xi_z = gust.step()
        wind = W_MEAN + xi_h
        if dr.mode == 'vessel':
            vp = S2.vessel_position(t)
            if float(np.hypot(*(vp - dr.pos[:2]))) <= D_DOCK:
                enter_final(dr, t, sea, law, xi_h, xi_z)
        if dr.mode == 'vessel':
            new, vz = step_vessel_pursuit(dr, t, wind, xi_z, sea, law)
            dr.vel = (new - dr.pos) / DT
            dr.pos = new
            contact = None
        else:
            vz, contact = approach_update(dr, t, xi_h, xi_z, sea, law, wind)
        burn(dr, wind, vz)
        assert dr.pos[2] > 0.0
        zs_c = sea.surface_z(t + DT, dr.pos[:2])
        assert zs_c is None or dr.pos[2] >= zs_c - 1e-9, 'below the vessel deck'
        gs = gs_sample(dr, t + DT, sea)
        pd_ = sea.deck(t + DT)
        track.append((t + DT - t0, gs[0] if gs else 0.0, dr.pos[2] - pd_[2], dr.mode))
        if contact is not None:
            break
    a = dr.app
    ok = a['contact'] == 'touchdown' and a['v_impact'] < EPS_V
    return dict(law=law, scale=scale, seed=seed, contact=a['contact'] or 'timeout',
                safe=ok, v_impact=a['v_impact'], t_total=(a['t_contact'] or t0 + TRIAL_TMAX) - t0,
                t_final=((a['t_contact'] - a['t_dock']) if a['t_contact'] and a['t_dock']
                         else np.nan),
                tilt_hold=a['tilt_hold'], gs_frac=a['gs_viol'] / max(a['gs_n'], 1),
                gs_max=a['gs_max'], min_clear=a['min_clear'], e_v=dr.e_v, e_h=dr.e_h,
                track=track, samples=a['samples'], t_pred=t_pred,
                t_dock=a['t_dock'] - t0 if a['t_dock'] else None)


def run_approach_study():
    out = {(law, s): [approach_trial(law, s, i) for i in range(N_TRIALS)]
           for s in SEA_SCALES for law in LAWS}
    out[('direct', 1.0)] = [approach_trial('direct', 1.0, i) for i in range(N_TRIALS)]
    return out


def calibrate_overhead():
    """Gust-free card-law approaches from TRIAL_R0 (same start states as the
    study): mean extra time and energy beyond the 2-D model, which docks on
    reaching the predicted intercept point. Used by the 3-D planner."""
    res = [approach_trial('card', 1.0, i, gusty=False) for i in range(N_TRIALS)]
    dt_extra = float(np.mean([r['t_total'] - r['t_pred'] for r in res]))
    e_v = float(np.mean([r['e_v'] for r in res]))
    return dt_extra, e_v, res


# ── 3-D planner (experiment 2) ──────────────────────────────────────────────
PLAN_OVH = {'dt': 0.0, 'ev': 0.0}     # vessel-approach overhead, set by calibration


def ramp_energy(dz):
    """Vertical energy of a linear ramp of height dz over D_RAMP at airspeed."""
    return K_VERT * dz * dz * V_AIRSPEED / D_RAMP if dz > 0 else 0.0


def fly3(p, q, z_from, to_vessel=False):
    """(time, planned energy) of a 3-D leg: the 2-D mean-wind leg plus the
    climb-out ramp, the platform descent ramp or the calibrated vessel approach."""
    t, e = fly_cost(p, q)
    e += ramp_energy(Z_CRUISE - z_from)
    if to_vessel:
        t += PLAN_OVH['dt']
        e += P_PLAN * PLAN_OVH['dt'] + PLAN_OVH['ev']
    else:
        e += ramp_energy(Z_CRUISE - Z_PLATFORM)
    return t, e


def plan_options3(pos, energy, t_free, z, req_idx, reqs):
    """3-D copy of the 2-D plan_options: same three route families, 3-D costs."""
    o, d, _, _ = reqs[req_idx]
    po, pd = PLATFORMS[o], PLATFORMS[d]
    opts = []
    t1, e1 = fly3(pos, po, z)
    t2, e2 = fly3(po, pd, Z_PLATFORM)
    if energy - e1 - e2 >= E_RES:
        opts.append(dict(steps=[('fly', o), ('pickup', req_idx), ('fly', d), ('deliver', req_idx)],
                         t_deliver=t_free + t1 + T_HANDLE + t2, e_used=e1 + e2,
                         e_end=energy - e1 - e2, swaps=0))
    t_pick = t_free + t1 + T_HANDLE
    if energy - e1 >= E_RES:
        for w in WAIT_GRID:
            t_dep = t_pick + w
            ts, pv = intercept(po, t_dep)
            _, ev = fly3(po, pv, Z_PLATFORM, to_vessel=True)
            tf, ef = fly3(pv, pd, Z_REF)
            if energy - e1 - ev >= E_RES and BATTERY_FULL - ef >= E_RES:
                steps = [('fly', o), ('pickup', req_idx)]
                if w > 0:
                    steps.append(('wait', t_dep))
                steps += [('vessel',), ('fly', d), ('deliver', req_idx)]
                opts.append(dict(steps=steps, t_deliver=ts + PLAN_OVH['dt'] + T_SWAP + tf,
                                 e_used=e1 + ev + ef, e_end=BATTERY_FULL - ef, swaps=1))
    ts, pv = intercept(pos, t_free)
    _, ev = fly3(pos, pv, z, to_vessel=True)
    ta, ea = fly3(pv, po, Z_REF)
    if energy - ev >= E_RES and BATTERY_FULL - ea - e2 >= E_RES:
        opts.append(dict(steps=[('vessel',), ('fly', o), ('pickup', req_idx), ('fly', d),
                                ('deliver', req_idx)],
                         t_deliver=ts + PLAN_OVH['dt'] + T_SWAP + ta + T_HANDLE + t2,
                         e_used=ev + ea + e2, e_end=BATTERY_FULL - ea - e2, swaps=1))
    return [op for op in opts if op['t_deliver'] <= T_MAX]


def build_plan3(states, req_list, mode, reqs):
    """3-D copy of the 2-D build_plan (deadline order, chained per drone).
    states: {k: (pos_xy, energy, t_free, z)}."""
    states = {k: (np.array(v[0], float), v[1], v[2], v[3]) for k, v in states.items()}
    queues = {k: [] for k in states}
    rejected = []
    for r in sorted(req_list, key=lambda r: (reqs[r][3], r)):
        best = None
        for k, (pos, en, tf, z) in states.items():
            op = S2.choose(plan_options3(pos, en, tf, z, r, reqs), reqs[r][3], mode)
            if op is None:
                continue
            key = (op['t_deliver'] > reqs[r][3], op['t_deliver'], k)
            if best is None or key < best[0]:
                best = (key, k, op)
        if best is None:
            rejected.append(r)
            continue
        _, k, op = best
        queues[k] += op['steps']
        states[k] = (PLATFORMS[reqs[r][1]].copy(), op['e_end'], op['t_deliver'] + T_HANDLE,
                     Z_PLATFORM)
    return queues, rejected


def build_plan_any(states, req_list, mode, reqs, plan3d):
    if plan3d:
        return build_plan3(states, req_list, mode, reqs)
    return S2.build_plan({k: v[:3] for k, v in states.items()}, req_list, mode, reqs)


# ── Mission simulation (experiment 2) ───────────────────────────────────────
_PLAT_L = [(float(x), float(y)) for x, y in PLATFORMS]


def on_pad(xy, m):
    px, py = _PLAT_L[m]
    return abs(xy[0] - px) < PAD_HALF and abs(xy[1] - py) < PAD_HALF


def run_strategy3d(strategy, seed, reqs, plan3d=True, sea=SEA_CARD, law='card'):
    drones = [Drone3D(k, [*PLATFORMS[DRONE_START[k]], Z_PLATFORM]) for k in range(N_DRONES)]
    gusts = [Gust3D(seed, k) for k in range(N_DRONES)]
    n_req = len(reqs)
    status = ['pending'] * n_req
    t_deliver = [None] * n_req
    pad_busy = [None] * N_PLATFORMS
    deck_busy = None
    events, pad_log = [], []
    hold_time = 0.0
    pad_contacts = 0
    deck_lifts = 0

    def give(dr, steps, r_ids):
        dr.queue = list(steps)
        dr.assigned = list(r_ids)
        dr.swaps_task = 0
        for r in r_ids:
            status[r] = 'assigned'

    def reqs_in(steps):
        return sorted({s[1] for s in steps if s[0] == 'pickup'})

    def plan_static():
        states = {k: (dr.pos[:2], dr.energy, 0.0, dr.pos[2]) for k, dr in enumerate(drones)}
        queues, rej = build_plan_any(states, list(range(n_req)), 'max_soc', reqs, plan3d)
        for k, q in queues.items():
            give(drones[k], q, reqs_in(q))
        for r in rej:
            status[r] = 'rejected'

    def plan_rolling(t):
        states = {}
        for k, dr in enumerate(drones):
            if dr.cargo is None and dr.mode in ('idle', 'fly', 'wait'):
                states[k] = (dr.pos[:2], dr.energy, t, dr.pos[2])
        locked = {r for k, dr in enumerate(drones) if k not in states for r in dr.assigned}
        open_reqs = [r for r in range(n_req)
                     if status[r] in ('pending', 'assigned', 'rejected') and r not in locked]
        if not open_reqs or not states:
            return
        queues, rej = build_plan_any(states, open_reqs, 'min_energy', reqs, plan3d)
        for k in states:
            dr = drones[k]
            for r in dr.assigned:
                if status[r] == 'assigned':
                    status[r] = 'pending'
            if queues[k] or dr.mode in ('fly', 'wait'):
                if dr.mode == 'wait':
                    dr.mode = 'idle'
                give(dr, queues[k], reqs_in(queues[k]))
                if dr.mode == 'fly' and not queues[k]:
                    m = int(np.argmin(np.linalg.norm(PLATFORMS - dr.pos[:2], axis=1)))
                    dr.queue = [('fly', m)]
                if dr.mode == 'fly':
                    dr.mode = 'idle'
        for r in rej:
            if status[r] != 'assigned':
                status[r] = 'rejected'

    def assign_greedy():
        for r in range(n_req):
            if status[r] != 'pending':
                continue
            idle = [dr for dr in drones if dr.mode == 'idle' and not dr.queue]
            if not idle:
                return
            o, d, _, _ = reqs[r]
            dr = min(idle, key=lambda x: (np.hypot(*(x.pos[:2] - PLATFORMS[o])), x.k))
            give(dr, [('fly', o), ('pickup', r), ('fly', d), ('deliver', r)], [r])

    def need(dr):
        if dr.mode != 'fly' or not dr.queue or dr.queue[0][0] != 'fly':
            return 0.0
        tgt = PLATFORMS[dr.queue[0][1]]
        if plan3d:
            return fly3(dr.pos[:2], tgt, dr.pos[2])[1]
        return S2.fly_cost(dr.pos[:2], tgt)[1]

    def nearest_platform(dr):
        return int(np.argmin([fly_cost(dr.pos[:2], P)[1] for P in PLATFORMS]))

    def start_leg(dr):
        dr.dep_xy, dr.dep_z = dr.pos[:2].copy(), dr.pos[2]
        dr.zc_prev = None

    def start_vessel(dr, t):
        start_leg(dr)
        dr.mode = 'vessel'
        if dr.app is None:
            dr.app = new_approach(t)

    def emergency(dr, t):
        if dr.mode != 'fly':
            return
        if dr.energy - need(dr) >= E_RES:
            return
        if dr.energy >= 0.98 * BATTERY_FULL and dr.swaps_task >= 1:
            r_ab = dr.cargo if dr.cargo is not None else (dr.assigned[0] if dr.assigned else None)
            if r_ab is not None:
                status[r_ab] = 'aborted'
                events.append((t, dr.k, 'abort', r_ab))
            dr.cargo = None
            dr.assigned = []
            dr.queue = [('fly', nearest_platform(dr))]
            start_leg(dr)
            return
        events.append((t, dr.k, 'divert', None))
        dr.queue.insert(0, ('vessel',))
        start_vessel(dr, t)

    def ditch(dr, t):
        nonlocal deck_busy
        dr.mode = 'ditched'
        dr.queue = []
        if dr.cargo is not None:
            status[dr.cargo] = 'lost'
            dr.cargo = None
        for r in dr.assigned:
            if status[r] == 'assigned':
                status[r] = 'pending'
        dr.assigned = []
        if deck_busy == dr.k:
            deck_busy = None
        zs = sea.surface_z(t + DT, dr.pos[:2])
        dr.ditch_over_deck = zs is not None and dr.pos[2] - zs < 2.0
        events.append((t, dr.k, 'ditched', None))

    n_steps = int(round(T_MAX / DT))
    deck_log = []
    if strategy == 'relay':
        plan_static()
    for step in range(n_steps + 1):
        t = step * DT
        vp = S2.vessel_position(t)
        deck_log.append(sea.deck(t))
        if strategy == 'rolling' and step % int(round(DT_PLAN / DT)) == 0:
            plan_rolling(t)
        if strategy == 'greedy':
            assign_greedy()

        for k, dr in enumerate(drones):
            xi_h, xi_z = gusts[k].step()
            wind = W_MEAN + xi_h
            p_prev = dr.pos.copy()
            if dr.mode == 'idle' and dr.queue:
                kind = dr.queue[0][0]
                if kind == 'fly':
                    m = dr.queue[0][1]
                    if np.hypot(*(dr.pos[:2] - PLATFORMS[m])) < 1.0:
                        dr.queue.pop(0)
                        dr.land_platform = m
                        dr.mode = 'land' if abs(dr.pos[2] - Z_PLATFORM) < 1e-6 else 'descend'
                        dr.zc_prev = None
                        dr.zc = dr.pos[2]
                    else:
                        dr.mode = 'fly'
                        start_leg(dr)
                elif kind == 'vessel':
                    start_vessel(dr, t)
                elif kind == 'wait':
                    dr.mode = 'wait'
                elif kind in ('pickup', 'deliver'):
                    dr.queue.pop(0)
            if dr.mode == 'fly':
                emergency(dr, t)

            flying, vz = False, 0.0
            if dr.mode == 'fly':
                m = dr.queue[0][1]
                tgt = PLATFORMS[m]
                vec = tgt - dr.pos[:2]
                dist = float(np.hypot(*vec))
                step_len = groundspeed(vec / dist, wind) * DT
                flying = True
                arrived = dist <= step_len
                new_xy = tgt.copy() if arrived else dr.pos[:2] + vec / dist * step_len
                d_to = float(np.hypot(*(tgt - new_xy)))
                z_cmd = max(Z_PLATFORM + H_HOVER,
                            min(Z_CRUISE, climb_profile(dr, new_xy),
                                Z_PLATFORM + H_HOVER
                                + (Z_CRUISE - Z_PLATFORM - H_HOVER) * d_to / D_RAMP))
                vz = track_z(dr, z_cmd)
                dr.pos = np.array([new_xy[0], new_xy[1], dr.pos[2] + (vz + xi_z) * DT])
                low = on_pad(new_xy, m) and dr.pos[2] <= Z_PLATFORM
                if low:
                    dr.pos[2] = Z_PLATFORM        # gust pushed it onto the pad: contact
                if arrived:
                    dr.queue.pop(0)
                    dr.mode = 'land' if low else 'descend'
                    dr.land_platform = m
                    dr.zc = dr.pos[2]
            elif dr.mode == 'descend':
                flying = True
                rate = V_Z_MAX if dr.zc > Z_PLATFORM + H_HOVER else V_TD
                dr.zc = max(Z_PLATFORM - 0.2, dr.zc - rate * DT)
                vz = track_z(dr, dr.zc)
                dr.pos[2] += (vz + xi_z) * DT
                if dr.pos[2] <= Z_PLATFORM:
                    dr.pos[2] = Z_PLATFORM
                    dr.mode = 'land'
            elif dr.mode == 'land':
                m = dr.land_platform
                if pad_busy[m] is None:
                    if dr.queue and dr.queue[0][0] in ('pickup', 'deliver') and \
                            reqs[dr.queue[0][1]][0 if dr.queue[0][0] == 'pickup' else 1] == m:
                        pad_busy[m] = dr.k
                        dr.mode = 'handle'
                        dr.timer = T_HANDLE
                        dr.handle = dr.queue.pop(0)
                        dr.pad_start = t
                        if dr.handle[0] == 'deliver':
                            r = dr.handle[1]
                            t_deliver[r] = t
                            status[r] = 'delivered'
                            dr.cargo = None
                            events.append((t, dr.k, 'deliver', r))
                    else:
                        dr.mode = 'idle'
                else:
                    dr.mode = 'hold'
                    dr.hold_angle = 0.0
                    dr.zc_prev = None
                    events.append((t, dr.k, 'hold', m))
            elif dr.mode in ('hold', 'reland'):
                m = dr.land_platform
                flying = True
                hold_time += DT
                if dr.mode == 'hold':
                    dr.hold_angle += 0.8 * V_AIRSPEED / R_HOLD * DT
                    tgt = PLATFORMS[m] + R_HOLD * np.array([np.cos(dr.hold_angle),
                                                            np.sin(dr.hold_angle)])
                else:
                    tgt = PLATFORMS[m]
                vec = tgt - dr.pos[:2]
                dist = float(np.hypot(*vec))
                mv = vec if dist <= V_AIRSPEED * DT else vec / dist * V_AIRSPEED * DT
                vz = track_z(dr, Z_HOLD)
                dr.pos = np.array([dr.pos[0] + mv[0], dr.pos[1] + mv[1],
                                   dr.pos[2] + (vz + xi_z) * DT])
                if dr.mode == 'hold' and pad_busy[m] is None:
                    dr.mode = 'reland'
                elif dr.mode == 'reland' and dist <= V_AIRSPEED * DT:
                    dr.mode = 'descend'
                    dr.zc = dr.pos[2]
                    dr.zc_prev = None
            elif dr.mode == 'handle':
                dr.timer -= DT
                if dr.timer <= 1e-9:
                    m = dr.land_platform
                    pad_busy[m] = None
                    pad_log.append((m, dr.k, dr.pad_start, t, dr.handle[0]))
                    if dr.handle[0] == 'pickup':
                        r = dr.handle[1]
                        dr.cargo = r
                        status[r] = 'picked'
                        if r in dr.assigned:
                            dr.assigned.remove(r)
                    dr.mode = 'idle'
            elif dr.mode == 'wait':
                if t >= dr.queue[0][1] - 1e-9:
                    dr.queue.pop(0)
                    dr.mode = 'idle'
            elif dr.mode in ('vessel', 'vhold'):
                flying = True
                dist = float(np.hypot(*(vp - dr.pos[:2])))
                if dist <= D_DOCK and deck_busy is None:
                    deck_busy = dr.k
                    enter_final(dr, t, sea, law, xi_h, xi_z)
                elif dist <= D_DOCK or dr.mode == 'vhold':
                    if dr.mode != 'vhold':
                        dr.mode = 'vhold'
                        dr.zc_prev = None
                    u = (dr.pos[:2] - vp) / max(dist, 1e-6)
                    new_xy = S2.vessel_position(t + DT) + u * D_DOCK
                    vz = track_z(dr, sea.deck(t + DT, law)[2] + D_DOCK * TAN_GS)
                    new = np.array([new_xy[0], new_xy[1], dr.pos[2] + (vz + xi_z) * DT])
                    dr.vel = (new - dr.pos) / DT
                    dr.pos = new
                    hold_time += DT
                else:
                    new, vz = step_vessel_pursuit(dr, t, wind, xi_z, sea, law)
                    dr.vel = (new - dr.pos) / DT
                    dr.pos = new
            if dr.mode in ('final', 'touch', 'goaround'):
                flying = True
                vz, contact = approach_update(dr, t, xi_h, xi_z, sea, law, wind)
                if contact is not None and (contact == 'strike'
                                            or dr.app['v_impact'] >= EPS_V):
                    # unsafe contact: abort and go around (back to the 0.5 m
                    # hover above the pad, then wait for the gates again)
                    a = dr.app
                    if contact == 'strike':
                        events.append((t, dr.k, 'strike', None))
                    events.append((t, dr.k, 'go_around', contact))
                    a['go_arounds'] += 1
                    a['failed'].append((contact, a['v_impact']))
                    a['t_contact'], a['contact'], a['v_impact'] = None, '', None
                    a['t_gate'] = None
                    dr.mode = 'goaround'
                    contact = None
                if contact is not None:
                    assert contact == 'touchdown' and dr.app['v_impact'] < EPS_V, \
                        'unsafe touch-down counted as docked'
                    q = sea.frame(t + DT).T @ (dr.pos - sea.deck(t + DT))
                    dr.q_dock = np.array([q[0], q[1], 0.0])
                    dr.mode = 'docked'
                    dr.timer = T_SWAP
                    events.append((t, dr.k, 'dock', None))
            elif dr.mode == 'docked':
                dr.pos = sea.deck(t + DT) + sea.frame(t + DT) @ dr.q_dock
                dr.vel = np.zeros(3)
                dr.timer -= DT
                if dr.timer <= 1e-9:
                    dr.energy = BATTERY_FULL
                    dr.swaps += 1
                    dr.swaps_task += 1
                    deck_busy = None
                    if dr.queue and dr.queue[0][0] == 'vessel':
                        dr.queue.pop(0)
                    if not dr.queue:
                        dr.queue = [('fly', nearest_platform(dr))]
                    dr.approaches.append(dr.app)
                    dr.app = None
                    dr.mode = 'idle'
                    events.append((t, dr.k, 'swap', None))
            elif dr.mode == 'ditched':
                # falls at V_Z_MAX; a wreck above the vessel deck comes to rest on
                # it (rising with the heave, never through it) until the vessel
                # has moved on from under it, then falls into the sea
                zs = sea.surface_z(t + DT, dr.pos[:2])
                if zs is not None and dr.pos[2] >= zs - 1.0:
                    dr.pos[2] = max(zs, dr.pos[2] - V_Z_MAX * DT)
                else:
                    dr.pos[2] = max(0.0, dr.pos[2] - V_Z_MAX * DT)

            # take-off from (or any other flight over) the moving deck: a drone
            # whose climb is slower than the deck's heave plus a downdraft is
            # carried up by the deck instead of passing through it.  The final
            # approach handles deck contact itself (approach_update), a docked
            # drone moves with the deck and a wreck rests on it (below).
            if dr.mode not in ('final', 'touch', 'goaround', 'docked', 'ditched'):
                zs_l = sea.surface_z(t + DT, dr.pos[:2])
                if zs_l is not None and dr.pos[2] < zs_l:
                    dr.pos[2] = zs_l
                    deck_lifts += 1

            if flying:
                gs_sample(dr, t + DT, sea)
                if not burn(dr, wind, vz):
                    ditch(dr, t)
            for m in range(N_PLATFORMS):
                if dr.mode != 'ditched' and on_pad(dr.pos[:2], m) and dr.pos[2] < Z_PLATFORM:
                    dr.pos[2] = Z_PLATFORM        # resting on the pad (take-off in a downdraft)
                    pad_contacts += 1
            # physical invariants
            assert dr.energy >= 0.0
            assert dr.pos[2] >= 0.0, 'below sea level'
            for m in range(N_PLATFORMS):
                if on_pad(dr.pos[:2], m):
                    assert dr.pos[2] >= Z_PLATFORM - 1e-9, ('inside a platform', dr.mode, dr.pos, p_prev, t)
            zs_c = sea.surface_z(t + DT, dr.pos[:2])
            assert zs_c is None or dr.pos[2] >= zs_c - 1e-9, \
                ('below the vessel deck', dr.mode, dr.pos, zs_c, t)
            assert np.linalg.norm(dr.pos - p_prev) <= 25.0 * DT, 'teleport'
            dr.min_soc = min(dr.min_soc, dr.energy / BATTERY_FULL)
            dr.log_pos.append(dr.pos.copy())
            dr.log_soc.append(dr.energy / BATTERY_FULL)
            dr.log_mode.append(dr.mode)

    delivered = [r for r in range(n_req) if status[r] == 'delivered']
    late = [r for r in delivered if t_deliver[r] > reqs[r][3]]
    apps = [a for dr in drones for a in dr.approaches] + \
        [dr.app for dr in drones if dr.app is not None]
    return dict(strategy=strategy, seed=seed, reqs=reqs, drones=drones, status=status,
                t_deliver=t_deliver, events=events, pad_log=pad_log,
                deck_log=np.array(deck_log), hold_time=hold_time,
                energy_kJ=sum(dr.e_h + dr.e_v for dr in drones) / 1000.0,
                e_h_kJ=sum(dr.e_h for dr in drones) / 1000.0,
                e_v_kJ=sum(dr.e_v for dr in drones) / 1000.0,
                swaps=sum(dr.swaps for dr in drones), delivered=len(delivered),
                on_time=len(delivered) - len(late), late=len(late),
                min_soc=min(dr.min_soc for dr in drones),
                ditched=sum(1 for e in events if e[2] == 'ditched'),
                strikes=sum(1 for e in events if e[2] == 'strike'), approaches=apps,
                pad_contacts=pad_contacts, deck_lifts=deck_lifts)


# ── Experiment driver ───────────────────────────────────────────────────────
def run_simulation():
    dt_extra, ev_extra, calib = calibrate_overhead()
    PLAN_OVH.update(dt=dt_extra, ev=ev_extra)
    study = run_approach_study()
    runs3 = {name: {s: [run_strategy3d(s, seed, reqs) for seed in range(N_SEEDS)]
                    for s in STRATEGIES} for name, reqs in INSTANCES.items()}
    runs_ab = {name: {s: [run_strategy3d(s, seed, reqs, plan3d=False)
                          for seed in range(N_SEEDS)] for s in ('relay', 'rolling')}
               for name, reqs in INSTANCES.items()}
    runs2 = {name: {s: [S2.run_strategy(s, seed, reqs) for seed in range(N_SEEDS)]
                    for s in STRATEGIES} for name, reqs in INSTANCES.items()}
    return dict(calib=calib, study=study, runs3=runs3, runs_ab=runs_ab, runs2=runs2)


# ── Printing ────────────────────────────────────────────────────────────────
def gate_closed_fraction(scale):
    sea = SeaState(scale)
    ts = np.arange(0.0, T_W, 0.001)
    return float(np.mean([sea.tilt(t) >= PHI_MAX for t in ts]))


def print_model_checks():
    print('=== 3-D model checks ===')
    print(f'Glideslope intercept range D_GS = (z_cruise - z_ref) / tan(gamma) = '
          f'{Z_CRUISE - Z_REF:.0f} / {TAN_GS:.4f} = {D_GS:.1f} m; hover floor reached at '
          f'{D_EPS_MIN:.2f} m')
    print(f'Glideslope tolerance at d_dock: {EPS_GS_MAX} x {D_DOCK:.0f} m = '
          f'{EPS_GS_MAX * D_DOCK:.2f} m (heave amplitude {A_H} m); a level-deck reference '
          f'violates it whenever |z_heave| > 0.03 d, i.e. for d < {A_H / EPS_GS_MAX:.1f} m at '
          f'heave peaks')
    tb = math.degrees(math.hypot(THETA_P, PHI_R))
    print(f'Tilt bound: |phi_comb| <= sqrt(Theta_p^2 + Phi_r^2) = {tb:.2f} deg = phi_max '
          f'{math.degrees(PHI_MAX):.1f} deg -> the card gate can never close at the card sea state')
    for s in SEA_SCALES:
        print(f'  sea scale {s:.1f}: A_h {A_H * s:.1f} m, pitch {math.degrees(THETA_P) * s:.1f} deg, '
              f'roll {math.degrees(PHI_R) * s:.1f} deg, max tilt '
              f'{math.degrees(max(SeaState(s).tilt(t) for t in np.arange(0, T_W, 0.01))):.2f} deg, '
              f'min tilt {math.degrees(min(SeaState(s).tilt(t) for t in np.arange(0, T_W, 0.01))):.2f} deg, '
              f'gate closed {gate_closed_fraction(s) * 100:.1f} % of the time')
    ts = np.arange(0, T_W, 0.01)
    err = max(abs(math.acos(math.cos(SEA_CARD.motion(t)[1]) * math.cos(SEA_CARD.motion(t)[2]))
                  - SEA_CARD.tilt(t)) for t in ts)
    print(f'Small-angle tilt sqrt(th^2 + ph^2) vs exact acos(cos th cos ph): max difference '
          f'{math.degrees(err) * 3600:.2f} arcsec at the card sea state')
    hv = A_H * OMEGA_W
    print(f'Heave rate amplitude A_h omega = {hv:.3f} m/s -> heave-following power '
          f'k_vert (A_h omega)^2 / 2 = {K_VERT * hv * hv / 2:.2f} W on average')
    print(f'Platform ramp: {Z_CRUISE - Z_PLATFORM:.0f} m over {D_RAMP:.0f} m -> vertical speed '
          f'{(Z_CRUISE - Z_PLATFORM) / D_RAMP:.2f} x groundspeed; planner charges '
          f'{ramp_energy(Z_CRUISE - Z_PLATFORM):.0f} J per platform ramp at V_a')


def print_calibration(calib):
    ext = [r['t_total'] - r['t_pred'] for r in calib]
    print('\n=== Planner calibration: gust-free card-law approaches from 400 m (20 start states) ===')
    print(f'outcomes: {sum(r["contact"] == "touchdown" for r in calib)}/{len(calib)} touchdowns, '
          f'impact {min(r["v_impact"] for r in calib):.2f}-{max(r["v_impact"] for r in calib):.2f} m/s')
    print(f'extra time over the 2-D intercept prediction: mean {np.mean(ext):.2f} s '
          f'(range {min(ext):.2f}-{max(ext):.2f} s); d_dock -> contact '
          f'{np.mean([r["t_final"] for r in calib]):.2f} s; vertical energy per approach '
          f'{np.mean([r["e_v"] for r in calib]):.0f} J')
    print(f'3-D planner vessel-approach overhead: {PLAN_OVH["dt"]:.2f} s and '
          f'{P_PLAN * PLAN_OVH["dt"] + PLAN_OVH["ev"]:.0f} J '
          f'({(P_PLAN * PLAN_OVH["dt"] + PLAN_OVH["ev"]) / BATTERY_FULL * 100:.1f} % of a battery)')


def print_feasibility():
    print('\n=== Earliest feasible delivery from a full battery at the origin at t = 0 ===')
    for r, (o, d, _, dl) in enumerate(REQUESTS_EXT):
        po = PLATFORMS[o]
        o2 = S2.plan_options(po, BATTERY_FULL, 0.0, r, REQUESTS_EXT)
        o3 = plan_options3(po, BATTERY_FULL, 0.0, Z_PLATFORM, r, REQUESTS_EXT)

        def fmt(opts):
            if not opts:
                return 'none by T_max'
            b = min(opts, key=lambda op: op['t_deliver'])
            return f'{b["t_deliver"]:.0f} s ({b["swaps"]} swap)'
        e2 = S2.fly_cost(po, PLATFORMS[d])[1]
        e3 = fly3(po, PLATFORMS[d], Z_PLATFORM)[1]
        print(f'  R{r + 1}: P{o + 1}->P{d + 1}, deadline {dl:.0f} s; direct energy 2-D '
              f'{e2 / BATTERY_FULL * 100:.0f} % / 3-D {e3 / BATTERY_FULL * 100:.0f} % of a battery; '
              f'2-D planner {fmt(o2)}; 3-D planner {fmt(o3)}')


def _fmt_rng(v, f='.2f'):
    v = [x for x in v if x is not None and not (isinstance(x, float) and np.isnan(x))]
    if not v:
        return 'n/a'
    return f'{np.mean(v):{f}} [{min(v):{f}}-{max(v):{f}}]'


def print_study(study):
    print(f'\n=== Deck-approach study: {N_TRIALS} seeded approaches per law and sea state ===')
    print(f'{"law":<7}{"scale":>6}{"touch":>6}{"strike":>7}{"tmout":>6}{"safe":>5}  '
          f'{"impact m/s mean [min-max]":<26}{"GS viol %":>10}{"max|eps|":>9}'
          f'{"tilt hold s":>12}{"dock->contact s":>16}{"min clear m":>12}{"E_v J":>7}')
    for (law, s), R in study.items():
        n_t = sum(r['contact'] == 'touchdown' for r in R)
        n_s = sum(r['contact'] == 'strike' for r in R)
        n_o = sum(r['contact'] == 'timeout' for r in R)
        print(f'{law:<7}{s:>6.1f}{n_t:>6}{n_s:>7}{n_o:>6}{sum(r["safe"] for r in R):>5}  '
              f'{_fmt_rng([r["v_impact"] for r in R]):<26}'
              f'{np.mean([r["gs_frac"] for r in R]) * 100:>10.1f}'
              f'{np.mean([r["gs_max"] for r in R]):>9.3f}'
              f'{np.mean([r["tilt_hold"] for r in R]):>7.1f} ({max(r["tilt_hold"] for r in R):.1f})'
              f'{np.nanmean([r["t_final"] for r in R]):>12.1f}'
              f'{min(r["min_clear"] for r in R):>12.3f}{np.mean([r["e_v"] for r in R]):>7.0f}')
    print('Paired outcome per seed (same start state and gusts), card law vs 2-D deck law:')
    for s in SEA_SCALES:
        a, b = study[('card', s)], study[('level', s)]
        cw = sum(x['safe'] and not y['safe'] for x, y in zip(a, b))
        lw = sum(y['safe'] and not x['safe'] for x, y in zip(a, b))
        both = sum(x['safe'] and y['safe'] for x, y in zip(a, b))
        print(f'  scale {s:.1f}: only card safe {cw}, only 2-D deck safe {lw}, both {both}, '
              f'neither {N_TRIALS - cw - lw - both}')
    a, b = study[('card', 1.0)], study[('direct', 1.0)]
    print(f'Glide reference vs card PD straight on the deck point (scale 1.0): glide safe '
          f'{sum(x["safe"] for x in a)}/{N_TRIALS}, direct safe {sum(x["safe"] for x in b)}/{N_TRIALS}, '
          f'direct strikes {sum(x["contact"] == "strike" for x in b)}')


def _req_outcomes(R, n_req):
    out = []
    for r in range(n_req):
        d = sum(res['status'][r] == 'delivered' for res in R)
        on = sum(res['status'][r] == 'delivered' and res['t_deliver'][r] <= res['reqs'][r][3]
                 for res in R)
        out.append(f'R{r + 1} {d}/{len(R)}' + (f' ({on} on time)' if d else ''))
    return ', '.join(out)


def print_mission(name, res):
    reqs = INSTANCES[name]
    r3, ab, r2 = res['runs3'][name], res['runs_ab'][name], res['runs2'][name]
    print(f'\n=== {name} instance, 3-D world + 3-D planner: per-run results ===')
    print(f'{"strategy":<8} {"seed":>4} {"E(kJ)":>7} {"E_v":>5} {"swaps":>5} {"deliv":>5} '
          f'{"late":>4} {"minSoC":>6} {"hold(s)":>7}  request status (delivery time s)')
    for s in STRATEGIES:
        for x in r3[s]:
            st = ' '.join(f'R{r + 1}:{x["status"][r]}' + (f'({x["t_deliver"][r]:.0f})'
                          if x['t_deliver'][r] is not None else '') for r in range(len(reqs)))
            print(f'{s:<8} {x["seed"]:>4} {x["energy_kJ"]:>7.2f} {x["e_v_kJ"]:>5.2f} '
                  f'{x["swaps"]:>5} {x["delivered"]:>5} {x["late"]:>4} {x["min_soc"] * 100:>5.1f}% '
                  f'{x["hold_time"]:>7.1f}  {st}')
    print(f'--- {name} instance: summary over {N_SEEDS} seeds (same horizontal gusts in every model) ---')
    rows = [('2-D script', s, r2[s]) for s in STRATEGIES] + \
        [('3-D, 3-D planner', s, r3[s]) for s in STRATEGIES] + \
        [('3-D, 2-D planner', s, ab[s]) for s in ('relay', 'rolling')]
    for model, s, R in rows:
        ev = (f' (vertical {np.mean([x["e_v_kJ"] for x in R]):.2f} kJ = '
              f'{np.mean([x["e_v_kJ"] / x["energy_kJ"] for x in R]) * 100:.1f} %)'
              if 'e_v_kJ' in R[0] else '')
        print(f'{model:<17} {STRAT_LABEL[s]:<23} energy {np.mean([x["energy_kJ"] for x in R]):6.2f} '
              f'+- {np.std([x["energy_kJ"] for x in R]):.2f} kJ{ev}, delivered '
              f'{np.mean([x["delivered"] for x in R]):.1f}/{len(reqs)}, on time '
              f'{np.mean([x["on_time"] for x in R]):.1f}, swaps {np.mean([x["swaps"] for x in R]):.1f}, '
              f'lowest SoC {np.mean([x["min_soc"] for x in R]) * 100:.1f} % '
              f'(min {min(x["min_soc"] for x in R) * 100:.1f} %), below reserve '
              f'{sum(x["min_soc"] < D_SAFETY for x in R)}/{len(R)}, ditched '
              f'{sum(x["ditched"] for x in R)}'
              + (f', deck strikes {sum(x["strikes"] for x in R)}, go-arounds '
                 f'{sum(a["go_arounds"] for x in R for a in x["approaches"])}'
                 if 'strikes' in R[0] else ''))
        print(f'{"":<17} requests: {_req_outcomes(R, len(reqs))}')
    print(f'--- {name} instance: 3-D vs 2-D per seed (3-D planner) ---')
    for s in STRATEGIES:
        dd = [x['delivered'] - y['delivered'] for x, y in zip(r3[s], r2[s])]
        do = [x['on_time'] - y['on_time'] for x, y in zip(r3[s], r2[s])]
        print(f'{STRAT_LABEL[s]:<23} delivered 3-D minus 2-D per seed {dd}, on time {do}')


def print_mission_landings(res):
    apps = [a for name in INSTANCES for s in STRATEGIES for x in res['runs3'][name][s]
            for a in x['approaches'] if a['t_contact'] is not None]
    n = len(apps)
    gs_n = sum(a['gs_n'] for a in apps)
    gs_v = sum(a['gs_viol'] for a in apps)
    print(f'\n=== Vessel landings in all 3-D missions (3-D planner, both instances, {N_SEEDS} seeds) ===')
    print(f'{n} landings: {sum(a["contact"] == "touchdown" for a in apps)} commanded touch-downs, '
          f'{sum(a["contact"] == "strike" for a in apps)} deck strikes; impact speed '
          f'{_fmt_rng([a["v_impact"] for a in apps])} m/s, '
          f'{sum(a["v_impact"] >= EPS_V for a in apps)} at or above eps_v')
    allapps = [a for name in INSTANCES for s in STRATEGIES for x in res['runs3'][name][s]
               for a in x['approaches']]
    fail = [f for a in allapps for f in a['failed']]
    print(f'Go-arounds (unsafe contact -> climb back to the {H_HOVER} m hover, retry): '
          f'{len(fail)} in {sum(a["go_arounds"] > 0 for a in allapps)} approaches '
          f'(max {max(a["go_arounds"] for a in allapps)} in one approach); '
          f'{sum(f[0] == "strike" for f in fail)} deck strikes, '
          f'{sum(f[0] == "touchdown" for f in fail)} hard touch-downs (>= eps_v), impact speed '
          f'{_fmt_rng([f[1] for f in fail])} m/s')
    open_dr = [dr for name in INSTANCES for s in STRATEGIES for x in res['runs3'][name][s]
               for dr in x['drones'] if dr.app is not None and dr.app['t_dock'] is not None
               and dr.app['t_contact'] is None]
    print(f'Approaches past d_dock that never docked: '
          f'{sum(dr.mode == "ditched" for dr in open_dr)} ended with an empty battery '
          f'({sum(dr.mode == "ditched" and dr.app["go_arounds"] > 0 for dr in open_dr)} of them '
          f'after a go-around; over the deck when the battery ran out: '
          f'{sum(getattr(dr, "ditch_over_deck", False) for dr in open_dr)}), '
          f'{sum(dr.mode != "ditched" for dr in open_dr)} still in progress at T_MAX')
    for name in INSTANCES:
        for s in STRATEGIES:
            R = res['runs3'][name][s]
            print(f'  {name:<9} {STRAT_LABEL[s]:<23} go-arounds per seed '
                  f'{[sum(a["go_arounds"] for a in x["approaches"]) for x in R]}')
    print(f'd_dock -> contact {_fmt_rng([a["t_contact"] - a["t_dock"] for a in apps], ".1f")} s; '
          f'tilt-gate hold {sum(a["tilt_hold"] for a in apps):.1f} s in total; glideslope samples '
          f'outside +-0.03: {gs_v}/{gs_n} ({gs_v / max(gs_n, 1) * 100:.1f} %), max |eps| '
          f'{max(a["gs_max"] for a in apps):.3f}; min deck clearance before the touch-down command '
          f'{min(a["min_clear"] for a in apps):.3f} m')
    slow = [a for a in apps if a['v_entry'] < 1.0]
    print(f'Landings entering the PD phase at < 1 m/s closing speed (after waiting beside a busy '
          f'deck): {len(slow)}, 15 m -> contact {_fmt_rng([a["t_contact"] - a["t_dock"] for a in slow], ".1f")} s; '
          f'others {_fmt_rng([a["t_contact"] - a["t_dock"] for a in apps if a["v_entry"] >= 1.0], ".1f")} s')
    vh = sum(1 for name in INSTANCES for s in STRATEGIES for x in res['runs3'][name][s]
             for dr in x['drones'] for i in range(1, len(dr.log_mode))
             if dr.log_mode[i] == 'vhold' and dr.log_mode[i - 1] != 'vhold')
    print(f'Times a drone had to wait beside a busy deck (vhold): {vh}')
    pc = sum(x['pad_contacts'] for name in INSTANCES for s in STRATEGIES
             for x in res['runs3'][name][s])
    print(f'Take-off / arrival steps where a downdraft held a drone on its pad: {pc}')
    all_runs = [x for name in INSTANCES for s in STRATEGIES for x in res['runs3'][name][s]] + \
        [x for name in INSTANCES for s in ('relay', 'rolling') for x in res['runs_ab'][name][s]]
    dl3 = sum(x['deck_lifts'] for name in INSTANCES for s in STRATEGIES
              for x in res['runs3'][name][s])
    dlab = sum(x['deck_lifts'] for name in INSTANCES for s in ('relay', 'rolling')
               for x in res['runs_ab'][name][s])
    print(f'Steps where the rising deck carried a drone up (take-off slower than the heave '
          f'plus a downdraft): {dl3} with the 3-D planner, {dlab} with the 2-D planner')
    n_over, n_bad, cmin = 0, 0, np.inf
    for x in all_runs:
        n, b, c = deck_clearance_posthoc(x)
        n_over, n_bad, cmin = n_over + n, n_bad + b, min(cmin, c)
    print(f'Post-hoc deck check (independent plane formula, every logged position of every '
          f'drone in all {len(all_runs)} mission runs): {n_over} samples over the deck, '
          f'{n_bad} below it (tolerance 1e-9 m), min clearance {max(cmin, 0.0):.3f} m')
    assert n_bad == 0


def deck_clearance_posthoc(x, scale=1.0):
    """Independent check of all logged positions of one mission run against the
    deck plane, written out from the card formulas (not via SeaState). Sample i
    is the position at t = (i + 1) DT. Returns (samples over the deck,
    samples below it, min clearance over the deck)."""
    n_over, n_bad, cmin = 0, 0, np.inf
    for dr in x['drones']:
        P = np.array(dr.log_pos)
        t = (np.arange(len(P)) + 1) * DT
        a = OMEGA_W * t
        heave = Z_REF + A_H * scale * np.sin(a + PH_H)
        th = THETA_P * scale * np.sin(a + PH_P)
        ph = PHI_R * scale * np.sin(a + PH_R)
        vxy = np.array([S2.vessel_position(ti) for ti in t])
        psi = np.array([vessel_heading(ti) for ti in t])
        c, s = np.cos(psi), np.sin(psi)
        # normal n = Rz Ry Rx e_z, ship axes ex = Rz Ry Rx e_x, ey = Rz Ry Rx e_y
        sx, sy = np.sin(th) * np.cos(ph), -np.sin(ph)
        n = np.stack([c * sx - s * sy, s * sx + c * sy, np.cos(th) * np.cos(ph)], 1)
        ex = np.stack([c * np.cos(th), s * np.cos(th), -np.sin(th)], 1)
        ey = np.stack([c * np.sin(th) * np.sin(ph) - s * np.cos(ph),
                       s * np.sin(th) * np.sin(ph) + c * np.cos(ph),
                       np.cos(th) * np.sin(ph)], 1)
        dx, dy = P[:, 0] - vxy[:, 0], P[:, 1] - vxy[:, 1]
        zs = heave - (n[:, 0] * dx + n[:, 1] * dy) / n[:, 2]
        d = np.stack([dx, dy, zs - heave], 1)
        over = (np.abs((d * ex).sum(1)) <= DECK_HALF[0]) & \
            (np.abs((d * ey).sum(1)) <= DECK_HALF[1])
        clear = P[over, 2] - zs[over]
        n_over += int(over.sum())
        n_bad += int((clear < -1e-9).sum())
        if clear.size:
            cmin = min(cmin, float(clear.min()))
    return n_over, n_bad, cmin


# ── Plots ───────────────────────────────────────────────────────────────────
Z_ASPECT = (1.0, 0.85, 0.35)


def _base_3d(ax):
    ax.plot(VESSEL_ROUTE[:, 0], VESSEL_ROUTE[:, 1], np.zeros(len(VESSEL_ROUTE)), '--',
            color='gray', lw=1.0)
    for m, (px, py) in enumerate(PLATFORMS):
        ax.plot([px, px], [py, py], [0, Z_PLATFORM], color=PLAT_COLORS[m], lw=4)
        ax.scatter([px], [py], [Z_PLATFORM], s=50, marker='s', color=PLAT_COLORS[m])
        ax.text(px, py, Z_PLATFORM + 5, f'P{m + 1}', fontsize=8, fontweight='bold')
    ax.set_xlim(-200, 6300)
    ax.set_ylim(-200, 5200)
    ax.set_zlim(0, 35)
    ax.set_box_aspect(Z_ASPECT)
    ax.set_xlabel('X (m)', fontsize=8)
    ax.set_ylabel('Y (m)', fontsize=8)
    ax.set_zlabel('Z (m)', fontsize=8)
    ax.tick_params(labelsize=7)
    ax.view_init(elev=28, azim=-60)


def plot_sea_area_3d(res, out_dir):
    runs = res['runs3'][SHOW_INSTANCE]
    fig = plt.figure(figsize=(21, 7.5))
    for i, s in enumerate(STRATEGIES):
        ax = fig.add_subplot(1, 3, i + 1, projection='3d')
        _base_3d(ax)
        x = runs[s][SHOW_SEED]
        dl = x['deck_log']
        ax.plot(dl[:, 0], dl[:, 1], dl[:, 2], '-', color='steelblue', lw=1.0, alpha=0.8)
        for dr in x['drones']:
            tr = np.array(dr.log_pos)
            ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=DRONE_COLORS[dr.k], lw=1.1)
        for (t, k, kind, info) in x['events']:
            p = x['drones'][k].log_pos[min(int(round(t / DT)), len(x['drones'][k].log_pos) - 1)]
            if kind == 'swap':
                ax.scatter(*p, s=140, marker='*', color=DRONE_COLORS[k], edgecolors='black')
            elif kind == 'abort':
                ax.scatter(*p, s=100, marker='X', color=DRONE_COLORS[k], edgecolors='black')
            elif kind == 'ditched':
                ax.scatter(*p, s=120, marker='v', color='black', edgecolors=DRONE_COLORS[k])
        st = x['status']
        txt = ', '.join(f'R{r + 1}:{st[r]}' for r in range(len(x['reqs']))).replace('R5', '\nR5')
        ax.set_title(f'{STRAT_LABEL[s]} (seed {SHOW_SEED})\n{txt}', fontsize=9)
    h = [Line2D([], [], color=DRONE_COLORS[k], lw=2) for k in range(N_DRONES)] + \
        [Line2D([], [], color='steelblue', lw=2), Line2D([], [], color='gray', ls='--'),
         Line2D([], [], marker='*', ls='', color='gray', mec='black', ms=12),
         Line2D([], [], marker='X', ls='', color='gray', mec='black', ms=10),
         Line2D([], [], marker='v', ls='', color='black', ms=10)]
    lab = [f'Drone {k + 1}' for k in range(N_DRONES)] + \
        ['Vessel deck (heaving)', 'Vessel route', 'Battery swap', 'Request aborted', 'Ditched']
    fig.legend(h, lab, loc='lower center', ncol=10, fontsize=9)
    fig.suptitle(f'S039 3D Offshore Platform Exchange — {SHOW_INSTANCE} instance, 3-D planner '
                 '(vertical axis exaggerated)', fontsize=13, fontweight='bold')
    plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    path = os.path.join(out_dir, 'sea_area_3d.png')
    plt.savefig(path, dpi=110)
    plt.close()
    print(f'Saved: {path}')


def _first_landing(res):
    for s in ('relay', 'rolling', 'greedy'):
        x = res['runs3'][SHOW_INSTANCE][s][SHOW_SEED]
        for dr in x['drones']:
            for a in dr.approaches:
                if a['t_contact'] is not None:
                    return s, x, dr, a
    return None


def plot_altitude_profiles(res, out_dir):
    x = res['runs3'][SHOW_INSTANCE]['rolling'][SHOW_SEED]
    t = np.arange(len(x['drones'][0].log_pos)) * DT
    fig, axes = plt.subplots(2, 1, figsize=(13, 8.5))
    ax = axes[0]
    for dr in x['drones']:
        ax.plot(t, np.array(dr.log_pos)[:, 2], color=DRONE_COLORS[dr.k], lw=1.2,
                label=f'Drone {dr.k + 1}')
    ax.plot(t, x['deck_log'][:, 2], color='steelblue', lw=0.6, label='Vessel deck $z_{deck}(t)$')
    for zz, lab in ((Z_CRUISE, 'cruise 30 m'), (Z_HOLD, 'platform hold 13 m'),
                    (Z_PLATFORM, 'platform pad 3 m')):
        ax.axhline(zz, color='gray', ls=':', lw=0.8)
        ax.text(T_MAX + 3, zz, lab, fontsize=7, va='center')
    ax.set_xlim(0, T_MAX)
    ax.set_ylim(0, 34)
    ax.set_ylabel('Altitude (m)')
    ax.set_title(f'Rolling-horizon, {SHOW_INSTANCE} instance, seed {SHOW_SEED}: cruise, '
                 'ramp-down, pad work, climb-out', fontsize=10)
    ax.legend(fontsize=8, ncol=5, loc='lower center')
    ax.grid(True, alpha=0.3)
    ax = axes[1]
    fl = _first_landing(res)
    if fl is not None:
        s, xs, dr, a = fl
        i0 = int(round((a['t_contact'] - 60.0) / DT))
        i1 = min(int(round((a['t_contact'] + 8.0) / DT)), len(dr.log_pos) - 1)
        tt = np.arange(i0, i1 + 1) * DT
        zz = np.array(dr.log_pos)[i0:i1 + 1, 2]
        ax.plot(tt, zz, color=DRONE_COLORS[dr.k], lw=1.6, label=f'Drone {dr.k + 1}')
        ax.plot(tt, xs['deck_log'][i0:i1 + 1, 2], color='steelblue', lw=1.2, label='Deck $z_{deck}$')
        for key, lab, c in (('t_gs', 'glideslope capture (178 m)', 'green'),
                            ('t_dock', 'd_dock = 15 m: PD phase', 'orange'),
                            ('t_gate', 'gates pass: touch-down', 'purple'),
                            ('t_contact', 'deck contact', 'black')):
            if a[key] is not None and a[key] >= tt[0]:
                ax.axvline(a[key], color=c, ls='--', lw=1, label=lab)
        ax.set_title(f'Zoom: first vessel landing in the {SHOW_INSTANCE} runs '
                     f'({STRAT_LABEL[s]}, drone {dr.k + 1}, seed {SHOW_SEED})', fontsize=10)
        ax.legend(fontsize=8, ncol=3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_profiles.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_heave_tilt(res, out_dir):
    fl = _first_landing(res)
    tc = fl[3]['t_contact'] if fl else 300.0
    t = np.arange(tc - 40.0, tc + 10.0, 0.05)
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    m = np.array([SEA_CARD.motion(x) for x in t])
    axes[0].plot(t, m[:, 0], color='steelblue')
    axes[0].set_ylabel('$z_{heave}$ (m)')
    axes[0].set_title(f'Card sea state ($A_h$ = {A_H} m, $T_w$ = {T_W:.0f} s); '
                      'vertical line = first mission deck contact', fontsize=10)
    axes[1].plot(t, np.degrees(m[:, 1]), label=r'pitch $\theta_p$')
    axes[1].plot(t, np.degrees(m[:, 2]), label=r'roll $\phi_r$')
    axes[1].plot(t, np.degrees([SEA_CARD.tilt(x) for x in t]), 'k', lw=1.6,
                 label=r'$|\phi_{combined}|$')
    axes[1].axhline(math.degrees(PHI_MAX), color='red', ls='--', label=r'$\phi_{max}$ = 5°')
    axes[1].set_ylabel('Angle (deg)')
    axes[1].set_title(r'Card amplitudes: $|\phi_{combined}| \leq \sqrt{3^2+4^2}$ = 5° = '
                      r'$\phi_{max}$, so the gate never closes', fontsize=10)
    axes[1].legend(fontsize=8, ncol=4, loc='lower left')
    for s, c in ((2.0, 'darkorange'), (3.0, 'crimson')):
        tilt = np.degrees([SeaState(s).tilt(x) for x in t])
        axes[2].plot(t, tilt, color=c, label=f'sea scale {s:.0f}x')
        axes[2].fill_between(t, 0, tilt.max(), where=tilt >= math.degrees(PHI_MAX), color=c,
                             alpha=0.12, step='mid')
    axes[2].axhline(math.degrees(PHI_MAX), color='red', ls='--')
    axes[2].set_ylabel(r'$|\phi_{combined}|$ (deg)')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_title('Escalated sea states (card extension 1): shaded = gate closed, drone '
                      'must hold', fontsize=10)
    axes[2].legend(fontsize=8)
    for ax in axes:
        ax.axvline(tc, color='black', lw=0.8)
        ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'heave_tilt.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_deck_approach(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    dd = np.linspace(0, D_GS, 200)
    for ax, dmax in zip(axes, (D_GS + 10, 30.0)):
        ax.fill_between(dd, dd * (TAN_GS - EPS_GS_MAX), dd * (TAN_GS + EPS_GS_MAX), color='green',
                        alpha=0.15, label=r'glideslope 8° $\pm$ 0.03 d')
        ax.plot(dd, dd * TAN_GS, 'g--', lw=1)
        ax.axhline(H_HOVER, color='gray', ls=':', lw=1)
        ax.axvline(D_DOCK, color='orange', ls=':', lw=1)
        ax.set_xlim(0, dmax)
    # mission landings (true deck height, card law)
    first = True
    for name in INSTANCES:
        for s in STRATEGIES:
            for x in res['runs3'][name][s]:
                for dr in x['drones']:
                    lp = np.array(dr.log_pos)
                    for a in dr.approaches:
                        if a['t_gs'] is None or a['t_contact'] is None:
                            continue
                        i0 = int(round(a['t_gs'] / DT))
                        i1 = int(round(a['t_contact'] / DT))
                        dl = x['deck_log'][i0:i1 + 1]
                        d = np.hypot(lp[i0:i1 + 1, 0] - dl[:, 0], lp[i0:i1 + 1, 1] - dl[:, 1])
                        h = lp[i0:i1 + 1, 2] - dl[:, 2]
                        axes[0].plot(d, h, color='red', lw=0.6, alpha=0.5,
                                     label='mission landings (card law)' if first else None)
                        first = False
    axes[0].set_title('All 3-D mission vessel landings: height above the heaving deck vs range',
                      fontsize=10)
    for law, c in (('card', 'red'), ('level', 'blue')):
        for i, r in enumerate(res['study'][(law, 1.0)][:8]):
            tr = [(d, h) for (_, d, h, mode) in r['track'] if d <= 30.0]
            if not tr:
                continue
            tr = np.array(tr)
            axes[1].plot(tr[:, 0], tr[:, 1], color=c, lw=0.9, alpha=0.7,
                         label=LAW_LABEL[law] if i == 0 else None)
            if r['contact'] == 'strike':
                axes[1].scatter(tr[-1, 0], tr[-1, 1], marker='x', color='black', zorder=5,
                                label='deck strike' if law == 'level' and i == 0 else None)
    axes[1].set_title('Study, card sea state, first 8 seeds: last 30 m (true deck height)',
                      fontsize=10)
    axes[1].set_ylim(-1.2, 5.0)
    axes[1].axhline(0, color='steelblue', lw=1)
    for ax in axes:
        ax.set_xlabel('Horizontal range to deck centre $d_{horiz}$ (m)')
        ax.set_ylabel('Height above deck reference point (m)')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc='upper left')
    plt.tight_layout()
    path = os.path.join(out_dir, 'deck_approach.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_deck_convergence(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for name in INSTANCES:
        for s in STRATEGIES:
            for x in res['runs3'][name][s]:
                for a in x['approaches']:
                    if not a['samples'] or a['t_dock'] is None:
                        continue
                    sm = np.array(a['samples'])
                    tt = sm[:, 0] - a['t_dock']
                    axes[0].plot(tt, sm[:, 1], color='red', lw=0.6, alpha=0.5)
                    axes[1].plot(tt, sm[:, 2], color='red', lw=0.6, alpha=0.5)
                    if a['t_gate'] is not None:
                        axes[0].scatter(a['t_gate'] - a['t_dock'], EPS_P, s=8, color='purple')
    axes[0].axhline(EPS_P, color='black', ls='--', label=r'$\epsilon_p$ = 1.0 m')
    axes[1].axhline(EPS_V, color='black', ls='--', label=r'$\epsilon_v$ = 0.5 m/s')
    axes[0].set_ylabel(r'$\|\Delta\mathbf{r}\|$ to the true deck point (m)')
    axes[1].set_ylabel(r'$\|\Delta\mathbf{v}\|$ to the true deck (m/s)')
    axes[0].set_title('Relative distance (dots = all gates pass)', fontsize=10)
    axes[1].set_title('Relative velocity', fontsize=10)
    for ax in axes:
        ax.set_yscale('log')
        ax.set_xlabel('Time since d_dock = 15 m (s)')
        ax.grid(True, alpha=0.3, which='both')
        ax.legend(fontsize=8)
    fig.suptitle('All 3-D mission vessel landings (card law, card sea state)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'deck_convergence.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_sea_state_sweep(res, out_dir):
    st = res['study']
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.6))
    cols = {'card': 'red', 'level': 'blue'}
    for law in LAWS:
        R = [st[(law, s)] for s in SEA_SCALES]
        axes[0].plot(SEA_SCALES, [sum(r['safe'] for r in x) for x in R], 'o-', color=cols[law],
                     label=LAW_LABEL[law])
        axes[1].plot(SEA_SCALES, [sum(r['contact'] == 'strike' for r in x) for x in R], 'o-',
                     color=cols[law])
        axes[2].plot(SEA_SCALES, [np.mean([r['tilt_hold'] for r in x]) for x in R], 'o-',
                     color=cols[law])
        axes[3].plot(SEA_SCALES, [np.mean([r['gs_frac'] for r in x]) * 100 for x in R], 'o-',
                     color=cols[law])
    d = st[('direct', 1.0)]
    axes[0].plot([1.0], [sum(r['safe'] for r in d)], 's', color='black', ms=8,
                 label='card PD straight on the deck point')
    axes[1].plot([1.0], [sum(r['contact'] == 'strike' for r in d)], 's', color='black', ms=8)
    axes[2].plot([1.0], [np.mean([r['tilt_hold'] for r in d])], 's', color='black', ms=8)
    axes[3].plot([1.0], [np.mean([r['gs_frac'] for r in d]) * 100], 's', color='black', ms=8)
    titles = [f'Safe touch-downs (of {N_TRIALS})', f'Deck strikes (of {N_TRIALS})',
              'Mean tilt-gate hold (s)', 'Glideslope samples outside ±0.03 (%)']
    for ax, ti in zip(axes, titles):
        ax.set_title(ti, fontsize=10)
        ax.set_xlabel('Sea-state multiplier on $A_h$, $\\Theta_p$, $\\Phi_r$')
        ax.grid(True, alpha=0.3)
    axes[0].set_ylim(-1, N_TRIALS + 1)
    axes[1].set_ylim(-1, N_TRIALS + 1)
    axes[0].legend(fontsize=7, loc='lower left')
    fig.suptitle(f'S039 3D deck-approach study: {N_TRIALS} seeded approaches per point '
                 '(same start states and gusts for every law)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'sea_state_sweep.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_battery_soc(res, out_dir):
    runs = res['runs3'][SHOW_INSTANCE]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    t = np.arange(len(runs['greedy'][SHOW_SEED]['drones'][0].log_soc)) * DT
    ls = {'greedy': '-', 'relay': '--', 'rolling': ':'}
    for k, ax in enumerate(axes.flatten()):
        for s in STRATEGIES:
            dr = runs[s][SHOW_SEED]['drones'][k]
            ax.plot(t, np.array(dr.log_soc) * 100, ls[s], color=DRONE_COLORS[k], lw=1.6,
                    label=f'{STRAT_LABEL[s]} ({(dr.e_h + dr.e_v) / 1000:.1f} kJ, of which '
                          f'vertical {dr.e_v / 1000:.2f}; {dr.swaps} swaps)')
        ax.axhline(D_SAFETY * 100, color='red', ls='--', lw=1)
        ax.axhspan(0, D_SAFETY * 100, color='red', alpha=0.08)
        ax.set_ylim(0, 105)
        ax.set_title(f'Drone {k + 1}', fontsize=10)
        ax.set_ylabel('SoC (%)')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, loc='lower left')
    for ax in axes[1]:
        ax.set_xlabel('Time (s)')
    fig.suptitle(f'S039 3D Battery SoC ({SHOW_INSTANCE} instance, seed {SHOW_SEED}, 3-D planner; '
                 'red band = 15 % reserve)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'battery_soc.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


MODE_COL = {'fly': '#4c72b0', 'descend': '#4c72b0', 'vessel': '#55a868', 'final': '#2ca02c',
            'touch': '#2ca02c', 'goaround': '#2ca02c', 'vhold': '#c44e52', 'hold': '#c44e52', 'reland': '#c44e52',
            'docked': '#ffd92f', 'handle': '#8172b2', 'wait': '#bbbbbb', 'ditched': '#000000'}


def plot_platform_gantt(res, out_dir):
    runs = res['runs3'][SHOW_INSTANCE]
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for ax, s in zip(axes, STRATEGIES):
        x = runs[s][SHOW_SEED]
        for dr in x['drones']:
            modes = dr.log_mode
            start = 0
            for i in range(1, len(modes) + 1):
                if i == len(modes) or modes[i] != modes[start]:
                    m = modes[start]
                    if m in MODE_COL:
                        ax.barh(dr.k, (i - start) * DT, left=start * DT, height=0.6,
                                color=MODE_COL[m], edgecolor='none')
                    start = i
        for (m, k, t0, t1, kind) in x['pad_log']:
            ax.text((t0 + t1) / 2, k + 0.36, f'P{m + 1}', ha='center', fontsize=6)
        for (t, k, kind, info) in x['events']:
            if kind == 'hold':
                ax.text(t, k - 0.45, f'hold P{info + 1} @ {Z_HOLD:.0f} m', fontsize=6,
                        color='#c44e52')
        for r, (o, d, w, dl) in enumerate(x['reqs']):
            ax.axvline(dl, color='black', lw=0.6, ls=':')
            ax.text(dl, N_DRONES - 0.45, f'dl R{r + 1}', fontsize=7, rotation=90, va='top')
        ax.set_yticks(range(N_DRONES))
        ax.set_yticklabels([f'Drone {k + 1}' for k in range(N_DRONES)])
        ax.set_xlim(0, T_MAX)
        ax.set_ylim(-0.7, N_DRONES - 0.3)
        ax.set_title(f'{STRAT_LABEL[s]}: delivered {x["delivered"]}/{len(x["reqs"])} '
                     f'({x["late"]} late), energy {x["energy_kJ"]:.1f} kJ, {x["swaps"]} swaps, '
                     f'pad hold {x["hold_time"]:.1f} s', fontsize=10)
        ax.grid(True, axis='x', alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    handles = [mpatches.Patch(color=c, label=l) for l, c in
               (('Flying to / descending onto a platform', '#4c72b0'),
                ('Pursuing the vessel (cruise + glideslope)', '#55a868'),
                ('PD deck approach + touch-down', '#2ca02c'),
                ('Holding (pad 13 m orbit / deck busy)', '#c44e52'),
                ('Docked: battery swap', '#ffd92f'), ('On pad: load / unload', '#8172b2'),
                ('Waiting on platform', '#bbbbbb'), ('Ditched', '#000000'))]
    fig.legend(handles=handles, loc='lower center', ncol=4, fontsize=8)
    fig.suptitle(f'S039 3D Drone Activity and Pad Use ({SHOW_INSTANCE} instance, seed {SHOW_SEED}, '
                 '3-D planner; white = parked)', fontsize=12)
    plt.tight_layout(rect=[0, 0.07, 1, 0.96])
    path = os.path.join(out_dir, 'platform_gantt.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_energy_comparison(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    for ax, name in zip(axes, INSTANCES):
        labels, eh, ev, e2, dlv = [], [], [], [], []
        for s in STRATEGIES:
            R = res['runs3'][name][s]
            labels.append(f'{STRAT_LABEL[s]}\n3-D planner')
            eh.append(np.mean([x['e_h_kJ'] for x in R]))
            ev.append(np.mean([x['e_v_kJ'] for x in R]))
            e2.append(np.mean([x['energy_kJ'] for x in res['runs2'][name][s]]))
            dlv.append((np.mean([x['delivered'] for x in R]),
                        np.mean([x['delivered'] for x in res['runs2'][name][s]])))
        for s in ('relay', 'rolling'):
            R = res['runs_ab'][name][s]
            labels.append(f'{STRAT_LABEL[s]}\n2-D planner')
            eh.append(np.mean([x['e_h_kJ'] for x in R]))
            ev.append(np.mean([x['e_v_kJ'] for x in R]))
            e2.append(np.mean([x['energy_kJ'] for x in res['runs2'][name][s]]))
            dlv.append((np.mean([x['delivered'] for x in R]),
                        np.mean([x['delivered'] for x in res['runs2'][name][s]])))
        xx = np.arange(len(labels))
        ax.bar(xx, eh, color='#4c72b0', edgecolor='black', label='3-D: horizontal power')
        ax.bar(xx, ev, bottom=eh, color='#dd8452', edgecolor='black',
               label=r'3-D: vertical power $k_{vert} v_z^2$')
        ax.scatter(xx, e2, marker='D', color='black', zorder=5, label='2-D script (same seeds)')
        for i in range(len(xx)):
            ax.text(xx[i], eh[i] + ev[i] + 3, f'{eh[i] + ev[i]:.1f} kJ\nv {ev[i]:.2f}\n'
                    f'deliv {dlv[i][0]:.1f} (2-D {dlv[i][1]:.1f})', ha='center', fontsize=7)
        ax.set_xticks(xx)
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_ylabel(f'Fleet energy (kJ, mean of {N_SEEDS} seeds)')
        ax.set_ylim(0, max(np.array(eh) + np.array(ev)) * 1.3)
        ax.set_title(f'{name} instance ({len(INSTANCES[name])} requests)', fontsize=10)
        ax.grid(True, axis='y', alpha=0.3)
        ax.legend(fontsize=8, loc='upper right')
    fig.suptitle('S039 3D fleet energy: horizontal + vertical components', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'energy_comparison.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    runs = res['runs3'][SHOW_INSTANCE]
    n = len(runs['greedy'][SHOW_SEED]['drones'][0].log_pos)
    n_frames = math.ceil((n - 1) / GIF_STEP) + 1
    idx = [min(i * GIF_STEP, n - 1) for i in range(n_frames)]
    fig = plt.figure(figsize=(15, 5.4))
    art = []
    for i, s in enumerate(STRATEGIES):
        ax = fig.add_subplot(1, 3, i + 1, projection='3d')
        _base_3d(ax)
        ax.set_title(STRAT_LABEL[s], fontsize=10)
        x = runs[s][SHOW_SEED]
        pos = [np.array(dr.log_pos) for dr in x['drones']]
        dots = [ax.plot([], [], [], 'o', color=DRONE_COLORS[k], ms=5)[0] for k in range(N_DRONES)]
        trails = [ax.plot([], [], [], '-', color=DRONE_COLORS[k], lw=0.8, alpha=0.6)[0]
                  for k in range(N_DRONES)]
        ves = ax.plot([], [], [], 's', color='steelblue', ms=7)[0]
        art.append((x, pos, dots, trails, ves))
    clock = fig.suptitle('')

    def update(fi):
        i = idx[fi]
        out = []
        for x, pos, dots, trails, ves in art:
            vp = x['deck_log'][i]
            ves.set_data([vp[0]], [vp[1]])
            ves.set_3d_properties([vp[2]])
            for k in range(N_DRONES):
                tr = pos[k][:i + 1]
                dots[k].set_data([tr[-1, 0]], [tr[-1, 1]])
                dots[k].set_3d_properties([tr[-1, 2]])
                trails[k].set_data(tr[:, 0], tr[:, 1])
                trails[k].set_3d_properties(tr[:, 2])
            out += dots + trails + [ves]
        clock.set_text(f'S039 3D Offshore Platform Exchange — t = {i * DT:.0f} s '
                       f'({SHOW_INSTANCE} instance, seed {SHOW_SEED}; square = vessel deck)')
        return out + [clock]

    ani = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    plt.tight_layout()
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=15, dpi=72)
    plt.close()
    print(f'Saved: {path} ({n_frames} frames)')


# ── Main ────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print_model_checks()
    res = run_simulation()
    print_calibration(res['calib'])
    print_feasibility()
    print_study(res['study'])
    for name in INSTANCES:
        print_mission(name, res)
    print_mission_landings(res)

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plot_sea_area_3d(res, OUTPUT_DIR)
    plot_altitude_profiles(res, OUTPUT_DIR)
    plot_heave_tilt(res, OUTPUT_DIR)
    plot_deck_approach(res, OUTPUT_DIR)
    plot_deck_convergence(res, OUTPUT_DIR)
    plot_sea_state_sweep(res, OUTPUT_DIR)
    plot_battery_soc(res, OUTPUT_DIR)
    plot_platform_gantt(res, OUTPUT_DIR)
    plot_energy_comparison(res, OUTPUT_DIR)
    save_animation(res, OUTPUT_DIR)
