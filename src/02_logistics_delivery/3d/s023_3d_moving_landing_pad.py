"""
S023 3D Moving Landing Pad
==========================
A delivery drone leaves a shore station and lands on a vessel that sails at
4 m/s along +x while its deck heaves, pitches and rolls.  Guidance has three
latched phases:

  Phase 1 - 3-D proportional navigation (N = 3) plus an along-LOS speed hold
            (the fix from the 2-D S023) toward the predicted ramp-entry point
            astern of the pad at z_aim.
  Phase 2 - constrained approach ramp: a straight line from the entry point to
            the pad, expressed in the pad frame, so it rides the heaving deck.
  Phase 3 - flare onto the deck.  Four vertical laws are compared on the same
            sea-phase seeds (Phases 1-2 are shared and bit-identical):
              static2d  - 2-D S023 flare: tanh law on height above the MEAN deck
              rel_noff  - tanh law on height above the LIVE pad, no z_p-dot FF
              rel_ff    - as rel_noff plus the z_p-dot feed-forward (card law)
              rel_gate  - rel_ff, but the flare starts only when the motion
                          window (heave near crest/trough, low heave rate) opens

Pad telemetry arrives with a 2-step (0.04 s) delay; every strategy uses the same
constant-velocity latency compensation.  Contact is detected against the tilted
deck plane under the drone.  A Monte-Carlo over 100 random sea phases is run for
the card's heave amplitude (0.6 m) and two larger sea states.

Outputs (outputs/02_logistics_delivery/3d/s023_3d_moving_landing_pad/):
  trajectory_3d.png, altitude_tracking.png, deck_motion.png,
  relative_velocity.png, monte_carlo.png, animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s023_3d_moving_landing_pad.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D            # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation, PillowWriter

# ── Output directory ──────────────────────────────────────────────────────────
OUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's023_3d_moving_landing_pad'))
os.makedirs(OUT_DIR, exist_ok=True)

# ── Card parameters ───────────────────────────────────────────────────────────
DT          = 0.02                 # s   (50 Hz)
V_V         = 4.0                  # m/s vessel speed along +x (heading psi_V = 0)
A_H         = 0.6                  # m   heave amplitude
F_H         = 0.10                 # Hz  heave frequency
A_THETA     = math.radians(4.0)    # rad pitch amplitude
A_PHI       = math.radians(6.0)    # rad roll amplitude
F_THETA     = 0.08                 # Hz  pitch frequency (card range 0.08-0.12 Hz)
F_ROLL      = 0.12                 # Hz  roll frequency  (card range 0.08-0.12 Hz)
V_MAX       = 8.0                  # m/s drone max speed
A_MAX       = 5.0                  # m/s^2 drone max acceleration
N_PNG       = 3                    # PN navigation constant
GAMMA_MIN   = math.radians(6.0)    # rad minimum ramp descent angle
Z_RAMP      = 8.0                  # m   Phase 1->2 altitude
R_APPROACH  = 5.0                  # m   Phase 1->2 horizontal radius
Z_FLARE_REL = 2.5                  # m   Phase 2->3 height above live pad (also tanh scale)
DELTA_HEAVE = 0.2                  # m   motion-window heave gate
ZDOT_GATE   = 0.3                  # m/s motion-window heave-rate gate
R_PAD       = 0.5                  # m   pad radius
V_LAND_MAX  = 0.8                  # m/s max relative touchdown vertical speed (and flare rate)
DV_MAX      = 0.5                  # m/s max horizontal velocity mismatch
LAT_STEPS   = 2                    # telemetry delay in steps (tau_s = 0.04 s)
TAU_S       = LAT_STEPS * DT
Z_LIM       = (0.0, 20.0)          # m   altitude range

# ── Added parameters (not in the card; see README) ────────────────────────────
Z_DECK_MEAN = 2.0      # m   mean deck height above sea level (keeps the pad centre above z = 0)
DECK_HALF_L = 6.0      # m   deck half-length (body x)
DECK_HALF_W = 3.0      # m   deck half-width  (body y)
P_DRONE0    = np.array([-20.0, 8.0, 15.0])   # shore-station start (2-D S023 start), at rest
X_VESSEL0   = 0.0      # m
D_ENTRY     = 4.0      # m   Phase-1 aim point: this far astern of the predicted pad
Z_AIM       = 7.5      # m   Phase-1 aim altitude (just below Z_RAMP so the switch triggers)
V_APP       = 1.5      # m/s Phase-2 closing speed along the ramp
R_FLARE     = 2.5      # m   Phase 2->3 horizontal radius
Z_HOLD      = 2.5      # m   gated hold height above the MEAN deck
KP, KV      = 1.2, 1.8 # lateral / ramp PD gains (2-D S023 values)
K_ZR        = 2.0      # 1/s vertical rate-loop gain (2-D S023 KZ_RATE)
K_AX        = 1.0      # 1/s Phase-1 along-LOS speed hold (2-D S023 fix)
PRED_TOL    = 1e-6     # s   intercept-predictor tolerance (2-D S023 fix)
PRED_MAX_IT = 100
Z_TOUCHDOWN = 0.05     # m   contact when height above deck surface <= this
SIM_TIME    = 60.0     # s   budget
SEED        = 0        # showcase sea-phase seed
N_MC        = 100      # sea-phase seeds per sea state
A_H_SWEEP   = [0.6, 1.2, 1.8]   # m heave amplitudes (0.6 = card)

STRATS = ["static2d", "rel_noff", "rel_ff", "rel_gate"]
LABELS = {"static2d": "2-D flare (mean deck)",
          "rel_noff": "Deck-relative, no FF",
          "rel_ff":   "Deck-relative + zp-dot FF",
          "rel_gate": "Deck-relative + FF + gate"}
COLORS = {"static2d": "#7f7f7f", "rel_noff": "#1f77b4",
          "rel_ff": "#d62728", "rel_gate": "#9467bd"}

W_H, W_TH, W_R = 2 * math.pi * F_H, 2 * math.pi * F_THETA, 2 * math.pi * F_ROLL


# ── Sea / deck model ──────────────────────────────────────────────────────────

def sea_phases(seed):
    """Random initial phases (heave, pitch, roll), shared by every strategy."""
    rng = np.random.default_rng(seed)
    return rng.uniform(0.0, 2 * math.pi, size=3)


def deck_state(t, ph, a_h):
    """Pad-centre state.  The pad sits at the vessel's rotation centre
    (d_offset = 0), so pitch and roll tilt the deck but do not move the pad.
    Returns dict with pad position, velocity, heave zp, zp_dot, pitch, roll and
    their rates."""
    s_h = W_H * t + ph[0]
    s_t = W_TH * t + ph[1]
    s_r = W_R * t + ph[2]
    zp = a_h * math.sin(s_h)
    zpd = a_h * W_H * math.cos(s_h)
    return dict(x=X_VESSEL0 + V_V * t, y=0.0, z=Z_DECK_MEAN + zp,
                vx=V_V, vy=0.0, vz=zpd, zp=zp, zpd=zpd,
                th=A_THETA * math.sin(s_t), thd=A_THETA * W_TH * math.cos(s_t),
                ro=A_PHI * math.sin(s_r), rod=A_PHI * W_R * math.cos(s_r))


def deck_normal(d):
    n = np.array([-d["th"], d["ro"], 1.0])
    return n / np.linalg.norm(n)


def surface_height(d, x, y):
    """Height of the deck plane (normal ~ [-theta, phi, 1]) at (x, y); sea level
    (0) outside the deck footprint."""
    dx, dy = x - d["x"], y - d["y"]
    if abs(dx) <= DECK_HALF_L and abs(dy) <= DECK_HALF_W:
        return d["z"] + d["th"] * dx - d["ro"] * dy, True
    return 0.0, False


def surface_point_vz(d, x, y):
    """Vertical velocity of the deck material point under (x, y)."""
    dx, dy = x - d["x"], y - d["y"]
    return d["vz"] + d["thd"] * dx - d["rod"] * dy


def gate_open(zp, zpd, a_h):
    """Motion window (corrected card gate): heave within DELTA_HEAVE of a crest
    or trough AND |heave rate| <= ZDOT_GATE."""
    return abs(abs(zp) - a_h) <= DELTA_HEAVE and abs(zpd) <= ZDOT_GATE


def gate_open_literal(zp, zpd):
    """Card gate taken literally (|zp - mean| <= delta and |zp_dot| <= gate)."""
    return abs(zp) <= DELTA_HEAVE and abs(zpd) <= ZDOT_GATE


def measured_pad(t, ph, a_h, compensate=True):
    """Telemetry delayed by LAT_STEPS steps, optionally extrapolated forward by
    tau_s at constant velocity (latency compensation)."""
    d = deck_state(t - TAU_S, ph, a_h)
    k = TAU_S if compensate else 0.0
    p = np.array([d["x"] + d["vx"] * k, d["y"] + d["vy"] * k, d["z"] + d["vz"] * k])
    v = np.array([d["vx"], d["vy"], d["vz"]])
    return p, v, d["zp"] + d["zpd"] * k, d["zpd"]


# ── Phase 1: predictor + PN (from the fixed 2-D S023) ─────────────────────────

def predict_aim(p_d, p_pad, v_pad):
    """Fixed-point intercept prediction of the ramp-entry point
    (pad + v*dt_fly - D_ENTRY*x_hat, z = Z_AIM), iterated to PRED_TOL."""
    dx0, dy0 = p_pad[0] - D_ENTRY - p_d[0], p_pad[1] - p_d[1]
    dz = Z_AIM - p_d[2]
    dt_fly = math.hypot(dx0, dy0) / V_MAX
    for _ in range(PRED_MAX_IT):
        ax = dx0 + v_pad[0] * dt_fly
        ay = dy0 + v_pad[1] * dt_fly
        dt_new = math.sqrt(ax * ax + ay * ay + dz * dz) / V_MAX
        conv = abs(dt_new - dt_fly) < PRED_TOL
        dt_fly = dt_new
        if conv:
            break
    return np.array([p_pad[0] - D_ENTRY + v_pad[0] * dt_fly,
                     p_pad[1] + v_pad[1] * dt_fly, Z_AIM])


def png_acceleration(p_d, v_d, p_t, los_prev, r_prev):
    r = p_t - p_d
    rn = float(np.linalg.norm(r)) + 1e-9
    los = r / rn
    if los_prev is None:
        a_pn = np.zeros(3)
    else:
        vc = -(rn - r_prev) / DT
        a_pn = N_PNG * vc * (los - los_prev) / DT
        n = float(np.linalg.norm(a_pn))
        if n > A_MAX:
            a_pn *= A_MAX / n
    a_ax = K_AX * (V_MAX - float(np.dot(v_d, los)))
    budget = math.sqrt(max(A_MAX ** 2 - float(np.dot(a_pn, a_pn)), 0.0))
    a_ax = min(max(a_ax, -budget), budget)
    return a_pn + a_ax * los, los, rn


# ── Simulation ────────────────────────────────────────────────────────────────

def simulate(strategy, seed, a_h=A_H, record=False, compensate=True):
    ph = sea_phases(seed)
    p = P_DRONE0.astype(float).copy()
    v = np.zeros(3)
    phase, committed = 1, strategy != "rel_gate"
    los_prev, r_prev = None, None
    ramp = None                       # (r_entry, L_entry, t_entry, gamma)
    t3 = None
    t_commit = None
    hist = {k: [] for k in ("t", "p", "v", "pad", "vpad", "phase", "zp", "zpd",
                            "th", "ro", "gate", "committed")} if record else None
    n_steps = int(round(SIM_TIME / DT))
    td = None
    for k in range(n_steps + 1):
        t = k * DT
        d = deck_state(t, ph, a_h)
        pm, vm, zp_m, zpd_m = measured_pad(t, ph, a_h, compensate)
        g_now = gate_open(zp_m, zpd_m, a_h)

        # ── physical invariants and contact test on the current state ──
        z_s, on_deck = surface_height(d, p[0], p[1])
        h_s = p[2] - z_s
        assert h_s >= -1e-9, f"drone below deck/sea surface (h={h_s:.4f} m) at t={t:.2f}"
        assert Z_LIM[0] - 1e-9 <= p[2] <= Z_LIM[1] + 1e-9, "altitude out of card range"
        assert np.linalg.norm(v) <= V_MAX + 1e-9, "speed limit exceeded"
        if record:
            hist["t"].append(t); hist["p"].append(p.copy()); hist["v"].append(v.copy())
            hist["pad"].append([d["x"], d["y"], d["z"]]); hist["vpad"].append([d["vx"], d["vy"], d["vz"]])
            hist["phase"].append(phase); hist["zp"].append(d["zp"]); hist["zpd"].append(d["zpd"])
            hist["th"].append(d["th"]); hist["ro"].append(d["ro"])
            hist["gate"].append(gate_open(d["zp"], d["zpd"], a_h)); hist["committed"].append(committed)
        if h_s <= Z_TOUCHDOWN:
            n = deck_normal(d)
            miss = math.hypot(p[0] - d["x"], p[1] - d["y"])
            dvz = abs(v[2] - d["vz"])
            dvxy = math.hypot(v[0] - d["vx"], v[1] - d["vy"])
            v_pt = np.array([d["vx"], d["vy"], surface_point_vz(d, p[0], p[1])])
            td = dict(t=t, miss=miss, dvz=dvz, dvxy=dvxy, vz_abs=abs(v[2]),
                      on_deck=on_deck, phase=phase,
                      dvz_pt=abs(v[2] - v_pt[2]), vn=abs(float(np.dot(v - v_pt, n))),
                      tilt=math.degrees(math.acos(min(1.0, n[2]))),
                      dh_tilt=z_s - d["z"], zp=d["zp"], zpd=d["zpd"],
                      gate=gate_open(d["zp"], d["zpd"], a_h),
                      t3=t3, t_commit=t_commit)
            td["success"] = bool(on_deck and miss <= R_PAD and dvz <= V_LAND_MAX
                                 and dvxy <= DV_MAX)
            break
        if k == n_steps:
            break

        # ── phase logic (latched) ──
        d_xy = math.hypot(p[0] - pm[0], p[1] - pm[1])
        h_rel = p[2] - pm[2]
        if phase == 1 and d_xy <= R_APPROACH and p[2] <= Z_RAMP:
            phase = 2
            r_e = p - pm
            L_e = float(np.linalg.norm(r_e))
            gamma = math.atan2(r_e[2], math.hypot(r_e[0], r_e[1]))
            assert gamma >= GAMMA_MIN, "ramp shallower than gamma_min"
            ramp = (r_e, L_e, t, gamma)
        if phase == 2 and d_xy <= R_FLARE and h_rel <= Z_FLARE_REL:
            phase, t3 = 3, t
        if phase == 3 and not committed and g_now:
            committed, t_commit = True, t

        # ── control ──
        if phase == 1:
            aim = predict_aim(p, pm, vm)
            a, los_prev, r_prev = png_acceleration(p, v, aim, los_prev, r_prev)
        elif phase == 2:
            r_e, L_e, t_e, _ = ramp
            L = max(L_e - V_APP * (t - t_e), 0.0)
            r_cmd = r_e * (L / L_e)
            v_rel_cmd = -V_APP * r_e / L_e if L > 0 else np.zeros(3)
            a = KP * (r_cmd - (p - pm)) + KV * (v_rel_cmd - (v - vm))
        else:
            a = np.empty(3)
            a[0] = KP * (pm[0] - p[0]) + KV * (vm[0] - v[0])
            a[1] = KP * (pm[1] - p[1]) + KV * (vm[1] - v[1])
            if strategy == "static2d":
                zd_cmd = -V_LAND_MAX * math.tanh((p[2] - Z_DECK_MEAN) / Z_FLARE_REL)
            elif not committed:        # rel_gate hold above the mean deck
                zd_cmd = -V_LAND_MAX * math.tanh((p[2] - Z_DECK_MEAN - Z_HOLD) / Z_FLARE_REL)
            else:
                h = p[2] - pm[2]
                zd_cmd = -V_LAND_MAX * math.tanh(h / Z_FLARE_REL)
                if strategy in ("rel_ff", "rel_gate"):
                    zd_cmd += vm[2]                      # z_p-dot feed-forward
            a[2] = K_ZR * (zd_cmd - v[2])
        na = float(np.linalg.norm(a))
        if na > A_MAX:
            a *= A_MAX / na

        v = v + a * DT
        sp = float(np.linalg.norm(v))
        if sp > V_MAX:
            v *= V_MAX / sp
        p_new = p + v * DT
        # contact during the step: stop on the surface (no penetration)
        d_n = deck_state(t + DT, ph, a_h)
        z_sn, _ = surface_height(d_n, p_new[0], p_new[1])
        if p_new[2] < z_sn:
            p_new[2] = z_sn
        p = p_new

    if td is None:
        td = dict(t=None, success=False, miss=float("nan"), dvz=float("nan"),
                  dvxy=float("nan"), vz_abs=float("nan"), t3=t3, t_commit=t_commit,
                  on_deck=False, gate=False, phase=phase)
    td["ramp_gamma"] = math.degrees(ramp[3]) if ramp else float("nan")
    td["t2"] = ramp[2] if ramp else None
    if record:
        for kk in hist:
            hist[kk] = np.array(hist[kk])
        td["hist"] = hist
    return td


def run_simulation():
    """Showcase seed for all strategies + Monte-Carlo over sea states."""
    show = {s: simulate(s, SEED, A_H, record=True) for s in STRATS}
    mc = {}
    for a_h in A_H_SWEEP:
        mc[a_h] = {s: [simulate(s, sd, a_h) for sd in range(N_MC)] for s in STRATS}
        for sd in range(N_MC):          # Phases 1-2 are shared by construction
            t3s = {mc[a_h][s][sd]["t3"] for s in STRATS}
            assert len(t3s) == 1, "Phase-3 entry differs between strategies"
    nocomp = [simulate("rel_ff", sd, A_H, compensate=False) for sd in range(N_MC)]
    return show, mc, nocomp


# ── Reporting ─────────────────────────────────────────────────────────────────

def report(show, mc, nocomp):
    # literal vs corrected gate open fraction, card sea state
    ts = np.arange(0.0, 100.0, 0.001)
    zp = A_H * np.sin(W_H * ts)
    zpd = A_H * W_H * np.cos(W_H * ts)
    lit = np.mean((np.abs(zp) <= DELTA_HEAVE) & (np.abs(zpd) <= ZDOT_GATE))
    print("Motion-window gate check (card sea state, A_h = 0.6 m, f_h = 0.1 Hz):")
    print(f"  max |zp_dot| = {A_H * W_H:.3f} m/s; |zp_dot| where |zp| <= {DELTA_HEAVE} m "
          f">= {A_H * W_H * math.sqrt(1 - (DELTA_HEAVE / A_H) ** 2):.3f} m/s")
    print(f"  literal gate (|zp| <= {DELTA_HEAVE}, |zp_dot| <= {ZDOT_GATE}) open fraction: {lit * 100:.1f} %")
    for a_h in A_H_SWEEP:
        zp = a_h * np.sin(W_H * ts); zpd = a_h * W_H * np.cos(W_H * ts)
        cor = np.mean((np.abs(np.abs(zp) - a_h) <= DELTA_HEAVE) & (np.abs(zpd) <= ZDOT_GATE))
        print(f"  corrected gate open fraction, A_h = {a_h}: {cor * 100:.1f} %"
              f"  (max |zp_dot| = {a_h * W_H:.3f} m/s)")

    print(f"\nShowcase (sea-phase seed {SEED}, A_h = {A_H} m), phases = "
          f"{np.round(sea_phases(SEED), 3).tolist()} rad")
    r0 = show["rel_ff"]
    print(f"  Phase 1->2 at t = {r0['t2']:.2f} s, ramp angle gamma = {r0['ramp_gamma']:.1f} deg "
          f"(gamma_min = 6 deg)")
    print(f"  Phase 2->3 at t = {r0['t3']:.2f} s (identical for all strategies)")
    hdr = (f"  {'strategy':28s} {'t_td':>6s} {'miss':>6s} {'|dvz|rel':>8s} {'|vz|abs':>7s} "
           f"{'dvxy':>6s} {'v_n':>6s} {'zp':>6s} {'zp_dot':>7s} {'gate':>5s} {'result':>7s}")
    print(hdr)
    for s in STRATS:
        r = show[s]
        print(f"  {LABELS[s]:28s} {r['t']:6.2f} {r['miss']:6.3f} {r['dvz']:8.3f} {r['vz_abs']:7.3f} "
              f"{r['dvxy']:6.3f} {r['vn']:6.3f} {r['zp']:+6.2f} {r['zpd']:+7.3f} "
              f"{'open' if r['gate'] else 'shut':>5s} {'OK' if r['success'] else 'FAIL':>7s}")
    rg = show["rel_gate"]
    print(f"  rel_gate: flare committed at t = {rg['t_commit']:.2f} s "
          f"(held {rg['t_commit'] - rg['t3']:.2f} s)")
    print(f"  Deck tilt at touchdown (rel_ff): {r0['tilt']:.2f} deg; deck-plane height under the "
          f"drone minus pad-centre height: {r0['dh_tilt'] * 100:+.2f} cm; "
          f"|dvz| vs contact point {r0['dvz_pt']:.3f} m/s")

    print(f"\nMonte-Carlo: {N_MC} sea-phase seeds per sea state (same seeds for every strategy)")
    for a_h in A_H_SWEEP:
        print(f"  A_h = {a_h} m (max heave rate {a_h * W_H:.3f} m/s):")
        gam = [r["ramp_gamma"] for r in mc[a_h]["rel_ff"]]
        t3s = [r["t3"] for r in mc[a_h]["rel_ff"]]
        print(f"    shared Phases 1-2: ramp angle {min(gam):.1f}-{max(gam):.1f} deg "
              f"(gamma_min 6 deg never binds), Phase-3 entry t = {min(t3s):.2f}-{max(t3s):.2f} s")
        print(f"    {'strategy':28s} {'success':>8s} {'|dvz|rel mean+-std':>19s} {'max':>6s} "
              f"{'#>0.8':>5s} {'|vz|abs max':>11s} {'miss max':>8s} {'dvxy max':>8s} "
              f"{'t_td mean+-std':>15s} {'t_td max':>8s} {'gate@td':>7s}")
        for s in STRATS:
            rs = mc[a_h][s]
            ok = sum(r["success"] for r in rs)
            landed = [r for r in rs if r["t"] is not None]
            dvz = np.array([r["dvz"] for r in landed])
            tt = np.array([r["t"] for r in landed])
            print(f"    {LABELS[s]:28s} {ok:4d}/{N_MC:<3d} {dvz.mean():9.3f} +- {dvz.std():5.3f} "
                  f"{dvz.max():6.3f} {int(np.sum(dvz > V_LAND_MAX)):5d} "
                  f"{max(r['vz_abs'] for r in landed):11.3f} "
                  f"{max(r['miss'] for r in landed):8.3f} {max(r['dvxy'] for r in landed):8.3f} "
                  f"{tt.mean():7.2f} +- {tt.std():4.2f} {tt.max():8.2f} "
                  f"{sum(r['gate'] for r in landed):7d}")
            assert len(landed) == N_MC, "a run did not touch down"
        for s in ("static2d", "rel_noff"):
            hard = [r for r in mc[a_h][s] if r["dvz"] > V_LAND_MAX]
            if hard:
                print(f"    {LABELS[s]}: {len(hard)} hard contacts, deck RISING (zp_dot > 0) in "
                      f"{sum(r['zpd'] > 0 for r in hard)} of them")
        def wins(a, b, key):
            return sum(x[key] < y[key] for x, y in zip(mc[a_h][a], mc[a_h][b]))
        print(f"    head-to-head lower |dvz|rel: rel_ff < rel_noff {wins('rel_ff', 'rel_noff', 'dvz')}/{N_MC}, "
              f"rel_ff < static2d {wins('rel_ff', 'static2d', 'dvz')}/{N_MC}, "
              f"rel_noff < static2d {wins('rel_noff', 'static2d', 'dvz')}/{N_MC}, "
              f"rel_gate < rel_ff {wins('rel_gate', 'rel_ff', 'dvz')}/{N_MC}")
        print(f"    head-to-head earlier touchdown: rel_ff < rel_noff {wins('rel_ff', 'rel_noff', 't')}/{N_MC}, "
              f"rel_ff < static2d {wins('rel_ff', 'static2d', 't')}/{N_MC}, "
              f"rel_ff < rel_gate {wins('rel_ff', 'rel_gate', 't')}/{N_MC}")
        for s in STRATS:
            assert all(r["phase"] == 3 for r in mc[a_h][s]), "contact before Phase 3"
        rgs = mc[a_h]["rel_gate"]
        hold = np.array([r["t_commit"] - r["t3"] for r in rgs if r["t_commit"] is not None])
        unc = [r for r in rgs if r["t_commit"] is None]
        unc_txt = ""
        if unc:
            unc_txt = (f"; deck rose into the HOLDING drone (gate never opened) in {len(unc)}/{N_MC} runs: "
                       f"|dvz|rel max {max(r['dvz'] for r in unc):.3f} m/s, "
                       f"{sum(r['success'] for r in unc)} of them pass the criteria")
        print(f"    rel_gate hold before commit: mean {hold.mean():.2f} s, max {hold.max():.2f} s "
              f"over {len(hold)} committed runs{unc_txt}")
        print(f"    Phase-3 duration rel_ff mean "
              f"{np.mean([r['t'] - r['t3'] for r in mc[a_h]['rel_ff']]):.2f} s")
        rf = mc[a_h]["rel_ff"]
        print(f"    rel_ff tilt effects: max |deck-plane minus pad height| "
              f"{max(abs(r['dh_tilt']) for r in rf) * 100:.2f} cm, max | |dvz|pad - |dvz|contact pt | "
              f"{max(abs(r['dvz'] - r['dvz_pt']) for r in rf):.4f} m/s, "
              f"max deck-normal speed {max(r['vn'] for r in rf):.3f} m/s")

    # Deck geometry vs sea level (z = 0).  The deck is a rigid plane with a
    # fixed 2 m mean freeboard; its corners are lowest at -|theta| L - |phi| W.
    print("\nDeck height above sea level (z = 0) during each run (t = 0 .. latest touchdown):")
    for a_h in A_H_SWEEP:
        min_corner, n_awash, t_awash, min_contact = np.inf, 0, 0.0, np.inf
        for sd in range(N_MC):
            ph = sea_phases(sd)
            t_end = max(mc[a_h][s][sd]["t"] for s in STRATS)
            tt = np.arange(0.0, t_end + DT / 2, DT)
            zc = (Z_DECK_MEAN + a_h * np.sin(W_H * tt + ph[0])
                  - np.abs(A_THETA * np.sin(W_TH * tt + ph[1])) * DECK_HALF_L
                  - np.abs(A_PHI * np.sin(W_R * tt + ph[2])) * DECK_HALF_W)
            min_corner = min(min_corner, float(zc.min()))
            if zc.min() < 0.0:
                n_awash += 1
                t_awash += float(np.sum(zc < 0.0)) * DT
            for s in STRATS:
                r = mc[a_h][s][sd]
                z_contact = Z_DECK_MEAN + r["zp"] + r["dh_tilt"]   # deck plane under the drone
                min_contact = min(min_contact, z_contact)
        z_pad_min = Z_DECK_MEAN - a_h
        assert z_pad_min > 0.0, "pad centre below sea level"
        assert min_contact > 0.0, "contact point below sea level"
        print(f"  A_h = {a_h} m: pad centre >= {z_pad_min:.2f} m; lowest deck corner "
              f"{min_corner:+.3f} m (bound {Z_DECK_MEAN - a_h - A_THETA * DECK_HALF_L - A_PHI * DECK_HALF_W:+.3f} m); "
              f"a corner below sea level in {n_awash}/{N_MC} runs "
              f"({t_awash:.1f} s in total); lowest deck point under a touching-down drone {min_contact:.3f} m")

    m_c = np.array([r["miss"] for r in mc[A_H]["rel_ff"]])
    m_n = np.array([r["miss"] for r in nocomp])
    print(f"\n  Latency compensation (rel_ff, A_h = {A_H}): lateral miss mean "
          f"{m_c.mean():.3f} m with, {m_n.mean():.3f} m without (V_V * tau_s = {V_V * TAU_S:.2f} m)")


# ── Plots ─────────────────────────────────────────────────────────────────────

def _deck_poly(h, i):
    x, y, z = h["pad"][i]
    th, ro = h["th"][i], h["ro"][i]
    pts = []
    for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        dx, dy = sx * DECK_HALF_L, sy * DECK_HALF_W
        pts.append([x + dx, y + dy, z + th * dx - ro * dy])
    return pts


def plot_trajectory_3d(show):
    h = show["rel_ff"]["hist"]
    p, pad, ph = h["p"], h["pad"], h["phase"]
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(pad[:, 0], pad[:, 1], np.zeros(len(pad)), color="orange", lw=2, ls="--",
            label="Vessel track (sea level)")
    ax.plot(pad[:, 0], pad[:, 1], pad[:, 2], color="green", lw=1.5, label="Pad centre (heaving)")
    cols = {1: "red", 2: "darkorange", 3: "darkred"}
    names = {1: "Phase 1 (PN + speed hold)", 2: "Phase 2 (ramp)", 3: "Phase 3 (flare, rel_ff)"}
    for k in (1, 2, 3):
        idx = np.where(ph == k)[0]
        if len(idx):
            idx = np.append(idx, min(idx[-1] + 1, len(p) - 1))
            ax.plot(p[idx, 0], p[idx, 1], p[idx, 2], color=cols[k], lw=2, label=names[k])
    ax.add_collection3d(Poly3DCollection([_deck_poly(h, len(p) - 1)], color="gray", alpha=0.35))
    ax.scatter(*p[0], color="red", marker="^", s=80, label="Shore station (start)")
    ax.scatter(*p[-1], color="green", marker="*", s=160, label="Touchdown")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_zlim(0, 16)
    r = show["rel_ff"]
    ax.set_title(f"S023 3D Moving Landing Pad - seed {SEED}, deck-relative + FF\n"
                 f"touchdown t = {r['t']:.2f} s, miss = {r['miss']:.3f} m, "
                 f"|dvz|rel = {r['dvz']:.3f} m/s (grey: deck at touchdown)")
    ax.legend(loc="upper left", fontsize=8)
    ax.view_init(elev=22, azim=-60)
    out = os.path.join(OUT_DIR, "trajectory_3d.png")
    fig.savefig(out, dpi=120, bbox_inches="tight"); plt.close(fig)
    print(f"Saved: {out}")


def plot_altitude_tracking(show):
    t3 = show["rel_ff"]["t3"]
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    h0 = show["rel_ff"]["hist"]
    ax = axes[0]
    ax.plot(h0["t"], h0["pad"][:, 2], color="green", lw=2.5, label="Pad height $z_{pad}$")
    ax.axhline(Z_DECK_MEAN, color="green", ls=":", lw=1, label="Mean deck")
    for s in STRATS:
        h = show[s]["hist"]
        ax.plot(h["t"], h["p"][:, 2], color=COLORS[s], lw=1.5, label=f"$z_D$ {LABELS[s]}")
        ax.scatter(h["t"][-1], h["p"][-1, 2], color=COLORS[s], s=40, zorder=5)
    ax.axvline(t3, color="k", ls="--", lw=1, label=f"Phase 3 entry {t3:.2f} s")
    ax.set_ylabel("Altitude (m)"); ax.set_ylim(0, 9)
    ax.set_title(f"Drone altitude vs heaving pad (seed {SEED}, A_h = {A_H} m); dots = touchdown")
    ax.legend(fontsize=7, ncol=2); ax.grid(alpha=0.3)
    ax = axes[1]
    for s in STRATS:
        h = show[s]["hist"]
        ax.plot(h["t"], h["p"][:, 2] - h["pad"][:, 2], color=COLORS[s], lw=1.5, label=LABELS[s])
    ax.axhline(Z_FLARE_REL, color="k", ls=":", lw=1, label="$z_{flare,rel}$")
    ax.axvline(t3, color="k", ls="--", lw=1)
    ax.set_ylabel("$h_{rel} = z_D - z_{pad}$ (m)"); ax.set_xlabel("Time (s)")
    ax.set_ylim(-0.2, 7); ax.legend(fontsize=7); ax.grid(alpha=0.3)
    ax.set_xlim(t3 - 4, max(show[s]["t"] for s in STRATS) + 1)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "altitude_tracking.png")
    fig.savefig(out, dpi=120, bbox_inches="tight"); plt.close(fig)
    print(f"Saved: {out}")


def _shade(ax, t, mask, color, alpha):
    edges = np.diff(np.concatenate([[0], mask.astype(int), [0]]))
    for a, b in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]):
        ax.axvspan(t[a], t[min(b, len(t) - 1)], color=color, alpha=alpha, lw=0)


def plot_deck_motion(show):
    h = show["rel_gate"]["hist"]
    t, g = h["t"], h["gate"]
    rg, rf = show["rel_gate"], show["rel_ff"]
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True,
                             gridspec_kw=dict(height_ratios=[3, 3, 3, 1]))
    for ax, y, lab in ((axes[0], h["zp"], "Heave $z_p$ (m)"),
                       (axes[1], np.degrees(h["th"]), "Pitch $\\theta_p$ (deg)"),
                       (axes[2], np.degrees(h["ro"]), "Roll $\\phi_r$ (deg)")):
        _shade(ax, t, g, "green", 0.15)
        ax.plot(t, y, color="navy", lw=1.5)
        ax.set_ylabel(lab); ax.grid(alpha=0.3)
    axes[0].axhline(A_H - DELTA_HEAVE, color="green", ls=":", lw=1)
    axes[0].axhline(-A_H + DELTA_HEAVE, color="green", ls=":", lw=1)
    axes[0].set_title(f"Deck motion (seed {SEED}); green = motion window open "
                      f"(| |zp| - A_h | <= {DELTA_HEAVE} m and |zp_dot| <= {ZDOT_GATE} m/s)")
    ax = axes[3]
    _shade(ax, t, g, "green", 0.8)
    _shade(ax, t, ~g, "red", 0.5)
    ax.axvline(rg["t3"], color="k", ls="--", lw=1.2)
    ax.axvline(rg["t_commit"], color="purple", lw=2)
    ax.axvline(rg["t"], color="purple", ls="-.", lw=2)
    ax.axvline(rf["t"], color="#d62728", ls=":", lw=2)
    ax.set_yticks([]); ax.set_xlabel("Time (s)")
    ax.legend(handles=[mpatches.Patch(color="green", alpha=0.8, label="gate open"),
                       mpatches.Patch(color="red", alpha=0.5, label="gate closed"),
                       plt.Line2D([], [], color="k", ls="--", label=f"Phase 3 entry {rg['t3']:.2f} s"),
                       plt.Line2D([], [], color="purple", lw=2, label=f"gate commit {rg['t_commit']:.2f} s"),
                       plt.Line2D([], [], color="purple", ls="-.", lw=2, label=f"touchdown rel_gate {rg['t']:.2f} s"),
                       plt.Line2D([], [], color="#d62728", ls=":", lw=2, label=f"touchdown rel_ff {rf['t']:.2f} s")],
              fontsize=7, ncol=3, loc="upper left", bbox_to_anchor=(0, -0.6))
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "deck_motion.png")
    fig.savefig(out, dpi=120, bbox_inches="tight"); plt.close(fig)
    print(f"Saved: {out}")


def plot_relative_velocity(show):
    t3 = show["rel_ff"]["t3"]
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for s in STRATS:
        h = show[s]["hist"]
        t = h["t"]
        axes[0].plot(t, np.abs(h["v"][:, 2] - h["vpad"][:, 2]), color=COLORS[s],
                     lw=1.3, label=LABELS[s])
        axes[1].plot(t, np.hypot(h["v"][:, 0] - h["vpad"][:, 0], h["v"][:, 1] - h["vpad"][:, 1]),
                     color=COLORS[s], lw=1.3, label=LABELS[s])
        axes[2].plot(t, np.hypot(h["p"][:, 0] - h["pad"][:, 0], h["p"][:, 1] - h["pad"][:, 1]),
                     color=COLORS[s], lw=1.3, label=LABELS[s])
        for ax, val in zip(axes, (show[s]["dvz"], show[s]["dvxy"], show[s]["miss"])):
            ax.scatter(show[s]["t"], val, color=COLORS[s], s=40, zorder=5)
    axes[0].axhline(V_LAND_MAX, color="k", ls="--", lw=1, label=f"limit {V_LAND_MAX} m/s")
    axes[0].set_ylabel("$|\\dot z_D - \\dot z_p|$ (m/s)"); axes[0].set_ylim(0, 2.5)
    axes[1].axhline(DV_MAX, color="k", ls="--", lw=1, label=f"limit {DV_MAX} m/s")
    axes[1].set_ylabel("$\\|\\dot p_{xy,D} - \\dot p_{xy,V}\\|$ (m/s)"); axes[1].set_ylim(0, 6)
    axes[2].axhline(R_PAD, color="k", ls="--", lw=1, label=f"$r_{{pad}}$ = {R_PAD} m")
    axes[2].set_ylabel("Lateral miss (m)"); axes[2].set_ylim(0, 8)
    axes[2].set_xlabel("Time (s)")
    axes[0].set_title(f"Relative velocity and lateral miss (seed {SEED}); dots = touchdown")
    for ax in axes:
        ax.axvline(t3, color="gray", ls=":", lw=1)
        ax.grid(alpha=0.3); ax.legend(fontsize=7, loc="upper right")
    axes[2].set_xlim(show["rel_ff"]["t2"] - 2, max(show[s]["t"] for s in STRATS) + 1)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "relative_velocity.png")
    fig.savefig(out, dpi=120, bbox_inches="tight"); plt.close(fig)
    print(f"Saved: {out}")


def plot_monte_carlo(mc):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))
    w = 0.2
    xs = np.arange(len(A_H_SWEEP))
    for j, s in enumerate(STRATS):
        succ = [sum(r["success"] for r in mc[a][s]) for a in A_H_SWEEP]
        axes[0].bar(xs + (j - 1.5) * w, succ, w, color=COLORS[s], label=LABELS[s])
        for xi, v in zip(xs + (j - 1.5) * w, succ):
            axes[0].text(xi, v + 1, str(v), ha="center", fontsize=7)
        rng = np.random.default_rng(123)
        for i, a in enumerate(A_H_SWEEP):
            dvz = np.array([r["dvz"] for r in mc[a][s]])
            xj = i + (j - 1.5) * w + rng.uniform(-0.06, 0.06, len(dvz))
            axes[1].scatter(xj, dvz, s=6, color=COLORS[s], alpha=0.5,
                            label=LABELS[s] if i == 0 else None)
            axes[1].plot([i + (j - 1.5) * w - 0.08, i + (j - 1.5) * w + 0.08],
                         [np.median(dvz)] * 2, color="k", lw=1.5)
    axes[0].set_xticks(xs); axes[0].set_xticklabels([f"A_h = {a} m" for a in A_H_SWEEP])
    axes[0].set_ylabel(f"Successful landings (of {N_MC})"); axes[0].set_ylim(0, N_MC * 1.35)
    axes[0].set_title("Landing success vs heave amplitude")
    axes[0].legend(fontsize=7, loc="upper center", ncol=2)
    axes[1].axhline(V_LAND_MAX, color="k", ls="--", lw=1, label=f"limit {V_LAND_MAX} m/s")
    axes[1].set_xticks(xs); axes[1].set_xticklabels([f"A_h = {a} m" for a in A_H_SWEEP])
    axes[1].set_ylabel("$|\\dot z_D - \\dot z_p|$ at touchdown (m/s)")
    axes[1].set_title("Relative vertical touchdown speed (black bar = median)")
    axes[1].legend(fontsize=7, loc="upper left"); axes[1].grid(alpha=0.3)
    fig.suptitle(f"S023 3D Monte-Carlo: {N_MC} sea-phase seeds per sea state, same seeds for all strategies")
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "monte_carlo.png")
    fig.savefig(out, dpi=120, bbox_inches="tight"); plt.close(fig)
    print(f"Saved: {out}")


def save_animation(show):
    h = show["rel_ff"]["hist"]
    p, pad = h["p"], h["pad"]
    n = len(p)
    step = 8
    n_frames = math.ceil((n - 1) / step) + 1
    idx = [min(k * step, n - 1) for k in range(n_frames)]
    fig = plt.figure(figsize=(13, 6))
    ax3 = fig.add_subplot(121, projection="3d")
    ax2 = fig.add_subplot(122)
    ax3.set_xlim(-22, pad[-1, 0] + 8); ax3.set_ylim(-8, 10); ax3.set_zlim(0, 16)
    ax3.set_xlabel("X (m)"); ax3.set_ylabel("Y (m)"); ax3.set_zlabel("Z (m)")
    ax3.view_init(elev=22, azim=-60)
    ax3.plot(pad[:, 0], pad[:, 1], np.zeros(n), color="orange", ls="--", lw=1)
    tr3, = ax3.plot([], [], [], color="red", lw=1.5)
    dot3, = ax3.plot([], [], [], "o", color="red", ms=6)
    padp, = ax3.plot([], [], [], "s", color="green", ms=6)
    deck = [None]
    ax2.set_xlim(h["t"][0], h["t"][-1] + 0.5); ax2.set_ylim(0, 16)
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Z (m)"); ax2.grid(alpha=0.3)
    ax2.set_title("Altitude: drone (red) vs pad (green)")
    l_d, = ax2.plot([], [], color="red", lw=1.5)
    l_p, = ax2.plot([], [], color="green", lw=1.5)
    title = fig.suptitle("")

    def update(fi):
        i = idx[fi]
        tr3.set_data(p[:i + 1, 0], p[:i + 1, 1]); tr3.set_3d_properties(p[:i + 1, 2])
        dot3.set_data([p[i, 0]], [p[i, 1]]); dot3.set_3d_properties([p[i, 2]])
        padp.set_data([pad[i, 0]], [pad[i, 1]]); padp.set_3d_properties([pad[i, 2]])
        if deck[0] is not None:
            deck[0].remove()
        deck[0] = Poly3DCollection([_deck_poly(h, i)], color="gray", alpha=0.4)
        ax3.add_collection3d(deck[0])
        l_d.set_data(h["t"][:i + 1], p[:i + 1, 2]); l_p.set_data(h["t"][:i + 1], pad[:i + 1, 2])
        title.set_text(f"S023 3D Moving Landing Pad (deck-relative + FF, seed {SEED}) - "
                       f"t = {h['t'][i]:.2f} s, Phase {h['phase'][i]}")
        return tr3, dot3, padp, l_d, l_p

    anim = FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    out = os.path.join(OUT_DIR, "animation.gif")
    anim.save(out, writer=PillowWriter(fps=20), dpi=80)
    plt.close(fig)
    print(f"Saved: {out}  ({n_frames} frames, every {step}th of {n} samples)")


if __name__ == "__main__":
    print("=" * 70)
    print("S023 3D Moving Landing Pad - Simulation")
    print("=" * 70)
    show, mc, nocomp = run_simulation()
    report(show, mc, nocomp)
    print("\nGenerating plots ...")
    plot_trajectory_3d(show)
    plot_altitude_tracking(show)
    plot_deck_motion(show)
    plot_relative_velocity(show)
    plot_monte_carlo(mc)
    print("Generating animation ...")
    save_animation(show)
    print(f"\nAll outputs saved to: {OUT_DIR}")
    print("Done.")
