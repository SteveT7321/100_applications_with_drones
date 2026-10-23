"""
S027 Aerial Refueling Relay
===========================
A delivery drone (Receiver) must fly base -> target (60 m) -> base, but its
battery only covers 75 m.  It evaluates the card's range trigger
d_rem < d_target + d_base + margin; when the trigger fires it requests a tanker,
which launches from base, flies to a loiter orbit around the route midpoint and
holds station.  The Receiver docks with a PD approach / soft-dock law, receives
energy and completes the mission.  A second run without refuelling shows the
forced landing the trigger predicts.

Every simulation step uses one consistent state: the dock condition is checked
on the recorded state of both drones *before* either is advanced.

Run:
    conda activate drones
    python src/02_logistics_delivery/s027_aerial_refueling_relay.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation

OUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..",
    "outputs", "02_logistics_delivery", "s027_aerial_refueling_relay"))
os.makedirs(OUT_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# Scenario constants (card values; energies scaled x3, see README)
# ---------------------------------------------------------------------------
V_CRUISE  = 5.0      # m/s  cruise speed (both drones)
V_T_ORB   = 2.0      # m/s  tanker loiter speed
R_ORB     = 3.0      # m    loiter radius
K_FLIGHT  = 0.3      # W·s²/m²
K_HOVER   = 0.5      # W
E0_R      = 120.0    # J    Receiver initial battery (card 40 J x3)
E0_T      = 240.0    # J    Tanker initial battery   (card 80 J x3)
E_XFER    = 90.0     # J    energy transferred       (card 30 J x3)
ETA_LOSS  = 0.05
D_DOCK    = 0.4      # m    soft-dock engagement radius
EPS_P     = 0.15     # m    dock position tolerance
EPS_V     = 0.05     # m/s  dock velocity tolerance
KP,  KD   = 2.0, 1.5     # approach gains
KP2, KD2  = 4.0, 2.5     # soft-dock gains
A_MAX     = 15.0     # m/s² Receiver acceleration limit while docking
MARGIN    = 5.0      # m    range safety margin
ALPHA     = 0.5      # rendezvous fraction along base -> target
BASE_DIST = 60.0     # m
ALT       = 10.0     # m    mission altitude
DT        = 0.02     # s
MAX_TIME  = 120.0    # s
ARRIVE_R  = 0.5      # m    waypoint arrival radius

pos_base   = np.array([0.0, 0.0, ALT])
pos_target = np.array([BASE_DIST, 0.0, ALT])
w_rvz      = (1.0 - ALPHA) * pos_base + ALPHA * pos_target
omega_orb  = V_T_ORB / R_ORB
PHI0       = np.pi           # orbit entry point = side of the circle facing base
TANKER_START = pos_base + np.array([0.0, 0.5, 0.0])

CRUISE_POWER = K_FLIGHT * V_CRUISE**2 + K_HOVER       # 8.0 W
ENERGY_PER_M = CRUISE_POWER / V_CRUISE                # 1.6 J/m


def battery_drain(speed, dt):
    return (K_FLIGHT * speed**2 + K_HOVER) * dt


def range_remaining(energy, speed=V_CRUISE):
    """Card model: d_rem = E / (k_f v² + k_h) * v."""
    return energy / (K_FLIGHT * speed**2 + K_HOVER) * speed


def orbit_state(tl):
    """Tanker loiter position, velocity, acceleration at loiter time tl."""
    a = omega_orb * tl + PHI0
    c, s = np.cos(a), np.sin(a)
    p = w_rvz + R_ORB * np.array([c, s, 0.0])
    v = V_T_ORB * np.array([-s, c, 0.0])
    acc = -R_ORB * omega_orb**2 * np.array([c, s, 0.0])
    return p, v, acc


def _toward(pos, goal, speed=V_CRUISE):
    d = goal - pos
    n = np.linalg.norm(d)
    return speed * d / n if n > 1e-9 else np.zeros(3)


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def run_simulation(refuel=True, soft_dock_ff=True):
    """Simulate the mission.  refuel=False ignores the trigger (no tanker).

    soft_dock_ff=False uses the card's soft-dock law without the tanker
    acceleration feed-forward (only used as a check in the report).
    """
    pR, vR, ER = pos_base.copy(), np.zeros(3), E0_R
    pT, vT, ET = TANKER_START.copy(), np.zeros(3), E0_T
    aT = np.zeros(3)
    fsm_R, fsm_T = "to_target", "at_base"
    t_loiter = 0.0
    ev = {}
    log = []
    for k in range(int(MAX_TIME / DT) + 1):
        t = k * DT
        dr, dv = pR - pT, vR - vT
        # Range trigger (card): needed = to target (if not yet delivered) + to base
        if fsm_R in ("to_target", "hold", "docking"):
            need = (np.linalg.norm(pos_target - pR) + BASE_DIST + MARGIN)
        elif fsm_R == "deliver":
            need = np.linalg.norm(pos_target - pR) + BASE_DIST + MARGIN
        elif fsm_R == "return":
            need = np.linalg.norm(pos_base - pR) + MARGIN
        else:
            need = 0.0
        d_rem = range_remaining(ER)
        log.append((t, *pR, *pT, ER, ET, np.linalg.norm(dr),
                    np.linalg.norm(dv), d_rem, need, fsm_R, fsm_T))

        # ---- termination ----
        if ER <= 0.0:
            ev["forced_landing"] = (t, pR.copy())
            break
        if fsm_R == "home" and fsm_T in ("home", "at_base"):
            ev["complete"] = t
            break

        # ---- events on the current (recorded) state ----
        if refuel and "trigger" not in ev and d_rem < need:
            ev["trigger"] = (t, d_rem, need)
            fsm_T = "to_orbit"
        if (fsm_R == "docking" and np.linalg.norm(dr) < EPS_P
                and np.linalg.norm(dv) < EPS_V):
            ev["dock"] = (t, np.linalg.norm(dr), np.linalg.norm(dv), k)
            ER += E_XFER
            ET -= E_XFER * (1 + ETA_LOSS)
            ev["after_xfer"] = (ER, ET)
            fsm_R, fsm_T = "deliver", "return"

        # ---- Tanker guidance ----
        if fsm_T == "to_orbit":
            entry, _, _ = orbit_state(0.0)
            if np.linalg.norm(entry - pT) > V_CRUISE * DT:
                vT, aT = _toward(pT, entry), np.zeros(3)
            else:
                fsm_T, t_loiter = "loiter", 0.0
                ev["loiter"] = t
        if fsm_T == "loiter":
            pT, vT, aT = orbit_state(t_loiter)
        elif fsm_T == "return":
            aT = np.zeros(3)
            if np.linalg.norm(TANKER_START - pT) > ARRIVE_R:
                vT = _toward(pT, TANKER_START)
            else:
                vT, fsm_T = np.zeros(3), "home"
                ev["tanker_home"] = t

        # ---- Receiver guidance ----
        if fsm_R == "to_target":
            near_rvz = np.linalg.norm(w_rvz - pR) < 2.0 * R_ORB
            if refuel and "trigger" in ev and near_rvz:
                fsm_R = "docking" if fsm_T == "loiter" else "hold"
                if fsm_R == "docking":
                    ev["dock_start"] = t
                else:
                    ev["hold"] = t
            if fsm_R == "to_target":
                if np.linalg.norm(pos_target - pR) > ARRIVE_R:
                    vR = _toward(pR, pos_target)
                else:
                    fsm_R = "return"
                    ev["delivered"] = t
        if fsm_R == "hold":                      # wait for the tanker
            vR = np.zeros(3)
            if fsm_T == "loiter":
                fsm_R = "docking"
                ev["dock_start"] = t
        if fsm_R == "docking":
            if np.linalg.norm(dr) > D_DOCK:
                acc = -KP * dr - KD * dv + aT
            else:
                acc = -KP2 * dr - KD2 * dv + (aT if soft_dock_ff else 0.0)
            n = np.linalg.norm(acc)
            if n > A_MAX:
                acc *= A_MAX / n
            vR = vR + acc * DT
        elif fsm_R == "deliver":
            if np.linalg.norm(pos_target - pR) > ARRIVE_R:
                vR = _toward(pR, pos_target)
            else:
                fsm_R = "return"
                ev["delivered"] = t
        if fsm_R == "return":
            if np.linalg.norm(pos_base - pR) > ARRIVE_R:
                vR = _toward(pR, pos_base)
            else:
                vR, fsm_R = np.zeros(3), "home"
                ev["receiver_home"] = t

        # ---- integrate, drain ----
        pR = pR + vR * DT
        if fsm_T in ("to_orbit", "return"):
            pT = pT + vT * DT
        if fsm_T == "loiter":
            t_loiter += DT
        ER = max(0.0, ER - battery_drain(np.linalg.norm(vR), DT))
        if fsm_T != "at_base" and fsm_T != "home":
            ET = max(0.0, ET - battery_drain(np.linalg.norm(vT), DT))

    cols = list(zip(*log))
    h = {"t": np.array(cols[0]), "pR": np.array(cols[1:4]).T,
         "pT": np.array(cols[4:7]).T, "ER": np.array(cols[7]),
         "ET": np.array(cols[8]), "dr": np.array(cols[9]),
         "dv": np.array(cols[10]), "d_rem": np.array(cols[11]),
         "need": np.array(cols[12]), "fsm_R": np.array(cols[13]),
         "fsm_T": np.array(cols[14]), "ev": ev}
    return h


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
PHASE_COL = {"to_target": "red", "hold": "grey", "docking": "orange",
             "deliver": "green", "return": "darkgreen", "home": "darkgreen"}


def _phase_segments(ax, h, three_d):
    f = h["fsm_R"]
    cuts = [0] + [i for i in range(1, len(f)) if f[i] != f[i-1]] + [len(f)]
    for a, b in zip(cuts[:-1], cuts[1:]):
        seg = h["pR"][a:min(b + 1, len(f))]
        args = (seg[:, 0], seg[:, 1], seg[:, 2]) if three_d else \
            (seg[:, 0], seg[:, 1])
        ax.plot(*args, color=PHASE_COL[f[a]], lw=2)


def _legend_elems():
    return [Line2D([0], [0], color=c, lw=2, label=f"Receiver – {p}")
            for p, c in [("to target / RVZ", "red"), ("docking", "orange"),
                         ("deliver", "green"), ("return", "darkgreen")]] + [
        Line2D([0], [0], color="blue", lw=2, label="Tanker"),
        Line2D([0], [0], marker="s", color="w", markerfacecolor="black",
               ms=8, label="Base"),
        Line2D([0], [0], marker="*", color="w", markerfacecolor="green",
               ms=11, label="Target"),
        Line2D([0], [0], marker="^", color="w", markerfacecolor="purple",
               ms=8, label="RVZ waypoint"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="lime",
               ms=8, label="Dock success")]


def plot_trajectory_3d(h):
    fig = plt.figure(figsize=(13, 8))
    ax = fig.add_subplot(111, projection="3d")
    _phase_segments(ax, h, True)
    ax.plot(*h["pT"].T, color="blue", lw=1.5, alpha=0.8)
    ax.scatter(*pos_base, marker="s", s=120, color="black")
    ax.scatter(*pos_target, marker="*", s=200, color="green")
    ax.scatter(*w_rvz, marker="^", s=150, color="purple")
    if "dock" in h["ev"]:
        ax.scatter(*h["pR"][h["ev"]["dock"][3]], marker="o", s=180,
                   color="lime")
    ax.set_zlim(ALT - 3, ALT + 3)
    ax.legend(handles=_legend_elems(), loc="upper left", fontsize=8)
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
    ax.set_title("S027 Aerial Refueling Relay – 3-D trajectory (refuelled run)")
    plt.tight_layout()
    _save(fig, "trajectory_3d.png")


def plot_battery(h):
    ev = h["ev"]
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(h["t"], h["ER"], color="red", lw=2, label="Receiver energy")
    ax.plot(h["t"], h["ET"], color="blue", lw=2, label="Tanker energy")
    for key, col, lbl in [("loiter", "green", "Tanker on orbit"),
                          ("dock_start", "purple", "Docking phase start"),
                          ("dock", "lime", "Dock success / transfer"),
                          ("delivered", "black", "Package delivered")]:
        if key in ev:
            tv = ev[key][0] if isinstance(ev[key], tuple) else ev[key]
            ax.axvline(tv, color=col, ls="--", lw=1.4, label=lbl)
    ax.set_xlabel("Time [s]"); ax.set_ylabel("Battery energy [J]")
    ax.set_title("S027 – Battery energy vs time (refuelled run)")
    ax.legend(fontsize=9); ax.grid(True, alpha=0.4)
    plt.tight_layout()
    _save(fig, "battery_plot.png")


def plot_docking(h):
    ev = h["ev"]
    s = int(round(ev["dock_start"] / DT))
    e = ev["dock"][3]
    sl = slice(s, min(e + 50, len(h["t"])))
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    a1.semilogy(h["t"][sl], np.maximum(h["dr"][sl], 1e-6), color="orange",
                lw=2)
    a1.axhline(EPS_P, color="red", ls=":", label=f"ε_p = {EPS_P} m")
    a1.axhline(D_DOCK, color="k", ls="--", lw=1, label=f"d_dock = {D_DOCK} m")
    a2.semilogy(h["t"][sl], np.maximum(h["dv"][sl], 1e-6), color="purple",
                lw=2)
    a2.axhline(EPS_V, color="red", ls=":", label=f"ε_v = {EPS_V} m/s")
    for a in (a1, a2):
        a.axvline(h["t"][e], color="lime", ls="--", label="Dock success")
        a.grid(True, alpha=0.4, which="both"); a.legend(fontsize=9)
    a1.set_ylabel("|Δr| [m]"); a2.set_ylabel("|Δv| [m/s]")
    a2.set_xlabel("Time [s]  (after dock the Receiver leaves the tanker)")
    a1.set_title("S027 – Relative distance and velocity during docking")
    plt.tight_layout()
    _save(fig, "docking_relative_motion.png")


def plot_loiter_topdown(h):
    fig, ax = plt.subplots(figsize=(9, 7))
    ev = h["ev"]
    s = int(round(ev["dock_start"] / DT)) - 75
    e = ev["dock"][3] + 75
    pR, pT = h["pR"][s:e], h["pT"][s:e]
    sub = {k: (v[s:e] if isinstance(v, np.ndarray) else v)
           for k, v in h.items()}
    _phase_segments(ax, sub, False)
    ax.plot(pT[:, 0], pT[:, 1], color="blue", lw=1.5, label="Tanker")
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(w_rvz[0] + R_ORB * np.cos(th), w_rvz[1] + R_ORB * np.sin(th),
            "k--", lw=1, alpha=0.5)
    ax.scatter(*w_rvz[:2], marker="^", s=120, color="purple")
    k = ev["dock"][3]
    ax.scatter(*h["pR"][k, :2], marker="o", s=150, color="lime", zorder=6)
    ax.set_aspect("equal"); ax.grid(True, alpha=0.4)
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
    ax.legend(handles=_legend_elems()[:5] + [
        Line2D([0], [0], color="k", ls="--", label=f"Loiter circle R={R_ORB} m")],
        fontsize=8, loc="upper left")
    ax.set_title("S027 – Rendezvous detail (top-down, ±1.5 s around docking)")
    plt.tight_layout()
    _save(fig, "loiter_orbit_topdown.png")


def plot_comparison(h, h0):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(15, 5))
    for d, col, ls, lbl in [(h, "red", "-", "With refuelling"),
                            (h0, "black", "--", "Without refuelling")]:
        a1.plot(d["t"], d["ER"], color=col, ls=ls, lw=2, label=lbl)
        a2.plot(d["t"], d["pR"][:, 0], color=col, ls=ls, lw=2, label=lbl)
    a1.axvline(h["ev"]["dock"][0], color="lime", ls="--",
               label="Energy transfer")
    fl_t, fl_p = h0["ev"]["forced_landing"]
    for a, y in [(a1, 0.0), (a2, fl_p[0])]:
        a.scatter([fl_t], [y], marker="X", s=150, color="black", zorder=6,
                  label="Forced landing (no refuel)")
    deficit = 2 * BASE_DIST - range_remaining(E0_R)
    a1.annotate(f"range {range_remaining(E0_R):.0f} m < {2*BASE_DIST:.0f} m "
                f"needed\n(deficit {deficit:.0f} m = "
                f"{deficit*ENERGY_PER_M:.0f} J)", xy=(fl_t, 0),
                xytext=(fl_t - 8, 60), arrowprops=dict(arrowstyle="->"))
    a1.axhline(0, color="k", lw=0.8, ls=":")
    a1.set_ylabel("Receiver energy [J]")
    a2.axhline(BASE_DIST, color="green", ls=":", label="Target x")
    a2.set_ylabel("Receiver x [m]")
    for a in (a1, a2):
        a.set_xlabel("Time [s]"); a.grid(True, alpha=0.4); a.legend(fontsize=8)
    a1.set_title("Receiver battery")
    a2.set_title("Receiver progress along the route")
    plt.suptitle("S027 – Mission with vs without aerial refuelling")
    plt.tight_layout()
    _save(fig, "mission_comparison.png")


def _save(fig, name):
    fig.savefig(os.path.join(OUT_DIR, name), dpi=130)
    plt.close(fig)
    print(f"Saved {name}")


def save_animation(h, n_frames=140, fps=12):
    """Top-down 2-D animation of the refuelled mission (<= 150 frames)."""
    n = len(h["t"])
    # denser sampling around the docking manoeuvre
    s = int(round(h["ev"]["dock_start"] / DT))
    e = h["ev"]["dock"][3] + 25
    idx = np.unique(np.concatenate([
        np.linspace(0, s, 40).astype(int),
        np.linspace(s, e, 50).astype(int),
        np.linspace(e, n - 1, n_frames - 90).astype(int)]))
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.set_xlim(-3, BASE_DIST + 3); ax.set_ylim(-5, 5)
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
    th = np.linspace(0, 2 * np.pi, 100)
    ax.plot(w_rvz[0] + R_ORB * np.cos(th), w_rvz[1] + R_ORB * np.sin(th),
            "k--", lw=0.8, alpha=0.5)
    ax.scatter(*pos_base[:2], marker="s", s=80, color="black")
    ax.scatter(*pos_target[:2], marker="*", s=160, color="green")
    trR, = ax.plot([], [], color="red", lw=1.5)
    trT, = ax.plot([], [], color="blue", lw=1.5)
    dR, = ax.plot([], [], "o", color="red", ms=7, label="Receiver")
    dT, = ax.plot([], [], "o", color="blue", ms=7, label="Tanker")
    txt = ax.text(0.01, 0.92, "", transform=ax.transAxes, fontsize=9)
    ax.legend(loc="lower right", fontsize=8)
    ax.set_title("S027 Aerial Refueling Relay – top-down")

    def update(i):
        k = idx[i]
        lo = max(0, k - 150)
        trR.set_data(h["pR"][lo:k+1, 0], h["pR"][lo:k+1, 1])
        trT.set_data(h["pT"][lo:k+1, 0], h["pT"][lo:k+1, 1])
        col = PHASE_COL[h["fsm_R"][k]]
        trR.set_color(col); dR.set_color(col)
        dR.set_data([h["pR"][k, 0]], [h["pR"][k, 1]])
        dT.set_data([h["pT"][k, 0]], [h["pT"][k, 1]])
        txt.set_text(f"t = {h['t'][k]:5.2f} s   Receiver: {h['fsm_R'][k]:9s}"
                     f"  E_R = {h['ER'][k]:5.1f} J   E_T = {h['ET'][k]:5.1f} J")
        return trR, trT, dR, dT, txt

    anim = FuncAnimation(fig, update, frames=len(idx), blit=False)
    anim.save(os.path.join(OUT_DIR, "animation.gif"), writer="pillow", fps=fps)
    plt.close(fig)
    print(f"Saved animation.gif  ({len(idx)} frames @ {fps} fps)")


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def print_report(h, h0):
    ev = h["ev"]
    print("Energy budget:")
    print(f"  Cruise power {CRUISE_POWER:.2f} W -> {ENERGY_PER_M:.3f} J/m")
    print(f"  Round trip {2*BASE_DIST:.0f} m needs "
          f"{2*BASE_DIST*ENERGY_PER_M:.1f} J; E0_R = {E0_R:.1f} J gives "
          f"range {range_remaining(E0_R):.1f} m")
    print(f"  E0_R + E_XFER = {E0_R + E_XFER:.1f} J gives range "
          f"{range_remaining(E0_R + E_XFER):.1f} m")
    print()
    print("=" * 62)
    print("  S027 Aerial Refueling Relay – refuelled run")
    print("=" * 62)
    tt, dr0, nd = ev["trigger"]
    print(f"  Refuel trigger           : t = {tt:.2f} s  (d_rem {dr0:.1f} m "
          f"< need {nd:.1f} m)")
    if "hold" in ev:
        print(f"  Receiver holds (tanker not yet on orbit): from "
              f"{ev['hold']:.2f} s, {np.linalg.norm(w_rvz - h['pR'][int(round(ev['hold'] / DT))]):.2f} m before the RVZ")
    print(f"  Tanker on orbit          : {ev['loiter']:.2f} s")
    print(f"  Docking phase start      : {ev['dock_start']:.2f} s")
    td, drd, dvd, k = ev["dock"]
    print(f"  Docking success          : {td:.2f} s  (duration "
          f"{td - ev['dock_start']:.2f} s)")
    print(f"  |dr| / |dv| at dock      : {drd:.4f} m / {dvd:.4f} m/s  "
          f"(tol {EPS_P} m / {EPS_V} m/s)")
    print(f"  Same state in history    : {h['dr'][k]:.4f} m / "
          f"{h['dv'][k]:.4f} m/s")
    print(f"  Energy before transfer   : Receiver {h['ER'][k]:.2f} J, "
          f"Tanker {h['ET'][k]:.2f} J")
    print(f"  Energy after transfer    : Receiver {ev['after_xfer'][0]:.2f} J, "
          f"Tanker {ev['after_xfer'][1]:.2f} J")
    print(f"  Package delivered        : {ev['delivered']:.2f} s")
    print(f"  Receiver home            : {ev['receiver_home']:.2f} s, "
          f"final energy {h['ER'][-1]:.2f} J")
    print(f"  Tanker home              : {ev['tanker_home']:.2f} s, "
          f"final energy {h['ET'][-1]:.2f} J")
    post = np.isin(h["fsm_R"], ["deliver", "return"])
    margin = (h["d_rem"] - h["need"])[post]
    print(f"  Min range margin after refuel: {margin.min():.2f} m "
          f"(d_rem - needed, needed includes {MARGIN} m margin)")
    print(f"  Mission complete         : {ev['complete']:.2f} s")
    print()
    print("  Without refuelling:")
    fl_t, fl_p = h0["ev"]["forced_landing"]
    print(f"  Forced landing           : t = {fl_t:.2f} s at x = "
          f"{fl_p[0]:.2f} m ({np.linalg.norm(fl_p - pos_base):.2f} m from "
          f"base, delivered: {'delivered' in h0['ev']})")
    flown = np.sum(np.linalg.norm(np.diff(h0["pR"], axis=0), axis=1))
    print(f"  Distance flown           : {flown:.2f} m of {2*BASE_DIST:.0f} m")
    print("=" * 62)


if __name__ == "__main__":
    hist = run_simulation(refuel=True)
    hist0 = run_simulation(refuel=False)
    print_report(hist, hist0)
    # Check: card soft-dock law without tanker-acceleration feed-forward
    hc = run_simulation(refuel=True, soft_dock_ff=False)
    soft = (hc["fsm_R"] == "docking") & (hc["dr"] < D_DOCK)
    print("Check – card soft-dock law without a_T feed-forward:")
    print(f"  docked: {'dock' in hc['ev']};  min |dr| in soft-dock "
          f"{hc['dr'][soft].min():.3f} m, mean of last 4 s "
          f"{hc['dr'][soft][-200:].mean():.3f} m  (predicted lag "
          f"V_orb²/R_orb/K_p2 = {V_T_ORB**2 / R_ORB / KP2:.3f} m)")
    if "forced_landing" in hc["ev"]:
        print(f"  Receiver battery empty while still chasing the tanker at "
              f"t = {hc['ev']['forced_landing'][0]:.2f} s")
    plot_trajectory_3d(hist)
    plot_battery(hist)
    plot_docking(hist)
    plot_loiter_topdown(hist)
    plot_comparison(hist, hist0)
    save_animation(hist)
