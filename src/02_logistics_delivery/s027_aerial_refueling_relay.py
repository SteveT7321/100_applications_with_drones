"""
S027 Aerial Refueling Relay
===========================
A delivery drone (Receiver) rendezvouses with a tanker drone for mid-air
refueling.  Simulates: battery depletion model, rendezvous guidance
(approach + docking), energy transfer, and mission continuation.

Run:
    conda run -n drones python src/02_logistics_delivery/s027_aerial_refueling_relay.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401  needed for 3-D projection
from matplotlib.lines import Line2D

# ---------------------------------------------------------------------------
# Output directory
# ---------------------------------------------------------------------------
OUT_DIR = "outputs/02_logistics_delivery/s027_aerial_refueling_relay"
os.makedirs(OUT_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# Scenario constants  (energies and distances consistent for 60 m mission)
# ---------------------------------------------------------------------------
V_CRUISE    = 5.0      # m/s  cruise speed (both drones)
V_T_ORB     = 1.0      # m/s  tanker loiter speed  (slow orbit for easier docking)
R_ORB       = 2.0      # m    loiter radius
K_FLIGHT    = 0.3      # W·s²/m²  flight power coefficient (from scenario card)
K_HOVER     = 0.5      # W    baseline hover drain
E0_R        = 400.0    # J    Receiver initial battery  (scaled for 60 m mission)
E0_T        = 800.0    # J    Tanker initial battery
E_XFER      = 300.0    # J    energy transferred at dock
ETA_LOSS    = 0.05     # –    transfer loss fraction
D_DOCK      = 0.4      # m    docking engagement radius
EPS_P       = 0.15     # m    position tolerance (success)
EPS_V       = 0.05     # m/s  velocity tolerance (success)
KP,  KD     = 3.0, 2.0        # approach PD gains
KP2, KD2    = 6.0, 4.0        # soft-dock PD gains
MARGIN      = 5.0      # m    range safety margin
ALPHA       = 0.5      # rendezvous fraction along base→target
BASE_DIST   = 60.0     # m    base-to-target distance

# Simulation time-step and max duration
DT          = 0.02     # s  small step for accurate docking dynamics
MAX_TIME    = 120.0    # s

# Altitude for the mission
ALT_MISSION = 10.0     # m

# ---------------------------------------------------------------------------
# Derived geometry
# ---------------------------------------------------------------------------
pos_base   = np.array([0.0,        0.0, ALT_MISSION])
pos_target = np.array([BASE_DIST,  0.0, ALT_MISSION])
w_rvz      = (1.0 - ALPHA) * pos_base + ALPHA * pos_target   # midpoint

omega_orb  = V_T_ORB / R_ORB      # rad/s loiter angular rate
phi0       = 0.0                   # loiter initial phase

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

def battery_drain(speed: float, dt: float) -> float:
    """Energy consumed in one time-step [J]."""
    return (K_FLIGHT * speed ** 2 + K_HOVER) * dt


def range_remaining(energy: float, speed: float) -> float:
    """Predicted remaining range [m] given current energy and cruise speed."""
    power = K_FLIGHT * speed ** 2 + K_HOVER
    if power <= 0 or speed <= 0:
        return 0.0
    return (energy / power) * speed


def loiter_pos(tl: float) -> np.ndarray:
    """Tanker position on loiter orbit at internal loiter time tl."""
    angle = omega_orb * tl + phi0
    return w_rvz + R_ORB * np.array([np.cos(angle), np.sin(angle), 0.0])


def loiter_vel(tl: float) -> np.ndarray:
    """Tanker velocity on loiter orbit at internal loiter time tl."""
    angle = omega_orb * tl + phi0
    return R_ORB * omega_orb * np.array([-np.sin(angle), np.cos(angle), 0.0])


def loiter_acc(tl: float) -> np.ndarray:
    """Tanker centripetal acceleration on loiter orbit."""
    angle = omega_orb * tl + phi0
    return -R_ORB * omega_orb**2 * np.array([np.cos(angle), np.sin(angle), 0.0])


# ---------------------------------------------------------------------------
# Energy feasibility report
# ---------------------------------------------------------------------------
cruise_power   = K_FLIGHT * V_CRUISE**2 + K_HOVER
energy_per_m   = cruise_power / V_CRUISE

print("Energy cost analysis:")
print(f"  Cruise power          : {cruise_power:.3f} W")
print(f"  Energy / metre        : {energy_per_m:.4f} J/m")
full_trip = BASE_DIST * 2 * energy_per_m
print(f"  Full trip ({BASE_DIST}m x2)    : {full_trip:.2f} J")
print(f"  E0_R                  : {E0_R:.1f} J  (ratio {E0_R/full_trip:.2f}x)")
print(f"  Refueled total        : {E0_R + E_XFER:.1f} J  "
      f"(ratio {(E0_R + E_XFER)/full_trip:.2f}x)")
print()

# ---------------------------------------------------------------------------
# State initialisation
# ---------------------------------------------------------------------------
# Tanker starts from base (slightly offset in Y to avoid zero relative dist)
pos_T  = np.array([0.0, 0.5, ALT_MISSION])
vel_T  = np.zeros(3)
E_T    = E0_T

pos_R  = pos_base.copy().astype(float)
vel_R  = np.zeros(3)
E_R    = E0_R

t_loiter = 0.0   # internal loiter clock (tracks tanker orbit phase)

# FSM
fsm_R = "fly_to_rvz"
fsm_T = "fly_to_rvz_T"

docked             = False
dock_success_time  = None
energy_xfer_done   = False
loiter_start_time  = None
docking_start_time = None
delivery_done      = False
tanker_home        = False

# ---------------------------------------------------------------------------
# History buffers
# ---------------------------------------------------------------------------
t_hist     = []
posR_hist  = []
posT_hist  = []
ER_hist    = []
ET_hist    = []
dr_hist    = []
dv_hist    = []
fsm_R_hist = []

# ---------------------------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------------------------
t         = 0.0
max_steps = int(MAX_TIME / DT)

for _step in range(max_steps):
    # Record
    t_hist.append(t)
    posR_hist.append(pos_R.copy())
    posT_hist.append(pos_T.copy())
    ER_hist.append(E_R)
    ET_hist.append(E_T)
    rel_r = pos_R - pos_T
    rel_v = vel_R - vel_T
    dr_hist.append(np.linalg.norm(rel_r))
    dv_hist.append(np.linalg.norm(rel_v))
    fsm_R_hist.append(fsm_R)

    # Termination
    if E_R <= 0.0:
        print(f"[t={t:.2f}s] RECEIVER BATTERY EMPTY – forced landing!")
        break
    if delivery_done and (tanker_home or E_T <= 0.0):
        print(f"[t={t:.2f}s] Mission complete – both drones done.")
        break

    # ------------------------------------------------------------------
    # Tanker FSM
    # ------------------------------------------------------------------
    acc_T_ff = np.zeros(3)   # feedforward acceleration for docking

    if fsm_T == "fly_to_rvz_T":
        to_rvz = w_rvz - pos_T
        dist   = np.linalg.norm(to_rvz)
        if dist > 0.3:
            vel_T = V_CRUISE * to_rvz / dist
            pos_T = pos_T + vel_T * DT
        else:
            vel_T             = np.zeros(3)
            fsm_T             = "loiter"
            loiter_start_time = t
            t_loiter          = 0.0
            pos_T             = loiter_pos(0.0)
            print(f"[t={t:.2f}s] Tanker on loiter orbit (R={R_ORB} m).")

    elif fsm_T == "loiter":
        pos_T     = loiter_pos(t_loiter)
        vel_T     = loiter_vel(t_loiter)
        acc_T_ff  = loiter_acc(t_loiter)
        t_loiter += DT
        if energy_xfer_done:
            fsm_T = "return_home_T"
            print(f"[t={t:.2f}s] Tanker returning to base.")

    elif fsm_T == "return_home_T":
        to_base = pos_base - pos_T
        dist    = np.linalg.norm(to_base)
        if dist > 0.3:
            vel_T = V_CRUISE * to_base / dist
            pos_T = pos_T + vel_T * DT
        else:
            vel_T       = np.zeros(3)
            pos_T       = pos_base.copy()
            tanker_home = True

    # ------------------------------------------------------------------
    # Receiver FSM
    # ------------------------------------------------------------------
    acc_R = np.zeros(3)

    if fsm_R == "fly_to_rvz":
        to_rvz = w_rvz - pos_R
        dist   = np.linalg.norm(to_rvz)
        # Wait for tanker to reach loiter before starting docking approach
        if fsm_T != "loiter" or dist > 2.5 * R_ORB:
            if dist > 0.5:
                vel_R = V_CRUISE * to_rvz / dist
            else:
                vel_R = np.zeros(3)
        else:
            # Tanker is loitering and we're within range – begin docking
            fsm_R              = "docking"
            docking_start_time = t
            print(f"[t={t:.2f}s] Receiver entering docking phase  "
                  f"(dist_to_rvz={dist:.1f} m).")

    elif fsm_R == "docking":
        if not energy_xfer_done:
            dr    = pos_R - pos_T
            dv    = vel_R - vel_T
            dnorm = np.linalg.norm(dr)

            if dnorm > D_DOCK:
                # Phase 1 – approach with feedforward
                acc_R = -KP * dr - KD * dv + acc_T_ff
            else:
                # Phase 2 – soft-dock: zero relative velocity, close gap gently
                acc_R = -KP2 * dr - KD2 * dv + acc_T_ff

            # Clamp to max 15 m/s²
            amag = np.linalg.norm(acc_R)
            if amag > 15.0:
                acc_R = acc_R * 15.0 / amag

            vel_R = vel_R + acc_R * DT

            # Check docking tolerances AFTER velocity update
            dv_now = vel_R - vel_T
            if dnorm < EPS_P and np.linalg.norm(dv_now) < EPS_V:
                dock_success_time = t
                docked            = True
                E_R              += E_XFER
                E_T              -= E_XFER * (1.0 + ETA_LOSS)
                energy_xfer_done  = True
                fsm_R             = "deliver"
                print(f"[t={t:.2f}s] DOCKING SUCCESS!  "
                      f"|Δr|={dnorm:.4f} m  "
                      f"|Δv|={np.linalg.norm(dv_now):.4f} m/s")
                print(f"  +{E_XFER:.1f} J to Receiver → SoC = {E_R:.1f} J")
                print(f"  -{E_XFER*(1+ETA_LOSS):.1f} J from Tanker → SoC = {E_T:.1f} J")

    elif fsm_R == "deliver":
        to_tgt = pos_target - pos_R
        dist   = np.linalg.norm(to_tgt)
        if dist > 0.5:
            vel_R = V_CRUISE * to_tgt / dist
        else:
            fsm_R = "return_home_R"
            vel_R = np.zeros(3)
            print(f"[t={t:.2f}s] Package delivered – returning to base.")

    elif fsm_R == "return_home_R":
        to_base = pos_base - pos_R
        dist    = np.linalg.norm(to_base)
        if dist > 0.5:
            vel_R = V_CRUISE * to_base / dist
        else:
            vel_R         = np.zeros(3)
            pos_R         = pos_base.copy()
            delivery_done = True
            print(f"[t={t:.2f}s] Receiver at base.  Final SoC: {E_R:.2f} J")

    # ------------------------------------------------------------------
    # Integrate Receiver position  (tanker integrated in FSM above)
    # ------------------------------------------------------------------
    pos_R = pos_R + vel_R * DT

    # ------------------------------------------------------------------
    # Battery drain
    # ------------------------------------------------------------------
    E_R = max(0.0, E_R - battery_drain(np.linalg.norm(vel_R), DT))
    E_T = max(0.0, E_T - battery_drain(np.linalg.norm(vel_T), DT))

    t += DT

# ---------------------------------------------------------------------------
# Arrays
# ---------------------------------------------------------------------------
t_arr    = np.array(t_hist)
posR_arr = np.array(posR_hist)
posT_arr = np.array(posT_hist)
ER_arr   = np.array(ER_hist)
ET_arr   = np.array(ET_hist)
dr_arr   = np.array(dr_hist)
dv_arr   = np.array(dv_hist)
fsm_arr  = np.array(fsm_R_hist)


def t2idx(ev_t):
    if ev_t is None:
        return None
    return int(np.argmin(np.abs(t_arr - ev_t)))


dock_idx   = t2idx(dock_success_time)
dstart_idx = t2idx(docking_start_time)
lorb_idx   = t2idx(loiter_start_time)

# ---------------------------------------------------------------------------
# Key metrics print
# ---------------------------------------------------------------------------
print()
print("=" * 62)
print("  S027 Aerial Refueling Relay – Simulation Summary")
print("=" * 62)
print(f"  Simulation duration      : {t_arr[-1]:.2f} s")
print(f"  Rendezvous waypoint      : {w_rvz}")
if loiter_start_time:
    print(f"  Tanker loiter start      : {loiter_start_time:.2f} s")
if docking_start_time:
    print(f"  Docking phase start      : {docking_start_time:.2f} s")
if dock_success_time:
    print(f"  Docking success          : {dock_success_time:.2f} s")
    if dstart_idx is not None:
        print(f"  Docking duration         : {dock_success_time - docking_start_time:.2f} s")
else:
    print("  Docking success          : FAILED")
print(f"  Energy transferred       : {E_XFER:.1f} J  (loss {ETA_LOSS*100:.0f}%)")
print(f"  Receiver final SoC       : {ER_arr[-1]:.2f} J")
print(f"  Tanker final SoC         : {ET_arr[-1]:.2f} J")
if dock_idx is not None:
    print(f"  |Δr| at dock             : {dr_arr[dock_idx]:.4f} m  (tol {EPS_P} m)")
    print(f"  |Δv| at dock             : {dv_arr[dock_idx]:.4f} m/s (tol {EPS_V} m/s)")
mission_ok = delivery_done and dock_success_time is not None
print(f"  Mission status           : {'SUCCESS' if mission_ok else 'PARTIAL / FAILED'}")
print("=" * 62)

# ---------------------------------------------------------------------------
# Figure 1 – 3D trajectory
# ---------------------------------------------------------------------------
phase_colours = {
    "fly_to_rvz":     "steelblue",
    "docking":        "limegreen",
    "deliver":        "royalblue",
    "return_home_R":  "cornflowerblue",
}

fig = plt.figure(figsize=(13, 8))
ax  = fig.add_subplot(111, projection="3d")

# Draw receiver path coloured by FSM phase
prev_phase = fsm_arr[0]
seg_start  = 0
for i in range(1, len(fsm_arr)):
    if fsm_arr[i] != prev_phase or i == len(fsm_arr) - 1:
        seg = posR_arr[seg_start:i+1]
        ax.plot(seg[:, 0], seg[:, 1], seg[:, 2],
                color=phase_colours.get(prev_phase, "blue"), linewidth=2)
        seg_start  = i
        prev_phase = fsm_arr[i]

# Tanker path
ax.plot(posT_arr[:, 0], posT_arr[:, 1], posT_arr[:, 2],
        color="orange", linewidth=1.5, alpha=0.85)

# Markers
ax.scatter(*pos_base,    marker="s", s=120, color="black",  zorder=5)
ax.scatter(*pos_target,  marker="*", s=200, color="red",    zorder=5)
ax.scatter(*w_rvz,       marker="^", s=150, color="purple", zorder=5)
if dock_idx is not None:
    ax.scatter(*posR_arr[dock_idx], marker="o", s=180, color="lime", zorder=6)

legend_elems = [
    Line2D([0],[0], color="steelblue",     lw=2, label="Receiver – fly to RVZ"),
    Line2D([0],[0], color="limegreen",     lw=2, label="Receiver – docking"),
    Line2D([0],[0], color="royalblue",     lw=2, label="Receiver – deliver"),
    Line2D([0],[0], color="cornflowerblue",lw=2, label="Receiver – return"),
    Line2D([0],[0], color="orange",        lw=2, label="Tanker"),
    Line2D([0],[0], marker="s", color="w", markerfacecolor="black",  ms=8,  label="Base"),
    Line2D([0],[0], marker="*", color="w", markerfacecolor="red",    ms=10, label="Target"),
    Line2D([0],[0], marker="^", color="w", markerfacecolor="purple", ms=8,  label="RVZ waypoint"),
    Line2D([0],[0], marker="o", color="w", markerfacecolor="lime",   ms=8,  label="Dock success"),
]
ax.legend(handles=legend_elems, loc="upper left", fontsize=8)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.set_title("S027 Aerial Refueling Relay – 3-D Trajectory")
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "trajectory_3d.png"), dpi=150)
plt.close()
print("Saved trajectory_3d.png")

# ---------------------------------------------------------------------------
# Figure 2 – Battery SoC vs time
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(11, 5))
ax.plot(t_arr, ER_arr, color="steelblue", lw=2, label="Receiver SoC")
ax.plot(t_arr, ET_arr, color="orange",    lw=2, label="Tanker SoC")
for idx, col, lbl in [(lorb_idx, "green",  "Tanker starts loiter"),
                       (dstart_idx, "purple","Docking phase start"),
                       (dock_idx,  "lime",  "Dock success / E-transfer")]:
    if idx is not None:
        ax.axvline(t_arr[idx], color=col, linestyle="--", lw=1.5, label=lbl)
ax.set_xlabel("Time [s]")
ax.set_ylabel("Battery Energy [J]")
ax.set_title("S027 – Battery State-of-Charge vs Time")
ax.legend(fontsize=9)
ax.grid(True, alpha=0.4)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "battery_soc.png"), dpi=150)
plt.savefig(os.path.join(OUT_DIR, "battery_plot.png"), dpi=150)
plt.close()
print("Saved battery_soc.png / battery_plot.png")

# ---------------------------------------------------------------------------
# Figure 3 – Relative distance & velocity during docking
# ---------------------------------------------------------------------------
if dstart_idx is not None:
    end_dock = dock_idx if dock_idx is not None else len(t_arr) - 1
    # Show a window around docking
    window_end = min(end_dock + 200, len(t_arr) - 1)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax1.semilogy(t_arr[dstart_idx:window_end],
                 np.maximum(dr_arr[dstart_idx:window_end], 1e-6),
                 color="green", lw=2)
    ax1.axhline(EPS_P, color="red",    linestyle=":", lw=1.5, label=f"ε_p = {EPS_P} m")
    ax1.axhline(D_DOCK, color="orange", linestyle="--", lw=1.2, label=f"d_dock = {D_DOCK} m")
    if dock_idx is not None:
        ax1.axvline(t_arr[dock_idx], color="lime", linestyle="--", lw=1.5, label="Dock success")
    ax1.set_ylabel("|Δr| [m]  (log scale)")
    ax1.set_title("S027 – Relative Distance & Velocity During Docking Phase")
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.4, which="both")

    ax2.semilogy(t_arr[dstart_idx:window_end],
                 np.maximum(dv_arr[dstart_idx:window_end], 1e-6),
                 color="purple", lw=2)
    ax2.axhline(EPS_V, color="red", linestyle=":", lw=1.5, label=f"ε_v = {EPS_V} m/s")
    if dock_idx is not None:
        ax2.axvline(t_arr[dock_idx], color="lime", linestyle="--", lw=1.5, label="Dock success")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("|Δv| [m/s]  (log scale)")
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.4, which="both")

    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "docking_relative_motion.png"), dpi=150)
    plt.close()
    print("Saved docking_relative_motion.png")

# ---------------------------------------------------------------------------
# Figure 4 – Top-down XY loiter orbit detail
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(8, 8))

if lorb_idx is not None:
    ax.plot(posT_arr[:lorb_idx, 0], posT_arr[:lorb_idx, 1],
            color="orange", lw=2, label="Tanker approach")
    ax.plot(posT_arr[lorb_idx:, 0], posT_arr[lorb_idx:, 1],
            color="darkorange", lw=1.5, linestyle="--", label="Tanker loiter / return")
else:
    ax.plot(posT_arr[:, 0], posT_arr[:, 1], color="orange", lw=2, label="Tanker")

# Colour-code receiver
prev_phase = fsm_arr[0]
seg_start  = 0
for i in range(1, len(fsm_arr)):
    if fsm_arr[i] != prev_phase or i == len(fsm_arr) - 1:
        seg = posR_arr[seg_start:i+1]
        ax.plot(seg[:, 0], seg[:, 1],
                color=phase_colours.get(prev_phase, "blue"), lw=2)
        seg_start  = i
        prev_phase = fsm_arr[i]

theta = np.linspace(0, 2 * np.pi, 200)
ax.plot(w_rvz[0] + R_ORB * np.cos(theta),
        w_rvz[1] + R_ORB * np.sin(theta),
        "k--", lw=1, alpha=0.5, label="Loiter circle")

ax.scatter(pos_base[0],   pos_base[1],   marker="s", s=120, color="black",  zorder=5, label="Base")
ax.scatter(pos_target[0], pos_target[1], marker="*", s=200, color="red",    zorder=5, label="Target")
ax.scatter(w_rvz[0],      w_rvz[1],      marker="^", s=150, color="purple", zorder=5, label="RVZ")
if dock_idx is not None:
    ax.scatter(posR_arr[dock_idx, 0], posR_arr[dock_idx, 1],
               marker="o", s=150, color="lime", zorder=6, label="Dock success")

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("S027 – Top-Down XY View  (Loiter Orbit Detail)")
ax.legend(fontsize=9, loc="upper right")
ax.set_aspect("equal")
ax.grid(True, alpha=0.4)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "loiter_orbit_topdown.png"), dpi=150)
plt.close()
print("Saved loiter_orbit_topdown.png")

# ---------------------------------------------------------------------------
# Figure 5 – Comparison: with vs without refueling
# ---------------------------------------------------------------------------
posR2 = pos_base.copy().astype(float)
velR2 = np.zeros(3)
E_R2  = E0_R
t2_list  = []
ER2_list = []
fsm2 = "fly_to_target"

for _ in range(max_steps):
    t2_list.append(len(t2_list) * DT)
    ER2_list.append(E_R2)
    if E_R2 <= 0:
        break
    if fsm2 == "fly_to_target":
        to_tgt = pos_target - posR2
        dist   = np.linalg.norm(to_tgt)
        if dist > 0.5:
            velR2 = V_CRUISE * to_tgt / dist
        else:
            fsm2 = "return"
    else:
        to_base = pos_base - posR2
        dist    = np.linalg.norm(to_base)
        if dist > 0.5:
            velR2 = V_CRUISE * to_base / dist
        else:
            velR2 = np.zeros(3)
            break
    posR2 += velR2 * DT
    E_R2   = max(0.0, E_R2 - battery_drain(np.linalg.norm(velR2), DT))

t2_arr  = np.array(t2_list)
ER2_arr = np.array(ER2_list)

fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(t_arr,  ER_arr,  color="steelblue", lw=2, label="With refueling (Receiver)")
ax.plot(t2_arr, ER2_arr, color="red",       lw=2, linestyle="--", label="Without refueling (Receiver)")
if dock_idx is not None:
    ax.axvline(t_arr[dock_idx], color="lime", linestyle="--", lw=1.5, label="Energy transfer event")
ax.axhline(0.0, color="black", lw=1, linestyle=":")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Battery Energy [J]")
ax.set_title("S027 – Mission Comparison: With vs Without Refueling")
ax.legend(fontsize=9)
ax.grid(True, alpha=0.4)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "mission_comparison.png"), dpi=150)
plt.close()
print("Saved mission_comparison.png")

print(f"\nAll output files saved to: {OUT_DIR}/")
