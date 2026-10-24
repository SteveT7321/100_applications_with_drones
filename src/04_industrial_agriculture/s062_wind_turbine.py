"""
S062 Wind Turbine Blade Inspection
==================================
A three-blade rotor (hub 30 m, blades 15 m, omega = 0.5 rad/s) keeps turning while a
drone inspects every 1 m segment of every blade. The drone flies in a plane 1.0 m in
front of the rotor plane and co-rotates with the blade it inspects: it sits 1.73 m
behind the blade's trailing edge (2.0 m from the blade line) and slides from tip to
root at 0.8 m/s relative to the blade, then returns to an orbit radius of 17 m (outside
the blade tips) and moves on to the next blade.

Three controllers are compared on the same wind-gust and GPS-noise sequences:
  open-loop          : feed-forward of the planned co-rotating velocity, no feedback
  phase-locked       : feed-forward + PD feedback on GPS position against the blade-phase
                       (SCADA) reference
  phase-locked+safety: phase-locked + a repulsive velocity when the measured clearance
                       to any blade falls below d_safe + 0.3 m

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s062_wind_turbine.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters ────────────────────────────────────────────────────────────────
HUB_HEIGHT   = 30.0          # m
BLADE_LENGTH = 15.0          # m
OMEGA        = 0.5           # rad/s
N_BLADES     = 3
BLADE_PHASES = 2 * np.pi * np.arange(N_BLADES) / N_BLADES
P_HUB        = np.array([0.0, 0.0, HUB_HEIGHT])
TOWER_Y      = -3.0          # m   tower axis behind the rotor plane (added)
TOWER_R      = 1.5           # m   tower radius (added)

R_ORBIT      = 17.0          # m   transit orbit radius (outside the tips)
Y_OFFSET     = 1.0           # m   drone plane in front of the rotor plane
PHI_OFFSET   = np.deg2rad(60.0)   # start: 60 deg ahead of blade 0 (midway between blades)
D_SAFE       = 1.5           # m   hard safety margin to the blade line
D_WARN       = D_SAFE + 0.3  # m   safety-controller warning zone
D_INSPECT    = 2.0           # m   target standoff (card: 1.0 m, below d_safe; see README)
EPS_INSPECT  = 0.2           # m
V_INSPECT    = 1.0           # m/s max speed relative to the blade while imaging
CAM_HALF     = np.deg2rad(30.0)
N_SEG        = 15            # 1 m segments per blade
D_STRIKE     = 0.3           # m   blade half-thickness: closer = blade strike (added)

LAT_OFF      = np.sqrt(D_INSPECT ** 2 - Y_OFFSET ** 2)   # in-plane offset behind the blade
R_IN         = 1.0           # m   innermost sweep radius
V_SWEEP      = 0.8           # m/s relative radial speed while imaging (below V_INSPECT)
V_RETURN     = 3.0           # m/s relative radial speed going back out
V_TRANSFER   = 4.0           # m/s relative tangential speed on the 17 m orbit
BLADE_ORDER  = [1, 2, 0]     # blade 1 is the nearest trailing edge from the start

KP           = 2.0           # 1/s  position feedback (card K_p)
KD           = 0.3           # s    rate feedback (card K_d)
K_SAFE       = 3.0           # 1/s  repulsive velocity per metre inside the warning zone
TAU_DRONE    = 0.2           # s    velocity-command lag (added)
V_MAX        = 15.0          # m/s  airframe speed limit (added)
GPS_SIGMA    = 0.1           # m
WIND_SIGMA   = 0.5           # m/s  gust (OU) standard deviation (added)
WIND_TAU     = 2.0           # s    gust correlation time (added)

DT           = 0.05          # s
T_MAX        = 120.0         # s
N_SEEDS      = 20
SEEDS        = list(range(N_SEEDS))
SWEEP_WIND   = [0.0, 0.25, 0.5, 1.0]
SWEEP_SEEDS  = list(range(10))
PLOT_SEED    = 0
ANIM_STEP    = 13

STRATS = ['open', 'pll', 'pll_safe']
LABELS = {'open': 'Open-loop', 'pll': 'Phase-locked', 'pll_safe': 'Phase-locked + safety'}
COLORS = {'open': '#888888', 'pll': '#E87722', 'pll_safe': '#1F77B4'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '04_industrial_agriculture', 's062_wind_turbine')

Y_HAT = np.array([0.0, 1.0, 0.0])
SEG_S = (np.arange(N_SEG) + 0.5) / N_SEG * BLADE_LENGTH      # segment midpoints (m from hub)


# ── Geometry ──────────────────────────────────────────────────────────────────

def blade_angle(t, b):
    return OMEGA * t + BLADE_PHASES[b]


def e_r(a):
    return np.array([np.cos(a), 0.0, np.sin(a)])


def e_t(a):
    return np.array([-np.sin(a), 0.0, np.cos(a)])


def blade_dists(p, t):
    """Distance to each blade line segment and the nearest point on it."""
    ang = OMEGA * t + BLADE_PHASES
    U = np.stack([np.cos(ang), np.zeros(N_BLADES), np.sin(ang)], axis=1)
    s = np.clip(U @ (p - P_HUB), 0.0, BLADE_LENGTH)
    near = P_HUB + s[:, None] * U
    return np.linalg.norm(p - near, axis=1), near


# ── Mission plan (blade-relative) ─────────────────────────────────────────────

def build_plan():
    """Phases in the frame of the blade being inspected.

    Each phase: (kind, blade, t_start, duration, params). Position in the blade frame is
    (radial a, tangential c): world = hub + a e_r + c e_t + Y_OFFSET y.
    """
    plan = []
    t = 0.0
    # start: 60 deg ahead of blade 0 on the 17 m orbit
    ang0 = PHI_OFFSET
    for n, b in enumerate(BLADE_ORDER):
        # current angle relative to blade b (radians, measured from the blade, +ve ahead)
        rel_now = ang0 - BLADE_PHASES[b] if n == 0 else None
        if n > 0:
            prev = BLADE_ORDER[n - 1]
            rel_now = -np.arctan2(LAT_OFF, R_ORBIT) + (BLADE_PHASES[prev] - BLADE_PHASES[b])
        rel_now = (rel_now + np.pi) % (2 * np.pi) - np.pi
        rel_tgt = -np.arctan2(LAT_OFF, R_ORBIT)
        if n > 0 and rel_now > rel_tgt:          # always advance in +rotation direction
            rel_now -= 2 * np.pi
        r_o = np.hypot(R_ORBIT, LAT_OFF)
        dur = abs(rel_tgt - rel_now) * r_o / V_TRANSFER
        plan.append(('transfer', b, t, dur, (rel_now, rel_tgt, r_o)))
        t += dur
        dur = (R_ORBIT - R_IN) / V_SWEEP
        plan.append(('sweep_in', b, t, dur, (R_ORBIT, R_IN)))
        t += dur
        dur = (R_ORBIT - R_IN) / V_RETURN
        plan.append(('sweep_out', b, t, dur, (R_IN, R_ORBIT)))
        t += dur
    return plan, t


PLAN, T_MISSION = build_plan()


def reference(t):
    """Reference world position/velocity and the active blade at time t (exact blade phase)."""
    ph = PLAN[-1]
    for ph in PLAN:
        if t < ph[2] + ph[3]:
            break
    kind, b, t0, dur, par = ph
    tau = min(max(t - t0, 0.0), dur)
    phi = blade_angle(t, b)
    if kind == 'transfer':
        rel0, rel1, r_o = par
        rel = rel0 + (rel1 - rel0) * tau / dur
        rel_dot = (rel1 - rel0) / dur if t < t0 + dur else 0.0
        ang = phi + rel
        pos = P_HUB + r_o * e_r(ang) + Y_OFFSET * Y_HAT
        vel = r_o * (OMEGA + rel_dot) * e_t(ang)
        imaging = False
    else:
        a0, a1 = par
        a = a0 + (a1 - a0) * tau / dur
        a_dot = (a1 - a0) / dur if t < t0 + dur else 0.0
        c = -LAT_OFF
        pos = P_HUB + a * e_r(phi) + c * e_t(phi) + Y_OFFSET * Y_HAT
        # d/dt of a e_r + c e_t with phi_dot = OMEGA: e_r' = OMEGA e_t, e_t' = -OMEGA e_r
        vel = a_dot * e_r(phi) + a * OMEGA * e_t(phi) - c * OMEGA * e_r(phi)
        imaging = kind == 'sweep_in'
    return pos, vel, b, imaging


# ── Simulation ────────────────────────────────────────────────────────────────

def noise_streams(seed, wind_sigma, n):
    """Common random numbers: OU gust and GPS noise, identical for every controller."""
    rng = np.random.default_rng(seed)
    a = np.exp(-DT / WIND_TAU)
    w = np.zeros((n, 3))
    w[0] = rng.normal(0, wind_sigma, 3)
    kick = rng.normal(0, 1, (n, 3)) * wind_sigma * np.sqrt(1 - a * a)
    for k in range(1, n):
        w[k] = a * w[k - 1] + kick[k]
    gps = rng.normal(0, GPS_SIGMA, (n, 3))
    return w, gps


def simulate(strategy, seed, wind_sigma=WIND_SIGMA, keep_log=False):
    n = int(round(T_MAX / DT))
    wind, gps = noise_streams(seed, wind_sigma, n)
    pos, vel, _, _ = reference(0.0)
    pos = pos.copy()
    v_air = vel.copy()
    inspected = np.zeros((N_BLADES, N_SEG), dtype=bool)
    viol_steps = 0
    strike = False
    crash = ''
    insp_geom = np.zeros((N_BLADES, N_SEG), dtype=bool)
    min_clear = np.inf
    clear_insp, clear_transit = [], []
    rep_steps = 0
    lg = {k: [] for k in ('t', 'pos', 'clear', 'phase_err')} if keep_log else None

    k_end = n
    for k in range(n):
        t = k * DT
        if t > T_MISSION:
            k_end = k
            break
        p_ref, v_ref, b, imaging = reference(t)
        v_true = v_air + wind[k]

        # ── safety and inspection bookkeeping on the true state ──
        d_true, near_true = blade_dists(pos, t)
        dmin = float(d_true.min())
        min_clear = min(min_clear, dmin)
        (clear_insp if imaging else clear_transit).append(dmin)
        if dmin < D_SAFE:
            viol_steps += 1
        tower_clear = np.hypot(pos[0], pos[1] - TOWER_Y) - TOWER_R
        if dmin < D_STRIKE:
            strike = True
            crash = 'blade'
        elif tower_clear <= 0 and pos[2] <= HUB_HEIGHT:
            crash = 'tower'
        elif pos[2] <= 0.0:
            crash = 'ground'
        # camera boresight: towards the nearest point of the target blade (gimbal on SCADA)
        bore = near_true[b] - pos
        bore /= np.linalg.norm(bore)
        ang = OMEGA * t + BLADE_PHASES
        U = np.stack([np.cos(ang), np.zeros(N_BLADES), np.sin(ang)], axis=1)
        Tn = np.stack([-np.sin(ang), np.zeros(N_BLADES), np.cos(ang)], axis=1)
        rel = P_HUB + SEG_S[None, :, None] * U[:, None, :] - pos        # (3, 15, 3)
        dist = np.linalg.norm(rel, axis=2)
        v_rel = np.linalg.norm(v_true - SEG_S[None, :, None] * OMEGA * Tn[:, None, :], axis=2)
        geom = (dist <= D_INSPECT + EPS_INSPECT) & ((rel @ bore) / dist >= np.cos(CAM_HALF))
        inspected |= geom & (v_rel <= V_INSPECT)
        insp_geom |= geom
        if keep_log:
            lg['t'].append(t)
            lg['pos'].append(pos.copy())
            lg['clear'].append(dmin)
            lg['phase_err'].append(float(np.linalg.norm(pos - p_ref)))
        if crash:                  # a collision ends the flight
            k_end = k + 1
            break

        # ── controller ──
        # Feed-forward = exact plan displacement over the next step, so that Euler
        # integration of the feed-forward alone reproduces the plan (no discretisation drift)
        v_ff = (reference(t + DT)[0] - p_ref) / DT
        v_cmd = v_ff.copy()
        if strategy != 'open':
            meas = pos + gps[k]
            # rate term uses the true ground velocity (incl. wind), assumed measured noise-free
            fb = KP * (p_ref - meas) + KD * (v_ref - v_true)
            if strategy == 'pll_safe':
                d_m, near_m = blade_dists(meas, t)
                for bb in range(N_BLADES):
                    if d_m[bb] < D_WARN:
                        nvec = (meas - near_m[bb]) / d_m[bb]
                        toward = fb @ nvec
                        if toward < 0:                 # drop feedback that pushes inwards
                            fb = fb - toward * nvec
                        v_cmd = v_cmd + K_SAFE * (D_WARN - d_m[bb]) * nvec
                        rep_steps += 1
            v_cmd = v_cmd + fb
        sp = np.linalg.norm(v_cmd)
        if sp > V_MAX:
            v_cmd *= V_MAX / sp
        v_air = v_air + (v_cmd - v_air) * DT / TAU_DRONE
        assert np.linalg.norm(v_air) <= V_MAX + 1e-9
        pos = pos + (v_air + wind[k]) * DT

    res = {
        'coverage': 100.0 * inspected.mean(),
        'inspected': inspected,
        'viol_steps': viol_steps,
        'viol_frac': 100.0 * viol_steps / k_end,
        'strike': strike,
        'crash': crash,
        'coverage_geom': 100.0 * insp_geom.mean(),
        'min_clear': float(min_clear),
        'mean_clear_insp': float(np.mean(clear_insp)) if clear_insp else np.nan,
        'mean_clear_transit': float(np.mean(clear_transit)) if clear_transit else np.nan,
        't_end': k_end * DT,
        'rep_steps': rep_steps,
    }
    if keep_log:
        for kk in lg:
            lg[kk] = np.array(lg[kk])
        res['log'] = lg
    return res


def run_simulation():
    mc = {s: [simulate(s, seed, keep_log=(seed == PLOT_SEED)) for seed in SEEDS] for s in STRATS}
    sweep = {s: {w: [simulate(s, seed, wind_sigma=w) for seed in SWEEP_SEEDS]
                 for w in SWEEP_WIND} for s in STRATS}
    return mc, sweep


# ── Plotting ──────────────────────────────────────────────────────────────────

def _draw_turbine(ax, t_snaps):
    zt = np.linspace(0, HUB_HEIGHT, 2)
    th = np.linspace(0, 2 * np.pi, 24)
    TH, ZT = np.meshgrid(th, zt)
    ax.plot_surface(TOWER_R * np.cos(TH), TOWER_Y + TOWER_R * np.sin(TH), ZT,
                    color='grey', alpha=0.35, linewidth=0)
    ring = np.linspace(0, 2 * np.pi, 200)
    ax.plot(BLADE_LENGTH * np.cos(ring), np.zeros_like(ring),
            HUB_HEIGHT + BLADE_LENGTH * np.sin(ring), color='silver', ls='--', lw=1)
    for ts in t_snaps:
        for b in range(N_BLADES):
            tip = P_HUB + BLADE_LENGTH * e_r(blade_angle(ts, b))
            ax.plot(*zip(P_HUB, tip), color='dimgray', lw=1.5, alpha=0.5)


def plot_trajectory_3d(mc, out_dir):
    lg = mc['pll_safe'][PLOT_SEED]['log']
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    _draw_turbine(ax, [0.0, T_MISSION / 3, 2 * T_MISSION / 3])
    P, c = lg['pos'], lg['clear']
    col = np.where(c < D_SAFE, 'red', np.where(c < D_WARN, 'gold', 'tab:blue'))
    ax.scatter(P[:, 0], P[:, 1], P[:, 2], c=col, s=2)
    ax.scatter(*P[0], color='green', s=60, label='Start')
    ax.scatter(*P[-1], color='red', s=60, label='End')
    ax.plot([], [], color='tab:blue', label=f'clearance ≥ {D_WARN:.1f} m')
    ax.plot([], [], color='gold', label='warning zone')
    ax.plot([], [], color='red', label=f'< d_safe = {D_SAFE} m')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_ylim(-6, 6)
    ax.set_box_aspect((1, 0.35, 1))
    ax.view_init(elev=15, azim=-60)
    r = mc['pll_safe'][PLOT_SEED]
    ax.set_title(f'S062 Phase-locked + safety, seed {PLOT_SEED} (world frame)\n'
                 f"coverage {r['coverage']:.1f}%, steps below d_safe: {r['viol_steps']}")
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _rotor_frame(lg):
    """Drone position in the rotor-fixed frame (blade 0 along +x'), in-plane coordinates."""
    P = lg['pos'] - P_HUB
    ang = -OMEGA * lg['t']
    x = P[:, 0] * np.cos(ang) - P[:, 2] * np.sin(ang)
    z = P[:, 0] * np.sin(ang) + P[:, 2] * np.cos(ang)
    return x, z


def plot_rotor_frame(mc, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 6.3))
    for ax, s in zip(axes, STRATS):
        r = mc[s][PLOT_SEED]
        x, z = _rotor_frame(r['log'])
        for b in range(N_BLADES):
            u = e_r(BLADE_PHASES[b])
            ax.plot([0, BLADE_LENGTH * u[0]], [0, BLADE_LENGTH * u[2]], color='dimgray', lw=4)
            ax.text(16 * u[0], 16 * u[2], f'B{b}', ha='center', fontsize=10)
        ring = np.linspace(0, 2 * np.pi, 200)
        ax.plot(R_ORBIT * np.cos(ring), R_ORBIT * np.sin(ring), ':', color='silver')
        ax.plot(x, z, color=COLORS[s], lw=1.0)
        ax.plot(x[0], z[0], 'o', color='green', ms=8)
        ax.plot(x[-1], z[-1], 'X' if r['crash'] else 's', color='red', ms=10)
        ax.set_aspect('equal')
        ax.set_xlim(-22, 22)
        ax.set_ylim(-22, 22)
        ax.set_xlabel("x' (m, rotor-fixed)")
        end = f"{r['crash'].upper()} CRASH at t = {r['t_end']:.1f} s" if r['crash'] else 'completed'
        ax.set_title(f"{LABELS[s]} — {end}\ncoverage {r['coverage']:.1f}%, "
                     f"min clearance {r['min_clear']:.2f} m", fontsize=11)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("z' (m, rotor-fixed)")
    fig.suptitle(f'S062 Drone path in the co-rotating rotor frame (seed {PLOT_SEED}, '
                 f'gust σ = {WIND_SIGMA} m/s); blades are static here. Green = start, red = end', y=1.03)
    plt.tight_layout()
    path = os.path.join(out_dir, 'rotor_frame_paths.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_clearance(mc, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for ax, s in zip(axes, STRATS):
        r = mc[s][PLOT_SEED]
        lg = r['log']
        ax.plot(lg['t'], lg['clear'], color=COLORS[s], lw=1.0)
        ax.axhline(D_SAFE, color='red', ls='--', lw=1, label=f'd_safe = {D_SAFE} m')
        ax.axhline(D_INSPECT, color='orange', ls=':', lw=1, label=f'd_inspect = {D_INSPECT} m')
        ax.fill_between([0, T_MISSION], 0, D_SAFE, color='red', alpha=0.08)
        for ph in PLAN:
            if ph[0] == 'sweep_in':
                ax.axvspan(ph[2], ph[2] + ph[3], color='green', alpha=0.06)
        ax.set_ylim(0, 6)
        ax.set_ylabel('min clearance (m)')
        ax.set_title(f"{LABELS[s]} — coverage {r['coverage']:.1f}%, "
                     f"{r['viol_steps']} steps below d_safe", fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel('time (s)   (green bands = imaging sweeps)')
    fig.suptitle(f'S062 Minimum blade clearance, seed {PLOT_SEED}')
    plt.tight_layout()
    path = os.path.join(out_dir, 'clearance_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_coverage_heatmap(mc, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(16, 3.8), gridspec_kw={'wspace': 0.4})
    for ax, s in zip(axes, STRATS):
        frac = np.mean([r['inspected'] for r in mc[s]], axis=0)
        im = ax.imshow(frac, aspect='auto', cmap='RdYlGn', vmin=0, vmax=1, origin='lower',
                       extent=[0, BLADE_LENGTH, -0.5, N_BLADES - 0.5])
        ax.set_yticks(range(N_BLADES))
        ax.set_yticklabels([f'Blade {b}' for b in range(N_BLADES)])
        ax.set_xlabel('distance from hub (m)')
        cov = np.mean([r['coverage'] for r in mc[s]])
        ax.set_title(f'{LABELS[s]}\nmean coverage {cov:.1f}%', fontsize=10)
    fig.colorbar(im, ax=axes, fraction=0.02, label=f'fraction of {N_SEEDS} seeds inspected')
    fig.suptitle('S062 Segment inspection rate (green = inspected in every seed)', y=1.04)
    path = os.path.join(out_dir, 'coverage_heatmap.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, sweep, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.5))
    fields = [('coverage', 'coverage (%)'), ('viol_frac', 'time below d_safe (%)'),
              ('min_clear', 'minimum clearance (m)')]
    for ax, (f, lab) in zip(axes[:3], fields):
        data = [[r[f] for r in mc[s]] for s in STRATS]
        bp = ax.boxplot(data, patch_artist=True, widths=0.6)
        for patch, s in zip(bp['boxes'], STRATS):
            patch.set_facecolor(COLORS[s])
            patch.set_alpha(0.7)
        ax.set_xticks([1, 2, 3])
        ax.set_xticklabels([LABELS[s] for s in STRATS], fontsize=8)
        ax.set_title(lab)
        ax.grid(alpha=0.3)
    axes[2].axhline(D_SAFE, color='red', ls='--', lw=1)
    axes[2].axhline(D_STRIKE, color='k', ls=':', lw=1, label='strike distance')
    axes[2].legend(fontsize=8)
    ax = axes[3]
    for s in STRATS:
        m = [np.mean([r['viol_frac'] for r in sweep[s][w]]) for w in SWEEP_WIND]
        ax.plot(SWEEP_WIND, m, 'o-', color=COLORS[s], label=LABELS[s])
    ax.set_yscale('symlog', linthresh=0.1)
    ax.set_xlabel('gust σ (m/s)')
    ax.set_title(f'time below d_safe (%) vs gust, {len(SWEEP_SEEDS)} seeds')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.suptitle(f'S062 Monte Carlo over {N_SEEDS} seeds (common random numbers, '
                 f'gust σ = {WIND_SIGMA} m/s)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(mc, out_dir):
    lg = mc['pll_safe'][PLOT_SEED]['log']
    n = len(lg['t'])
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    fig = plt.figure(figsize=(6.5, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    _draw_turbine(ax, [])
    ax.set_xlim(-20, 20)
    ax.set_ylim(-6, 6)
    ax.set_zlim(10, 50)
    ax.set_box_aspect((1, 0.3, 1))
    ax.view_init(elev=12, azim=-65)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    blades = [ax.plot([], [], [], color='steelblue', lw=3)[0] for _ in range(N_BLADES)]
    dot, = ax.plot([], [], [], 'o', color='red', ms=7)
    trail, = ax.plot([], [], [], color='tomato', lw=1, alpha=0.7)
    P = lg['pos']

    def update(fi):
        i = min(fi * ANIM_STEP, n - 1)
        t = lg['t'][i]
        for b, ln in enumerate(blades):
            tip = P_HUB + BLADE_LENGTH * e_r(blade_angle(t, b))
            ln.set_data([P_HUB[0], tip[0]], [P_HUB[1], tip[1]])
            ln.set_3d_properties([P_HUB[2], tip[2]])
        lo = max(0, i - 60 * ANIM_STEP // 4)
        trail.set_data(P[lo:i + 1, 0], P[lo:i + 1, 1])
        trail.set_3d_properties(P[lo:i + 1, 2])
        dot.set_data([P[i, 0]], [P[i, 1]])
        dot.set_3d_properties([P[i, 2]])
        ax.set_title(f"S062 phase-locked + safety — t = {t:.1f} s\n"
                     f"min clearance {lg['clear'][i]:.2f} m", fontsize=10)
        return blades + [dot, trail]

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10)
    plt.close(fig)
    print(f'Saved: {path}  ({n_frames} frames)')


def _ms(v, fmt='.2f'):
    v = np.asarray(v, dtype=float)
    return f'{v.mean():{fmt}} ± {v.std():{fmt}}'


if __name__ == '__main__':
    mc, sweep = run_simulation()
    print('\n=== S062 Wind Turbine Blade Inspection — Results ===')
    print(f'  In-plane offset behind the blade: {LAT_OFF:.3f} m (distance to blade line '
          f'{D_INSPECT:.1f} m at y = {Y_OFFSET:.1f} m)')
    print(f'  Inspection window along the blade: ±{np.sqrt((D_INSPECT + EPS_INSPECT) ** 2 - D_INSPECT ** 2):.3f} m')
    print(f'  Lateral error that reaches d_safe: {LAT_OFF - np.sqrt(D_SAFE ** 2 - Y_OFFSET ** 2):.3f} m; '
          f'outward error that loses imaging: '
          f'{np.sqrt((D_INSPECT + EPS_INSPECT) ** 2 - Y_OFFSET ** 2) - LAT_OFF:.3f} m')
    print(f'  Card orbit check: r = 17 m, y = 1 m gives min blade distance '
          f'{np.hypot(R_ORBIT - BLADE_LENGTH, Y_OFFSET):.3f} m (> d_inspect + eps = '
          f'{D_INSPECT + EPS_INSPECT:.1f} m with the card d_inspect = 1.0: '
          f'{np.hypot(R_ORBIT - BLADE_LENGTH, Y_OFFSET) > 1.2})')
    print(f'  Planned mission time: {T_MISSION:.1f} s (T_max {T_MAX:.0f} s)')
    vmax_plan = max(np.linalg.norm(reference(t)[1]) for t in np.arange(0, T_MISSION, DT))
    print(f'  Max planned ground speed: {vmax_plan:.2f} m/s (limit {V_MAX:.0f} m/s)')
    p_chk = reference(0.0)[0].copy()
    dev_ff, dev_an = 0.0, 0.0
    p_an = p_chk.copy()
    for kk in range(int(T_MISSION / DT)):
        t = kk * DT
        p_now, v_now = reference(t)[:2]
        p_chk = p_chk + (reference(t + DT)[0] - p_now)
        p_an = p_an + v_now * DT
        dev_ff = max(dev_ff, float(np.linalg.norm(p_chk - reference(t + DT)[0])))
        dev_an = max(dev_an, float(np.linalg.norm(p_an - reference(t + DT)[0])))
    print(f'  Plan self-consistency (no lag, no wind): Euler on the step feed-forward drifts '
          f'{dev_ff:.2e} m; Euler on the analytic reference velocity would drift {dev_an:.2f} m')
    print(f'  Monte Carlo: {N_SEEDS} seeds, gust sigma {WIND_SIGMA} m/s, GPS {GPS_SIGMA} m')
    for s in STRATS:
        R = mc[s]
        g = lambda f: [r[f] for r in R]          # noqa: E731
        print(f'  [{LABELS[s]}]')
        print(f"    Coverage              : {_ms(g('coverage'), '.1f')} %  "
              f"(min {np.min(g('coverage')):.1f}, max {np.max(g('coverage')):.1f})")
        print(f"    Steps below d_safe    : {_ms(g('viol_steps'), '.1f')}  "
              f"({_ms(g('viol_frac'))} % of flight); runs with zero: "
              f"{sum(v == 0 for v in g('viol_steps'))}/{N_SEEDS}")
        print(f"    Coverage, distance + camera only (speed test ignored): "
              f"{_ms(g('coverage_geom'), '.1f')} %")
        print(f"    Crashes: blade {g('crash').count('blade')}, tower {g('crash').count('tower')}, "
              f"ground {g('crash').count('ground')} of {N_SEEDS}  "
              f"(mean end time {np.mean(g('t_end')):.1f} s)")
        print(f"    Min clearance         : {_ms(g('min_clear'))} m (worst {np.min(g('min_clear')):.2f} m)")
        print(f"    Mean clearance imaging: {_ms(g('mean_clear_insp'))} m; transit: "
              f"{_ms(g('mean_clear_transit'))} m")
        if s == 'pll_safe':
            print(f"    Steps with repulsion  : {_ms(g('rep_steps'), '.1f')}")
    vp = np.array([r['viol_steps'] for r in mc['pll']])
    vs = np.array([r['viol_steps'] for r in mc['pll_safe']])
    cp = np.array([r['coverage'] for r in mc['pll']])
    cs = np.array([r['coverage'] for r in mc['pll_safe']])
    print(f'  Per-seed (safety vs phase-locked): fewer violation steps {int(np.sum(vs < vp))}, '
          f'equal {int(np.sum(vs == vp))}, more {int(np.sum(vs > vp))}; '
          f'coverage higher {int(np.sum(cs > cp))}, equal {int(np.sum(cs == cp))}, '
          f'lower {int(np.sum(cs < cp))}')
    for s in STRATS:
        r = mc[s][PLOT_SEED]
        print(f"  Seed {PLOT_SEED} {LABELS[s]}: coverage {r['coverage']:.1f} %, "
              f"{r['viol_steps']} steps below d_safe, min clearance {r['min_clear']:.2f} m, "
              f"crash: {r['crash'] or 'none'}")
    print('  Gust sweep (10 seeds): sigma -> coverage / time below d_safe / crashes')
    for s in STRATS:
        for w in SWEEP_WIND:
            R = sweep[s][w]
            print(f"    {LABELS[s]:22s} sigma={w:.2f}: coverage {_ms([r['coverage'] for r in R], '.1f')} %, "
                  f"below d_safe {_ms([r['viol_frac'] for r in R])} %, "
                  f"crashes {sum(bool(r['crash']) for r in R)}/{len(R)}, "
                  f"min clearance {np.min([r['min_clear'] for r in R]):.2f} m")

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_trajectory_3d(mc, out)
    plot_rotor_frame(mc, out)
    plot_clearance(mc, out)
    plot_coverage_heatmap(mc, out)
    plot_monte_carlo(mc, sweep, out)
    save_animation(mc, out)
