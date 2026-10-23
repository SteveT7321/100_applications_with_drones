"""
S016 Airspace Defense
======================
A protected zone (sphere of radius 2 m at the origin) must be defended against
intruders that fly straight toward its center.  A single defender starts on the
top of the zone surface at (0, 0, 2) and tries to intercept each intruder before
it crosses the zone surface.  Two guidance laws are compared:

* Pure Pursuit (PP): fly at v_D straight at the intruder's current position.
* Proportional Navigation (PN, N = 3): a constant-speed vehicle launched along
  the initial line of sight; the PN command a = N * v_c * d(lambda_hat)/dt
  turns its velocity vector (speed stays at v_D).

Both laws use the same speed, the same launch heading and the same capture test.

Besides the card's four intruder starts, the script sweeps the intruder start
position over the (rho, z) half-plane (the problem is symmetric about the z-axis)
to map the capture-guarantee region of each law, and compares it with
  - the card's time test  |p_D - p_I| / v_D <= (|p_I| - R_zone) / v_I, and
  - the exact optimal-intercept (Apollonius) test: some point of the intruder's
    path before the zone is reachable by the defender no later than the intruder.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s016_airspace_defense.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3D projection)
import matplotlib.animation as animation

# ── Parameters ─────────────────────────────────────────────────────────────
R_ZONE      = 2.0                           # m   — protected zone radius
DEFENDER_START = np.array([0.0, 0.0, 2.0])  # m   — defender initial position
V_DEFENDER  = 6.0                           # m/s — defender speed
V_INTRUDER  = 4.0                           # m/s — intruder speed
SPEED_RATIO = V_DEFENDER / V_INTRUDER       # 1.5
PN_GAIN     = 3.0                           # —   — proportional navigation gain N
CAPTURE_R   = 0.15                          # m   — capture radius
DT          = 0.02                          # s   — timestep
T_MAX       = 10.0                          # s   — max simulation time

INTRUDER_STARTS = [
    np.array([ 8.0,  0.0, 2.0]),
    np.array([-8.0,  0.0, 2.0]),
    np.array([ 0.0,  8.0, 2.0]),
    np.array([ 6.0,  6.0, 2.0]),
]
INTRUDER_LABELS = ['I1 (+x)', 'I2 (-x)', 'I3 (+y)', 'I4 (+x+y)']
INTRUDER_COLORS = ['#e63946', '#f4a261', '#2a9d8f', '#8338ec']
STRATEGIES = ['pure_pursuit', 'proportional_nav']
STRAT_SHORT = {'pure_pursuit': 'PP', 'proportional_nav': 'PN'}
STRATEGY_LINESTYLES = {'pure_pursuit': '-', 'proportional_nav': '--'}

# Guarantee-region sweep: intruder start (rho, 0, z) on a grid in the half-plane
SWEEP_RHO = np.arange(0.0, 9.0 + 1e-9, 0.25)    # m
SWEEP_Z   = np.arange(-8.0, 6.0 + 1e-9, 0.25)   # m
# Approach-angle sweep: intruder start at a range where interception is marginal
RING_RANGE = 4.0                                 # m
RING_ELEV_DEG = np.arange(-85.0, 85.0 + 1e-9, 5.0)   # deg

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's016_airspace_defense',
)


# ── Helpers ─────────────────────────────────────────────────────────────────

def sphere_mesh(radius, center=(0, 0, 0), n=30):
    """Return (X, Y, Z) mesh arrays for a sphere surface."""
    u = np.linspace(0, 2 * np.pi, n)
    v = np.linspace(0, np.pi, n)
    X = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    Y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    Z = center[2] + radius * np.outer(np.ones(n), np.cos(v))
    return X, Y, Z


def penetration_time(p_int_0):
    """Time for a straight-flying intruder to reach the zone surface."""
    return (np.linalg.norm(p_int_0) - R_ZONE) / V_INTRUDER


def card_time_test(p_int_0, p_def=DEFENDER_START):
    """Card condition: |p_D - p_I| / v_D <= (|p_I| - R_zone) / v_I."""
    return np.linalg.norm(p_def - p_int_0) / V_DEFENDER <= penetration_time(p_int_0)


def optimal_intercept(p_int_0, p_def=DEFENDER_START, n=4000):
    """
    Exact test for a straight-flying intruder and a defender that can turn
    instantly: the earliest time t <= T_pen at which the defender, flying a
    straight line at v_D, can be within CAPTURE_R of the intruder.
    Returns t* or None if no such t exists (penetration cannot be prevented).
    """
    t_pen = penetration_time(p_int_0)
    v_int = -p_int_0 / np.linalg.norm(p_int_0) * V_INTRUDER
    t = np.linspace(0.0, t_pen, n)
    p_i = p_int_0[None, :] + t[:, None] * v_int[None, :]
    gap = np.linalg.norm(p_i - p_def[None, :], axis=1) - CAPTURE_R - V_DEFENDER * t
    ok = np.nonzero(gap <= 0.0)[0]
    return float(t[ok[0]]) if len(ok) else None


def apollonius_sphere(p_int_0, p_def=DEFENDER_START, k=SPEED_RATIO):
    """
    Apollonius sphere {X : |X - p_D| = k |X - p_I|}: points both reach at the
    same time.  Inside it (around p_I) the intruder arrives first.
    """
    c = (k ** 2 * p_int_0 - p_def) / (k ** 2 - 1.0)
    r = k * np.linalg.norm(p_int_0 - p_def) / (k ** 2 - 1.0)
    return c, r


def first_hit(r0, d, radius):
    """
    Smallest s in [0, 1] with |r0 + s d| <= radius (linear motion over a step),
    or None.
    """
    if np.dot(r0, r0) <= radius ** 2:
        return 0.0
    a = np.dot(d, d)
    if a < 1e-15:
        return None
    b = 2.0 * np.dot(r0, d)
    c = np.dot(r0, r0) - radius ** 2
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    s = (-b - np.sqrt(disc)) / (2 * a)
    return s if 0.0 <= s <= 1.0 else None


# ── Guidance laws ────────────────────────────────────────────────────────────

def pure_pursuit_velocity(p_def, p_int):
    """Velocity of v_D straight at the intruder's current position."""
    d = p_int - p_def
    norm_d = np.linalg.norm(d)
    if norm_d < 1e-9:
        return np.zeros(3)
    return d / norm_d * V_DEFENDER


def pn_velocity(p_def, p_int, v_def, v_int):
    """
    Proportional Navigation for a constant-speed vehicle:
        lambda_dot = (v_rel - (v_rel . lambda_hat) lambda_hat) / r
        v_c        = -v_rel . lambda_hat
        a_cmd      = N * v_c * lambda_dot
    The command is integrated over one step and the velocity is rescaled to v_D
    (the card's "clipped to v_D"; the vehicle always flies at full speed).
    """
    los = p_int - p_def
    r = np.linalg.norm(los)
    if r < 1e-9:
        return v_def
    los_hat = los / r
    v_rel = v_int - v_def
    los_dot = (v_rel - np.dot(v_rel, los_hat) * los_hat) / r
    v_c = -np.dot(v_rel, los_hat)
    a_cmd = PN_GAIN * v_c * los_dot
    v_new = v_def + a_cmd * DT
    return v_new / np.linalg.norm(v_new) * V_DEFENDER


# ── Simulation ───────────────────────────────────────────────────────────────

def simulate(p_int_0, strategy='pure_pursuit', record=True):
    """
    Simulate one defender vs one intruder engagement.

    Each step both vehicles move with constant velocity; capture (|p_D - p_I| <
    CAPTURE_R) and penetration (|p_I| <= R_zone) are detected inside the step
    by solving for the first crossing, and whichever happens first decides.
    Returns dict(traj_d, traj_i, outcome, time, miss).
    """
    p_def = DEFENDER_START.astype(float).copy()
    p_int = np.asarray(p_int_0, dtype=float).copy()
    v_int = -p_int / np.linalg.norm(p_int) * V_INTRUDER
    # PN is launched along the initial line of sight at full speed (the same
    # initial heading PP uses on its first step).
    los0 = p_int - p_def
    v_def = los0 / np.linalg.norm(los0) * V_DEFENDER

    traj_d = [p_def.copy()]
    traj_i = [p_int.copy()]
    miss = np.linalg.norm(p_int - p_def)
    outcome, t_end = 'timeout', T_MAX

    for step in range(int(round(T_MAX / DT))):
        if strategy == 'pure_pursuit':
            v_def = pure_pursuit_velocity(p_def, p_int)
        else:
            v_def = pn_velocity(p_def, p_int, v_def, v_int)

        r0 = p_int - p_def
        d_rel = (v_int - v_def) * DT
        s_cap = first_hit(r0, d_rel, CAPTURE_R)
        s_pen = first_hit(p_int, v_int * DT, R_ZONE)

        # closest approach inside this step (for the miss-distance metric)
        a = np.dot(d_rel, d_rel)
        s_ca = 0.0 if a < 1e-15 else float(np.clip(-np.dot(r0, d_rel) / a, 0.0, 1.0))
        s_lim = min(x for x in (s_cap, s_pen, 1.0) if x is not None)
        miss = min(miss, np.linalg.norm(r0 + min(s_ca, s_lim) * d_rel))

        if s_cap is not None and (s_pen is None or s_cap <= s_pen):
            s, outcome = s_cap, 'defender_wins'
        elif s_pen is not None:
            s, outcome = s_pen, 'intruder_wins'
        else:
            s = 1.0
        p_def = p_def + s * v_def * DT
        p_int = p_int + s * v_int * DT
        if record:
            traj_d.append(p_def.copy())
            traj_i.append(p_int.copy())
        if s < 1.0 or outcome != 'timeout':
            t_end = (step + s) * DT
            break

    return {'traj_d': np.array(traj_d), 'traj_i': np.array(traj_i),
            'outcome': outcome, 'time': t_end, 'miss': float(miss),
            'margin': float(np.linalg.norm(p_int) - R_ZONE),
            'p0': np.asarray(p_int_0, dtype=float)}


def run_card_cases():
    """The card's 4 intruder starts x 2 guidance laws."""
    results = {}
    print(f'{"Intruder":12s} | {"law":3s} | {"outcome":14s} | {"time":>7s} | '
          f'{"zone margin":>11s} | {"T_pen":>6s} | {"t* opt":>6s} | card test')
    for p0, label in zip(INTRUDER_STARTS, INTRUDER_LABELS):
        t_opt = optimal_intercept(p0)
        for strat in STRATEGIES:
            res = simulate(p0, strat)
            res['t_opt'] = t_opt
            res['t_pen'] = penetration_time(p0)
            results[(label, strat)] = res
            print(f'{label:12s} | {STRAT_SHORT[strat]:3s} | {res["outcome"]:14s} | '
                  f'{res["time"]:6.3f}s | {res["margin"]:10.3f}m | {res["t_pen"]:5.3f}s | '
                  f'{t_opt:5.3f}s | {"pass" if card_time_test(p0) else "fail"}')
    return results


def run_region_sweep():
    """Capture-guarantee region over the (rho, z) half-plane."""
    R, Z = np.meshgrid(SWEEP_RHO, SWEEP_Z)
    shape = R.shape
    valid = np.zeros(shape, bool)
    cap = {s: np.zeros(shape, bool) for s in STRATEGIES}
    card = np.zeros(shape, bool)
    opt = np.zeros(shape, bool)
    for idx in np.ndindex(shape):
        p0 = np.array([R[idx], 0.0, Z[idx]])
        if np.linalg.norm(p0) <= R_ZONE + 1e-9 or np.linalg.norm(p0 - DEFENDER_START) < CAPTURE_R:
            continue
        valid[idx] = True
        card[idx] = card_time_test(p0)
        opt[idx] = optimal_intercept(p0) is not None
        for s in STRATEGIES:
            cap[s][idx] = simulate(p0, s, record=False)['outcome'] == 'defender_wins'
    n = int(valid.sum())
    print(f'\nGuarantee-region sweep: {n} intruder starts '
          f'(rho 0-{SWEEP_RHO[-1]:.0f} m, z {SWEEP_Z[0]:.0f}..{SWEEP_Z[-1]:.0f} m, 0.25 m grid)')
    print(f'  exact optimal-intercept test: {int(opt[valid].sum())}/{n} interceptable')
    print(f'  card time test              : {int(card[valid].sum())}/{n} pass')
    for s in STRATEGIES:
        c = cap[s][valid]
        print(f'  {STRAT_SHORT[s]} captures                 : {int(c.sum())}/{n}  '
              f'(of the {int(opt[valid].sum())} interceptable: {int((c & opt[valid]).sum())})')
    pp, pn = cap['pure_pursuit'][valid], cap['proportional_nav'][valid]
    print(f'  PN only: {int((pn & ~pp).sum())}   PP only: {int((pp & ~pn).sum())}')
    print(f'  card test pass but optimal fails: {int((card[valid] & ~opt[valid]).sum())}; '
          f'optimal ok but card test fails: {int((~card[valid] & opt[valid]).sum())}')
    return {'R': R, 'Z': Z, 'valid': valid, 'cap': cap, 'card': card, 'opt': opt}


def run_angle_sweep():
    """Intruder at RING_RANGE (4 m, where interception is marginal), approach elevation varied."""
    out = {s: {'miss': [], 'time': [], 'outcome': []} for s in STRATEGIES}
    t_opt = []
    for el in RING_ELEV_DEG:
        e = np.radians(el)
        p0 = RING_RANGE * np.array([np.cos(e), 0.0, np.sin(e)])
        t = optimal_intercept(p0)
        t_opt.append(np.nan if t is None else t)
        for s in STRATEGIES:
            res = simulate(p0, s, record=False)
            out[s]['miss'].append(res['miss'])
            out[s]['time'].append(res['time'])
            out[s]['outcome'].append(res['outcome'])
    print(f'\nApproach-angle sweep (range {RING_RANGE:.3f} m, elevation '
          f'{RING_ELEV_DEG[0]:.0f}..{RING_ELEV_DEG[-1]:.0f} deg, {len(RING_ELEV_DEG)} starts):')
    print('  elev | t* opt | PP: outcome  time  miss | PN: outcome  time  miss')
    for i, el in enumerate(RING_ELEV_DEG):
        row = f'  {el:4.0f} | ' + ('   --  ' if np.isnan(t_opt[i]) else f'{t_opt[i]:.3f}s')
        for s in STRATEGIES:
            o = out[s]['outcome'][i]
            row += (f' | {"capture" if o == "defender_wins" else "PENETR."} '
                    f'{out[s]["time"][i]:.3f}s {out[s]["miss"][i]:.3f}m')
        print(row)
    for s in STRATEGIES:
        n_w = sum(o == 'defender_wins' for o in out[s]['outcome'])
        print(f'  {STRAT_SHORT[s]}: {n_w}/{len(RING_ELEV_DEG)} captured')
    n_opt = int(np.sum(~np.isnan(t_opt)))
    print(f'  optimal-intercept test: {n_opt}/{len(RING_ELEV_DEG)} interceptable')
    for s in out:
        for k in out[s]:
            out[s][k] = np.array(out[s][k])
    out['t_opt'] = np.array(t_opt)
    return out


def run_simulation():
    """Card cases, guarantee-region sweep and approach-angle sweep."""
    print('S016 Airspace Defense — Simulation Results')
    print('=' * 60)
    cases = run_card_cases()
    print('\nCard time test (|D-I|/v_D <= (|I|-R)/v_I), Apollonius sphere:')
    for p0, label in zip(INTRUDER_STARTS, INTRUDER_LABELS):
        c, r = apollonius_sphere(p0)
        print(f'  {label:12s} | |D-I| = {np.linalg.norm(DEFENDER_START - p0):.3f} m  '
              f'threshold v_D/v_I(|I|-R) = {SPEED_RATIO * (np.linalg.norm(p0) - R_ZONE):.3f} m  '
              f'| Apollonius centre ({c[0]:.2f},{c[1]:.2f},{c[2]:.2f}) r={r:.2f} m')
    region = run_region_sweep()
    ring = run_angle_sweep()
    return {'cases': cases, 'region': region, 'ring': ring}


# ── Plots ────────────────────────────────────────────────────────────────────

def plot_3d_trajectories(cases, out_dir):
    """3D trajectory plot — protected zone + Apollonius sphere (I1) + all tracks."""
    fig = plt.figure(figsize=(13, 10))
    ax = fig.add_subplot(111, projection='3d')
    Xz, Yz, Zz = sphere_mesh(R_ZONE)
    ax.plot_surface(Xz, Yz, Zz, color='royalblue', alpha=0.12, linewidth=0)
    c, r = apollonius_sphere(INTRUDER_STARTS[0])
    Xa, Ya, Za = sphere_mesh(r, center=tuple(c), n=24)
    ax.plot_wireframe(Xa, Ya, Za, color='goldenrod', alpha=0.25, linewidth=0.5)

    for (label, strat), res in cases.items():
        color = INTRUDER_COLORS[INTRUDER_LABELS.index(label)]
        td, ti = res['traj_d'], res['traj_i']
        if strat == 'pure_pursuit':
            ax.plot(ti[:, 0], ti[:, 1], ti[:, 2], color=color, linewidth=1.6,
                    linestyle=':', alpha=0.8, label=f'{label} intruder')
            ax.scatter(*res['p0'], color=color, s=60, marker='^')
        ax.plot(td[:, 0], td[:, 1], td[:, 2], color=color, linewidth=1.6,
                linestyle=STRATEGY_LINESTYLES[strat], alpha=0.9,
                label=f'{label} defender ({STRAT_SHORT[strat]})')
        if res['outcome'] == 'defender_wins':
            ax.scatter(*td[-1], color=color, s=50, marker='x')

    ax.scatter(*DEFENDER_START, color='black', s=100, marker='D', label='Defender start')
    ax.set_xlim(-10, 24); ax.set_ylim(-12, 12); ax.set_zlim(-8, 12)
    ax.set_box_aspect((34, 24, 20))
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('S016 Airspace Defense — 3D Trajectories (x = capture)\n'
                 'Dotted = intruder, solid = PP defender, dashed = PN defender\n'
                 'Blue = protected zone, gold wireframe = Apollonius sphere of I1 '
                 '(intruder arrives first inside it)', fontsize=9)
    ax.legend(fontsize=7, loc='upper left', ncol=2)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_outcome_table(cases, out_dir):
    """Outcome table + intercept times vs optimal and penetration times."""
    strat_labels = ['Pure Pursuit', 'Prop. Nav.']
    n_int = len(INTRUDER_LABELS)
    colors_map = {'defender_wins': '#2a9d8f', 'intruder_wins': '#e63946', 'timeout': '#adb5bd'}
    cell_colors, cell_text = [], []
    for label in INTRUDER_LABELS:
        rc, rt = [], []
        for strat in STRATEGIES:
            res = cases[(label, strat)]
            rc.append(colors_map[res['outcome']])
            rt.append(f'{res["outcome"].replace("_", " ")}\n'
                      f't={res["time"]:.3f}s, intruder {res["margin"]:.2f} m from zone')
        cell_colors.append(rc); cell_text.append(rt)

    fig, axes = plt.subplots(1, 2, figsize=(14, 4.5))
    ax = axes[0]
    table = ax.table(cellText=cell_text, rowLabels=INTRUDER_LABELS, colLabels=strat_labels,
                     cellColours=cell_colors, loc='center', cellLoc='center')
    table.auto_set_font_size(False); table.set_fontsize(9); table.scale(1.5, 2.8)
    ax.axis('off')
    ax.set_title('Outcome, intercept time, intruder distance to zone surface at capture',
                 fontsize=10, fontweight='bold')

    ax2 = axes[1]
    x = np.arange(n_int); w = 0.25
    t_pp = [cases[(l, 'pure_pursuit')]['time'] for l in INTRUDER_LABELS]
    t_pn = [cases[(l, 'proportional_nav')]['time'] for l in INTRUDER_LABELS]
    t_opt = [cases[(l, 'pure_pursuit')]['t_opt'] for l in INTRUDER_LABELS]
    t_pen = [cases[(l, 'pure_pursuit')]['t_pen'] for l in INTRUDER_LABELS]
    for xs, vals, lab, col in [(x - w, t_opt, 'Optimal t* (straight line)', '#adb5bd'),
                               (x, t_pp, 'Pure Pursuit', '#457b9d'),
                               (x + w, t_pn, 'Prop. Nav.', '#e9c46a')]:
        bars = ax2.bar(xs, vals, w, label=lab, color=col)
        for b in bars:
            ax2.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.01,
                     f'{b.get_height():.3f}', ha='center', va='bottom', fontsize=7)
    ax2.scatter(x, t_pen, color='red', marker='_', s=600, linewidths=2,
                label='Intruder reaches zone (T_pen)')
    ax2.set_xticks(x); ax2.set_xticklabels(INTRUDER_LABELS, fontsize=8)
    ax2.set_ylabel('Time (s)'); ax2.set_ylim(0, 2.4)
    ax2.set_title('Intercept time per engagement', fontsize=10, fontweight='bold')
    ax2.legend(fontsize=8, loc='upper left', ncol=2); ax2.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    path = os.path.join(out_dir, 'outcome_table.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_apollonius(cases, out_dir):
    """
    Top-down view: Apollonius circle (cross-section of the Apollonius sphere at
    z = 2, which contains its centre) for each card intruder, the optimal
    intercept point on its path, and the protected zone.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    th = np.linspace(0, 2 * np.pi, 360)
    ax.fill(R_ZONE * np.cos(th), R_ZONE * np.sin(th), color='royalblue', alpha=0.18,
            label='Protected zone (projection)')
    ax.scatter(*DEFENDER_START[:2], color='black', s=150, marker='D', zorder=8,
               label='Defender start')
    for p0, label, color in zip(INTRUDER_STARTS, INTRUDER_LABELS, INTRUDER_COLORS):
        c, r = apollonius_sphere(p0)
        ax.add_patch(plt.Circle(c[:2], r, fill=False, edgecolor=color, linestyle='--',
                                linewidth=1.8, alpha=0.8))
        ax.scatter(*p0[:2], color=color, s=110, marker='^', zorder=7)
        ax.plot([p0[0], 0], [p0[1], 0], color=color, linestyle=':', linewidth=1.2)
        t_opt = cases[(label, 'pure_pursuit')]['t_opt']
        v_int = -p0 / np.linalg.norm(p0) * V_INTRUDER
        p_opt = p0 + v_int * t_opt
        ax.scatter(*p_opt[:2], color=color, s=90, marker='*', edgecolors='black', zorder=9)
        ax.text(p0[0] + 0.3, p0[1] + 0.3,
                f'{label}\nr_apo={r:.2f} m\nt*={t_opt:.3f} s', fontsize=8, color=color)
    ax.scatter([], [], color='grey', marker='*', s=90, edgecolors='black',
               label='Optimal intercept point (t*)')
    ax.set_aspect('equal'); ax.set_xlim(-13, 13); ax.set_ylim(-13, 13)
    ax.grid(True, alpha=0.3); ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('S016 — Apollonius circles (top-down, z = 2 cross-section)\n'
                 'Inside a dashed circle the intruder arrives first; its path leaves the circle '
                 'at the optimal intercept point', fontsize=9)
    ax.legend(fontsize=8, loc='lower left')
    plt.tight_layout()
    path = os.path.join(out_dir, 'apollonius_region.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_guarantee_region(region, out_dir):
    """Capture map for PP and PN over intruder start (rho, z) with analytic boundaries."""
    R, Z, valid = region['R'], region['Z'], region['valid']
    fig, axes = plt.subplots(1, 2, figsize=(14, 8), sharey=True)
    th = np.linspace(-np.pi / 2, np.pi / 2, 200)
    for ax, s in zip(axes, STRATEGIES):
        cap = region['cap'][s]
        m = valid & cap
        ax.scatter(R[m], Z[m], s=14, marker='s', color='#2a9d8f', label='captured')
        m = valid & ~cap
        ax.scatter(R[m], Z[m], s=14, marker='s', color='#e63946', label='penetrated')
        lost_opt = valid & ~cap & region['opt']
        ax.scatter(R[lost_opt], Z[lost_opt], s=30, marker='o', facecolors='none',
                   edgecolors='black', linewidths=0.7, label='penetrated but optimal test says interceptable')
        ax.contour(R, Z, np.where(valid, region['opt'].astype(float), np.nan), levels=[0.5],
                   colors='black', linewidths=1.8)
        ax.contour(R, Z, np.where(valid, region['card'].astype(float), np.nan), levels=[0.5],
                   colors='goldenrod', linewidths=1.8, linestyles='--')
        ax.plot([], [], color='black', lw=1.8, label='exact optimal-intercept boundary')
        ax.plot([], [], color='goldenrod', lw=1.8, ls='--', label='card time-test boundary')
        ax.fill(R_ZONE * np.cos(th), R_ZONE * np.sin(th), color='royalblue', alpha=0.3)
        ax.scatter([0], [DEFENDER_START[2]], color='black', marker='D', s=80, zorder=9)
        ax.scatter([np.linalg.norm(INTRUDER_STARTS[0][:2]), np.linalg.norm(INTRUDER_STARTS[3][:2])],
                   [2, 2], marker='^', color='white', edgecolors='black', s=90, zorder=9,
                   label='card intruders (rho = 8, 8.49)')
        ax.plot(RING_RANGE * np.cos(th), RING_RANGE * np.sin(th), color='purple', lw=1,
                ls=':', label=f'approach-angle sweep (range {RING_RANGE:.0f} m)')
        n = int(valid.sum())
        ax.set_title(f'{"Pure Pursuit" if s == "pure_pursuit" else "Proportional Navigation"}: '
                     f'{int((cap & valid).sum())}/{n} captured', fontsize=10, fontweight='bold')
        ax.set_xlabel('Intruder start rho = sqrt(x² + y²) (m)')
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    axes[0].set_ylabel('Intruder start z (m)')
    axes[1].legend(fontsize=7, loc='upper left', bbox_to_anchor=(1.02, 1.0))
    fig.suptitle('S016 — Capture-guarantee region (intruder flies straight to the origin; '
                 'defender at (0,0,2), v_D/v_I = 1.5)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'guarantee_region.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_miss_vs_angle(ring, out_dir):
    """Closest approach and intercept time vs intruder approach elevation."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for s, mk, col in [('pure_pursuit', 'o', '#457b9d'), ('proportional_nav', 's', '#e9c46a')]:
        d = ring[s]
        axes[0].plot(RING_ELEV_DEG, d['miss'], marker=mk, color=col, label=STRAT_SHORT[s],
                     markeredgecolor='black', markeredgewidth=0.5)
        won = d['outcome'] == 'defender_wins'
        axes[1].plot(RING_ELEV_DEG[won], d['time'][won], marker=mk, linestyle='',
                     color=col, label=f'{STRAT_SHORT[s]} capture time',
                     markeredgecolor='black', markeredgewidth=0.5)
    axes[0].axhline(CAPTURE_R, color='red', linestyle='--', label=f'Capture radius {CAPTURE_R} m')
    axes[0].set_yscale('log'); axes[0].set_ylabel('Closest approach before\ncapture/penetration (m)')
    axes[0].legend(fontsize=8); axes[0].grid(True, alpha=0.3, which='both')
    axes[1].plot(RING_ELEV_DEG, ring['t_opt'], color='grey', label='optimal t*')
    t_pen = (RING_RANGE - R_ZONE) / V_INTRUDER
    axes[1].axhline(t_pen, color='red', linestyle='--', label=f'T_pen = {t_pen:.3f} s')
    axes[1].set_ylabel('Time (s)'); axes[1].set_xlabel('Intruder start elevation (deg)')
    axes[1].legend(fontsize=8); axes[1].grid(True, alpha=0.3)
    axes[0].set_title(f'S016 — Intruder at range {RING_RANGE:.1f} m, approach elevation varied\n'
                      '(closest approach = capture radius when captured)', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'miss_vs_angle.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def save_animation(cases, out_dir):
    """Animate the 4 card intruders vs the defender, PP (left) and PN (right)."""
    fig = plt.figure(figsize=(14, 7))
    axes = [fig.add_subplot(1, 2, k + 1, projection='3d') for k in range(2)]
    Xz, Yz, Zz = sphere_mesh(R_ZONE, n=20)
    artists = []
    for ax, strat in zip(axes, STRATEGIES):
        ax.plot_wireframe(Xz, Yz, Zz, color='royalblue', alpha=0.15, linewidth=0.5)
        ax.scatter(*DEFENDER_START, color='black', s=60, marker='D')
        ax.set_xlim(-9, 9); ax.set_ylim(-9, 9); ax.set_zlim(-4, 6)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        lines = []
        for i, label in enumerate(INTRUDER_LABELS):
            col = INTRUDER_COLORS[i]
            il, = ax.plot([], [], [], color=col, lw=1.2, ls=':')
            dl, = ax.plot([], [], [], color=col, lw=1.5, ls=STRATEGY_LINESTYLES[strat])
            idot, = ax.plot([], [], [], color=col, marker='^', ms=7, ls='')
            ddot, = ax.plot([], [], [], color=col, marker='o', ms=7, ls='',
                            markeredgecolor='black')
            lines.append((cases[(label, strat)], il, dl, idot, ddot))
        artists.append((ax, strat, lines))

    max_len = max(len(r['traj_d']) for r in cases.values())
    t_last = max(r['time'] for r in cases.values())

    def update(frame):
        out = []
        for ax, strat, lines in artists:
            for res, il, dl, idot, ddot in lines:
                n = min(frame + 1, len(res['traj_d']))
                td, ti = res['traj_d'][:n], res['traj_i'][:n]
                il.set_data_3d(ti[:, 0], ti[:, 1], ti[:, 2])
                dl.set_data_3d(td[:, 0], td[:, 1], td[:, 2])
                idot.set_data_3d([ti[-1, 0]], [ti[-1, 1]], [ti[-1, 2]])
                ddot.set_data_3d([td[-1, 0]], [td[-1, 1]], [td[-1, 2]])
                out += [il, dl, idot, ddot]
            ax.set_title(f'{"Pure Pursuit" if strat == "pure_pursuit" else "Prop. Nav. (N=3)"}'
                         f'   t = {min(frame * DT, t_last):.2f} s', fontsize=10)
        return out

    ani = animation.FuncAnimation(fig, update, frames=max_len, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=80)
    plt.close(); print(f'Saved: {path}  ({max_len} frames)')


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    data = run_simulation()
    cases = data['cases']
    print('\nOutcome summary (card cases):')
    for s in STRATEGIES:
        wins = [r['time'] for (l, st), r in cases.items()
                if st == s and r['outcome'] == 'defender_wins']
        print(f'  {STRAT_SHORT[s]} wins {len(wins)}/{len(INTRUDER_STARTS)}'
              + (f', mean intercept time {np.mean(wins):.3f} s' if wins else ''))
    plot_3d_trajectories(cases, out_dir)
    plot_outcome_table(cases, out_dir)
    plot_apollonius(cases, out_dir)
    plot_guarantee_region(data['region'], out_dir)
    plot_miss_vs_angle(data['ring'], out_dir)
    save_animation(cases, out_dir)
