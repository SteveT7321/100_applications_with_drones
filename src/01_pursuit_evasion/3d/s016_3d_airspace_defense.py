"""
S016 3D Upgrade — Airspace Defense
===================================
A protected zone (sphere of radius 2 m at the origin) is threatened by intruders
that fly straight at its centre from all 3D directions.  A single defender starts
on the z-axis at (0, 0, z_D) and must intercept each intruder before it crosses
the zone surface.  The card compares two guidance laws:

* Pure Pursuit (PP): fly at v_D straight at the intruder's current position.
* 3D Proportional Navigation (PN, N = 3): a constant-speed vehicle launched at
  v_D along the initial line of sight; a = N * v_c * d(lambda_hat)/dt turns its
  velocity vector (speed stays at v_D).

Both laws use the same speed, the same launch heading and the same in-step
capture/penetration test (the 2D fix of S016, ported to 3D).

Ground is z = 0: no intruder starts below it, and the defender start altitudes
are z_D in {0, 0.5, 1, 2, 3.5, 5} m (z_D = 0 is the card's zone-centre start).
The script runs
  1. the card's 6 approach directions (range 8 m) from every defender altitude;
  2. the card's altitude grid search (worst-case margin t_zone - |D-I|/v_D) and
     the exact optimal-intercept margin, per altitude;
  3. a capture-guarantee-region sweep over intruder start (rho, z >= 0) for every
     defender altitude (the problem is symmetric about the z-axis);
  4. an approach-elevation sweep at 4 m range (where interception is marginal).

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s016_3d_airspace_defense.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's016_3d_airspace_defense'))

# ── Parameters ────────────────────────────────────────────────────────────────
R_ZONE      = 2.0      # m   protected zone radius
V_DEFENDER  = 6.0      # m/s
V_INTRUDER  = 4.0      # m/s
SPEED_RATIO = V_DEFENDER / V_INTRUDER   # 1.5
PN_GAIN     = 3.0
CAPTURE_R   = 0.15     # m
DT          = 0.02     # s
T_MAX       = 10.0     # s
R_INIT      = 8.0      # m   card intruder start range

# Defender start altitudes (card's grid {0.5,1,2,3.5,5} plus the card start z=0)
Z_CANDIDATES = [0.0, 0.5, 1.0, 2.0, 3.5, 5.0]
Z_CARD_START = 0.0

# Card approach directions (azimuth, elevation of the start position, deg).
# The card's (270, -45) and (0, -80) would start 5.7 m and 7.9 m underground;
# (0, -80) is labelled "dive from above", so it is mirrored to +80, and the
# "oblique low" approach is kept low but above ground at +15 deg.
APPROACH_ANGLES = [(0, 0), (180, 0), (90, 0), (45, 45), (270, 15), (0, 80)]
INTRUDER_LABELS = ['I1 az0 el0', 'I2 az180 el0', 'I3 az90 el0',
                   'I4 az45 el45', 'I5 az270 el15', 'I6 az0 el80']
INTRUDER_COLORS = ['#e63946', '#f4a261', '#2a9d8f', '#8338ec', '#fb8500', '#06d6a0']
STRATEGIES  = ['pure_pursuit', 'proportional_nav']
STRAT_SHORT = {'pure_pursuit': 'PP', 'proportional_nav': 'PN'}

# Guarantee-region sweep: intruder start (rho, 0, z), z >= 0 (above ground)
SWEEP_RHO = np.arange(0.0, 9.0 + 1e-9, 0.25)
SWEEP_Z   = np.arange(0.0, 9.0 + 1e-9, 0.25)
# Approach-elevation sweep at a range where interception is marginal
RING_RANGE    = 4.0
RING_ELEV_DEG = np.arange(0.0, 90.0 + 1e-9, 5.0)
RING_Z_D      = [0.0, 2.0, 3.5]


def angles_to_pos(az_deg, el_deg, r=R_INIT):
    az, el = np.radians(az_deg), np.radians(el_deg)
    return r * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])


INTRUDER_STARTS = [angles_to_pos(az, el) for az, el in APPROACH_ANGLES]


def defender_start(z_d):
    return np.array([0.0, 0.0, float(z_d)])


# ── Geometry helpers ─────────────────────────────────────────────────────────

def penetration_time(p_int_0):
    """Time for a straight-flying intruder to reach the zone surface."""
    return (np.linalg.norm(p_int_0) - R_ZONE) / V_INTRUDER


def card_margin(p_int_0, p_def):
    """Card score t_zone - |p_D - p_I| / v_D (>= 0: card test passes)."""
    return penetration_time(p_int_0) - np.linalg.norm(p_def - p_int_0) / V_DEFENDER


def optimal_intercept(p_int_0, p_def, n=4000):
    """
    Exact test for a straight-flying intruder and a defender that can turn
    instantly: earliest t <= T_pen at which a straight defender path at v_D can
    be within CAPTURE_R of the intruder.  Returns t* or None (cannot intercept).
    """
    t_pen = penetration_time(p_int_0)
    v_int = -p_int_0 / np.linalg.norm(p_int_0) * V_INTRUDER
    t = np.linspace(0.0, t_pen, n)
    p_i = p_int_0[None, :] + t[:, None] * v_int[None, :]
    gap = np.linalg.norm(p_i - p_def[None, :], axis=1) - CAPTURE_R - V_DEFENDER * t
    ok = np.nonzero(gap <= 0.0)[0]
    return float(t[ok[0]]) if len(ok) else None


def apollonius_sphere(p_int_0, p_def, k=SPEED_RATIO):
    """Sphere {X : |X - p_D| = k |X - p_I|}; inside it the intruder arrives first."""
    c = (k ** 2 * p_int_0 - p_def) / (k ** 2 - 1.0)
    r = k * np.linalg.norm(p_int_0 - p_def) / (k ** 2 - 1.0)
    return c, r


def first_hit(r0, d, radius):
    """Smallest s in [0, 1] with |r0 + s d| <= radius, or None."""
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
    d = p_int - p_def
    nd = np.linalg.norm(d)
    return np.zeros(3) if nd < 1e-9 else d / nd * V_DEFENDER


def pn_velocity(p_def, p_int, v_def, v_int):
    """3D PN for a constant-speed vehicle (velocity rescaled to v_D every step)."""
    los = p_int - p_def
    r = np.linalg.norm(los)
    if r < 1e-9:
        return v_def
    los_hat = los / r
    v_rel = v_int - v_def
    los_dot = (v_rel - np.dot(v_rel, los_hat) * los_hat) / r
    v_c = -np.dot(v_rel, los_hat)
    v_new = v_def + PN_GAIN * v_c * los_dot * DT
    return v_new / np.linalg.norm(v_new) * V_DEFENDER


# ── Simulation ───────────────────────────────────────────────────────────────

def simulate(p_int_0, z_d, strategy='pure_pursuit', record=True):
    """
    One defender vs one intruder.  Within each step both move at constant
    velocity; the first capture (|D-I| < CAPTURE_R) or penetration (|I| <= R_zone)
    inside the step decides the engagement.
    """
    p_def = defender_start(z_d)
    p_int = np.asarray(p_int_0, dtype=float).copy()
    v_int = -p_int / np.linalg.norm(p_int) * V_INTRUDER
    los0 = p_int - p_def
    v_def = los0 / np.linalg.norm(los0) * V_DEFENDER   # PN launch: along LOS, full speed

    traj_d, traj_i = [p_def.copy()], [p_int.copy()]
    miss = np.linalg.norm(los0)
    min_z = p_def[2]
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
        min_z = min(min_z, p_def[2])
        if record:
            traj_d.append(p_def.copy()); traj_i.append(p_int.copy())
        if s < 1.0 or outcome != 'timeout':
            t_end = (step + s) * DT
            break
    return {'traj_d': np.array(traj_d), 'traj_i': np.array(traj_i), 'outcome': outcome,
            'time': t_end, 'miss': float(miss), 'min_z': float(min_z),
            'margin': float(np.linalg.norm(p_int) - R_ZONE)}


def run_card_cases():
    """6 card intruders x 2 laws x every defender altitude."""
    cases = {}
    print('Card intruders (range 8 m) — outcome per defender altitude')
    print(f'  {"z_D":>4s} | {"intruder":14s} | T_pen  | t* opt | PP time  | PN time  | '
          f'zone margin PP/PN')
    for z_d in Z_CANDIDATES:
        p_def = defender_start(z_d)
        for p0, label in zip(INTRUDER_STARTS, INTRUDER_LABELS):
            t_opt = optimal_intercept(p0, p_def)
            row = {}
            for s in STRATEGIES:
                res = simulate(p0, z_d, s)
                res['t_opt'] = t_opt
                res['t_pen'] = penetration_time(p0)
                cases[(z_d, label, s)] = res
                row[s] = res
            def fmt(r):
                return (f'{r["time"]:.3f}s' if r['outcome'] == 'defender_wins'
                        else f'PEN {r["time"]:.2f}')
            print(f'  {z_d:4.1f} | {label:14s} | {penetration_time(p0):.3f}s | {t_opt:.3f}s | '
                  f'{fmt(row["pure_pursuit"]):8s} | {fmt(row["proportional_nav"]):8s} | '
                  f'{row["pure_pursuit"]["margin"]:.3f} / {row["proportional_nav"]["margin"]:.3f} m')
    for z_d in Z_CANDIDATES:
        for s in STRATEGIES:
            w = [cases[(z_d, l, s)] for l in INTRUDER_LABELS]
            nw = sum(r['outcome'] == 'defender_wins' for r in w)
            print(f'  z_D={z_d:.1f}  {STRAT_SHORT[s]}: {nw}/6 captured, mean time '
                  f'{np.mean([r["time"] for r in w]):.3f} s, worst {max(r["time"] for r in w):.3f} s')
    return cases


def run_altitude_search():
    """Card grid search: worst-case margin over the 6 card intruders per z_D."""
    out = {'card': [], 'opt': []}
    print('\nAltitude grid search (6 card intruders):')
    print('   z_D | card worst margin t_zone-|D-I|/v_D | exact worst margin T_pen - t*')
    for z_d in Z_CANDIDATES:
        p_def = defender_start(z_d)
        cm = min(card_margin(p0, p_def) for p0 in INTRUDER_STARTS)
        om = min(penetration_time(p0) - optimal_intercept(p0, p_def) for p0 in INTRUDER_STARTS)
        out['card'].append(cm); out['opt'].append(om)
        print(f'  {z_d:4.1f} | {cm:+.3f} s{"":26s} | {om:+.3f} s')
    k = int(np.argmax(out['card']))
    print(f'  card optimum z_D* = {Z_CANDIDATES[k]:.1f} m (worst margin {out["card"][k]:+.3f} s); '
          f'exact optimum z_D = {Z_CANDIDATES[int(np.argmax(out["opt"]))]:.1f} m')
    out['z_star'] = Z_CANDIDATES[k]
    return out


def run_region_sweep():
    """Capture-guarantee region over intruder start (rho, z >= 0) per defender altitude."""
    R, Z = np.meshgrid(SWEEP_RHO, SWEEP_Z)
    shape = R.shape
    regions = {}
    print(f'\nGuarantee-region sweep (rho 0-{SWEEP_RHO[-1]:.0f} m, z 0-{SWEEP_Z[-1]:.0f} m, '
          f'0.25 m grid, starts outside the zone):')
    print('   z_D | starts | optimal | card test | PP capt | PN capt | PN only | PP only | '
          'card pass & opt fail | min defender z')
    for z_d in Z_CANDIDATES:
        p_def = defender_start(z_d)
        valid = np.zeros(shape, bool)
        cap = {s: np.zeros(shape, bool) for s in STRATEGIES}
        card = np.zeros(shape, bool); opt = np.zeros(shape, bool)
        min_z = np.inf
        for idx in np.ndindex(shape):
            p0 = np.array([R[idx], 0.0, Z[idx]])
            if (np.linalg.norm(p0) <= R_ZONE + 1e-9
                    or np.linalg.norm(p0 - p_def) < CAPTURE_R):
                continue
            valid[idx] = True
            card[idx] = card_margin(p0, p_def) >= 0
            opt[idx] = optimal_intercept(p0, p_def) is not None
            for s in STRATEGIES:
                res = simulate(p0, z_d, s, record=False)
                cap[s][idx] = res['outcome'] == 'defender_wins'
                min_z = min(min_z, res['min_z'])
        n = int(valid.sum())
        pp, pn = cap['pure_pursuit'][valid], cap['proportional_nav'][valid]
        print(f'  {z_d:4.1f} | {n:6d} | {int(opt[valid].sum()):7d} | {int(card[valid].sum()):9d} | '
              f'{int(pp.sum()):7d} | {int(pn.sum()):7d} | {int((pn & ~pp).sum()):7d} | '
              f'{int((pp & ~pn).sum()):7d} | {int((card[valid] & ~opt[valid]).sum()):20d} | '
              f'{min_z:.3f} m')
        beyond = int(((pp | pn) & ~opt[valid]).sum())
        if beyond:
            print(f'        WARNING: {beyond} captures where the optimal test says impossible')
        regions[z_d] = {'valid': valid, 'cap': cap, 'card': card, 'opt': opt}
    return {'R': R, 'Z': Z, 'by_z': regions}


def run_angle_sweep():
    """Intruder at RING_RANGE, start elevation 0-90 deg, for several defender altitudes."""
    ring = {}
    t_pen = (RING_RANGE - R_ZONE) / V_INTRUDER
    print(f'\nApproach-elevation sweep (range {RING_RANGE:.1f} m, T_pen {t_pen:.3f} s, '
          f'elevation 0-90 deg in 5 deg steps):')
    for z_d in RING_Z_D:
        p_def = defender_start(z_d)
        out = {s: {'miss': [], 'time': [], 'outcome': []} for s in STRATEGIES}
        t_opt = []
        for el in RING_ELEV_DEG:
            e = np.radians(el)
            p0 = RING_RANGE * np.array([np.cos(e), 0.0, np.sin(e)])
            t = optimal_intercept(p0, p_def)
            t_opt.append(np.nan if t is None else t)
            for s in STRATEGIES:
                res = simulate(p0, z_d, s, record=False)
                for k in out[s]:
                    out[s][k].append(res[k])
        for s in STRATEGIES:
            for k in out[s]:
                out[s][k] = np.array(out[s][k])
        out['t_opt'] = np.array(t_opt)
        ring[z_d] = out
        print(f'  z_D = {z_d:.1f} m:  elev  | t* opt | PP: outcome time  miss | PN: outcome time  miss')
        for i, el in enumerate(RING_ELEV_DEG):
            row = f'            {el:4.0f} | ' + ('  --  ' if np.isnan(t_opt[i]) else f'{t_opt[i]:.3f}s')
            for s in STRATEGIES:
                o = out[s]['outcome'][i]
                row += (f' | {"capture" if o == "defender_wins" else "PENETR."} '
                        f'{out[s]["time"][i]:.3f}s {out[s]["miss"][i]:.3f}m')
            print(row)
        n_opt = int(np.sum(~np.isnan(out['t_opt'])))
        n_pp = int(np.sum(out['pure_pursuit']['outcome'] == 'defender_wins'))
        n_pn = int(np.sum(out['proportional_nav']['outcome'] == 'defender_wins'))
        print(f'            optimal {n_opt}/{len(RING_ELEV_DEG)}  PP {n_pp}/{len(RING_ELEV_DEG)}  '
              f'PN {n_pn}/{len(RING_ELEV_DEG)}')
    return ring


def run_simulation():
    print('S016 3D Airspace Defense — Simulation Results')
    print('=' * 70)
    cases = run_card_cases()
    p_def = defender_start(Z_CARD_START)
    print('\nCard time test at the card start (0,0,0):')
    for p0, label in zip(INTRUDER_STARTS, INTRUDER_LABELS):
        c, r = apollonius_sphere(p0, p_def)
        print(f'  {label:14s} start ({p0[0]:5.2f},{p0[1]:5.2f},{p0[2]:5.2f}) | |D-I| = '
              f'{np.linalg.norm(p0 - p_def):.3f} m vs v_D T_pen = {V_DEFENDER * penetration_time(p0):.3f} m '
              f'| Apollonius r = {r:.2f} m')
    alt = run_altitude_search()
    region = run_region_sweep()
    ring = run_angle_sweep()
    return {'cases': cases, 'alt': alt, 'region': region, 'ring': ring}


# ── Plots ────────────────────────────────────────────────────────────────────

def sphere_mesh(radius, center=(0, 0, 0), n=24):
    u = np.linspace(0, 2 * np.pi, n)
    v = np.linspace(0, np.pi, n)
    X = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    Y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    Z = center[2] + radius * np.outer(np.ones(n), np.cos(v))
    return X, Y, Z


TRAJ_Z = [0.0, 5.0]   # defender altitudes drawn in the trajectory plot


def plot_trajectories_3d(cases, out_dir):
    fig = plt.figure(figsize=(18, 12))
    styles = {(0.0, 'pure_pursuit'): ('#e63946', '-'), (0.0, 'proportional_nav'): ('#e63946', '--'),
              (5.0, 'pure_pursuit'): ('#9d0208', '-'), (5.0, 'proportional_nav'): ('#ff8fa3', '--')}
    for idx, (p0, label) in enumerate(zip(INTRUDER_STARTS, INTRUDER_LABELS)):
        ax = fig.add_subplot(2, 3, idx + 1, projection='3d')
        Xz, Yz, Zz = sphere_mesh(R_ZONE)
        ax.plot_surface(Xz, Yz, Zz, color='green', alpha=0.10, linewidth=0)
        if idx == 3:   # Apollonius sphere of one diagonal intruder (defender at card start)
            c, r = apollonius_sphere(p0, defender_start(Z_CARD_START))
            Xa, Ya, Za = sphere_mesh(r, center=tuple(c), n=40)
            out = (np.abs(Xa) > 8) | (np.abs(Ya) > 8) | (Za < 0) | (Za > 9)   # clip to the axes box
            Xa, Ya, Za = (np.where(out, np.nan, a) for a in (Xa, Ya, Za))
            ax.plot_wireframe(Xa, Ya, Za, color='goldenrod', alpha=0.35, linewidth=0.5)
        ti = cases[(0.0, label, 'pure_pursuit')]['traj_i']
        ax.plot(ti[:, 0], ti[:, 1], ti[:, 2], color='blue', lw=1.4, ls=':', label='intruder')
        ax.scatter(*p0, color='blue', s=60, marker='^')
        for z_d in TRAJ_Z:
            ax.scatter(*defender_start(z_d), color='black', s=40, marker='D')
            for s in STRATEGIES:
                r = cases[(z_d, label, s)]
                col, ls = styles[(z_d, s)]
                td = r['traj_d']
                ax.plot(td[:, 0], td[:, 1], td[:, 2], color=col, ls=ls, lw=1.5,
                        label=f'{STRAT_SHORT[s]} from z={z_d:.0f} ({r["time"]:.3f}s)')
        ax.set_xlim(-8, 8); ax.set_ylim(-8, 8); ax.set_zlim(0, 9)
        ax.set_xlabel('X', fontsize=7); ax.set_ylabel('Y', fontsize=7); ax.set_zlabel('Z', fontsize=7)
        ax.set_title(label + (' (+ Apollonius sphere)' if idx == 3 else ''), fontsize=9)
        ax.legend(fontsize=6, loc='upper left'); ax.tick_params(labelsize=6)
    fig.suptitle('S016 3D — card intruders (range 8 m) vs defender from z_D = 0 (card start) and z_D = 5\n'
                 'Green sphere = protected zone, solid = PP, dashed = PN; every engagement is a capture',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


def plot_outcome_table(cases, out_dir):
    cols = [(z, s) for z in TRAJ_Z for s in STRATEGIES]
    col_labels = [f'{STRAT_SHORT[s]}, z_D={z:.0f}' for z, s in cols] + ['t* opt z_D=0', 't* opt z_D=5']
    cmap = {'defender_wins': '#b7e4c7', 'intruder_wins': '#f4a3a8', 'timeout': '#dee2e6'}
    text, colors = [], []
    for label in INTRUDER_LABELS:
        rt, rc = [], []
        for z, s in cols:
            r = cases[(z, label, s)]
            rt.append(f'{"capture" if r["outcome"] == "defender_wins" else r["outcome"]}\n'
                      f't={r["time"]:.3f}s, {r["margin"]:.2f} m out')
            rc.append(cmap[r['outcome']])
        for z in TRAJ_Z:
            rt.append(f'{cases[(z, label, "pure_pursuit")]["t_opt"]:.3f}s'); rc.append('white')
        text.append(rt); colors.append(rc)
    fig, ax = plt.subplots(figsize=(15, 5))
    tb = ax.table(cellText=text, rowLabels=INTRUDER_LABELS, colLabels=col_labels,
                  cellColours=colors, loc='center', cellLoc='center')
    tb.auto_set_font_size(False); tb.set_fontsize(8); tb.scale(1.0, 2.6)
    ax.axis('off')
    ax.set_title('S016 3D — card engagements: outcome, intercept time, intruder distance to the zone '
                 'surface at capture, and the straight-line optimum t*', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'outcome_table.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


def plot_altitude_sweep(alt, region, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))
    ax = axes[0]
    ax.plot(Z_CANDIDATES, alt['card'], 'o-', color='goldenrod', label='card score: min_j t_zone - |D-I_j|/v_D')
    ax.plot(Z_CANDIDATES, alt['opt'], 's-', color='black', label='exact: min_j T_pen - t*_j')
    ax.axhline(0, color='red', ls='--', lw=1)
    ax.set_xlabel('Defender start altitude z_D (m)'); ax.set_ylabel('Worst-case margin over the 6 card intruders (s)')
    ax.set_title('Card altitude grid search', fontsize=10)
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax = axes[1]
    by = region['by_z']
    for key, lab, mk, col in [('opt', 'exact optimal test', 's', 'black'), ('card', 'card time test', 'o', 'goldenrod')]:
        ax.plot(Z_CANDIDATES, [int(by[z][key][by[z]['valid']].sum()) for z in Z_CANDIDATES],
                marker=mk, color=col, label=lab)
    for s, mk, col in [('pure_pursuit', '^', '#457b9d'), ('proportional_nav', 'v', '#e76f51')]:
        ax.plot(Z_CANDIDATES, [int(by[z]['cap'][s][by[z]['valid']].sum()) for z in Z_CANDIDATES],
                marker=mk, color=col, label=f'{STRAT_SHORT[s]} captures')
    ns = [int(by[z]['valid'].sum()) for z in Z_CANDIDATES]
    ax.set_xlabel('Defender start altitude z_D (m)')
    ax.set_ylabel(f'Interceptable / captured starts (of {min(ns)}-{max(ns)})')
    ax.set_title('Guarantee-region size vs defender altitude', fontsize=10)
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    fig.suptitle('S016 3D — defender altitude strategy', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_sweep.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


REGION_Z_PLOT = [0.0, 2.0, 5.0]


def plot_guarantee_region(region, out_dir):
    R, Z = region['R'], region['Z']
    fig, axes = plt.subplots(2, 3, figsize=(17, 11), sharex=True, sharey=True)
    th = np.linspace(0, np.pi / 2, 100)
    for j, z_d in enumerate(REGION_Z_PLOT):
        d = region['by_z'][z_d]
        valid = d['valid']
        for i, s in enumerate(STRATEGIES):
            ax = axes[i, j]
            cap = d['cap'][s]
            ax.scatter(R[valid & cap], Z[valid & cap], s=9, marker='s', color='#2a9d8f', label='captured')
            ax.scatter(R[valid & ~cap], Z[valid & ~cap], s=9, marker='s', color='#e63946', label='penetrated')
            lost = valid & ~cap & d['opt']
            ax.scatter(R[lost], Z[lost], s=24, marker='o', facecolors='none', edgecolors='black',
                       linewidths=0.7, label='penetrated, optimal says interceptable')
            ax.contour(R, Z, np.where(valid, d['opt'].astype(float), np.nan), levels=[0.5],
                       colors='black', linewidths=1.6)
            ax.contour(R, Z, np.where(valid, d['card'].astype(float), np.nan), levels=[0.5],
                       colors='goldenrod', linewidths=1.6, linestyles='--')
            ax.fill(np.r_[0, R_ZONE * np.cos(th)], np.r_[0, R_ZONE * np.sin(th)], color='green', alpha=0.3)
            ax.plot(RING_RANGE * np.cos(th), RING_RANGE * np.sin(th), color='purple', ls=':', lw=1)
            ax.scatter([0], [z_d], color='black', marker='D', s=60, zorder=9)
            ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
            ax.set_title(f'{STRAT_SHORT[s]}, defender z_D = {z_d:.0f} m: '
                         f'{int((cap & valid).sum())}/{int(valid.sum())} captured', fontsize=9)
            if i == 1:
                ax.set_xlabel('Intruder start rho (m)')
            if j == 0:
                ax.set_ylabel('Intruder start z (m)')
    axes[0, 2].plot([], [], color='black', lw=1.6, label='exact optimal boundary')
    axes[0, 2].plot([], [], color='goldenrod', lw=1.6, ls='--', label='card time-test boundary')
    axes[0, 2].plot([], [], color='purple', ls=':', label=f'elevation sweep (r = {RING_RANGE:.0f} m)')
    axes[0, 2].legend(fontsize=7, loc='upper right')
    fig.suptitle('S016 3D — capture-guarantee region over intruder start (rho, z >= 0); '
                 'intruder flies straight to the origin, v_D/v_I = 1.5', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'guarantee_region.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_miss_vs_elevation(ring, out_dir):
    fig, axes = plt.subplots(2, len(RING_Z_D), figsize=(16, 8), sharex=True)
    t_pen = (RING_RANGE - R_ZONE) / V_INTRUDER
    for j, z_d in enumerate(RING_Z_D):
        d = ring[z_d]
        for s, mk, col in [('pure_pursuit', 'o', '#457b9d'), ('proportional_nav', 's', '#e76f51')]:
            axes[0, j].plot(RING_ELEV_DEG, d[s]['miss'], marker=mk, color=col, label=STRAT_SHORT[s],
                            markeredgecolor='black', markeredgewidth=0.4)
            won = d[s]['outcome'] == 'defender_wins'
            axes[1, j].plot(RING_ELEV_DEG[won], d[s]['time'][won], marker=mk, ls='', color=col,
                            label=f'{STRAT_SHORT[s]} capture time', markeredgecolor='black',
                            markeredgewidth=0.4)
        axes[0, j].axhline(CAPTURE_R, color='red', ls='--', label=f'capture radius {CAPTURE_R} m')
        axes[0, j].set_ylim(0, max(1.2 * max(d[s]['miss'].max() for s in STRATEGIES), 0.3))
        axes[0, j].grid(True, alpha=0.3)
        axes[0, j].set_title(f'Defender z_D = {z_d:.1f} m', fontsize=10)
        axes[1, j].plot(RING_ELEV_DEG, d['t_opt'], color='grey', label='optimal t*')
        axes[1, j].axhline(t_pen, color='red', ls='--', label=f'T_pen = {t_pen:.2f} s')
        axes[1, j].set_xlabel('Intruder start elevation (deg)'); axes[1, j].grid(True, alpha=0.3)
        axes[0, j].legend(fontsize=7); axes[1, j].legend(fontsize=7)
    axes[0, 0].set_ylabel('Closest approach (m)')
    axes[1, 0].set_ylabel('Time (s)')
    fig.suptitle(f'S016 3D — intruder at range {RING_RANGE:.0f} m, start elevation varied '
                 '(closest approach = capture radius when captured)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'miss_vs_elevation.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


ANIM_Z = 5.0


def save_animation(cases, out_dir):
    """6 card intruders vs the defender starting at z_D = 5 m; PP left, PN right."""
    fig = plt.figure(figsize=(13, 6.5))
    axes = [fig.add_subplot(1, 2, k + 1, projection='3d') for k in range(2)]
    Xz, Yz, Zz = sphere_mesh(R_ZONE, n=18)
    groups = []
    for ax, s in zip(axes, STRATEGIES):
        ax.plot_wireframe(Xz, Yz, Zz, color='green', alpha=0.15, linewidth=0.5)
        ax.scatter(*defender_start(ANIM_Z), color='black', s=50, marker='D')
        ax.set_xlim(-8, 8); ax.set_ylim(-8, 8); ax.set_zlim(0, 9)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        items = []
        for label, col in zip(INTRUDER_LABELS, INTRUDER_COLORS):
            il, = ax.plot([], [], [], color='blue', lw=1.0, ls=':')
            dl, = ax.plot([], [], [], color=col, lw=1.5)
            idot, = ax.plot([], [], [], color='blue', marker='^', ms=6, ls='')
            ddot, = ax.plot([], [], [], color='red', marker='o', ms=6, ls='', markeredgecolor='black')
            items.append((cases[(ANIM_Z, label, s)], il, dl, idot, ddot))
        groups.append((ax, s, items))
    n_frames = max(len(r['traj_d']) for r in (cases[(ANIM_Z, l, s)] for l in INTRUDER_LABELS
                                                  for s in STRATEGIES))

    def update(frame):
        out = []
        for ax, s, items in groups:
            for res, il, dl, idot, ddot in items:
                n = min(frame + 1, len(res['traj_d']))
                td, ti = res['traj_d'][:n], res['traj_i'][:n]
                il.set_data_3d(ti[:, 0], ti[:, 1], ti[:, 2])
                dl.set_data_3d(td[:, 0], td[:, 1], td[:, 2])
                idot.set_data_3d([ti[-1, 0]], [ti[-1, 1]], [ti[-1, 2]])
                ddot.set_data_3d([td[-1, 0]], [td[-1, 1]], [td[-1, 2]])
                out += [il, dl, idot, ddot]
            ax.set_title(f'{"Pure Pursuit" if s == "pure_pursuit" else "3D PN (N=3)"}, '
                         f'defender from z={ANIM_Z:.0f} m   t = {frame * DT:.2f} s', fontsize=10)
        return out

    ani = FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=20), dpi=80)
    plt.close(); print(f'Saved: {path}  ({n_frames} frames)')


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_trajectories_3d(data['cases'], OUTPUT_DIR)
    plot_outcome_table(data['cases'], OUTPUT_DIR)
    plot_altitude_sweep(data['alt'], data['region'], OUTPUT_DIR)
    plot_guarantee_region(data['region'], OUTPUT_DIR)
    plot_miss_vs_elevation(data['ring'], OUTPUT_DIR)
    save_animation(data['cases'], OUTPUT_DIR)
    print('Done.')
