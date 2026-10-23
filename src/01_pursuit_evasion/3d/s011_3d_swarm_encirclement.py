"""
S011 3D Upgrade — Swarm Encirclement
=====================================
N pursuers sit on Fibonacci-sphere slots of a shrinking sphere around the
evader; the evader commits to the direction of the largest angular gap and
tries to break out.

Encirclement law (card):  p_i^target(t) = c(t) + R(t)·q_i,
    R(t) = max(R0 − (R0 − R_capture)·t/T, R_capture)
The card writes c(t) = p_E(t).  Re-centring on the *current* evader position
every step makes the sphere translate rigidly with the evader (V_P > V_E), so
capture just follows the shrink schedule and no breakout is possible.  Here
the swarm centres the sphere on its shared evader fix, which is τ = 0.1 s old
(tracking + communication latency):  c(t) = p_E(t − τ).  τ = 0 (the ideal card
law) is run as a reference.

Breakout: the evader is outside the convex hull of the pursuers iff some
direction makes > 90° with every pursuer bearing (3D analogue of the 2D
"gap > 180°" test).  Outcome per run: captured (any pursuer < R_capture) or
escaped (no capture within 30 s).

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s011_3d_swarm_encirclement.py
"""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from matplotlib.animation import FuncAnimation, PillowWriter
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'outputs', '01_pursuit_evasion', '3d', 's011_3d_swarm_encirclement')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ───────────────────────────────────────────────
N_PURSUERS  = 6
R_INIT      = 4.0    # m
T_CONVERGE  = 15.0   # s
R_CAPTURE   = 0.3    # m  (also the sphere floor, card)
V_PURSUER   = 5.0    # m/s
V_EVADER    = 3.5    # m/s (main run, v_E/v_P = 0.7)
Z_MIN, Z_MAX = 0.5, 6.0
DT          = 0.05   # s
MAX_TIME    = 30.0   # s
K_BREAKOUT  = 200    # random candidate directions per step (card)
GAP_HYST    = np.radians(20.0)   # evader switches direction only if gain > 20°
WALL_MARGIN = 0.6    # m  evader ignores directions into floor/ceiling this close (2·R_capture)
TAU_FIX     = 0.1    # s  age of the swarm's evader fix (sphere centre)
INIT_EVADER = np.array([0.0, 0.0, 2.0])

SWEEP_N      = [4, 6, 8, 10, 12]
SWEEP_RATIO  = [0.3, 0.5, 0.7, 0.9]       # v_E / v_P
N_SEEDS      = 20                         # common random numbers: seed s in every cell
MAIN_SEED    = 0

PHI_GOLDEN = (1 + np.sqrt(5)) / 2


# ── Geometry ─────────────────────────────────────────────────

def fibonacci_sphere(n):
    """Return n unit vectors approximately uniformly on S² (card formula)."""
    i = np.arange(n)
    y = 1 - 2 * i / max(n - 1, 1)
    r = np.sqrt(np.maximum(1 - y * y, 0))
    phi = i * 2 * np.pi / PHI_GOLDEN ** 2
    return np.column_stack([r * np.cos(phi), r * np.sin(phi), y])


MEAS_DIRS = fibonacci_sphere(1000)   # fixed, seed-independent gap measurement


def unit_bearings(pos_e, pos_p):
    d = pos_p - pos_e[np.newaxis, :]
    return d / (np.linalg.norm(d, axis=1, keepdims=True) + 1e-12)


def wall_bearings(pos_e, R):
    """Floor / ceiling closer than the sphere radius block vertical escape:
    they are treated as extra blocking bearings straight down / up."""
    w = []
    if pos_e[2] - Z_MIN < R:
        w.append([0.0, 0.0, -1.0])
    if Z_MAX - pos_e[2] < R:
        w.append([0.0, 0.0, 1.0])
    return np.array(w).reshape(-1, 3)


def blocking_bearings(pos_e, pos_p, R):
    return np.vstack([unit_bearings(pos_e, pos_p), wall_bearings(pos_e, R)])


def max_gap_deg(pos_e, pos_p, R):
    """Largest angular gap: max over directions of the min angle to any pursuer
    (or blocking floor/ceiling).  > 90° ⇔ evader outside the hull of pursuers
    and walls (breakout)."""
    b = blocking_bearings(pos_e, pos_p, R)
    ang = np.arccos(np.clip(MEAS_DIRS @ b.T, -1.0, 1.0)).min(axis=1)
    return float(np.degrees(ang.max()))


def min_pair_angle_deg(pos_e, pos_p):
    """Card's 'minimum angular gap between adjacent pursuers' (seen from evader)."""
    b = unit_bearings(pos_e, pos_p)
    c = np.clip(b @ b.T, -1.0, 1.0)
    np.fill_diagonal(c, -1.0)
    return float(np.degrees(np.arccos(c.max())))


def shrink_radius(t, r0=R_INIT, r_min=R_CAPTURE, t_conv=T_CONVERGE):
    v_shrink = (r0 - r_min) / t_conv
    return max(r0 - v_shrink * t, r_min)


def card_n_critical(ratio):
    """Card formula N_crit = ceil(2 / (1 − cos(arcsin r)))."""
    return int(np.ceil(2.0 / (1.0 - np.cos(np.arcsin(ratio)))))


# ── Evader: committed max-gap breakout ───────────────────────

def evader_direction(pos_e, pos_p, R, u_cur, rng, k=K_BREAKOUT):
    """Sample k directions (card), drop those into a nearby floor/ceiling, pick
    the one maximising the min angle to all pursuers and to a floor/ceiling
    closer than R; keep the current direction unless the best candidate is
    better by GAP_HYST."""
    b = blocking_bearings(pos_e, pos_p, R)
    u = rng.standard_normal((k, 3))
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    low = (pos_e[2] - Z_MIN) < WALL_MARGIN
    high = (Z_MAX - pos_e[2]) < WALL_MARGIN
    ok = ~((low & (u[:, 2] < 0)) | (high & (u[:, 2] > 0)))
    u = u[ok]
    score = np.arccos(np.clip(u @ b.T, -1.0, 1.0)).min(axis=1)
    best = int(np.argmax(score))
    if u_cur is not None and not ((low and u_cur[2] < 0) or (high and u_cur[2] > 0)):
        cur_score = np.arccos(np.clip(b @ u_cur, -1.0, 1.0)).min()
        if score[best] <= cur_score + GAP_HYST:
            return u_cur, False
    return u[best], u_cur is not None


# ── Main simulation ──────────────────────────────────────────

def run_simulation(n_pursuers=N_PURSUERS, v_e=V_EVADER, tau=TAU_FIX,
                   seed=MAIN_SEED, log=True):
    rng = np.random.default_rng(seed)
    q = fibonacci_sphere(n_pursuers)
    lag = int(round(tau / DT))

    pos_e = INIT_EVADER.copy()
    pos_p = pos_e[np.newaxis, :] + R_INIT * q
    pos_p[:, 2] = np.clip(pos_p[:, 2], Z_MIN, Z_MAX)
    hist_e = [pos_e.copy()]
    traj_p = [pos_p.copy()]
    r_hist, t_hist = [R_INIT], [0.0]
    gap_hist = [max_gap_deg(pos_e, pos_p, R_INIT)]
    pair_hist = [min_pair_angle_deg(pos_e, pos_p)]
    dmin_hist = [float(np.linalg.norm(pos_p - pos_e, axis=1).min())]

    u_cur = None
    n_switch = 0
    captured, cap_time, cap_who, breakout_time = False, None, None, None
    max_gap = gap_hist[0]

    for step in range(1, int(round(MAX_TIME / DT)) + 1):
        t = step * DT
        R = shrink_radius(t)
        centre = hist_e[max(0, len(hist_e) - 1 - lag)]      # p_E(t − τ)
        targets = centre[np.newaxis, :] + R * q
        targets[:, 2] = np.clip(targets[:, 2], Z_MIN, Z_MAX)
        d = targets - pos_p
        dn = np.linalg.norm(d, axis=1, keepdims=True)
        pos_p = pos_p + d / np.maximum(dn, 1e-12) * np.minimum(V_PURSUER, dn / DT) * DT
        pos_p[:, 2] = np.clip(pos_p[:, 2], Z_MIN, Z_MAX)

        u_cur, switched = evader_direction(pos_e, pos_p, R, u_cur, rng)
        n_switch += int(switched)
        pos_e = pos_e + v_e * u_cur * DT
        pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)

        hist_e.append(pos_e.copy())
        dists = np.linalg.norm(pos_p - pos_e[np.newaxis, :], axis=1)
        g = max_gap_deg(pos_e, pos_p, R)
        max_gap = max(max_gap, g)
        if g > 90.0 and breakout_time is None:
            breakout_time = t
        if log:
            traj_p.append(pos_p.copy()); r_hist.append(R); t_hist.append(t)
            gap_hist.append(g); pair_hist.append(min_pair_angle_deg(pos_e, pos_p))
            dmin_hist.append(float(dists.min()))
        if dists.min() < R_CAPTURE:
            captured, cap_time, cap_who = True, t, int(np.argmin(dists))
            break

    traj_e = np.array(hist_e)
    return dict(traj_e=traj_e, traj_p=np.array(traj_p), r_hist=np.array(r_hist),
                t_hist=np.array(t_hist), gap_hist=np.array(gap_hist),
                pair_hist=np.array(pair_hist), dmin_hist=np.array(dmin_hist),
                captured=captured, cap_time=cap_time, cap_who=cap_who,
                breakout_time=breakout_time, max_gap=max_gap, n_switch=n_switch,
                z_range=(float(traj_e[:, 2].min()), float(traj_e[:, 2].max())),
                final_dmin=float(dists.min()),
                net_disp=float(np.linalg.norm(traj_e[-1] - traj_e[0])))


def run_sweep(tau=TAU_FIX, ratios=SWEEP_RATIO, n_list=SWEEP_N, n_seeds=N_SEEDS):
    """Escape / breakout statistics over N × v_E/v_P × seeds (CRN)."""
    out = {}
    for ratio in ratios:
        for n in n_list:
            rs = [run_simulation(n, ratio * V_PURSUER, tau, s, log=False) for s in range(n_seeds)]
            caps = [r['cap_time'] for r in rs if r['captured']]
            out[(ratio, n)] = dict(
                esc=sum(not r['captured'] for r in rs),
                broke=sum(r['breakout_time'] is not None for r in rs),
                cap_mean=float(np.mean(caps)) if caps else None,
                cap_std=float(np.std(caps)) if caps else None,
                cap_times=[r['cap_time'] for r in rs],
                esc_dmin=(float(np.median([r['final_dmin'] for r in rs if not r['captured']]))
                          if any(not r['captured'] for r in rs) else None))
    return out


def sim_n_critical(sweep, ratio, n_list=SWEEP_N):
    for n in n_list:
        if sweep[(ratio, n)]['esc'] == 0:
            return n
    return None


# ── Plots ────────────────────────────────────────────────────

def _p_colors(n):
    return [plt.cm.Reds(0.4 + 0.5 * i / max(n - 1, 1)) for i in range(n)]


def plot_trajectories_3d(runs, labels, out_dir):
    fig = plt.figure(figsize=(15, 7))
    for k, (res, lab) in enumerate(zip(runs, labels)):
        ax = fig.add_subplot(1, len(runs), k + 1, projection='3d')
        ax.view_init(elev=22, azim=-55)
        tp, te = res['traj_p'], res['traj_e']
        n = tp.shape[1]
        for i, c in enumerate(_p_colors(n)):
            ax.plot(tp[:, i, 0], tp[:, i, 1], tp[:, i, 2], color=c, lw=1.2, alpha=0.85,
                    label=f'P{i+1}')
            ax.scatter(*tp[0, i], color=c, s=30)
        ax.plot(te[:, 0], te[:, 1], te[:, 2], color='royalblue', lw=2.0, label='Evader')
        ax.scatter(*te[0], color='blue', s=50)
        if res['captured']:
            ax.scatter(*te[-1], color='black', s=110, marker='X', zorder=7,
                       label=f"Captured by P{res['cap_who']+1}")
        out = (f"captured @ {res['cap_time']:.2f} s" if res['captured']
               else 'escaped (30 s timeout)')
        bt = res['breakout_time']
        ax.set_title(f"{lab}\n{out}; hull broken "
                     f"{'at %.2f s' % bt if bt is not None else 'never'}", fontsize=9)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.legend(fontsize=7, loc='upper left', ncol=2)
    fig.suptitle(f'S011 3D Swarm Encirclement — N={N_PURSUERS}, v_E/v_P={V_EVADER/V_PURSUER:.1f}',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=110, bbox_inches='tight'); plt.close('all')
    print(f'Saved: {path}')


def plot_altitude_time(res, out_dir):
    tp, te, t = res['traj_p'], res['traj_e'], res['t_hist']
    n = tp.shape[1]
    fig, ax = plt.subplots(figsize=(11, 5))
    for i, c in enumerate(_p_colors(n)):
        ax.plot(t, tp[:, i, 2], color=c, lw=1.3, alpha=0.85,
                label=f'P{i+1}' + (' (top slot)' if i == 0 else ' (bottom slot)' if i == n - 1 else ''))
    ax.plot(t, te[:, 2], color='royalblue', lw=2.0, label='Evader', zorder=5)
    ax.axhline(Z_MIN, color='grey', ls='--', lw=0.8, label='z bounds')
    ax.axhline(Z_MAX, color='grey', ls='--', lw=0.8)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude z (m)')
    ax.set_title(f'S011 3D — Altitude vs Time (N={n}, τ={TAU_FIX} s fix latency)', fontsize=10)
    ax.legend(fontsize=8, ncol=4); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all')
    print(f'Saved: {path}')


def plot_radius_time(runs, labels, out_dir):
    fig, ax = plt.subplots(figsize=(10, 4.5))
    t_full = np.arange(0, MAX_TIME + DT / 2, DT)
    ax.plot(t_full, [shrink_radius(x) for x in t_full], color='steelblue', lw=2.0, label='R(t)')
    for res, lab, ls in zip(runs, labels, ['-', '--']):
        ax.plot(res['t_hist'], res['dmin_hist'], color='red', ls=ls, lw=1.2,
                label=f'nearest pursuer distance — {lab}')
        if res['captured']:
            ax.axvline(res['cap_time'], color='black', ls=ls, lw=1.0,
                       label=f"capture @ {res['cap_time']:.2f} s — {lab}")
    ax.axhline(R_CAPTURE, color='red', ls=':', lw=1.2, label=f'R_capture = {R_CAPTURE} m')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Distance (m)'); ax.set_yscale('log')
    ax.set_title('S011 3D — Shrinking Sphere Radius R(t) and Nearest Pursuer Distance')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3, which='both')
    plt.tight_layout()
    path = os.path.join(out_dir, 'radius_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all')
    print(f'Saved: {path}')


def plot_angular_gap(runs, labels, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for res, lab, ls in zip(runs, labels, ['-', '--']):
        axes[0].plot(res['t_hist'], res['gap_hist'], color='royalblue', ls=ls, lw=1.5, label=lab)
        axes[1].plot(res['t_hist'], res['pair_hist'], color='red', ls=ls, lw=1.5, label=lab)
    axes[0].axhline(90, color='black', ls=':', lw=1.2, label='90° = evader outside hull')
    axes[0].set_ylabel('Largest gap (deg)\nmax_u min_i ∠(u, b_i)')
    axes[1].set_ylabel('Min angle between\npursuer bearings (deg)')
    axes[1].set_xlabel('Time (s)')
    axes[0].set_title(f'S011 3D — Angular Gaps Seen from the Evader (N={N_PURSUERS})')
    for ax in axes:
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'angular_gap.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all')
    print(f'Saved: {path}')


def plot_breakout_vs_N(sweep, ideal, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    colors = plt.cm.viridis(np.linspace(0.1, 0.85, len(SWEEP_RATIO)))
    for c, ratio in zip(colors, SWEEP_RATIO):
        ax.plot(SWEEP_N, [100 * sweep[(ratio, n)]['esc'] / N_SEEDS for n in SWEEP_N], 'o-',
                color=c, lw=1.8, label=f'τ={TAU_FIX} s, v_E/v_P={ratio}')
    ax.plot(SWEEP_N, [100 * ideal[(0.7, n)]['esc'] / N_SEEDS for n in SWEEP_N], 'k--s',
            lw=1.5, label='τ=0 (ideal card law), v_E/v_P=0.7')
    ax.set_xlabel('Number of pursuers N'); ax.set_ylabel('Escape rate (%)')
    ax.set_xticks(SWEEP_N); ax.set_ylim(-5, 105)
    ax.set_title(f'S011 3D — Breakout (escape) Rate vs N ({N_SEEDS} seeds per cell, CRN)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc='upper center', bbox_to_anchor=(0.5, -0.13), ncol=3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'breakout_vs_N.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all')
    print(f'Saved: {path}')


def plot_speed_ratio_sweep(sweep, out_dir):
    M = np.array([[100 * sweep[(r, n)]['esc'] / N_SEEDS for n in SWEEP_N] for r in SWEEP_RATIO])
    fig, ax = plt.subplots(figsize=(9, 5))
    im = ax.imshow(M, cmap='Blues', vmin=0, vmax=100, aspect='auto', origin='lower')
    for i in range(len(SWEEP_RATIO)):
        for j in range(len(SWEEP_N)):
            ax.text(j, i, f'{M[i, j]:.0f}%', ha='center', va='center',
                    color='white' if M[i, j] > 55 else 'black', fontsize=9)
    ax.set_xticks(range(len(SWEEP_N))); ax.set_xticklabels(SWEEP_N)
    ax.set_yticks(range(len(SWEEP_RATIO))); ax.set_yticklabels(SWEEP_RATIO)
    ax.set_xlabel('Number of pursuers N'); ax.set_ylabel('Speed ratio v_E / v_P')
    lab = [f'{r}: sim N_crit={sim_n_critical(sweep, r) or ">12"}, card={card_n_critical(r)}'
           for r in SWEEP_RATIO]
    ax.set_title('S011 3D — Escape Rate over N × v_E/v_P (τ=%.1f s)\n' % TAU_FIX + '; '.join(lab),
                 fontsize=8)
    fig.colorbar(im, ax=ax, label='Escape rate (%)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'speed_ratio_sweep.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all')
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    tp, te, t_hist, r_hist = res['traj_p'], res['traj_e'], res['t_hist'], res['r_hist']
    n = tp.shape[1]
    T = len(t_hist)
    frames = list(range(0, T, max(1, T // 150)))
    if frames[-1] != T - 1:
        frames.append(T - 1)
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d'); ax.view_init(elev=22, azim=-55)
    allp = np.vstack([te] + [tp[:, i] for i in range(n)])
    lo, hi = allp.min(0) - 0.5, allp.max(0) + 0.5
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(Z_MIN - 0.2, Z_MAX + 0.2)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    cols = _p_colors(n)
    lines_p = [ax.plot([], [], [], color=c, lw=1.0, alpha=0.7)[0] for c in cols]
    dots_p = [ax.plot([], [], [], '^', color=c, ms=7)[0] for c in cols]
    line_e, = ax.plot([], [], [], color='royalblue', lw=1.8)
    dot_e, = ax.plot([], [], [], 'bs', ms=8)
    title = ax.set_title('')

    def update(k):
        si = frames[k]
        for i in range(n):
            lines_p[i].set_data(tp[:si+1, i, 0], tp[:si+1, i, 1]); lines_p[i].set_3d_properties(tp[:si+1, i, 2])
            dots_p[i].set_data([tp[si, i, 0]], [tp[si, i, 1]]); dots_p[i].set_3d_properties([tp[si, i, 2]])
        line_e.set_data(te[:si+1, 0], te[:si+1, 1]); line_e.set_3d_properties(te[:si+1, 2])
        dot_e.set_data([te[si, 0]], [te[si, 1]]); dot_e.set_3d_properties([te[si, 2]])
        title.set_text(f"S011 3D  t={t_hist[si]:.2f}s  R={r_hist[si]:.2f}m  gap={res['gap_hist'][si]:.0f}°")
        return lines_p + dots_p + [line_e, dot_e, title]

    ani = FuncAnimation(fig, update, frames=len(frames), interval=70, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=15)); plt.close('all')
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

def _describe(res, lab):
    out = (f"captured by P{res['cap_who']+1} @ {res['cap_time']:.2f} s" if res['captured']
           else 'escaped (no capture in 30 s)')
    bt = res['breakout_time']
    print(f"  {lab}: {out}; hull broken {'at %.2f s' % bt if bt is not None else 'never'}; "
          f"max gap {res['max_gap']:.1f}°; direction switches {res['n_switch']}; "
          f"evader z {res['z_range'][0]:.2f}–{res['z_range'][1]:.2f} m; "
          f"net displacement {res['net_disp']:.2f} m; final nearest pursuer {res['final_dmin']:.2f} m")


if __name__ == '__main__':
    print(f'S011 3D Swarm Encirclement: N={N_PURSUERS}, v_E={V_EVADER}, v_P={V_PURSUER}, '
          f'seed={MAIN_SEED}')
    main = run_simulation()
    ideal = run_simulation(tau=0.0)
    lab_main, lab_ideal = f'τ={TAU_FIX} s fix latency', 'τ=0 (ideal card law)'
    _describe(main, lab_main)
    _describe(ideal, lab_ideal)
    print('  Fibonacci slots seen from the sphere centre (no walls): largest gap ' +
          ', '.join(f'N={n}: {max_gap_deg(np.zeros(3), fibonacci_sphere(n), 0.0):.1f}°'
                    for n in SWEEP_N))
    static = run_simulation(v_e=0.0)
    print(f"  static evader (τ={TAU_FIX} s): "
          f"{'captured @ %.2f s' % static['cap_time'] if static['captured'] else 'not captured'}")

    print(f'\nSweep: N={SWEEP_N} × v_E/v_P={SWEEP_RATIO}, {N_SEEDS} seeds (CRN), τ={TAU_FIX} s')
    sweep = run_sweep()
    print(f"{'v_E/v_P':>7} {'N':>3} {'escape':>7} {'hull broken':>11} {'capture time mean±sd (s)':>26}"
          f"  {'escapes: median final nearest pursuer (m)':>42}")
    for ratio in SWEEP_RATIO:
        for n in SWEEP_N:
            s = sweep[(ratio, n)]
            ct = f"{s['cap_mean']:.2f} ± {s['cap_std']:.2f}" if s['cap_mean'] is not None else '-'
            ed = f"{s['esc_dmin']:.2f}" if s['esc_dmin'] is not None else '-'
            print(f"{ratio:>7} {n:>3} {s['esc']:>4}/{N_SEEDS} {s['broke']:>8}/{N_SEEDS} {ct:>26}  {ed:>42}")
    print('\nN_crit (smallest swept N with 0 escapes) vs card formula ceil(2/(1-cos(asin r))):')
    for ratio in SWEEP_RATIO:
        print(f'  v_E/v_P={ratio}: simulated {sim_n_critical(sweep, ratio) or ">12"}, '
              f'card formula {card_n_critical(ratio)}')

    ideal_sweep = run_sweep(tau=0.0, ratios=[0.7])
    print('\nReference τ=0 (sphere re-centred on the current evader every step), v_E/v_P=0.7:')
    for n in SWEEP_N:
        s = ideal_sweep[(0.7, n)]
        ct = f"{s['cap_mean']:.2f} ± {s['cap_std']:.2f}" if s['cap_mean'] is not None else '-'
        print(f"  N={n:>2}: escape {s['esc']}/{N_SEEDS}, hull broken {s['broke']}/{N_SEEDS}, "
              f"capture time {ct} s")

    plot_trajectories_3d([main, ideal], [lab_main, lab_ideal], OUTPUT_DIR)
    plot_altitude_time(main, OUTPUT_DIR)
    plot_radius_time([main, ideal], [lab_main, lab_ideal], OUTPUT_DIR)
    plot_angular_gap([main, ideal], [lab_main, lab_ideal], OUTPUT_DIR)
    plot_breakout_vs_N(sweep, ideal_sweep, OUTPUT_DIR)
    plot_speed_ratio_sweep(sweep, OUTPUT_DIR)
    save_animation(main, OUTPUT_DIR)
    print('All outputs saved to:', OUTPUT_DIR)
