"""
S063 Single-Plant Precision Spraying
====================================
A drone visits 20 plants in a 20 x 20 m field in nearest-neighbour TSP order. It flies
each leg behind a reference moving at 1.0 m/s, waits until it has settled over the
plant, then hovers for 3 s while spraying. A PID position controller (double-integrator
drone, 3 m/s^2 acceleration limit, GPS noise 0.1 m) rejects a sinusoidal + random wind
disturbance. The dose delivered to each plant falls off with the hover RMS error.

Three PID gain sets from the card (Loose, Tuned, Tight) are compared on the same
wind/GPS random numbers over 20 seeds, and over a wind-intensity sweep.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s063_precision_spraying.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle

# ── Parameters ────────────────────────────────────────────────────────────────
FIELD_SIZE   = 20.0     # m
N_PLANTS     = 20
LAYOUT_SEED  = 0        # card: plants from default_rng(0)
DEPOT        = np.array([0.0, 0.0])
SPRAY_WIDTH  = 2.0      # m
SIGMA_SPRAY  = SPRAY_WIDTH / 4.0   # 0.5 m dose fall-off scale
GPS_SIGMA    = 0.1      # m   measurement noise (card lists it; used here)
V_TRANSIT    = 1.0      # m/s reference speed between plants
T_SPRAY      = 3.0      # s
DT           = 0.02     # s (50 Hz)
A_MAX        = 3.0      # m/s^2 per-axis acceleration limit (added)
V_MAX        = 10.0     # m/s   airframe speed limit (added)
SETTLE_POS   = 0.3      # m   measured-error threshold to start spraying (added)
SETTLE_VEL   = 0.1      # m/s speed threshold to start spraying (added)
SETTLE_HOLD  = 0.2      # s   both thresholds held this long (added)
SETTLE_MAX   = 10.0     # s   spray anyway after this wait (added)

A_WIND       = 0.3      # m/s^2 gust amplitude
T_WIND       = 4.0      # s     gust period
SIGMA_W_BASE = 0.1      # m/s^2 baseline turbulence
SIGMA_SWEEP  = np.round(np.linspace(0.0, 0.5, 6), 2)

PID_CONFIGS = {
    'Loose': dict(kp=0.5, ki=0.05, kd=0.1),
    'Tuned': dict(kp=2.0, ki=0.2, kd=0.5),
    'Tight': dict(kp=5.0, ki=0.5, kd=1.0),
}
CFG = list(PID_CONFIGS)
COLORS = {'Loose': '#888888', 'Tuned': '#1F77B4', 'Tight': '#E87722'}
MAIN = 'Tuned'

N_SEEDS     = 20
SEEDS       = list(range(N_SEEDS))
SWEEP_SEEDS = list(range(10))
PLOT_SEED   = 0
ANIM_STEP   = 50        # sim steps per GIF frame (1 s)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'outputs',
                          '04_industrial_agriculture', 's063_precision_spraying')


# ── Planning ──────────────────────────────────────────────────────────────────

def make_plants():
    rng = np.random.default_rng(LAYOUT_SEED)
    return rng.uniform(1.0, FIELD_SIZE - 1.0, size=(N_PLANTS, 2))


def tsp_nearest_neighbour(plants, depot=DEPOT):
    unvisited = list(range(len(plants)))
    tour, cur = [], depot.copy()
    while unvisited:
        d = [np.linalg.norm(plants[j] - cur) for j in unvisited]
        tour.append(unvisited.pop(int(np.argmin(d))))
        cur = plants[tour[-1]]
    return tour


def path_length(plants, tour, depot=DEPOT):
    pts = np.vstack([depot, plants[tour]])
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def two_opt(plants, tour, depot=DEPOT):
    """2-opt on the open path from the depot (reference only, not flown)."""
    best = list(tour)
    best_len = path_length(plants, best, depot)
    improved = True
    while improved:
        improved = False
        for i in range(len(best) - 1):
            for j in range(i + 1, len(best)):
                cand = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                L = path_length(plants, cand, depot)
                if L < best_len - 1e-9:
                    best, best_len, improved = cand, L, True
    return best, best_len


# ── Simulation ────────────────────────────────────────────────────────────────

def noise(seed, sigma_w, n_max):
    """Common random numbers: per-plant gust phases, per-step turbulence and GPS noise."""
    rng = np.random.default_rng(1000 + seed)
    phases = rng.uniform(0, 2 * np.pi, size=(N_PLANTS, 2))
    turb = rng.normal(0, 1, size=(n_max, 2)) * sigma_w
    gps = rng.normal(0, GPS_SIGMA, size=(n_max, 2))
    return phases, turb, gps


def run_mission(plants, tour, gains, sigma_w, seed, keep_log=False):
    """Full mission with scalar per-axis arithmetic (same model as the card, 50 Hz Euler)."""
    kp, ki, kd = gains['kp'], gains['ki'], gains['kd']
    n_max = int(600.0 / DT)
    phases, turb, gps = noise(seed, sigma_w, n_max)
    tx, ty = turb[:, 0].tolist(), turb[:, 1].tolist()
    gx, gy = gps[:, 0].tolist(), gps[:, 1].tolist()
    px, py = float(DEPOT[0]), float(DEPOT[1])
    vx = vy = ix = iy = 0.0
    k = 0
    n_sat = 0
    n_vsat = 0
    max_off = [0.0]
    rms = np.zeros(N_PLANTS)
    wait = np.zeros(N_PLANTS)
    timed_out = np.zeros(N_PLANTS, dtype=bool)
    log = {'t': [], 'pos': [], 'mode': [], 'plant': []} if keep_log else None
    hover_err = []
    w_g = 2 * math.pi / T_WIND
    n_spray = int(round(T_SPRAY / DT))

    def step(rx, ry, vrx, vry, ph, mode, i_visit):
        nonlocal px, py, vx, vy, ix, iy, k, n_sat, n_vsat
        t = k * DT
        ex, ey = rx - (px + gx[k]), ry - (py + gy[k])
        ix += ex * DT
        iy += ey * DT
        ax = kp * ex + ki * ix + kd * (vrx - vx)
        ay = kp * ey + ki * iy + kd * (vry - vy)
        if abs(ax) > A_MAX or abs(ay) > A_MAX:
            n_sat += 1
            ax = min(max(ax, -A_MAX), A_MAX)
            ay = min(max(ay, -A_MAX), A_MAX)
        assert abs(ax) <= A_MAX and abs(ay) <= A_MAX
        vx += (ax + A_WIND * math.sin(w_g * t + ph[0]) + tx[k]) * DT
        vy += (ay + A_WIND * math.sin(w_g * t + ph[1]) + ty[k]) * DT
        sp = math.hypot(vx, vy)
        if sp > V_MAX:
            vx *= V_MAX / sp
            vy *= V_MAX / sp
            n_vsat += 1
        assert math.hypot(vx, vy) <= V_MAX + 1e-9
        px += vx * DT
        py += vy * DT
        if abs(px - FIELD_SIZE / 2) > max_off[0]:
            max_off[0] = abs(px - FIELD_SIZE / 2)
        if abs(py - FIELD_SIZE / 2) > max_off[0]:
            max_off[0] = abs(py - FIELD_SIZE / 2)
        if keep_log:
            log['t'].append(t)
            log['pos'].append((px, py))
            log['mode'].append(mode)
            log['plant'].append(i_visit)
        k += 1

    prev = DEPOT.copy()
    for i_visit, p_idx in enumerate(tour):
        target = plants[p_idx]
        cx, cy = float(target[0]), float(target[1])
        ph = (float(phases[i_visit, 0]), float(phases[i_visit, 1]))
        leg = target - prev
        L = float(np.linalg.norm(leg))
        ux, uy = float(leg[0] / L), float(leg[1] / L)
        n_leg = int(np.ceil(L / (V_TRANSIT * DT)))
        for j in range(n_leg):
            s = min((j + 1) * V_TRANSIT * DT, L)
            moving = s < L
            step(prev[0] + s * ux, prev[1] + s * uy,
                 V_TRANSIT * ux if moving else 0.0, V_TRANSIT * uy if moving else 0.0,
                 ph, 0, i_visit)
        # settle: measured error and speed below thresholds for SETTLE_HOLD, or time-out
        held, n_wait = 0, 0
        n_wait_max = int(round(SETTLE_MAX / DT))
        while n_wait < n_wait_max:
            ok = (math.hypot(cx - px - gx[k], cy - py - gy[k]) < SETTLE_POS and
                  math.hypot(vx, vy) < SETTLE_VEL)
            held = held + 1 if ok else 0
            if held * DT >= SETTLE_HOLD - 1e-9:
                break
            step(cx, cy, 0.0, 0.0, ph, 1, i_visit)
            n_wait += 1
        wait[i_visit] = n_wait * DT
        timed_out[i_visit] = n_wait >= n_wait_max
        # spray window; integrator reset at its start (card)
        ix = iy = 0.0
        errs = np.empty(n_spray)
        for j in range(n_spray):
            step(cx, cy, 0.0, 0.0, ph, 2, i_visit)
            errs[j] = math.hypot(cx - px, cy - py)
        rms[i_visit] = math.sqrt(float(np.mean(errs ** 2)))
        if keep_log:
            hover_err.append(errs)
        prev = target.copy()

    dose = np.exp(-rms ** 2 / (2 * SIGMA_SPRAY ** 2))
    res = {'rms': rms, 'dose': dose, 'c_eff': float(dose.mean()), 'time': k * DT,
           'wait': wait, 'timeouts': int(timed_out.sum()), 'sat_frac': n_sat / k,
           'vsat_frac': n_vsat / k, 'max_off': max_off[0] - FIELD_SIZE / 2}
    if keep_log:
        for kk in ('t', 'pos', 'mode', 'plant'):
            log[kk] = np.array(log[kk])
        res['log'] = log
        res['hover_err'] = hover_err
    return res


def run_simulation():
    plants = make_plants()
    tour = tsp_nearest_neighbour(plants)
    mc = {c: [run_mission(plants, tour, PID_CONFIGS[c], SIGMA_W_BASE, s,
                          keep_log=(s == PLOT_SEED)) for s in SEEDS] for c in CFG}
    sweep = {c: {sw: [run_mission(plants, tour, PID_CONFIGS[c], sw, s)['c_eff']
                      for s in SWEEP_SEEDS] for sw in SIGMA_SWEEP} for c in CFG}
    return plants, tour, mc, sweep


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_field_overview(plants, tour, mc, out_dir):
    r = mc[MAIN][PLOT_SEED]
    fig, ax = plt.subplots(figsize=(7.5, 7))
    ax.add_patch(Rectangle((0, 0), FIELD_SIZE, FIELD_SIZE, fill=False, ec='grey', lw=2))
    pts = np.vstack([DEPOT, plants[tour]])
    ax.plot(pts[:, 0], pts[:, 1], 'k-', lw=0.8, label='NN TSP tour')
    P = r['log']['pos']
    ax.plot(P[:, 0], P[:, 1], color='tab:red', lw=0.6, alpha=0.6, label='drone path')
    dose_by_plant = np.empty(N_PLANTS)
    dose_by_plant[tour] = r['dose']
    sc = ax.scatter(plants[:, 0], plants[:, 1], c=dose_by_plant, cmap='RdYlGn', vmin=0.6,
                    vmax=1.0, s=60 + 140 * dose_by_plant, edgecolors='k', zorder=5)
    for n, p in enumerate(tour):
        ax.annotate(str(n + 1), plants[p], xytext=(6, 6) if n % 2 == 0 else (-16, -14),
                    textcoords='offset points',
                    fontsize=8, zorder=7,
                    bbox=dict(boxstyle='round,pad=0.1', fc='white', ec='none', alpha=0.7))
    ax.plot(*DEPOT, 's', color='k', ms=10, label='depot')
    fig.colorbar(sc, ax=ax, label='dose fraction $D_i/D_0$')
    ax.set_xlim(-1, 21)
    ax.set_ylim(-1, 21)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f"S063 Field overview — {MAIN} PID, seed {PLOT_SEED}: "
                 f"C_eff = {r['c_eff']:.3f}\n(numbers = visit order)")
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'field_overview.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_rms_bars(mc, out_dir):
    fig, ax = plt.subplots(figsize=(10, 7))
    y = np.arange(N_PLANTS)
    h = 0.27
    for j, c in enumerate(CFG):
        rms = np.mean([r['rms'] for r in mc[c]], axis=0) * 100
        ax.barh(y + (j - 1) * h, rms, height=h, color=COLORS[c], label=c)
    ax.axvline(SIGMA_SPRAY * 100, color='red', ls='--', label=r'$\sigma_{spray}$ = 50 cm')
    ax.axvline(SIGMA_SPRAY * 50, color='green', ls=':', label=r'$\sigma_{spray}/2$ = 25 cm')
    ax.set_yticks(y)
    ax.set_yticklabels([f'visit {i + 1}' for i in y], fontsize=8)
    ax.invert_yaxis()
    ax.set_xscale('log')
    ax.set_xlabel(f'hover RMS error (cm, log scale), mean over {N_SEEDS} seeds')
    ax.set_title(f'S063 Per-plant hover RMS error (σ_w = {SIGMA_W_BASE} m/s²)')
    ax.legend(fontsize=8)
    ax.grid(axis='x', alpha=0.3)
    path = os.path.join(out_dir, 'rms_per_plant.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_ceff_sweep(sweep, out_dir):
    fig, ax = plt.subplots(figsize=(8, 5))
    for c in CFG:
        m = [np.mean(sweep[c][sw]) for sw in SIGMA_SWEEP]
        sd = [np.std(sweep[c][sw]) for sw in SIGMA_SWEEP]
        ax.errorbar(SIGMA_SWEEP, m, yerr=sd, marker='o', capsize=3, color=COLORS[c], label=c)
    ax.axhline(0.9, color='k', ls='--', lw=1, label='0.9')
    ax.axhline(0.8, color='k', ls=':', lw=1, label='0.8')
    ax.axhspan(0, 0.8, color='red', alpha=0.08)
    ax.set_ylim(0.0, 1.02)
    ax.set_xlabel(r'turbulence $\sigma_w$ (m/s²)')
    ax.set_ylabel('$C_{eff}$')
    ax.set_title(f'S063 Effective coverage vs turbulence (mean ± std, {len(SWEEP_SEEDS)} seeds)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'ceff_vs_wind.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_hover_timeseries(mc, out_dir):
    rms_main = mc[MAIN][PLOT_SEED]['rms']
    worst, best = int(np.argmax(rms_main)), int(np.argmin(rms_main))
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), sharey=True)
    t = np.arange(int(round(T_SPRAY / DT))) * DT
    for ax, i, name in zip(axes, [worst, best], ['worst', 'best']):
        for c in CFG:
            e = mc[c][PLOT_SEED]['hover_err'][i]
            ax.plot(t, e * 100, color=COLORS[c],
                    label=f"{c} (RMS {mc[c][PLOT_SEED]['rms'][i] * 100:.1f} cm)")
        ax.set_title(f'visit {i + 1} ({name} for {MAIN}), seed {PLOT_SEED}')
        ax.set_xlabel('time in spray window (s)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axes[0].set_yscale('log')
    axes[0].set_ylabel('|e(t)| (cm, log scale)')
    fig.suptitle('S063 Hover error during the 3 s spray window')
    plt.tight_layout()
    path = os.path.join(out_dir, 'hover_timeseries.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_config_summary(mc, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.3))
    for ax, (f, lab) in zip(axes, [('c_eff', '$C_{eff}$'), ('time', 'mission time (s)'),
                                   ('sat_frac', 'fraction of steps at A_MAX')]):
        data = [[r[f] for r in mc[c]] for c in CFG]
        bp = ax.boxplot(data, patch_artist=True, widths=0.6)
        for patch, c in zip(bp['boxes'], CFG):
            patch.set_facecolor(COLORS[c])
            patch.set_alpha(0.7)
        ax.set_xticks([1, 2, 3])
        ax.set_xticklabels(CFG)
        ax.set_title(lab)
        ax.grid(alpha=0.3)
    fig.suptitle(f'S063 PID gain sets over {N_SEEDS} seeds (σ_w = {SIGMA_W_BASE} m/s², CRN)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'config_summary.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(plants, tour, mc, out_dir):
    r = mc[MAIN][PLOT_SEED]
    lg = r['log']
    n = len(lg['t'])
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.add_patch(Rectangle((0, 0), FIELD_SIZE, FIELD_SIZE, fill=False, ec='grey', lw=2))
    pts = np.vstack([DEPOT, plants[tour]])
    ax.plot(pts[:, 0], pts[:, 1], 'k:', lw=0.6)
    dots = ax.scatter(plants[tour, 0], plants[tour, 1], c=['lightgrey'] * N_PLANTS, s=70,
                      edgecolors='k', zorder=4)
    trail, = ax.plot([], [], color='tab:red', lw=0.8, alpha=0.6)
    drone, = ax.plot([], [], 'o', color='red', ms=8, zorder=6)
    halo = Circle((0, 0), SIGMA_SPRAY, fc='tab:blue', alpha=0.25, ec='tab:blue')
    ax.add_patch(halo)
    ax.set_xlim(-1, 21)
    ax.set_ylim(-1, 21)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    mode_name = {0: 'transit', 1: 'settling', 2: 'spraying'}

    def update(fi):
        i = min(fi * ANIM_STEP, n - 1)
        p = lg['pos'][i]
        trail.set_data(lg['pos'][:i + 1, 0], lg['pos'][:i + 1, 1])
        drone.set_data([p[0]], [p[1]])
        halo.center = (p[0], p[1])
        halo.set_visible(lg['mode'][i] == 2)
        cols = []
        cur = lg['plant'][i]
        for v in range(N_PLANTS):
            done = v < cur or (v == cur and lg['mode'][i] == 2 and i == n - 1)
            if done:
                cols.append('green' if r['dose'][v] > 0.9 else 'orange')
            else:
                cols.append('lightgrey')
        dots.set_facecolors(cols)
        ax.set_title(f"S063 {MAIN} PID — t = {lg['t'][i]:.0f} s, visit {cur + 1}: "
                     f"{mode_name[int(lg['mode'][i])]}\n(green: dose > 0.9, orange: ≤ 0.9)")
        return trail, drone, halo, dots

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10)
    plt.close(fig)
    print(f'Saved: {path}  ({n_frames} frames)')


def _ms(v, fmt='.3f'):
    v = np.asarray(v, dtype=float)
    return f'{v.mean():{fmt}} ± {v.std():{fmt}}'


if __name__ == '__main__':
    plants, tour, mc, sweep = run_simulation()
    L_nn = path_length(plants, tour)
    tour2, L_2opt = two_opt(plants, tour)
    print('\n=== S063 Single-Plant Precision Spraying — Results ===')
    print(f'  NN TSP path length (depot + 20 plants, open path): {L_nn:.2f} m')
    print(f'  2-opt path length (reference): {L_2opt:.2f} m -> NN is '
          f'{100 * (L_nn / L_2opt - 1):.1f} % longer')
    for c, g in PID_CONFIGS.items():
        zeta = g['kd'] / (2 * np.sqrt(g['kp']))
        wn = np.sqrt(g['kp'])
        w = 2 * np.pi / T_WIND
        amp = A_WIND / abs(complex(g['kp'] - w ** 2, g['kd'] * w))
        print(f"  {c}: Routh margin Kp*Kd - Ki = {g['kp'] * g['kd'] - g['ki']:.3f} "
              f"(> 0 needed for stability)")
        print(f"  {c}: omega_n = {wn:.3f} rad/s, zeta (PD part) = {zeta:.3f}, "
              f"steady gust amplitude (PD, no I) = {100 * amp:.1f} cm at omega_w = {w:.3f} rad/s")
        poles = np.roots([1.0, g['kd'], g['kp'], g['ki']])
        cplx = poles[np.abs(poles.imag) > 1e-9]
        dom = cplx[np.argmax(cplx.real)] if len(cplx) else poles[np.argmax(poles.real)]
        zeta3 = round(-dom.real / abs(dom), 6) + 0.0
        jw = 1j * w
        amp3 = A_WIND * abs(jw / (jw ** 3 + g['kd'] * jw ** 2 + g['kp'] * jw + g['ki']))
        print(f"  {c}: full PID loop poles " + ', '.join(f'{z.real:.3f}{z.imag:+.3f}j' for z in poles)
              + f"; dominant zeta = {zeta3:.3f}; steady gust amplitude (PID) = {100 * amp3:.1f} cm")
    print(f'  Monte Carlo: {N_SEEDS} seeds, sigma_w = {SIGMA_W_BASE} m/s^2, CRN across gain sets')
    for c in CFG:
        R = mc[c]
        g = lambda f: [r[f] for r in R]          # noqa: E731
        allrms = np.concatenate(g('rms'))
        print(f'  [{c}]')
        print(f"    C_eff              : {_ms(g('c_eff'))}  (min {np.min(g('c_eff')):.3f})")
        print(f"    Hover RMS (cm)     : mean {100 * allrms.mean():.1f}, "
              f"95th pct {100 * np.percentile(allrms, 95):.1f}, max {100 * allrms.max():.1f}")
        print(f"    Plants with dose > 0.9: {100 * np.mean(np.concatenate(g('dose')) > 0.9):.1f} %")
        print(f"    Mission time       : {_ms(g('time'), '.1f')} s")
        print(f"    Settle wait / plant: {np.mean(np.concatenate(g('wait'))):.2f} s; "
              f"timeouts ({SETTLE_MAX:.0f} s) {int(np.sum(g('timeouts')))} of {N_SEEDS * N_PLANTS}")
        print(f"    Steps at A_MAX     : {100 * np.mean(g('sat_frac')):.2f} %; at V_MAX "
              f"{100 * np.mean(g('vsat_frac')):.2f} %")
        print(f"    Farthest excursion outside the field: {np.max(g('max_off')):.1f} m")
    ce = {c: np.array([r['c_eff'] for r in mc[c]]) for c in CFG}
    print(f"  Per-seed C_eff wins: Tight > Tuned {int(np.sum(ce['Tight'] > ce['Tuned']))}/{N_SEEDS}, "
          f"Tuned > Loose {int(np.sum(ce['Tuned'] > ce['Loose']))}/{N_SEEDS}, "
          f"Tight > Loose {int(np.sum(ce['Tight'] > ce['Loose']))}/{N_SEEDS}")
    print('  C_eff vs sigma_w (10 seeds):')
    for c in CFG:
        print(f'    {c}: ' + ', '.join(f'{sw:.1f}→{np.mean(sweep[c][sw]):.3f}±{np.std(sweep[c][sw]):.3f}'
                                      for sw in SIGMA_SWEEP))
    # Loose without actuator limits: does the growth come from saturation or from the gains?
    a_keep, v_keep = A_MAX, V_MAX
    A_MAX, V_MAX = 1e9, 1e9
    r_nl = run_mission(plants, tour, PID_CONFIGS['Loose'], SIGMA_W_BASE, PLOT_SEED)
    A_MAX, V_MAX = a_keep, v_keep
    r_l = mc['Loose'][PLOT_SEED]
    print(f"  Loose seed {PLOT_SEED}, hover RMS visits 1-3 / 18-20 (m): with limits "
          f"{np.round(r_l['rms'][:3], 2).tolist()} / {np.round(r_l['rms'][-3:], 2).tolist()}; "
          f"without limits {np.round(r_nl['rms'][:3], 2).tolist()} / "
          f"{np.round(r_nl['rms'][-3:], 2).tolist()}")
    pd_loose = [run_mission(plants, tour, dict(PID_CONFIGS['Loose'], ki=0.0), SIGMA_W_BASE, s)['c_eff']
                for s in SEEDS]
    print(f"  Loose with Ki = 0 (PD only, stable): C_eff {np.mean(pd_loose):.3f} ± {np.std(pd_loose):.3f} "
          f"over {N_SEEDS} seeds")
    rm = mc[MAIN][PLOT_SEED]
    print(f"  Seed {PLOT_SEED} {MAIN}: C_eff {rm['c_eff']:.3f}, mean RMS "
          f"{100 * rm['rms'].mean():.1f} cm, mission {rm['time']:.1f} s, worst visit "
          f"{int(np.argmax(rm['rms'])) + 1} ({100 * rm['rms'].max():.1f} cm), best visit "
          f"{int(np.argmin(rm['rms'])) + 1} ({100 * rm['rms'].min():.1f} cm)")

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_field_overview(plants, tour, mc, out)
    plot_rms_bars(mc, out)
    plot_ceff_sweep(sweep, out)
    plot_hover_timeseries(mc, out)
    plot_config_summary(mc, out)
    save_animation(plants, tour, mc, out)
