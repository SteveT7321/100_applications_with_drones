"""
S076 Extreme Heat Cooling Strategy
==================================
A single inspection drone flies a boustrophedon (lawnmower) path over a 100 x 100 m field
at 45 degC ambient. Motor temperature follows a first-order thermal model driven by the
speed-dependent motor power P(v) = P_hover + k_v v^2. Four strategies are compared:
  1. No thermal management (constant v0 = 1 m/s)
  2. Reactive throttle, hold T_warn (above T_warn fly at the speed that holds the current
     temperature; cooling pause at T_max)
  3. Reactive throttle to T_max (above T_warn fly at the speed whose steady state is T_max;
     cooling pause at T_max)
  4. Predictive scheduled pauses (closed-form plan that never enters the warning zone)
The card's parameter set (T_amb = 45 degC) is run first; an ambient-temperature sweep
(40-55 degC) and a detailed hot case (52 degC) show where the strategies actually differ.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s076_thermal_management.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection

# ── Parameters (card) ─────────────────────────────────────────────────────────
FIELD_W     = 100.0      # m   cross-track width
FIELD_H     = 100.0      # m   along-track length
STRIP_D     = 5.0        # m   strip spacing
V0          = 1.0        # m/s nominal inspection speed
T_AMB       = 45.0       # degC ambient (card case)
T_MAX       = 85.0       # degC hard limit
T_WARN      = 80.0       # degC warning threshold
T_PAUSE     = 30.0       # s   cooling-pause duration
R_TH        = 0.8        # degC/W thermal resistance
C_TH        = 50.0       # J/degC thermal capacitance
TAU         = C_TH * R_TH  # s heating time constant (40 s)
TAU_COOL    = 20.0       # s   cooling time constant in a shaded pause
P_HOVER     = 30.0       # W   hover power
K_V         = 15.0       # W s^2/m^2 speed-power coefficient
T_TURN      = 3.0        # s   one 180 deg heading reversal
P_TURN      = P_HOVER    # W   motor power during a turn (hover)
DT          = 0.1        # s   time step
V_MIN       = 0.05       # m/s reactive speed floor (card)

# ── Added parameters ──────────────────────────────────────────────────────────
T_AMB_HOT   = 52.0       # degC detailed hot case (unmanaged flight breaches T_max)
T_AMB_SWEEP = np.arange(40.0, 55.01, 1.0)   # degC ambient sweep
ANIM_DT     = 30.0       # s   simulated time per GIF frame
ANIM_TRAIL_EVERY = 10    # log samples between drawn trail points in the GIF

STRATEGIES = ['none', 'reactive', 'reactive_max', 'predictive']
LABELS = {'none': 'No management', 'reactive': 'Reactive (hold T_warn)',
          'reactive_max': 'Reactive (throttle to T_max)',
          'predictive': 'Predictive pauses'}
COLORS = {'none': '#D62728', 'reactive': '#1F77B4', 'reactive_max': '#9467BD',
          'predictive': '#2CA02C'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '04_industrial_agriculture', 's076_thermal_management')


# ── Thermal model ─────────────────────────────────────────────────────────────

def p_motor(v):
    return P_HOVER + K_V * v ** 2


def t_ss(v, t_amb):
    """Steady-state motor temperature at constant speed v."""
    return t_amb + R_TH * p_motor(v)


def v_safe_static(t_amb, t_lim=T_MAX):
    """Largest constant speed whose steady-state temperature is t_lim (card eq.)."""
    return float(np.sqrt(max(0.0, (t_lim - t_amb) / (R_TH * K_V) - P_HOVER / K_V)))


def v_safe_card(t_m):
    """Card's instantaneous formula v_safe(T_m) (uses T_max - T_m). Kept for the check."""
    return float(np.sqrt(max(0.0, (T_MAX - t_m) / (R_TH * K_V) - P_HOVER / K_V)))


def v_hold(t_m, t_amb):
    """Speed at which dT/dt = 0 at the current temperature (corrected reactive law)."""
    return float(np.sqrt(max(0.0, ((t_m - t_amb) / R_TH - P_HOVER) / K_V)))


def heat(t_m, t_inf, h, tau):
    """Exact solution of tau dT/dt = t_inf - T over a step h (piecewise-constant power)."""
    return t_inf + (t_m - t_inf) * np.exp(-h / tau)


# ── Path ──────────────────────────────────────────────────────────────────────

def build_path():
    """Lawnmower path as a list of ('line', p0, p1) and ('turn',) elements."""
    n_strips = int(np.ceil(FIELD_W / STRIP_D))
    elems = []
    for i in range(n_strips):
        y = (i + 0.5) * STRIP_D
        xa, xb = (0.0, FIELD_H) if i % 2 == 0 else (FIELD_H, 0.0)
        elems.append(('line', np.array([xa, y]), np.array([xb, y])))
        if i < n_strips - 1:
            elems.append(('line', np.array([xb, y]), np.array([xb, y + STRIP_D])))
            elems.append(('turn',))
    return elems, n_strips


# ── Predictive planner (closed form, before flight) ───────────────────────────

def plan_pauses(t_amb, v=V0, thresh=T_WARN):
    """Path distances at which T_m would reach T_warn; a pause is inserted there."""
    elems, _ = build_path()
    t_inf, t_inf_turn = t_ss(v, t_amb), t_amb + R_TH * P_TURN
    assert t_inf_turn < thresh, 'hovering alone would enter the warning zone'
    tm, s, pauses = t_amb, 0.0, []
    for e in elems:
        if e[0] == 'turn':
            tm = heat(tm, t_inf_turn, T_TURN, TAU)
            continue
        rem = float(np.linalg.norm(e[2] - e[1]))
        while rem > 1e-12:
            if t_inf > thresh:
                t_w = TAU * np.log((t_inf - tm) / (t_inf - thresh)) if tm < thresh else 0.0
            else:
                t_w = np.inf
            if v * t_w >= rem:
                tm = heat(tm, t_inf, rem / v, TAU)
                s += rem
                rem = 0.0
            else:
                s += v * t_w
                rem -= v * t_w
                pauses.append(s)
                tm = t_amb + (thresh - t_amb) * np.exp(-T_PAUSE / TAU_COOL)
    return pauses


# ── Simulation engine ─────────────────────────────────────────────────────────

def simulate(strategy, t_amb=T_AMB, reactive_law='hold', pred_thresh=T_WARN):
    """Fly the full lawnmower path under one strategy; returns a log dict and an outcome."""
    elems, n_strips = build_path()
    if strategy == 'reactive_max':
        strategy, reactive_law = 'reactive', 'tmax'
    plan = plan_pauses(t_amb, thresh=pred_thresh) if strategy == 'predictive' else []
    pi = 0
    tm, clock, s = t_amb, 0.0, 0.0
    pos = elems[0][1].copy()
    L = dict(t=[0.0], s=[0.0], x=[pos[0]], y=[pos[1]], T=[tm], v=[0.0], mode=['cruise'])
    stats = dict(t_cruise=0.0, t_turn=0.0, t_pause=0.0, n_pauses=0, t_over=0.0,
                 n_switch=0, t_throttled=0.0, d_throttled=0.0)
    prev_slow = False

    def push(mode, v):
        L['t'].append(clock); L['s'].append(s); L['x'].append(pos[0]); L['y'].append(pos[1])
        L['T'].append(tm); L['v'].append(v); L['mode'].append(mode)

    def pause():
        nonlocal tm, clock
        stats['n_pauses'] += 1
        n = int(round(T_PAUSE / DT))
        for _ in range(n):
            tm = heat(tm, t_amb, DT, TAU_COOL)
            clock += DT
            push('pause', 0.0)
        stats['t_pause'] += n * DT

    for e in elems:
        if e[0] == 'turn':
            n = int(round(T_TURN / DT))
            for _ in range(n):
                tm = heat(tm, t_amb + R_TH * P_TURN, DT, TAU)
                clock += DT
                if tm > T_MAX:
                    stats['t_over'] += DT
                push('turn', 0.0)
            stats['t_turn'] += n * DT
            continue
        p0, p1 = e[1], e[2]
        seg = float(np.linalg.norm(p1 - p0))
        u = (p1 - p0) / seg
        done = 0.0
        while seg - done > 1e-9:
            v = V0
            limit = seg - done
            if strategy == 'reactive':
                if tm >= T_MAX:
                    pause()
                elif tm >= T_WARN:
                    if reactive_law == 'hold':
                        v = min(V0, max(V_MIN, v_hold(tm, t_amb)))
                    elif reactive_law == 'tmax':
                        # speed whose steady-state temperature is T_max (uses the 80-85 band)
                        v = min(V0, max(V_MIN, v_safe_static(t_amb)))
                    else:   # literal card formula
                        v = min(V0, max(V_MIN, v_safe_card(tm)))
                slow = v < V0 - 1e-12
                if slow != prev_slow:
                    stats['n_switch'] += 1
                prev_slow = slow
            elif strategy == 'predictive' and pi < len(plan):
                if plan[pi] - s <= 1e-9:
                    pause()
                    pi += 1
                    continue
                limit = min(limit, plan[pi] - s)
            ds = min(v * DT, limit)
            h = ds / v
            tm = heat(tm, t_ss(v, t_amb), h, TAU)
            done += ds
            s += ds
            clock += h
            pos = p0 + u * done
            stats['t_cruise'] += h
            if v < V0 - 1e-12:
                stats['t_throttled'] += h
                stats['d_throttled'] += ds
            if tm > T_MAX:
                stats['t_over'] += h
            push('cruise', v)
    if strategy == 'predictive':
        assert pi == len(plan), 'not every planned pause was executed'

    for k in L:
        L[k] = np.array(L[k]) if k != 'mode' else np.array(L[k])
    # Physical invariants
    assert np.all(np.diff(L['t']) > 0), 'time must advance'
    step_len = np.hypot(np.diff(L['x']), np.diff(L['y']))
    assert np.all(step_len <= np.maximum(L['v'][1:], 0) * np.diff(L['t']) + 1e-9), 'teleport'
    assert np.all(L['v'] <= V0 + 1e-12), 'speed above v0'
    assert np.all((L['x'] >= -1e-9) & (L['x'] <= FIELD_H + 1e-9) &
                  (L['y'] >= -1e-9) & (L['y'] <= FIELD_W + 1e-9)), 'left the field'
    assert np.all(L['T'] >= t_amb - 1e-9), 'motor below ambient'
    if strategy == 'predictive':
        assert L['T'].max() <= pred_thresh + 1e-6, 'predictive plan exceeded its threshold'
    if strategy == 'reactive' and reactive_law == 'tmax':
        assert L['T'].max() <= T_MAX, 'throttle-to-T_max variant exceeded T_max'
    out = dict(strategy=strategy, t_amb=t_amb, mission_s=clock, peak_T=float(L['T'].max()),
               path_m=s, n_strips=n_strips, **stats)
    return L, out


def run_simulation():
    """Card case, hot case, the literal card reactive law, and the ambient sweep."""
    res = {}
    for case, ta in (('card', T_AMB), ('hot', T_AMB_HOT)):
        res[case] = {st: simulate(st, ta) for st in STRATEGIES}
    res['card_literal'] = simulate('reactive', T_AMB, reactive_law='card')
    # fairness check: predictive schedule allowed to use the same 85 degC limit
    res['pred85_hot'] = simulate('predictive', T_AMB_HOT, pred_thresh=T_MAX)
    sweep = {st: [] for st in STRATEGIES}
    for ta in T_AMB_SWEEP:
        for st in STRATEGIES:
            _, o = simulate(st, float(ta))
            sweep[st].append(o)
    res['sweep'] = sweep
    return res


# ── Plotting ──────────────────────────────────────────────────────────────────

def _case_title(case):
    return (f'Card case, $T_{{amb}}$ = {T_AMB:.0f} °C' if case == 'card'
            else f'Hot case, $T_{{amb}}$ = {T_AMB_HOT:.0f} °C')


def plot_temperature_traces(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5), sharey=True)
    for ax, case in zip(axes, ('card', 'hot')):
        for st in STRATEGIES:
            L, o = res[case][st]
            ax.plot(L['t'] / 60, L['T'], color=COLORS[st], lw=1.2,
                    label=f"{LABELS[st]} (peak {o['peak_T']:.2f} °C)")
        ax.axhline(T_MAX, color='k', ls='--', lw=1, label=f'$T_{{max}}$ = {T_MAX:.0f} °C')
        ax.axhline(T_WARN, color='orange', ls=':', lw=1.4, label=f'$T_{{warn}}$ = {T_WARN:.0f} °C')
        ta = T_AMB if case == 'card' else T_AMB_HOT
        ax.axhline(ta, color='grey', ls=':', lw=1, label='$T_{amb}$')
        ax.set_xlabel('Time (min)')
        ax.set_title(_case_title(case))
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='lower right')
    axes[0].set_ylabel('Motor temperature (°C)')
    fig.suptitle('S076 Motor Temperature vs Time')
    plt.tight_layout()
    path = os.path.join(out_dir, 'temperature_traces.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_speed_profile(res, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(13, 6.5), sharex=True)
    for ax, case in zip(axes, ('card', 'hot')):
        for st in ('reactive', 'reactive_max', 'predictive'):
            L, o = res[case][st]
            ax.plot(L['t'] / 60, L['v'], color=COLORS[st], lw=0.9, alpha=0.85,
                    label=f"{LABELS[st]} (mission {o['mission_s'] / 60:.1f} min)")
        ax.set_ylabel('Speed (m/s)')
        ax.set_ylim(-0.05, 1.15)
        ax.set_title(_case_title(case) + ' — speed 0 = turn or cooling pause', fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='lower right')
    axes[1].set_xlabel('Time (min)')
    fig.suptitle('S076 Speed Profiles of the Managed Strategies')
    plt.tight_layout()
    path = os.path.join(out_dir, 'speed_profile.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_flight_paths(res, out_dir):
    fig, axes = plt.subplots(2, 4, figsize=(21, 10.5))
    norm = plt.Normalize(T_AMB, 90)
    for r, case in enumerate(('card', 'hot')):
        for c, st in enumerate(STRATEGIES):
            ax = axes[r, c]
            L, o = res[case][st]
            ax.add_patch(mpatches.Rectangle((0, 0), FIELD_H, FIELD_W, fc='wheat', alpha=0.3,
                                            ec='k'))
            pts = np.column_stack([L['x'], L['y']])
            segs = np.stack([pts[:-1], pts[1:]], axis=1)
            lc = LineCollection(segs, cmap='hot_r', norm=norm, lw=2)
            lc.set_array(L['T'][1:])
            ax.add_collection(lc)
            idx = np.where((L['mode'][1:] == 'pause') & (L['mode'][:-1] != 'pause'))[0] + 1
            if len(idx):
                ax.scatter(L['x'][idx], L['y'][idx], marker='x', s=70, color='blue', zorder=5)
            ax.set_xlim(-5, FIELD_H + 5)
            ax.set_ylim(-5, FIELD_W + 5)
            ax.set_aspect('equal')
            ax.set_title(f"{LABELS[st]} — {_case_title(case)}\npeak {o['peak_T']:.1f} °C, "
                         f"{o['mission_s'] / 60:.1f} min, {o['n_pauses']} pauses", fontsize=9)
            ax.set_xlabel('x (m)')
            ax.set_ylabel('y (m)')
    fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap='hot_r'), ax=axes, shrink=0.6,
                 label='Motor temperature (°C)')
    fig.suptitle('S076 Lawnmower Path Coloured by Motor Temperature (blue × = cooling pause)')
    path = os.path.join(out_dir, 'flight_paths.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _intervals(L, mode):
    m = L['mode'] == mode
    out, i = [], 1
    while i < len(m):
        if m[i]:
            j = i
            while j + 1 < len(m) and m[j + 1]:
                j += 1
            out.append((L['t'][i - 1], L['t'][j] - L['t'][i - 1]))
            i = j + 1
        else:
            i += 1
    return out


def plot_mission_timeline(res, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(13, 5.5), sharex=True)
    for ax, case in zip(axes, ('card', 'hot')):
        for k, st in enumerate(reversed(STRATEGIES)):
            L, o = res[case][st]
            ax.barh(k, o['mission_s'] / 60, color=COLORS[st], alpha=0.35, height=0.6)
            for t0, w in _intervals(L, 'turn'):
                ax.barh(k, w / 60, left=t0 / 60, color='grey', height=0.6)
            for t0, w in _intervals(L, 'pause'):
                ax.barh(k, w / 60, left=t0 / 60, color='blue', height=0.6)
            ax.text(o['mission_s'] / 60 + 0.3, k, f"{o['mission_s'] / 60:.1f} min",
                    va='center', fontsize=8)
        ax.set_yticks(range(len(STRATEGIES)))
        ax.set_yticklabels([LABELS[s] for s in reversed(STRATEGIES)], fontsize=8)
        ax.set_title(_case_title(case) + ' — blue = cooling pause, grey = turn', fontsize=10)
    axes[1].set_xlabel('Mission time (min)')
    fig.suptitle('S076 Mission Timeline')
    plt.tight_layout()
    path = os.path.join(out_dir, 'mission_timeline.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_steady_state(out_dir):
    fig, ax = plt.subplots(figsize=(8, 5))
    vr = np.linspace(0, 1.6, 200)
    for ta, ls in ((T_AMB, '-'), (T_AMB_HOT, '--')):
        ax.plot(vr, t_ss(vr, ta), 'k', ls=ls, lw=1.6,
                label=rf'$T_{{m,\infty}}(v)$ at $T_{{amb}}$ = {ta:.0f} °C')
        vs = v_safe_static(ta)
        ax.plot(vs, T_MAX, 'o', color='k', ms=7)
        ax.annotate(f'$v_{{safe}}$ = {vs:.3f} m/s', (vs, T_MAX), xytext=(5, 6),
                    textcoords='offset points', fontsize=8)
        vw = v_safe_static(ta, T_WARN)
        ax.plot(vw, T_WARN, 's', color='orange', ms=6)
        ax.annotate(f'{vw:.3f} m/s', (vw, T_WARN), xytext=(5, -12),
                    textcoords='offset points', fontsize=8)
    ax.axhline(T_MAX, color='k', ls='--', lw=1, label=r'$T_{max}$')
    ax.axhline(T_WARN, color='orange', ls=':', lw=1.4, label=r'$T_{warn}$')
    ax.axvline(V0, color='#D62728', ls='-.', lw=1, label=f'$v_0$ = {V0} m/s')
    ax.set_xlabel('Cruise speed v (m/s)')
    ax.set_ylabel('Steady-state motor temperature (°C)')
    ax.set_title('S076 Steady-State Temperature vs Speed')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc='upper left')
    path = os.path.join(out_dir, 'steady_state_curve.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics(res, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 4.8))
    x = np.arange(len(STRATEGIES))
    w = 0.38
    for j, case in enumerate(('card', 'hot')):
        o = [res[case][st][1] for st in STRATEGIES]
        hatch = '' if case == 'card' else '//'
        lbl = f'$T_{{amb}}$ = {T_AMB if case == "card" else T_AMB_HOT:.0f} °C'
        for ax, key, scale in zip(axes, ('mission_s', 'n_pauses', 'peak_T'), (1 / 60, 1, 1)):
            vals = [oo[key] * scale for oo in o]
            bars = ax.bar(x + (j - 0.5) * w, vals, w, color=[COLORS[s] for s in STRATEGIES],
                          hatch=hatch, edgecolor='k', label=lbl)
            for b, vv in zip(bars, vals):
                ax.text(b.get_x() + b.get_width() / 2, vv, f'{vv:.1f}' if key != 'n_pauses'
                        else f'{vv:d}', ha='center', va='bottom', fontsize=8)
    for ax, t in zip(axes, ('Mission time (min)', 'Cooling pauses', 'Peak motor temperature (°C)')):
        ax.set_xticks(x)
        ax.set_xticklabels([LABELS[s].replace(' (', '\n(') for s in STRATEGIES], fontsize=8)
        ax.set_title(t)
        ax.grid(alpha=0.3, axis='y')
    axes[2].axhline(T_MAX, color='k', ls='--', lw=1)
    axes[2].axhline(T_WARN, color='orange', ls=':', lw=1.4)
    axes[2].set_ylim(60, 92)
    axes[0].legend(fontsize=8, title='plain = card, hatched = hot')
    fig.suptitle('S076 Mission Metrics')
    plt.tight_layout()
    path = os.path.join(out_dir, 'metrics_bar.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_ambient_sweep(res, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 4.8))
    for st in STRATEGIES:
        o = res['sweep'][st]
        a1.plot(T_AMB_SWEEP, [oo['mission_s'] / 60 for oo in o], 'o-', color=COLORS[st],
                label=LABELS[st])
        a2.plot(T_AMB_SWEEP, [oo['peak_T'] for oo in o], 'o-', color=COLORS[st],
                label=LABELS[st])
    for a in (a1, a2):
        a.axvline(T_AMB, color='grey', ls=':', lw=1)
        a.axvline(T_AMB_HOT, color='grey', ls='--', lw=1)
        a.set_xlabel('Ambient temperature $T_{amb}$ (°C)')
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    a1.set_ylabel('Mission time (min)')
    a1.set_title('Mission time (dotted = card 45 °C, dashed = hot case)')
    a2.axhline(T_MAX, color='k', ls='--', lw=1)
    a2.axhline(T_WARN, color='orange', ls=':', lw=1.4)
    a2.set_ylabel('Peak motor temperature (°C)')
    a2.set_title('Peak temperature (dashed black = $T_{max}$)')
    fig.suptitle('S076 Ambient-Temperature Sweep')
    plt.tight_layout()
    path = os.path.join(out_dir, 'ambient_sweep.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, out_dir, case='hot'):
    logs = {st: res[case][st][0] for st in STRATEGIES}
    t_end = max(L['t'][-1] for L in logs.values())
    n = int(np.floor(t_end / DT)) + 1           # samples on a common DT grid
    step = int(round(ANIM_DT / DT))
    n_frames = int(np.ceil((n - 1) / step)) + 1
    t_grid = np.minimum(np.arange(n_frames) * step, n - 1) * DT

    fig = plt.figure(figsize=(15, 8))
    gs = fig.add_gridspec(2, 4, height_ratios=[1.3, 1])
    maps = [fig.add_subplot(gs[0, c]) for c in range(4)]
    axT = fig.add_subplot(gs[1, :])
    norm = plt.Normalize(T_AMB, 90)
    dots, trails, idx_of = {}, {}, {}
    for ax, st in zip(maps, STRATEGIES):
        L = logs[st]
        ax.add_patch(mpatches.Rectangle((0, 0), FIELD_H, FIELD_W, fc='wheat', alpha=0.3, ec='k'))
        ax.set_xlim(-5, FIELD_H + 5)
        ax.set_ylim(-5, FIELD_W + 5)
        ax.set_aspect('equal')
        ax.set_title(LABELS[st], color=COLORS[st], fontsize=10)
        trails[st] = ax.scatter([], [], c=[], cmap='hot_r', norm=norm, s=2)
        dots[st], = ax.plot([], [], 'o', color=COLORS[st], mec='k', ms=8)
        idx_of[st] = np.minimum(np.searchsorted(L['t'], t_grid, side='right') - 1, len(L['t']) - 1)
        axT.plot(L['t'] / 60, L['T'], color=COLORS[st], lw=0.6, alpha=0.25)
    lines = {st: axT.plot([], [], color=COLORS[st], lw=1.5, label=LABELS[st])[0]
             for st in STRATEGIES}
    axT.axhline(T_MAX, color='k', ls='--', lw=1)
    axT.axhline(T_WARN, color='orange', ls=':', lw=1.4)
    axT.set_xlim(0, t_end / 60)
    axT.set_ylim(T_AMB_HOT - 2, 92)
    axT.set_xlabel('Time (min)')
    axT.set_ylabel('Motor temperature (°C)')
    axT.legend(fontsize=8, loc='lower right')
    title = fig.suptitle('')

    def update(f):
        for st in STRATEGIES:
            L = logs[st]
            i = idx_of[st][f]
            sub = np.append(np.arange(0, i, ANIM_TRAIL_EVERY), i)   # thinned trail, ends at i
            trails[st].set_offsets(np.column_stack([L['x'][sub], L['y'][sub]]))
            trails[st].set_array(L['T'][sub])
            dots[st].set_data([L['x'][i]], [L['y'][i]])
            lines[st].set_data(L['t'][sub] / 60, L['T'][sub])
        title.set_text(f'S076 {_case_title(case)} — t = {t_grid[f] / 60:.1f} min')
        return []

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=15, dpi=60)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    res = run_simulation()
    elems, n_strips = build_path()
    o0 = res['card']['none'][1]
    print('=== S076 Extreme Heat Cooling Strategy — Results ===')
    print(f'  Strips: {n_strips}, path length {o0["path_m"]:.0f} m, turns '
          f'{sum(1 for e in elems if e[0] == "turn")} x {T_TURN:.0f} s')
    print(f'  P_motor(v0) = {p_motor(V0):.1f} W; T_ss(v0) at {T_AMB:.0f} degC = '
          f'{t_ss(V0, T_AMB):.1f} degC; T_ss(hover) = {t_ss(0.0, T_AMB):.1f} degC')
    print(f'  v_safe (T_ss = T_max) at {T_AMB:.0f} degC = {v_safe_static(T_AMB):.3f} m/s; '
          f'speed with T_ss = T_warn = {v_safe_static(T_AMB, T_WARN):.3f} m/s '
          f'(its square = {v_safe_static(T_AMB, T_WARN) ** 2:.3f})')
    print(f'  v_safe at {T_AMB_HOT:.0f} degC = {v_safe_static(T_AMB_HOT):.3f} m/s; '
          f'T_ss(v0) = {t_ss(V0, T_AMB_HOT):.1f} degC')
    for ta in (T_AMB, T_AMB_HOT):
        print(f'  At {ta:.0f} degC: T after a pause started at T_warn = '
              f'{ta + (T_WARN - ta) * np.exp(-T_PAUSE / TAU_COOL):.1f} degC; T_ss(hover) = '
              f'{t_ss(0.0, ta):.1f} degC; speed holding T_warn = {v_hold(T_WARN, ta):.3f} m/s')
    print(f'  Holding T_warn needs v = 0 once T_amb >= {T_WARN - R_TH * P_HOVER:.0f} degC')
    ta_cross = T_MAX - R_TH * p_motor(V0)
    print(f'  Unmanaged flight breaches T_max only for T_amb > {ta_cross:.1f} degC')
    vc = max(v_safe_card(t) for t in np.linspace(T_WARN, T_MAX, 51))
    print(f'  Card formula v_safe(T_m): max over T_m in [80, 85] = {vc:.3f} m/s '
          f'(zero for T_m >= {T_MAX - R_TH * P_HOVER:.0f} degC)')
    for case in ('card', 'hot'):
        print(f'  --- {case} case, T_amb = {res[case]["none"][1]["t_amb"]:.0f} degC ---')
        base = res[case]['none'][1]['mission_s']
        for st in STRATEGIES:
            o = res[case][st][1]
            print(f'  [{LABELS[st]}] mission {o["mission_s"]:.1f} s ({o["mission_s"] / 60:.2f} min), '
                  f'peak {o["peak_T"]:.2f} degC, pauses {o["n_pauses"]}, '
                  f'time > T_max {o["t_over"]:.1f} s')
            print(f'      cruise {o["t_cruise"]:.1f} s, turns {o["t_turn"]:.1f} s, '
                  f'pauses {o["t_pause"]:.1f} s, throttled {o["d_throttled"]:.1f} m in '
                  f'{o["t_throttled"]:.1f} s, penalty vs none {o["mission_s"] - base:+.1f} s')
    lo = res['card_literal'][1]
    print(f'  Reactive with the literal card formula (card case): mission '
          f'{lo["mission_s"]:.1f} s ({lo["mission_s"] / 60:.2f} min), peak {lo["peak_T"]:.2f} degC, '
          f'{lo["n_switch"]} speed switches, {lo["d_throttled"]:.1f} m at {V_MIN} m/s, '
          f'{lo["mission_s"] - res["card"]["reactive"][1]["mission_s"]:+.1f} s vs v_hold law')
    print('  Ambient sweep (T_amb: none / reactive-hold / reactive-Tmax / predictive mission min; '
          'peaks none / hold / Tmax; pauses hold / Tmax / pred):')
    for i, ta in enumerate(T_AMB_SWEEP):
        o = [res['sweep'][st][i] for st in STRATEGIES]
        print(f'    {ta:4.0f}: ' + ' / '.join(f'{oo["mission_s"] / 60:6.2f}' for oo in o) +
              f'   peaks {o[0]["peak_T"]:.1f} / {o[1]["peak_T"]:.2f} / {o[2]["peak_T"]:.2f}'
              f'   pauses {o[1]["n_pauses"]} / {o[2]["n_pauses"]} / {o[3]["n_pauses"]}')
    p85 = res['pred85_hot'][1]
    print(f'  Fairness check at {T_AMB_HOT:.0f} degC: predictive pauses at T_max instead of T_warn: '
          f'mission {p85["mission_s"] / 60:.2f} min, {p85["n_pauses"]} pauses, peak '
          f'{p85["peak_T"]:.2f} degC (throttle to T_max: '
          f'{res["hot"]["reactive_max"][1]["mission_s"] / 60:.2f} min)')
    for st in ('reactive', 'reactive_max'):
        rx = [res['sweep'][st][i]['mission_s'] - res['sweep']['predictive'][i]['mission_s']
              for i in range(len(T_AMB_SWEEP))]
        better = [f'{ta:.0f}' for ta, d in zip(T_AMB_SWEEP, rx) if d > 0]
        print(f'  Predictive faster than {LABELS[st]} at T_amb = '
              f'{", ".join(better) if better else "none"}')
    print(f'  Max peak of reactive (throttle to T_max) over the sweep: '
          f'{max(o["peak_T"] for o in res["sweep"]["reactive_max"]):.2f} degC; time above T_max '
          f'{max(o["t_over"] for o in res["sweep"]["reactive_max"]):.1f} s; pauses '
          f'{max(o["n_pauses"] for o in res["sweep"]["reactive_max"])}')

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_temperature_traces(res, out)
    plot_speed_profile(res, out)
    plot_flight_paths(res, out)
    plot_mission_timeline(res, out)
    plot_steady_state(out)
    plot_metrics(res, out)
    plot_ambient_sweep(res, out)
    save_animation(res, out)
