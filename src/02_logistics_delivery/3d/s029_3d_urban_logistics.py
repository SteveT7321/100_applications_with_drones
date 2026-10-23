"""
S029 3D Upgrade — Urban Logistics Scheduling
=============================================
Multi-depot drone delivery with zone altitudes: residential customers sit at
30 m, commercial at 60 m and rooftop at 100 m; depots are at 30 m.  A leg costs
the 3-D distance plus an altitude penalty gamma |dz| (card).  Each drone flies
one or more SORTIES; every sortie starts and ends at the drone's depot and must
respect the payload capacity Q_MAX and the energy budget E_MAX (asserted).

Strategies (all evaluated in true 3-D: 3-D flight times, 3-D energy):
  1. 2D Clarke-Wright   -- savings and feasibility on flat geometry (z ignored)
  2. 3D Clarke-Wright   -- savings and feasibility with the 3-D energy metric
  3. 3D TW insertion    -- time-window-aware cheapest insertion of the card's
                           objective  J = alpha TWT + (1 - alpha) E_3D  with a
                           relocate local search (VRPTW heuristic), run twice:
                           restricted to each customer's nearest depot (same
                           freedom as Clarke-Wright) and with any depot allowed

Two instances: the card's time windows (seed 42) and a tight-window variant
(same customers, windows from seed 2029) where the windows actually bind.

Outputs (outputs/02_logistics_delivery/3d/s029_3d_urban_logistics/):
  routes_3d.png, altitude_profiles.png, energy_comparison.png, animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s029_3d_urban_logistics.py
"""

import matplotlib
matplotlib.use('Agg')

import os
from itertools import combinations
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.lines import Line2D

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's029_3d_urban_logistics',
))

# ── Parameters (card) ─────────────────────────────────────────────
N_DEPOTS    = 3
N_CUSTOMERS = 12
N_DRONES    = 6           # 2 per depot
V_DRONE     = 15.0        # m/s 3-D cruise speed
Q_MAX       = 3.0         # kg per sortie
E_MAX       = 5000.0      # m-equivalent energy per sortie
SVC_TIME    = 10.0        # s per stop
ALPHA       = 0.7         # objective weight: alpha TWT + (1 - alpha) energy
GAMMA       = 1.5         # altitude penalty factor
DT          = 0.5         # s animation / profile time step
SEED        = 42          # card instance
SEED_TIGHT  = 2029        # tight time windows (same customers)

DEPOT_ALT   = 30.0
ZONE_ALTS   = np.array([30.0, 60.0, 100.0])
ZONE_NAMES  = ['Residential', 'Commercial', 'Rooftop']
ZONE_COLORS = ['cyan', 'orange', 'magenta']
DEPOT_XY    = np.array([[100., 100.], [500., 900.], [900., 200.]])
DEPOTS      = np.column_stack([DEPOT_XY, np.full(N_DEPOTS, DEPOT_ALT)])
DEPOT_OF_DRONE = [k // 2 for k in range(N_DRONES)]
DRONE_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#42d4f4']

# Card instance (same draw order as the original script)
_rng = np.random.default_rng(SEED)
CUST_XY = _rng.uniform(50, 950, (N_CUSTOMERS, 2))
ZONE    = _rng.integers(0, 3, N_CUSTOMERS)
EARLY   = _rng.uniform(0, 120, N_CUSTOMERS)
LATE    = EARLY + _rng.uniform(60, 180, N_CUSTOMERS)
DEMAND  = _rng.uniform(0.3, 1.5, N_CUSTOMERS)
WEIGHT  = np.ones(N_CUSTOMERS)            # card gives no weight distribution
CUST    = np.column_stack([CUST_XY, ZONE_ALTS[ZONE]])
_rng_t  = np.random.default_rng(SEED_TIGHT)
EARLY_T = _rng_t.uniform(0, 240, N_CUSTOMERS)
LATE_T  = EARLY_T + _rng_t.uniform(20, 60, N_CUSTOMERS)
INSTANCES = {'card': (EARLY, LATE), 'tight': (EARLY_T, LATE_T)}
STRATS = ['2D Clarke-Wright', '3D Clarke-Wright', '3D TW insertion (nearest depot)',
          '3D TW insertion']
NEAREST = np.argmin(np.linalg.norm(CUST_XY[:, None, :] - DEPOT_XY[None, :, :], axis=2), axis=1)


# ── Costs and feasibility ─────────────────────────────────────────

def energy_3d(a, b):
    return float(np.linalg.norm(b - a) + GAMMA * abs(b[2] - a[2]))


def energy_flat(a, b):
    return float(np.linalg.norm((b - a)[:2]))


def sortie_energy(sortie, dep, metric=energy_3d):
    pts = [DEPOTS[dep]] + [CUST[c] for c in sortie] + [DEPOTS[dep]]
    return sum(metric(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def feasible(sortie, dep, metric=energy_3d):
    return (sum(DEMAND[c] for c in sortie) <= Q_MAX + 1e-9
            and sortie_energy(sortie, dep, metric) <= E_MAX + 1e-9)


# ── Schedule (3-D timing, depot return between sorties) ───────────

def drone_schedule(plan, k, early, late):
    dep = DEPOTS[DEPOT_OF_DRONE[k]]
    t, energy, twt = 0.0, 0.0, 0.0
    events, deliv = [], {}
    for sortie in plan:
        pos = dep
        for c in sortie:
            arrive = t + np.linalg.norm(CUST[c] - pos) / V_DRONE
            events.append(('fly', t, arrive, pos, CUST[c]))
            start = max(arrive, early[c])
            if start > arrive:
                events.append(('wait', arrive, start, CUST[c], CUST[c]))
            events.append(('svc', start, start + SVC_TIME, CUST[c], CUST[c]))
            twt += WEIGHT[c] * max(0.0, start - late[c])
            deliv[c] = (arrive, start)
            energy += energy_3d(pos, CUST[c])
            t, pos = start + SVC_TIME, CUST[c]
        back = t + np.linalg.norm(dep - pos) / V_DRONE
        events.append(('fly', t, back, pos, dep))
        energy += energy_3d(pos, dep)
        t = back
    return dict(energy=energy, twt=twt, events=events, deliv=deliv, end=t,
                obj=ALPHA * twt + (1 - ALPHA) * energy)


def evaluate(plans, inst):
    early, late = INSTANCES[inst]
    sch = [drone_schedule(p, k, early, late) for k, p in enumerate(plans)]
    deliv = {}
    for s in sch:
        deliv.update(s['deliv'])
    sorties = [(k, s) for k, p in enumerate(plans) for s in p]
    loads = [sum(DEMAND[c] for c in s) for _, s in sorties]
    e3 = [sortie_energy(s, DEPOT_OF_DRONE[k]) for k, s in sorties]
    assert sorted(deliv) == list(range(N_CUSTOMERS)), 'every customer served once'
    extra = []                      # extra flight time of legs that change altitude
    for k, s in sorties:
        pts = [DEPOTS[DEPOT_OF_DRONE[k]]] + [CUST[c] for c in s] + [DEPOTS[DEPOT_OF_DRONE[k]]]
        for a, b in zip(pts[:-1], pts[1:]):
            if abs(b[2] - a[2]) > 1e-9:
                extra.append((np.linalg.norm(b - a) - np.linalg.norm((b - a)[:2])) / V_DRONE)
    assert max(loads) <= Q_MAX + 1e-9, f'capacity violated: {max(loads):.2f} kg'
    return dict(
        sched=sch, deliv=deliv, plans=plans,
        energy=sum(s['energy'] for s in sch), twt=sum(s['twt'] for s in sch),
        objective=sum(s['obj'] for s in sch), makespan=max(s['end'] for s in sch),
        n_sorties=len(sorties), n_drones=sum(1 for p in plans if p),
        n_late=sum(1 for c in deliv if deliv[c][1] > late[c] + 1e-9),
        max_load=max(loads), max_e3=max(e3), extra_t=extra,
        climb=sum(abs(b[2] - a[2]) for k, s in sorties
                  for a, b in zip(*(lambda pts: (pts[:-1], pts[1:]))(
                      [DEPOTS[DEPOT_OF_DRONE[k]]] + [CUST[c] for c in s]
                      + [DEPOTS[DEPOT_OF_DRONE[k]]]))))


# ── Strategies ────────────────────────────────────────────────────

def assign_sorties(sorties_by_depot, inst):
    """Give each depot's sorties, in order, to the depot drone free first."""
    early, late = INSTANCES[inst]
    plans = [[] for _ in range(N_DRONES)]
    for d, sorties in sorties_by_depot.items():
        drones = [k for k in range(N_DRONES) if DEPOT_OF_DRONE[k] == d]
        for s in sorties:
            k = min(drones, key=lambda kk: (drone_schedule(plans[kk], kk, early, late)['end'], kk))
            plans[k].append(s)
    return plans


def nearest_depot_groups():
    d = np.linalg.norm(CUST_XY[:, None, :] - DEPOT_XY[None, :, :], axis=2)
    nd = np.argmin(d, axis=1)
    return {dd: [c for c in range(N_CUSTOMERS) if nd[c] == dd] for dd in range(N_DEPOTS)}


def clarke_wright(inst, metric):
    sorties_by_depot = {}
    for d, custs in nearest_depot_groups().items():
        dep = DEPOTS[d]
        routes = [[c] for c in custs]
        sav = sorted(((metric(dep, CUST[i]) + metric(dep, CUST[j]) - metric(CUST[i], CUST[j]), i, j)
                      for i, j in combinations(custs, 2)), reverse=True)
        for _, i, j in sav:
            ri = next(r for r in routes if i in r)
            rj = next(r for r in routes if j in r)
            if ri is rj or i not in (ri[0], ri[-1]) or j not in (rj[0], rj[-1]):
                continue
            a = ri if ri[-1] == i else ri[::-1]
            b = rj if rj[0] == j else rj[::-1]
            if feasible(a + b, d, metric):
                routes.remove(ri); routes.remove(rj); routes.append(a + b)
        sorties_by_depot[d] = routes
    return assign_sorties(sorties_by_depot, inst)


def _best_insertion(plans, c, early, late, base, restrict=False):
    best = None
    for k in range(N_DRONES):
        dep = DEPOT_OF_DRONE[k]
        if restrict and dep != NEAREST[c]:
            continue
        cands = []
        for si, s in enumerate(plans[k]):
            for pos in range(len(s) + 1):
                ns = s[:pos] + [c] + s[pos:]
                if feasible(ns, dep):
                    cands.append(plans[k][:si] + [ns] + plans[k][si + 1:])
        for si in range(len(plans[k]) + 1):
            cands.append(plans[k][:si] + [[c]] + plans[k][si:])
        for p in cands:
            delta = drone_schedule(p, k, early, late)['obj'] - base[k]
            if best is None or delta < best[0] - 1e-9:
                best = (delta, k, p)
    return best


def tw_insertion(inst, restrict=False):
    early, late = INSTANCES[inst]
    plans = [[] for _ in range(N_DRONES)]
    base = [0.0] * N_DRONES
    for c in sorted(range(N_CUSTOMERS), key=lambda c: (late[c], c)):
        _, k, p = _best_insertion(plans, c, early, late, base, restrict)
        plans[k] = p
        base[k] = drone_schedule(p, k, early, late)['obj']
    history = [sum(base)]
    improved = True
    while improved:
        improved = False
        for c in range(N_CUSTOMERS):
            k0 = next(k for k in range(N_DRONES) if any(c in s for s in plans[k]))
            reduced = [s for s in ([x for x in s if x != c] for s in plans[k0]) if s]
            trial = [list(map(list, p)) for p in plans]
            trial[k0] = reduced
            tbase = list(base)
            tbase[k0] = drone_schedule(reduced, k0, early, late)['obj']
            _, k, p = _best_insertion(trial, c, early, late, tbase, restrict)
            new_k = drone_schedule(p, k, early, late)['obj']
            if sum(tbase) - tbase[k] + new_k < sum(base) - 1e-6:
                trial[k] = p
                tbase[k] = new_k
                plans, base = trial, tbase
                history.append(sum(base))
                improved = True
    return plans, history


# ── Driver ────────────────────────────────────────────────────────

def _fmt_plan(plans):
    return '; '.join(f'drone{k + 1}:' + '|'.join('-'.join(f'C{c + 1}' for c in s) for s in p)
                     for k, p in enumerate(plans) if p)


def run_simulation():
    print('S029 3D Urban Logistics Scheduling')
    print('Zones: ' + ', '.join(f'{ZONE_NAMES[z]}={int((ZONE == z).sum())}' for z in range(3)))
    for c in range(N_CUSTOMERS):
        print(f'  C{c + 1:<2d} {ZONE_NAMES[ZONE[c]]:<11s} z={CUST[c, 2]:5.0f} m  '
              f'q={DEMAND[c]:.2f} kg  card window [{EARLY[c]:6.1f}, {LATE[c]:6.1f}] s  '
              f'tight [{EARLY_T[c]:6.1f}, {LATE_T[c]:6.1f}] s')
    res, hist = {}, {}
    for inst in INSTANCES:
        res[inst] = {}
        res[inst]['2D Clarke-Wright'] = evaluate(clarke_wright(inst, energy_flat), inst)
        res[inst]['3D Clarke-Wright'] = evaluate(clarke_wright(inst, energy_3d), inst)
        plans, hn = tw_insertion(inst, restrict=True)
        res[inst]['3D TW insertion (nearest depot)'] = evaluate(plans, inst)
        plans, h = tw_insertion(inst)
        res[inst]['3D TW insertion'] = evaluate(plans, inst)
        hist[inst] = h
        early, late = INSTANCES[inst]
        print(f'\n=== {inst} time windows ===')
        for s in STRATS:
            r = res[inst][s]
            print(f'  {s:<31s} energy {r["energy"]:7.1f} m-eq  TWT {r["twt"]:6.1f} s '
                  f'({r["n_late"]} late)  J {r["objective"]:7.1f}  makespan {r["makespan"]:6.1f} s  '
                  f'sorties {r["n_sorties"]}/drones {r["n_drones"]}  max load {r["max_load"]:.2f} kg  '
                  f'max sortie E {r["max_e3"]:.1f}  climb+descent {r["climb"]:.0f} m')
            print(f'      plan: {_fmt_plan(r["plans"])}')
            off = [c for k, p in enumerate(r['plans']) for sx in p for c in sx
                   if DEPOT_OF_DRONE[k] != NEAREST[c]]
            print(f'      customers served from a non-nearest depot: '
                  f'{", ".join(f"C{c + 1}" for c in sorted(off)) if off else "none"}')
            ex = r['extra_t']
            print(f'      altitude-change legs: {len(ex)}, extra flight time vs flat '
                  f'{min(ex):.2f}-{max(ex):.2f} s per leg, {sum(ex):.2f} s in total')
            for c in sorted(r['deliv']):
                arr, st = r['deliv'][c]
                if st > late[c] + 1e-9:
                    print(f'      late C{c + 1} ({ZONE_NAMES[ZONE[c]]}): arrive {arr:.1f} s, '
                          f'window [{early[c]:.1f}, {late[c]:.1f}] s, late by {st - late[c]:.1f} s')
        b2, b3 = res[inst]['2D Clarke-Wright'], res[inst]['3D Clarke-Wright']
        de = (b3['energy'] - b2['energy']) / b2['energy'] * 100
        dt_ = b3['twt'] - b2['twt']
        rel = (f'{dt_ / b2["twt"] * 100:+.1f}%' if b2['twt'] > 1e-9
               else 'n/a, 2D TWT is 0')
        print(f'  3D vs 2D Clarke-Wright: energy {de:+.1f}%, TWT {dt_:+.1f} s ({rel})')
        for lab, hh in [('TW insertion (nearest depot)', hn), ('TW insertion (any depot)', h)]:
            if len(hh) > 1:
                print(f'  {lab} local search: J {hh[0]:.1f} -> {hh[-1]:.1f} in {len(hh) - 1} moves')
            else:
                print(f'  {lab} local search: no improving move (J {hh[0]:.1f})')
        rn, ra = res[inst]['3D TW insertion (nearest depot)'], res[inst]['3D TW insertion']
        print(f'  Effect of time-window awareness (nearest depot, vs 3D Clarke-Wright): '
              f'J {b3["objective"]:.1f} -> {rn["objective"]:.1f}, TWT {b3["twt"]:.1f} -> {rn["twt"]:.1f} s, '
              f'energy {b3["energy"]:.1f} -> {rn["energy"]:.1f}')
        print(f'  Effect of free depot choice (vs nearest-depot TW insertion): '
              f'J {rn["objective"]:.1f} -> {ra["objective"]:.1f}, TWT {rn["twt"]:.1f} -> {ra["twt"]:.1f} s, '
              f'energy {rn["energy"]:.1f} -> {ra["energy"]:.1f}')
    return res, hist


# ── Plots ─────────────────────────────────────────────────────────

def pos_at(sched, k, t):
    for ev in sched[k]['events']:
        kind, t0, t1, p0, p1 = ev
        if t0 - 1e-9 <= t <= t1 + 1e-9:
            if kind == 'fly' and t1 > t0:
                return p0 + (p1 - p0) * (t - t0) / (t1 - t0)
            return p0
    return DEPOTS[DEPOT_OF_DRONE[k]]


def _draw_map(ax, r, title):
    for d in range(N_DEPOTS):
        ax.scatter(*DEPOTS[d], color='black', marker='s', s=90)
        ax.text(DEPOTS[d, 0], DEPOTS[d, 1], DEPOTS[d, 2] + 6, f'Depot {d + 1}', fontsize=8)
    for c in range(N_CUSTOMERS):
        ax.scatter(*CUST[c], color=ZONE_COLORS[ZONE[c]], s=60, edgecolors='black', linewidths=0.5)
        ax.text(CUST[c, 0] + 15, CUST[c, 1], CUST[c, 2] + 3, f'C{c + 1}', fontsize=7)
        ax.plot([CUST[c, 0]] * 2, [CUST[c, 1]] * 2, [0, CUST[c, 2]], color='grey', lw=0.4, ls=':')
    for k, p in enumerate(r['plans']):
        dep = DEPOTS[DEPOT_OF_DRONE[k]]
        for si, s in enumerate(p):
            pts = np.array([dep] + [CUST[c] for c in s] + [dep])
            ax.plot(*pts.T, color=DRONE_COLORS[k], lw=1.6, ls='-' if si == 0 else '--')
    ax.set_xlim(0, 1000); ax.set_ylim(0, 1000); ax.set_zlim(0, 110)
    ax.set_xlabel('X (m)', fontsize=8); ax.set_ylabel('Y (m)', fontsize=8); ax.set_zlabel('Z (m)', fontsize=8)
    ax.set_title(title, fontsize=10)


def plot_3d_routes(res, out_dir):
    fig = plt.figure(figsize=(16, 7.5))
    for i, (inst, s) in enumerate([('card', '3D Clarke-Wright'), ('tight', '3D TW insertion (nearest depot)')]):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        r = res[inst][s]
        _draw_map(ax, r, f'{s}, {inst} windows: E = {r["energy"]:.0f} m-eq, TWT = {r["twt"]:.1f} s')
    handles = [Line2D([0], [0], marker='s', color='w', markerfacecolor='black', ms=9, label='Depot (30 m)')]
    handles += [Line2D([0], [0], marker='o', color='w', markerfacecolor=ZONE_COLORS[z], ms=8,
                       label=f'{ZONE_NAMES[z]} ({ZONE_ALTS[z]:.0f} m)') for z in range(3)]
    handles += [Line2D([0], [0], color=DRONE_COLORS[k], lw=2, label=f'Drone {k + 1} (depot {k // 2 + 1})')
                for k in range(N_DRONES)]
    handles += [Line2D([0], [0], color='k', lw=1.5, ls='--', label='2nd sortie of a drone')]
    fig.legend(handles=handles, loc='lower center', ncol=6, fontsize=8)
    fig.suptitle('S029 3D Urban Logistics — sorties (each starts and ends at its depot)', fontsize=12)
    plt.tight_layout(rect=[0, 0.08, 1, 0.95])
    path = os.path.join(out_dir, 'routes_3d.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_profiles(res, out_dir):
    cases = [('card', '3D Clarke-Wright'), ('tight', '3D TW insertion (nearest depot)')]
    fig, axes = plt.subplots(2, 1, figsize=(13, 8.5), sharex=True)
    for ax, (inst, s) in zip(axes, cases):
        r = res[inst][s]
        early, late = INSTANCES[inst]
        tg = np.arange(0.0, r['makespan'] + DT, DT)
        for k in range(N_DRONES):
            if not r['plans'][k]:
                continue
            z = [pos_at(r['sched'], k, t)[2] for t in tg]
            ax.plot(tg, z, color=DRONE_COLORS[k], lw=1.6, label=f'Drone {k + 1}')
            for c, (arr, st) in r['sched'][k]['deliv'].items():
                lateflag = st > late[c] + 1e-9
                ax.plot(st, CUST[c, 2], marker='X' if lateflag else 'o', ms=8 if lateflag else 5,
                        color='red' if lateflag else DRONE_COLORS[k], mec='black', mew=0.5)
                ax.text(st + 2, CUST[c, 2] + 2, f'C{c + 1}', fontsize=7)
        for z, col in zip(ZONE_ALTS, ZONE_COLORS):
            ax.axhline(z, color=col, ls='--', lw=0.8)
        ax.set_ylim(0, 115); ax.set_ylabel('Altitude (m)'); ax.grid(True, alpha=0.3)
        ax.set_title(f'{s}, {inst} windows (dots: service start; red X: late)')
        ax.legend(fontsize=7, ncol=6, loc='upper right')
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S029 3D Urban Logistics — altitude profiles per drone (with depot returns)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_profiles.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_energy_comparison(res, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    cols = ['steelblue', 'coral', 'khaki', 'seagreen']
    x = np.arange(len(INSTANCES))
    w = 0.2
    for ax, key, lab in zip(axes, ['energy', 'twt', 'objective'],
                            ['Total 3-D energy (m-equiv)', 'Total weighted tardiness (s)',
                             f'Objective J = {ALPHA} TWT + {1 - ALPHA:.1f} E']):
        for i, s in enumerate(STRATS):
            vals = [res[inst][s][key] for inst in INSTANCES]
            bars = ax.bar(x + (i - 1.5) * w, vals, w, color=cols[i], edgecolor='black', lw=0.6, label=s)
            for b, v in zip(bars, vals):
                ax.text(b.get_x() + b.get_width() / 2, b.get_height(), f'{v:.0f}' if key != 'twt' else f'{v:.1f}',
                        ha='center', va='bottom', fontsize=7)
        ax.set_xticks(x); ax.set_xticklabels([f'{inst} windows' for inst in INSTANCES])
        ax.set_title(lab, fontsize=10); ax.grid(True, axis='y', alpha=0.3)
    axes[0].legend(fontsize=8, loc='lower right')
    fig.suptitle('S029 3D Urban Logistics — 2D vs 3D Clarke-Wright vs time-window-aware insertion '
                 '(all evaluated in 3-D)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'energy_comparison.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def save_animation(res, out_dir, n_frames=110):
    r = res['card']['3D Clarke-Wright']
    tg = np.linspace(0.0, r['makespan'], n_frames)
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    for d in range(N_DEPOTS):
        ax.scatter(*DEPOTS[d], color='black', marker='s', s=90)
    for c in range(N_CUSTOMERS):
        ax.scatter(*CUST[c], color=ZONE_COLORS[ZONE[c]], s=50, edgecolors='black', linewidths=0.5)
    ax.set_xlim(0, 1000); ax.set_ylim(0, 1000); ax.set_zlim(0, 110)
    ax.set_xlabel('X (m)', fontsize=8); ax.set_ylabel('Y (m)', fontsize=8); ax.set_zlabel('Z (m)', fontsize=8)
    active = [k for k in range(N_DRONES) if r['plans'][k]]
    paths = {k: np.array([pos_at(r['sched'], k, t) for t in tg]) for k in active}
    trails = {k: ax.plot([], [], [], color=DRONE_COLORS[k], lw=1.3)[0] for k in active}
    dots = {k: ax.plot([], [], [], '^', color=DRONE_COLORS[k], ms=8)[0] for k in active}
    title = ax.set_title('')
    ax.legend(handles=[Line2D([0], [0], color=DRONE_COLORS[k], lw=2, label=f'Drone {k + 1}') for k in active],
              fontsize=8, loc='upper left')

    def update(f):
        for k in active:
            p = paths[k][:f + 1]
            trails[k].set_data(p[:, 0], p[:, 1]); trails[k].set_3d_properties(p[:, 2])
            dots[k].set_data([p[-1, 0]], [p[-1, 1]]); dots[k].set_3d_properties([p[-1, 2]])
        title.set_text(f'S029 3D — 3D Clarke-Wright, card windows, t = {tg[f]:.1f} s')
        return list(trails.values()) + list(dots.values()) + [title]

    ani = FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=12), dpi=80)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    res, hist = run_simulation()
    out_dir = OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    print()
    plot_3d_routes(res, out_dir)
    plot_altitude_profiles(res, out_dir)
    plot_energy_comparison(res, out_dir)
    save_animation(res, out_dir)
    print('Done.')
