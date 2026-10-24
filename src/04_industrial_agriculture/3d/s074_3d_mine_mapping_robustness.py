"""
S074 3D Mine Mapping — held-out robustness set of the exploration planner
=========================================================================
Companion of s074_3d_mine_mapping.py. Re-runs the true-pose '3d' mission of the main
script (same code) under settings the planner was not tuned against:
  - the eleven settings of the round-2 review (fresh seeds, layout D, constant changes),
  - ten fresh noise seeds with default constants (each also with the altitude-held
    '3d_fixed' ablation as the fair comparison),
  - the two alternative layouts B and C (used while the planner was developed),
  - every planner / traverse / safety-filter constant that can change the path, one at a
    time at two values (where a review setting already covers one side, one new value).
The run list is split into parts of about five 3d missions (each part well under 7 min):
    python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py h01
    ...
    python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py h15
    python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py summary
'list' prints the parts. Each part writes its lines to
outputs/04_industrial_agriculture/3d/s074_3d_mine_mapping/robustness_<part>.txt; 'summary'
reads them all, prints the per-run table with totals and writes robustness_summary.txt.
A run that trips a safety assertion (exact wall clearance < 0.25 m, altitude bounds, speed)
is recorded as an abort.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py <part>
"""

import os
import re
import sys
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import s074_3d_mine_mapping as M  # noqa: E402

OUT_DIR = M.OUTPUT_DIR
REACHED = 0.5                    # a level counts as reached when half of its free voxels are mapped
PART_COST = 5.0                  # 3d missions per part (a 3d_fixed mission counts 0.2)

REVIEW = [('review: seed 2026', dict(seed=2026)), ('review: seed 99', dict(seed=99)),
          ('review: layout D', dict(layout='D')),
          ('review: SHAFT_DZ = 2', dict(overrides={'SHAFT_DZ': 2.0})),
          ('review: LAT_DZ = 0.5, seed 7', dict(seed=7, overrides={'LAT_DZ': 0.5})),
          ('review: CARROT = 0.9', dict(overrides={'CARROT': 0.9})),
          ('review: GOAL_CELLS = 3', dict(overrides={'GOAL_CELLS': 3})),
          ('review: GUARD_DZ = 0.4', dict(overrides={'GUARD_DZ': 0.4})),
          ('review: SOFT_COST = 2', dict(overrides={'SOFT_COST': 2.0})),
          ('review: ALIGN_TOL = 0.25', dict(overrides={'ALIGN_TOL': 0.25})),
          ('review: REJECT_R = 0.2, seed 43', dict(seed=43, overrides={'REJECT_R': 0.2}))]
FRESH_SEEDS = list(range(100, 110))
LAYOUTS = ['B', 'C']
CONSTANTS = {                    # default -> held-out values (one-at-a-time, seed 42)
    'LAT_DZ': (0.45, 0.75), 'SHAFT_DZ': (1.7,), 'SHAFT_PAD': (0.5, 0.7), 'SHAFT_ZPAD': (2.5, 3.5),
    'REJECT_R': (0.25, 0.4), 'REJECT_EXCL': (0.8, 1.25), 'MAX_VISITS': (2, 4), 'MIN_FRONTIER': (6, 10),
    'GOAL_CELLS': (1,), 'SOFT_COST': (8.0,), 'CARROT': (0.4,), 'REPLAN_EVERY': (20, 45),
    'IGNORE_R': (0.8, 1.2), 'STOP_DIST': (0.4, 0.6), 'VISIT_GAP': (10, 25), 'CLR_VOX': (0, 2),
    'ALIGN_TOL': (0.1,), 'ALIGN_MAX': (0.4, 0.8), 'GUARD_DZ': (0.1,), 'CLIMB_VXY': (0.2, 0.4),
    'CLIMB_HTOL': (0.05, 0.15), 'TRAV_TOL': (0.3, 0.7), 'TRAV_STOP': (0.05, 0.2), 'CENTRE_WIN': (1.2, 2.0),
    'T_TRAV_MAX': (90.0, 180.0), 'SAFE_R': (0.6, 0.8), 'K_REPEL': (0.5, 2.0)}


def layout_boxes(name):
    HW, SR = M.HALF_W, M.SHAFT_R
    L1, L2, L3 = M.Z_LEVELS
    if name == 'B':              # shafts at x = 40 and 70, Level 3 cross drift toward -x
        boxes = [((0.0, -HW, L1 - 1.5), (80.0, HW, L1 + 1.5)),
                 ((50.0 - HW, -20.0, L1 - 1.5), (50.0 + HW, 0.0, L1 + 1.5)),
                 ((40.0 - HW, -HW, L2 - 1.5), (71.5, HW, L2 + 1.5)),
                 ((55.0 - HW, 0.0, L2 - 1.5), (55.0 + HW, 20.0, L2 + 1.5)),
                 ((70.0 - HW, -HW, L3 - 1.5), (70.0 + HW, 20.0 + HW, L3 + 1.5)),
                 ((50.0, 20.0 - HW, L3 - 1.5), (70.0, 20.0 + HW, L3 + 1.5)),
                 ((40.0 - SR, -SR, L1), (40.0 + SR, SR, L2)),
                 ((70.0 - SR, -SR, L2), (70.0 + SR, SR, L3))]
    elif name == 'C':            # shaft 1 at x = 20, Level 2 branch toward -y
        boxes = [((0.0, -HW, L1 - 1.5), (80.0, HW, L1 + 1.5)),
                 ((50.0 - HW, -20.0, L1 - 1.5), (50.0 + HW, 0.0, L1 + 1.5)),
                 ((20.0 - HW, -HW, L2 - 1.5), (61.5, HW, L2 + 1.5)),
                 ((35.0 - HW, -20.0, L2 - 1.5), (35.0 + HW, 0.0, L2 + 1.5)),
                 ((60.0 - HW, -HW, L3 - 1.5), (60.0 + HW, 20.0 + HW, L3 + 1.5)),
                 ((60.0, 20.0 - HW, L3 - 1.5), (80.0, 20.0 + HW, L3 + 1.5)),
                 ((20.0 - SR, -SR, L1), (20.0 + SR, SR, L2)),
                 ((60.0 - SR, -SR, L2), (60.0 + SR, SR, L3))]
    else:                        # 'D' (review): shafts at x = 25 and 65, Level 3 cross drift toward -x
        boxes = [((0.0, -HW, L1 - 1.5), (80.0, HW, L1 + 1.5)),
                 ((50.0 - HW, -20.0, L1 - 1.5), (50.0 + HW, 0.0, L1 + 1.5)),
                 ((25.0 - HW, -HW, L2 - 1.5), (66.5, HW, L2 + 1.5)),
                 ((40.0 - HW, 0.0, L2 - 1.5), (40.0 + HW, 20.0, L2 + 1.5)),
                 ((65.0 - HW, -HW, L3 - 1.5), (65.0 + HW, 20.0 + HW, L3 + 1.5)),
                 ((45.0, 20.0 - HW, L3 - 1.5), (65.0, 20.0 + HW, L3 + 1.5)),
                 ((25.0 - SR, -SR, L1), (25.0 + SR, SR, L2)),
                 ((65.0 - SR, -SR, L2), (65.0 + SR, SR, L3))]
    return np.array([b[0] for b in boxes]), np.array([b[1] for b in boxes])


def all_runs():
    runs = list(REVIEW)
    for s in FRESH_SEEDS:
        runs.append((f'fresh seed {s}', dict(seed=s)))
        runs.append((f'fresh seed {s}', dict(seed=s, mode='3d_fixed')))
    runs += [(f'layout {lay} (development)', dict(layout=lay)) for lay in LAYOUTS]
    for k, vals in CONSTANTS.items():
        for v in vals:
            runs.append((f'{k} = {v:g} (default {getattr(M, k):g})', dict(overrides={k: v})))
    return runs


def parts():
    out, cur, cost = [], [], 0.0
    for r in all_runs():
        c = 0.2 if r[1].get('mode') == '3d_fixed' else 1.0
        if cur and cost + c > PART_COST + 1e-9:
            out.append(cur)
            cur, cost = [], 0.0
        cur.append(r)
        cost += c
    out.append(cur)
    return {f'h{i + 1:02d}': p for i, p in enumerate(out)}


def run(name, mode='3d', seed=M.SEED, layout=None, overrides=None):
    saved = {k: getattr(M, k) for k in (overrides or {})}
    blo, bhi = M.BLO, M.BHI
    try:
        for k, v in (overrides or {}).items():
            setattr(M, k, v)
        if layout is not None:
            M.BLO, M.BHI = layout_boxes(layout)
        M.GT = M.ground_truth()
        try:
            m = M.mission(mode, seed=seed)
        except AssertionError as e:
            return f'{name:<34} {mode:<8} ABORT   ({e})'
        c = m['cov_log'][-1][1]
        lv = sum(c[f'L{i}'] > REACHED for i in (1, 2, 3))
        return (f"{name:<34} {mode:<8} levels {lv}/3  xi {100 * c['xi']:6.2f}%  L1/L2/L3 {100 * c['L1']:5.2f}/"
                f"{100 * c['L2']:5.2f}/{100 * c['L3']:5.2f}%  T {m['t']:6.1f} s  min clearance {m['min_clear']:.3f} m  "
                f"traverses {m['n_traverse']} (given up {m['n_trav_fail']})  {m['status']}")
    finally:
        for k, v in saved.items():
            setattr(M, k, v)
        M.BLO, M.BHI = blo, bhi
        M.GT = None


def summary():
    ps = parts()
    lines = []
    for p in ps:
        with open(os.path.join(OUT_DIR, f'robustness_{p}.txt'), encoding='utf-8') as f:
            lines += [l.rstrip('\n') for l in f if not l.startswith('===')]
    out = ['=== S074 3D held-out robustness set: per-run table (true pose; seed 42 unless named) ===']
    out += ['  ' + l for l in lines]
    l3 = [l for l in lines if re.search(r' 3d +(levels|ABORT)', l)]
    lf = [l for l in lines if ' 3d_fixed ' in l]

    def tally(group, title):
        n = len(group)
        ab = [l for l in group if 'ABORT' in l]
        ok = [l for l in group if 'ABORT' not in l]
        lv = [int(re.search(r'levels (\d)/3', l).group(1)) for l in ok]
        xs = np.array([float(re.search(r'xi +([\d.]+)%', l).group(1)) for l in ok])
        cl = np.array([float(re.search(r'min clearance ([\d.]+) m', l).group(1)) for l in ok])
        out.append(f'  {title}: {n} runs; all three levels {sum(v == 3 for v in lv)}/{n}, two levels '
                   f'{sum(v == 2 for v in lv)}/{n}, one level {sum(v == 1 for v in lv)}/{n}, aborted {len(ab)}/{n}')
        if len(xs):
            x3 = np.array([x for x, v in zip(xs, lv) if v == 3])
            out.append(f'    xi over completed runs: median {np.median(xs):.2f}%, min {xs.min():.2f}%, max {xs.max():.2f}%'
                       + (f'; runs with all three levels {x3.min():.2f}-{x3.max():.2f}%' if len(x3) else '')
                       + f'; min wall clearance over completed runs {cl.min():.3f} m')

    out.append('  [Totals, 3d missions]')
    tally([l for l in l3 if l.startswith('review:')], 'review settings')
    tally([l for l in l3 if l.startswith('fresh seed')], 'fresh default seeds')
    tally([l for l in l3 if l.startswith('layout')], 'development layouts')
    tally([l for l in l3 if re.match(r'[A-Z_]+ = ', l)], 'planner constants')
    tally(l3, 'all')
    for l in l3:
        if 'ABORT' in l or 'levels 3/3' not in l:
            out.append('    failed: ' + re.sub(r' {2,}', '  ', l.strip()))
    if lf:
        fx = [float(re.search(r'xi +([\d.]+)%', l).group(1)) for l in lf if 'levels' in l]
        out.append(f'  3d_fixed (altitude held) on the fresh seeds: {len(lf)} runs, xi {min(fx):.2f}-{max(fx):.2f}%, '
                   f'levels {sorted(set(re.search(r"levels (.)/3", l).group(1) for l in lf if "levels" in l))}')
    for l in out:
        print(l)
    with open(os.path.join(OUT_DIR, 'robustness_summary.txt'), 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(out) + '\n')


if __name__ == '__main__':
    part = sys.argv[1] if len(sys.argv) > 1 else 'list'
    os.makedirs(OUT_DIR, exist_ok=True)
    ps = parts()
    if part == 'list':
        for p, rs in ps.items():
            print(p, '; '.join(f"{n} [{kw.get('mode', '3d')}]" for n, kw in rs))
        sys.exit(0)
    if part == 'summary':
        summary()
        sys.exit(0)
    head = f'=== S074 3D held-out robustness, part {part} ==='
    lines = [head]
    print(head, flush=True)
    for name, kw in ps[part]:
        t0 = time.time()
        line = run(name, **kw)
        lines.append(line)
        print(line, flush=True)
        print(f'    [{time.time() - t0:.0f} s wall]', file=sys.stderr, flush=True)
    with open(os.path.join(OUT_DIR, f'robustness_{part}.txt'), 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
