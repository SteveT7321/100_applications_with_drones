"""
S074 3D Mine Mapping — robustness of the exploration planner
============================================================
Companion of s074_3d_mine_mapping.py. Re-runs the true-pose '3d' mission of the main
script (same code, same settings) under
  part 'seeds'     : other noise seeds and two other mine layouts (plus the altitude-held
                     '3d_fixed' ablation for each, as the fair comparison)
  part 'planner_a' : one-at-a-time changes of four planner constants (low / high value)
  part 'planner_b' : one-at-a-time changes of four more planner constants
  part 'summary'   : reads the three result files and prints the distribution
Each part writes its printed output to
outputs/04_industrial_agriculture/3d/s074_3d_mine_mapping/robustness_<part>.txt.
A run that trips a safety assertion (e.g. wall clearance) is recorded as an abort.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py seeds
    python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py planner_a
    python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py planner_b
    python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py summary
"""

import os
import re
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import s074_3d_mine_mapping as M  # noqa: E402

OUT_DIR = M.OUTPUT_DIR
SEEDS = [43, 44, 7, 1000]
LAYOUTS = ['B', 'C']
PLANNER_A = {'LAT_DZ': (0.4, 0.8), 'SHAFT_DZ': (1.6, 2.2), 'SHAFT_PAD': (0.4, 0.8),
             'SHAFT_ZPAD': (2.0, 4.0)}
PLANNER_B = {'REJECT_R': (0.2, 0.5), 'REJECT_EXCL': (0.6, 1.5), 'MAX_VISITS': (2, 5),
             'MIN_FRONTIER': (4, 12)}
REACHED = 0.5                    # a level counts as reached when half of its free voxels are mapped


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
    else:                        # 'C': shaft 1 at x = 20, Level 2 branch toward -y
        boxes = [((0.0, -HW, L1 - 1.5), (80.0, HW, L1 + 1.5)),
                 ((50.0 - HW, -20.0, L1 - 1.5), (50.0 + HW, 0.0, L1 + 1.5)),
                 ((20.0 - HW, -HW, L2 - 1.5), (61.5, HW, L2 + 1.5)),
                 ((35.0 - HW, -20.0, L2 - 1.5), (35.0 + HW, 0.0, L2 + 1.5)),
                 ((60.0 - HW, -HW, L3 - 1.5), (60.0 + HW, 20.0 + HW, L3 + 1.5)),
                 ((60.0, 20.0 - HW, L3 - 1.5), (80.0, 20.0 + HW, L3 + 1.5)),
                 ((20.0 - SR, -SR, L1), (20.0 + SR, SR, L2)),
                 ((60.0 - SR, -SR, L2), (60.0 + SR, SR, L3))]
    return np.array([b[0] for b in boxes]), np.array([b[1] for b in boxes])


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
            return f'{name:<26} {mode:<8} ABORT   ({e})'
        c = m['cov_log'][-1][1]
        lv = sum(c[f'L{i}'] > REACHED for i in (1, 2, 3))
        return (f"{name:<26} {mode:<8} levels {lv}/3  xi {100 * c['xi']:6.2f}%  L1/L2/L3 {100 * c['L1']:5.2f}/"
                f"{100 * c['L2']:5.2f}/{100 * c['L3']:5.2f}%  T {m['t']:6.1f} s  min clearance {m['min_clear']:.2f} m  "
                f"{m['status']}")
    finally:
        for k, v in saved.items():
            setattr(M, k, v)
        M.BLO, M.BHI = blo, bhi
        M.GT = None


def part_runs(part):
    if part == 'seeds':
        out = []
        for s in SEEDS:
            out.append((f'seed {s}', dict(seed=s)))
        for lay in LAYOUTS:
            out.append((f'layout {lay}', dict(layout=lay)))
        out += [(n + ' [fixed]', dict(kw, mode='3d_fixed')) for n, kw in list(out)]
        return out
    table = PLANNER_A if part == 'planner_a' else PLANNER_B
    return [(f'{k} = {v:g} (default {getattr(M, k):g})', dict(overrides={k: v}))
            for k, vals in table.items() for v in vals]


def summary():
    lines = []
    for part in ('seeds', 'planner_a', 'planner_b'):
        with open(os.path.join(OUT_DIR, f'robustness_{part}.txt'), encoding='utf-8') as f:
            lines += [l.rstrip('\n') for l in f if re.search(r' 3d +(levels|ABORT)', l)]
    out = ['=== S074 3D robustness summary (3d missions, true pose; the seed-42 default run is in the main script) ===']
    xs, lv, ab = [], [], 0
    for l in lines:
        if 'ABORT' in l:
            ab += 1
            continue
        lv.append(int(re.search(r'levels (\d)/3', l).group(1)))
        xs.append(float(re.search(r'xi +([\d.]+)%', l).group(1)))
    n = len(lines)
    xs = np.array(xs)
    out.append(f'  runs {n} (seeds/layouts and one-at-a-time planner changes): all three levels in '
               f'{sum(v == 3 for v in lv)}/{n}, two levels in {sum(v == 2 for v in lv)}/{n}, one level in '
               f'{sum(v == 1 for v in lv)}/{n}, aborted {ab}/{n}')
    if len(xs):
        x3 = np.array([x for x, v in zip(xs, lv) if v == 3])
        out.append(f'  xi over completed runs: median {np.median(xs):.2f}%, min {xs.min():.2f}%, max {xs.max():.2f}%; '
                   f'runs reaching all three levels: {x3.min():.2f}-{x3.max():.2f}%')
    for l in lines:
        if 'ABORT' in l or 'levels 3/3' not in l:
            out.append('  not all levels: ' + l.strip())
    with open(os.path.join(OUT_DIR, 'robustness_seeds.txt'), encoding='utf-8') as f:
        fx = [float(re.search(r'xi +([\d.]+)%', l).group(1)) for l in f if '3d_fixed' in l and 'levels' in l]
    if fx:
        out.append(f'  3d_fixed (altitude held) on the same seeds and layouts: xi {min(fx):.2f}-{max(fx):.2f}%')
    for l in out:
        print(l)
    with open(os.path.join(OUT_DIR, 'robustness_summary.txt'), 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(out) + '\n')


if __name__ == '__main__':
    part = sys.argv[1] if len(sys.argv) > 1 else 'seeds'
    os.makedirs(OUT_DIR, exist_ok=True)
    if part == 'summary':
        summary()
        sys.exit(0)
    head = f'=== S074 3D robustness, part {part} (base: seed {M.SEED}, default layout) ==='
    lines = [head]
    print(head, flush=True)
    for name, kw in part_runs(part):
        line = run(name, **kw)
        lines.append(line)
        print(line, flush=True)
    with open(os.path.join(OUT_DIR, f'robustness_{part}.txt'), 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
