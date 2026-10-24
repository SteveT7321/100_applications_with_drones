"""
S064 3D Upgrade — Greenhouse Interior Navigation
================================================
The 2D S064 base flies a serpentine through five aisles of a GPS-denied greenhouse at a
fixed z = 1.0 m; the plant rows stop 2 m short of each gable wall so that the drone can
change aisle sideways in the headlands.  Its lateral position comes from dead reckoning
fused with two side-looking ultrasonic sensors (complementary filter, alpha = 0.85).

The 3D variant makes altitude a planned state:
  * every plant row is split into 2 m canopy segments of height h ~ U(1.0, 1.6) m and
    carries two trellis wires (z = 0.6 and 1.2 m) along its centre line;
  * the ultrasonic beams only return from a row where the drone is BELOW the local canopy
    top (above it the beam passes over the plants), so the aisle altitude decides whether
    the lateral fix exists;
  * the altitude is no longer known: barometric/IMU dead reckoning (sigma 0.008 m/s) is
    fused with a downward ToF sensor (beta = 0.75, innovation gate 0.3 m);
  * in the "full-length rows" greenhouse the rows run to the gable walls (no headlands)
    and the drone changes aisle with the card's climb - translate - descend manoeuvre.

Compared on the same random numbers (30 seeds, common random numbers):
  2D base (headlands)  : base layout and base strategy, z held at 1.0 m, sideways turns
  3D crossing z=1.0    : full-length rows, aisles at 1.0 m, over-row crossing at 2.0 m
  3D card altitude rule: full-length rows, the card's per-aisle cruise altitude formula
plus the base strategy flown in the full-length greenhouse (it must collide), and
sweeps of every added or changed parameter a conclusion rests on.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/3d/s064_3d_greenhouse.py
"""

import os
import math
import numpy as np
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
sys.path.insert(0, os.path.join(SCRIPT_DIR, '..'))
import s064_greenhouse as base_s064                     # noqa: E402  (2D base, equivalence check only)

OUTPUT_DIR = os.path.join(PROJECT_ROOT, 'outputs', '04_industrial_agriculture', '3d',
                          's064_3d_greenhouse')

# ── Parameters ────────────────────────────────────────────────────────────────
GH_X, GH_Y, GH_Z = 40.0, 10.0, 3.0
ROW_Y = [2.0, 4.0, 6.0, 8.0, 10.0]
ROW_THICK = 0.2
H_LO, H_HI = 1.0, 1.6              # canopy height range (card)
SEG_LEN = 2.0                      # m  canopy segment length (added; swept)
WIRE_Z = (0.6, 1.2)                # trellis wire heights (card)
WIRE_R = 0.01                      # wire radius (card)
WIRE_X = (1.0, 39.0)               # wires absent at x < 1 and x > 39 (card)
HALF = (0.15, 0.15, 0.075)         # drone half-extents of the 0.3 x 0.3 x 0.15 m body
LAYOUT_SPAN = {'headland': (2.0, 38.0), 'full': (0.0, 40.0)}
HEAD_W = 2.0                       # base headland width (rows x = 2-38 m); swept 2.0 / 1.5 / 1.0 m
HEAD_SWEEP = [2.0, 1.5, 1.375, 1.25, 1.2, 1.125, 1.0]
HEAD_SEEDS = list(range(100))       # safety verdict of the headland sweep (seeds 0-99)
HEAD_EXTRA = [1.5, 1.375, 1.25]     # widths re-checked on fresh seeds 100-299 (rare-event check)
HEAD_SEEDS_EXTRA = list(range(100, 300))
TURN_PROBES = [(1.0, 0.4), (1.0, 0.45), (1.5, 1.0)]   # (headland width, turn line x) geometry probes

V_FWD = 1.0
V_LAT_MAX = 1.0                    # base: lateral speed limit of the tracking law in aisles
V_LAT_TURN = 0.5                   # card: lateral speed during aisle changes (both layouts)
V_MAX = 3.0                        # base: airframe speed limit incl. repulsion
D_SAFE = 0.25
D_WP = 0.5
SIGMA_XY = 0.02                    # m/s/sqrt(s)
SIGMA_Z = 0.008                    # m/s/sqrt(s) (card)
SIGMA_RANGE = 0.05
SIGMA_TOF = 0.03                   # card
RANGE_MIN, RANGE_MAX = 0.1, 2.0
TOF_MAX = 4.0
GATE = 0.5                         # base: lateral fix gate
GATE_Z = 0.3                       # added: ToF innovation gate (swept)
ALPHA = 0.85
BETA = 0.75                        # card
KP_LAT = 0.8
K_ALT = 1.5                        # card (base used 1.2); used by every strategy
KP_X_TURN = 0.8
K_REP = 1.5
Z_AISLE = 1.0                      # base cruise altitude, kept for the 3D aisles
Z_CROSS = 2.0                      # card 1.9 corrected (see README)
Z_CAPTURE = 0.05                   # card: climb done when z_hat > z_cross - 0.05
Y_CAPTURE = 0.1                    # card: translate done within 0.1 m
RHO_CARD = 0.19                    # card: r_wire + r_drone used by the card altitude rule
DT = 0.05
T_MAX = 300.0
X_END = [1.0, 39.0]
START = (1.0, 1.0, 0.4)            # card (0, 1, 0.4); x = 0 is the gable wall

AISLE_Y = [(0.0 + ROW_Y[0] - ROW_THICK / 2) / 2] + \
          [(ROW_Y[k] + ROW_Y[k + 1]) / 2 for k in range(4)]   # 0.95, 3, 5, 7, 9
N_AISLE = len(AISLE_Y)

N_SEEDS = 30
SEEDS = list(range(N_SEEDS))
SWEEP_SEEDS = list(range(10))
PLOT_SEED = 0
ANIM_STEP = 25
GEO_BASE = 10_000                  # geometry stream: default_rng(GEO_BASE + seed)

MAIN = [  # key, label, layout, turn mode, aisle altitude rule
    ('2d', '2D base (headlands, z=1.0)', 'headland', 'side', 'fixed'),
    ('3d', '3D crossing, aisles z=1.0', 'full', 'over', 'fixed'),
    ('card', '3D card altitude rule', 'full', 'over', 'card'),
]
COLORS = {'2d': '#888888', '3d': '#1F77B4', 'card': '#E87722'}


# ── Geometry ──────────────────────────────────────────────────────────────────

def canopy(seed, seg_len=SEG_LEN):
    """Canopy height of every row segment, (5, n_seg); same draws for both layouts."""
    n_seg = int(round(GH_X / seg_len))
    rng = np.random.default_rng(GEO_BASE + seed)
    return rng.uniform(H_LO, H_HI, (len(ROW_Y), n_seg))


class World:
    def __init__(self, heights, layout, head_w=None):
        self.H = heights
        self.n_seg = heights.shape[1]
        self.seg_len = GH_X / self.n_seg
        self.layout = layout
        self.x0, self.x1 = LAYOUT_SPAN[layout]
        if layout == 'headland' and head_w is not None:
            self.x0, self.x1 = head_w, GH_X - head_w
        self.wx0, self.wx1 = (max(WIRE_X[0], self.x0), min(WIRE_X[1], self.x1))

    def present(self, x):
        return self.x0 <= x <= self.x1

    def h_at(self, ri, x):
        s = min(max(int(x // self.seg_len), 0), self.n_seg - 1)
        return self.H[ri, s]

    def faces_map(self, x):
        """Map faces (+y facing, -y facing) known to the filter (no canopy heights)."""
        left, right = [0.0], [GH_Y]
        if self.present(x):
            left += [r + ROW_THICK / 2 for r in ROW_Y]
            right += [r - ROW_THICK / 2 for r in ROW_Y]
        return left, right

    def ranges(self, x, y, z):
        """True centre-to-face ranges; a row returns an echo only where z < canopy top."""
        dl = y - 0.0
        dr = GH_Y - y
        if self.present(x):
            for ri, r in enumerate(ROW_Y):
                if self.h_at(ri, x) <= z:
                    continue
                f_up, f_dn = r + ROW_THICK / 2, r - ROW_THICK / 2
                if f_up <= y + 1e-12:
                    dl = min(dl, y - f_up)
                if f_dn >= y - 1e-12:
                    dr = min(dr, f_dn - y)
        return dl, dr

    def top_below(self, x, y, z):
        """Height of the highest surface under the drone centre (ToF target)."""
        top = 0.0
        if self.present(x):
            for ri, r in enumerate(ROW_Y):
                if abs(y - r) <= ROW_THICK / 2:
                    h = self.h_at(ri, x)
                    if h < z:
                        top = max(top, h)
                    if abs(y - r) <= WIRE_R and self.wx0 <= x <= self.wx1:
                        for zw in WIRE_Z:
                            if zw + WIRE_R < z:
                                top = max(top, zw + WIRE_R)
        return top

    def clearance_base(self, x, y):
        """The 2D S064 base's clearance: nearest wall or row box in the plane, minus the 0.15 m
        half-size (a rounded-corner measure; no floor, no canopy height)."""
        best = min(x, GH_X - x, y, GH_Y - y)
        for r in ROW_Y:
            dx = max(self.x0 - x, 0.0, x - self.x1)
            dy = max(r - ROW_THICK / 2 - y, 0.0, y - r - ROW_THICK / 2)
            best = min(best, math.hypot(dx, dy))
        return best - HALF[0]

    def clearance(self, x, y, z):
        """3D edge clearance (box-box distance, negative = penetration) to
        walls/floor/ceiling, row segments and trellis wires."""
        hx, hy, hz = HALF
        c_wall = min(x - hx, GH_X - x - hx, y - hy, GH_Y - y - hy, z - hz, GH_Z - z - hz)
        c_row = c_wire = 9.0
        for ri, r in enumerate(ROW_Y):
            if abs(y - r) > 1.6:
                continue
            ylo, yhi = r - ROW_THICK / 2, r + ROW_THICK / 2
            xa, xb = max(self.x0, x - 1.6), min(self.x1, x + 1.6)
            if xa < xb:
                s0 = int(xa // self.seg_len)
                s1 = min(int(xb // self.seg_len), self.n_seg - 1)
                for s in range(s0, s1 + 1):
                    lo = (max(s * self.seg_len, self.x0), ylo, 0.0)
                    hi = (min((s + 1) * self.seg_len, self.x1), yhi, self.H[ri, s])
                    c_row = min(c_row, _box_dist((x, y, z), lo, hi))
            if self.wx0 <= x + 1.6 and x - 1.6 <= self.wx1:
                for zw in WIRE_Z:
                    lo = (self.wx0, r - WIRE_R, zw - WIRE_R)
                    hi = (self.wx1, r + WIRE_R, zw + WIRE_R)
                    c_wire = min(c_wire, _box_dist((x, y, z), lo, hi))
        return min(c_wall, c_row, c_wire), c_row, c_wire, c_wall


def _box_dist(c, lo, hi):
    gaps, over = [], []
    for ci, hi_d, l, h in zip(c, HALF, lo, hi):
        g = max(l - (ci + hi_d), (ci - hi_d) - h, 0.0)
        gaps.append(g)
        over.append(min(ci + hi_d, h) - max(ci - hi_d, l))
    if gaps[0] == 0.0 and gaps[1] == 0.0 and gaps[2] == 0.0:
        return -min(over)
    return math.sqrt(gaps[0] ** 2 + gaps[1] ** 2 + gaps[2] ** 2)


def aisle_altitudes(world, rule, z_fixed=Z_AISLE):
    if rule == 'fixed':
        return [z_fixed] * N_AISLE
    zs = []
    for k in range(N_AISLE):
        rows = [0] if k == 0 else [k - 1, k]          # aisle 0 is bounded by the wall
        hmin = min(float(world.H[ri].min()) for ri in rows)
        z_max = hmin - D_SAFE                         # card: min(h_k, h_k+1) - d_safe
        zs.append((WIRE_Z[1] + RHO_CARD + z_max) / 2) # card midpoint rule
    return zs


# ── Simulation ────────────────────────────────────────────────────────────────

def noise(seed, n):
    """Per-seed streams; the first three draws are the base S064 noise()."""
    rng = np.random.default_rng(seed)
    eta_true = rng.normal(0, SIGMA_XY * math.sqrt(DT), (n, 3))
    eta_est = rng.normal(0, SIGMA_XY * math.sqrt(DT), n)
    rng_noise = rng.normal(0, SIGMA_RANGE, (n, 2))
    eta_true[:, 2] *= SIGMA_Z / SIGMA_XY                       # vertical drift (card)
    eta_est_z = rng.normal(0, SIGMA_Z * math.sqrt(DT), n)
    tof_noise = rng.normal(0, SIGMA_TOF, n)
    return eta_true, eta_est, rng_noise, eta_est_z, tof_noise


def simulate(seed, layout='full', turn='over', alt_rule='fixed', z_aisle=Z_AISLE,
             z_cross=Z_CROSS, beta=BETA, gate_z=GATE_Z, v_lat_turn=V_LAT_TURN,
             seg_len=SEG_LEN, head_w=HEAD_W, k_alt=K_ALT, start=START, x_turn=None, keep_log=False):
    n = int(round(T_MAX / DT))
    eta_t, eta_e, rn, eta_ez, tofn = noise(seed, n)
    world = World(canopy(seed, seg_len), layout, head_w)
    za = aisle_altitudes(world, alt_rule, z_aisle)
    x_end = [head_w / 2, GH_X - head_w / 2] if layout == 'headland' else X_END
    if x_turn is not None:
        x_end = [x_turn, GH_X - x_turn]
    c_base_min = 9.0
    viol_base = 0
    x, y, z = start
    y_hat, z_hat = y, z
    y_pure, z_pure = y, z          # parallel pure dead-reckoning integrators (logging only)
    k_aisle, mode = 0, 'leg'
    lat_err, alt_err, clr, in_turn = [], [], [], []
    c_row_min = c_wire_min = c_wall_min = 9.0
    viol = 0
    n_fix = n_tof = 0
    crash = complete = False
    v_peak = 0.0
    turn_steps = 0
    peak_lat_turn = None
    reach = []
    lg = {k: [] for k in ('t', 'x', 'y', 'z', 'yhat', 'zhat', 'zdr', 'ztof', 'ywall', 'ydr',
                          'top', 'clr', 'mode')} if keep_log else None
    step = 0
    for step in range(n):
        t = step * DT
        yc = AISLE_Y[k_aisle]
        xf = x_end[1] if k_aisle % 2 == 0 else x_end[0]
        y_next = AISLE_Y[k_aisle + 1] if k_aisle + 1 < N_AISLE else yc
        # ── sensing (true state) ──
        dl_t, dr_t = world.ranges(x, y, z)
        zl, zr = dl_t + rn[step, 0], dr_t + rn[step, 1]
        zl = zl if RANGE_MIN <= zl <= RANGE_MAX else None
        zr = zr if RANGE_MIN <= zr <= RANGE_MAX else None
        top = world.top_below(x, y, z)
        tof = z - top + tofn[step]
        tof = tof if 0.02 <= tof <= TOF_MAX else None
        # ── commands (x known as in the base; y and z from the estimates) ──
        if mode == 'leg':
            dx = xf - x
            vx = math.copysign(V_FWD, dx) if abs(dx) > D_WP else KP_X_TURN * dx
            vy = max(-V_LAT_MAX, min(V_LAT_MAX, -KP_LAT * (y_hat - yc)))
            z_tgt = za[k_aisle]
        else:
            vx = KP_X_TURN * (xf - x)
            if mode in ('side', 'translate'):
                vy = max(-v_lat_turn, min(v_lat_turn, -KP_LAT * (y_hat - y_next)))
            elif mode == 'climb':
                vy = max(-V_LAT_MAX, min(V_LAT_MAX, -KP_LAT * (y_hat - yc)))
            else:                                   # descend
                vy = max(-V_LAT_MAX, min(V_LAT_MAX, -KP_LAT * (y_hat - y_next)))
            z_tgt = {'side': za[k_aisle], 'climb': z_cross, 'translate': z_cross,
                     'descend': za[min(k_aisle + 1, N_AISLE - 1)]}[mode]
            turn_steps += 1
        vz = k_alt * (z_tgt - z_hat)
        # repulsion from measured ranges (lateral, base) and the ToF clearance (vertical)
        if zl is not None and zl - HALF[1] < D_SAFE:
            vy += K_REP * (D_SAFE - (zl - HALF[1])) / D_SAFE
        if zr is not None and zr - HALF[1] < D_SAFE:
            vy -= K_REP * (D_SAFE - (zr - HALF[1])) / D_SAFE
        if tof is not None and tof - HALF[2] < D_SAFE:
            vz += K_REP * (D_SAFE - (tof - HALF[2])) / D_SAFE
        sp = math.sqrt(vx * vx + vy * vy + vz * vz)
        if sp > V_MAX:
            vx, vy, vz = vx * V_MAX / sp, vy * V_MAX / sp, vz * V_MAX / sp
            sp = V_MAX
        assert sp <= V_MAX + 1e-9, 'commanded speed above V_MAX'
        v_peak = max(v_peak, sp)
        # ── truth propagation ──
        x += vx * DT + eta_t[step, 0]
        y += vy * DT + eta_t[step, 1]
        z += vz * DT + eta_t[step, 2]
        assert HALF[2] < z < GH_Z - HALF[2], f'drone outside floor/ceiling: z = {z:.3f}'
        # ── lateral estimator (base) ──
        y_dr = y_hat + vy * DT + eta_e[step]
        y_pure += vy * DT + eta_e[step]
        y_wall = None
        left, right = world.faces_map(x)
        cands = []
        if zl is not None:
            lf = max((f for f in left if f <= y_dr + 1e-9), default=None)
            if lf is not None:
                cands.append(lf + zl)
        if zr is not None:
            rf = min((f for f in right if f >= y_dr - 1e-9), default=None)
            if rf is not None:
                cands.append(rf - zr)
        cands = [c for c in cands if abs(c - y_dr) < GATE]
        if cands:
            y_wall = sum(cands) / len(cands)
            n_fix += 1
        y_hat = ALPHA * y_dr + (1 - ALPHA) * y_wall if y_wall is not None else y_dr
        # ── altitude estimator: baro/IMU DR + downward ToF (floor assumed) ──
        z_dr = z_hat + vz * DT + eta_ez[step]
        z_pure += vz * DT + eta_ez[step]
        z_tof = None
        if tof is not None and abs(tof - z_dr) < gate_z and beta < 1.0:
            z_tof = tof
            n_tof += 1
        z_hat = beta * z_dr + (1 - beta) * z_tof if z_tof is not None else z_dr

        c, c_row, c_wire, c_wall = world.clearance(x, y, z)
        c_wall_min = min(c_wall_min, c_wall)
        cb = world.clearance_base(x, y)
        c_base_min = min(c_base_min, cb)
        viol_base += cb < D_SAFE
        c_row_min = min(c_row_min, c_row)
        c_wire_min = min(c_wire_min, c_wire)
        lat_err.append(abs(y_hat - y))
        in_turn.append(mode != 'leg')
        alt_err.append(abs(z_hat - z))
        clr.append(c)
        if c < D_SAFE:
            viol += 1
        if keep_log:
            for key, val in (('t', t + DT), ('x', x), ('y', y), ('z', z), ('yhat', y_hat),
                             ('zhat', z_hat), ('zdr', z_pure),
                             ('ztof', np.nan if z_tof is None else z_tof),
                             ('ywall', np.nan if y_wall is None else y_wall), ('ydr', y_pure),
                             ('top', top), ('clr', c), ('mode', mode)):
                lg[key].append(val)
        if c <= 0.0:
            crash = True
            break
        # ── mission logic ──
        if mode == 'leg':
            if math.hypot(x - xf, y_hat - yc) < D_WP:
                reach.append(t + DT)
                if k_aisle == N_AISLE - 1:
                    complete = True
                    break
                mode = 'side' if turn == 'side' else 'climb'
        elif mode == 'side':
            if math.hypot(x - xf, y_hat - y_next) < D_WP:
                k_aisle += 1
                mode = 'leg'
        elif mode == 'climb':
            if z_hat > z_cross - Z_CAPTURE:
                mode = 'translate'
        elif mode == 'translate':
            if abs(y_hat - y_next) < Y_CAPTURE:
                mode = 'descend'
        elif mode == 'descend':
            if abs(z_hat - za[k_aisle + 1]) < Z_CAPTURE:
                k_aisle += 1
                mode = 'leg'
    lat_err, alt_err = np.array(lat_err), np.array(alt_err)
    nst = step + 1
    res = {
        'complete': complete, 'crash': crash, 'time': nst * DT, 'row_ends': len(reach),
        'reach': reach, 'mean_lat': float(lat_err.mean()), 'max_lat': float(lat_err.max()),
        'mean_alt': float(alt_err.mean()), 'max_alt': float(alt_err.max()),
        'mean_3d': float(np.mean(np.hypot(lat_err, alt_err))),
        'min_clear': float(np.min(clr)), 'min_row': c_row_min, 'min_wire': c_wire_min, 'min_wall': c_wall_min, 'min_base': c_base_min, 'viol_base': viol_base,
        'viol': viol, 'fix_frac': n_fix / nst, 'tof_frac': n_tof / nst,
        'turn_time': turn_steps * DT, 'v_peak': v_peak, 'z_aisle': za,
        'peak_in_turn': bool(in_turn[int(np.argmax(lat_err))]),
        'turn_frac': float(np.mean(in_turn)),
    }
    if keep_log:
        for key in lg:
            lg[key] = np.array(lg[key])
        res['log'] = lg
        res['world'] = world
    return res


def _job(args):
    seed, kw = args
    if kw.get('__base__'):                         # the 2D S064 base itself (equivalence check)
        r = base_s064.simulate('fusion', seed)
        return {'mean_lat': r['mean_err'], 'max_lat': r['max_err'], 'time': r['time'],
                'complete': r['complete']}
    return simulate(seed, **kw)


SWEEPS = {  # name -> {value: simulate() keyword arguments}; nominal values reuse main runs
    'z_aisle': {v: dict(z_aisle=v) for v in (0.5, 0.75, 1.0, 1.25, 1.5)},
    'z_cross': {v: dict(z_cross=v) for v in (1.9, 1.95, 2.0, 2.1, 2.2)},
    'beta': {v: dict(beta=v) for v in (0.5, 0.75, 0.9, 1.0)},
    'gate_z': {v: dict(gate_z=v) for v in (0.15, 0.3, 1.5)},
    'v_turn': {(lay, v): dict(layout=lay, turn='side' if lay == 'headland' else 'over',
                              v_lat_turn=v) for lay in ('headland', 'full') for v in (0.5, 1.0)},
    'seg': {(sl, rule): dict(alt_rule=rule, seg_len=sl)
            for sl in (1.0, 2.0, 4.0, 40.0) for rule in ('fixed', 'card')},
}
NOMINAL = {'z_aisle': Z_AISLE, 'z_cross': Z_CROSS, 'beta': BETA, 'gate_z': GATE_Z,
           'v_turn': None, 'seg': None}


def run_simulation(workers=6):
    """All runs are independent and deterministic; results are collected in job order."""
    from concurrent.futures import ProcessPoolExecutor
    jobs, slots = [], []
    for key, _, layout, turn, rule in MAIN:
        for sd in SEEDS:
            if sd != PLOT_SEED:
                jobs.append((sd, dict(layout=layout, turn=turn, alt_rule=rule)))
                slots.append(('main', key, sd))
    for sd in SWEEP_SEEDS:
        jobs.append((sd, dict(layout='full', turn='side')))
        slots.append(('main', '2d_full', sd))
    for name, cfgs in SWEEPS.items():
        for val, kw in cfgs.items():
            if val == NOMINAL[name] or val in (('headland', V_LAT_TURN), ('full', V_LAT_TURN),
                                               (SEG_LEN, 'fixed'), (SEG_LEN, 'card')):
                continue                       # identical to a main configuration
            for sd in SWEEP_SEEDS:
                jobs.append((sd, kw))
                slots.append((name, val, sd))
    for hw in HEAD_SWEEP:                          # headland width of the 2D arm (100 seeds each)
        for sd in HEAD_SEEDS:
            if hw == HEAD_W and sd in SEEDS:
                continue                           # identical to the main '2d' runs
            jobs.append((sd, dict(layout='headland', turn='side', head_w=hw)))
            slots.append(('head', hw, sd))
    for hw in HEAD_EXTRA:
        for sd in HEAD_SEEDS_EXTRA:
            jobs.append((sd, dict(layout='headland', turn='side', head_w=hw)))
            slots.append(('headx', hw, sd))
    for hw, xt in TURN_PROBES:                     # turn-line geometry probes (30 seeds)
        for sd in SEEDS:
            jobs.append((sd, dict(layout='headland', turn='side', head_w=hw, x_turn=xt)))
            slots.append(('turnp', (hw, xt), sd))
    for sd in SEEDS:                               # 2D arm with the base's K_alt, start and turn speed
        jobs.append((sd, dict(layout='headland', turn='side', k_alt=1.2, start=(1.0, 1.0, 1.0),
                              v_lat_turn=1.0)))
        slots.append(('equiv', 'arm', sd))
        jobs.append((sd, {'__base__': True}))
        slots.append(('equiv', 'base', sd))
    with ProcessPoolExecutor(max_workers=workers) as ex:
        results = list(ex.map(_job, jobs, chunksize=4))
    out = {'main': {k: {} for k in ('2d', '3d', 'card', '2d_full')}, 'sweep': {}, 'head': {},
           'equiv': {}, 'turnp': {}, 'headx': {}}
    for key, _, layout, turn, rule in MAIN:      # plotted seed with logs, in this process
        out['main'][key][PLOT_SEED] = simulate(PLOT_SEED, layout, turn, rule, keep_log=True)
    for (a, b, sd), r in zip(slots, results):
        if a == 'main':
            out['main'][b][sd] = r
        elif a in ('head', 'equiv', 'turnp', 'headx'):
            out[a].setdefault(b, {})[sd] = r
        else:
            out['sweep'].setdefault(a, {}).setdefault(b, {})[sd] = r
    for key in out['main']:
        d = out['main'][key]
        out['main'][key] = [d[sd] for sd in sorted(d)]
    for sd in SEEDS:
        out['head'].setdefault(HEAD_W, {})[sd] = out['main']['2d'][sd]
    for grp in ('head', 'equiv', 'turnp', 'headx'):
        for key in out[grp]:
            out[grp][key] = [out[grp][key][sd] for sd in sorted(out[grp][key])]
    alias = {'z_aisle': '3d', 'z_cross': '3d', 'beta': '3d', 'gate_z': '3d'}
    sw = out['sweep']
    for name in SWEEPS:
        sw.setdefault(name, {})
        for val in list(SWEEPS[name]):
            if val in sw[name]:
                sw[name][val] = [sw[name][val][sd] for sd in SWEEP_SEEDS]
            else:
                src = alias.get(name)
                if name == 'v_turn':
                    src = '2d' if val[0] == 'headland' else '3d'
                if name == 'seg':
                    src = '3d' if val[1] == 'fixed' else 'card'
                sw[name][val] = out['main'][src][:len(SWEEP_SEEDS)]
    return out


# ── Plotting ──────────────────────────────────────────────────────────────────

def _draw_rows(ax, world, alpha=0.22):
    for ri, r in enumerate(ROW_Y):
        for s in range(world.n_seg):
            xa = max(s * world.seg_len, world.x0)
            xb = min((s + 1) * world.seg_len, world.x1)
            if xa >= xb:
                continue
            h = world.H[ri, s]
            yy = r - ROW_THICK / 2
            verts = [[(xa, yy, 0), (xb, yy, 0), (xb, yy, h), (xa, yy, h)],
                     [(xa, yy, h), (xb, yy, h), (xb, yy + ROW_THICK, h), (xa, yy + ROW_THICK, h)]]
            ax.add_collection3d(Poly3DCollection(verts, alpha=alpha, facecolor='grey',
                                                 edgecolor='none'))
        for zw in WIRE_Z:
            ax.plot([world.wx0, world.wx1], [r, r], [zw, zw], color='k', lw=0.5, alpha=0.6)


def plot_trajectory(out, out_dir):
    fig = plt.figure(figsize=(17, 10))
    for i, key in enumerate(('3d', '2d')):
        r = out['main'][key][PLOT_SEED]
        lg, world = r['log'], r['world']
        ax = fig.add_subplot(2, 1, i + 1, projection='3d')
        _draw_rows(ax, world)
        ax.plot(lg['x'], lg['y'], lg['z'], color='tab:blue', lw=1.2, label='true path')
        ax.plot(lg['x'], lg['yhat'], lg['zhat'], color='red', lw=0.8, ls='--',
                label='estimated path')
        ax.scatter(*START, color='green', s=50, label='start')
        ax.scatter(lg['x'][-1], lg['y'][-1], lg['z'][-1], color='red', marker='x', s=60,
                   label='end')
        ax.set_xlim(0, GH_X)
        ax.set_ylim(0, GH_Y)
        ax.set_zlim(0, GH_Z)
        ax.set_box_aspect((4, 1.3, 0.75), zoom=1.15)
        ax.set_zticks([0, 1, 2, 3])
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.view_init(elev=24, azim=-62)
        title = ('3D: full-length rows, climb–translate–descend crossings at z = 2.0 m'
                 if key == '3d' else '2D base: rows x = 2–38 m, sideways headland turns at z = 1.0 m')
        ax.set_title(f'{title} (seed {PLOT_SEED}; grey = canopy segments, black = trellis wires)',
                     fontsize=10)
        ax.legend(loc='upper left', fontsize=7)
    fig.suptitle('S064 3D Greenhouse — true and estimated 3D paths')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _turn_spans(lg):
    spans, cur = [], None
    for t, m in zip(lg['t'], lg['mode']):
        if m != 'leg' and cur is None:
            cur = t
        if m == 'leg' and cur is not None:
            spans.append((cur, t))
            cur = None
    if cur is not None:
        spans.append((cur, lg['t'][-1]))
    return spans


def plot_altitude(out, out_dir):
    r = out['main']['3d'][PLOT_SEED]
    lg = r['log']
    fig, ax = plt.subplots(figsize=(14, 5))
    for a, b in _turn_spans(lg):
        ax.axvspan(a, b, color='gold', alpha=0.25, lw=0)
    for zw in WIRE_Z:
        ax.axhline(zw, color='k', lw=0.6, ls=':')
    ax.plot(lg['t'], lg['top'], color='green', lw=1.0, label='surface under the drone (floor / canopy top / wire)')
    ax.plot(lg['t'], lg['top'] + HALF[2] + D_SAFE, color='green', lw=0.8, ls='--',
            label='surface + body half-height + d_safe')
    ax.plot(lg['t'], lg['z'], color='tab:blue', lw=1.2, label='true z')
    ax.plot(lg['t'], lg['zhat'], color='red', lw=0.8, ls='--', label='estimated ẑ')
    m = ~np.isnan(lg['ztof'])
    ax.plot(lg['t'][m], lg['ztof'][m], '.', color='c', ms=1.0, label='accepted ToF fixes')
    ax.axhline(Z_CROSS, color='purple', lw=0.8, ls='--', label=f'z_cross = {Z_CROSS} m')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('z (m)')
    ax.set_ylim(0, 2.6)
    ax.set_title(f'S064 3D altitude, seed {PLOT_SEED} (gold = aisle changes; dotted = trellis wire '
                 f'heights 0.6 / 1.2 m; the green spikes are the 0.2 m wide rows passing under the drone)')
    ax.legend(loc='upper right', fontsize=8, ncol=2)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_analysis(out, out_dir):
    r = out['main']['3d'][PLOT_SEED]
    lg = r['log']
    spans = _turn_spans(lg)
    fig, axes = plt.subplots(3, 2, figsize=(15, 11))

    def shade(ax):
        for a, b in spans:
            ax.axvspan(a, b, color='gold', alpha=0.25, lw=0)

    ax = axes[0, 0]
    shade(ax)
    ax.plot(lg['t'], lg['y'], color='tab:blue', lw=1.0, label='true y')
    ax.plot(lg['t'], lg['yhat'], color='red', lw=0.8, ls='--', label='ŷ')
    for rw in ROW_Y:
        ax.axhline(rw, color='grey', lw=0.6)
    ax.set_ylabel('y (m)')
    ax.set_title('(a) Lateral position (grey = row centre lines)')
    ax.legend(fontsize=8)
    ax = axes[0, 1]
    shade(ax)
    ax.plot(lg['t'], lg['z'], color='tab:blue', lw=1.0, label='true z')
    ax.plot(lg['t'], lg['zhat'], color='red', lw=0.8, ls='--', label='ẑ')
    for zw in WIRE_Z:
        ax.axhline(zw, color='k', lw=0.6, ls=':')
    ax.set_ylabel('z (m)')
    ax.set_title('(b) Altitude (dotted = trellis wire heights)')
    ax.legend(fontsize=8)
    ax = axes[1, 0]
    shade(ax)
    ax.plot(lg['t'], lg['ywall'] - lg['y'], '.', color='c', ms=1.0, label='wall-derived − true')
    ax.plot(lg['t'], lg['ydr'] - lg['y'], color='orange', lw=1.0,
            label='pure dead reckoning − true (same flight)')
    ax.plot(lg['t'], lg['yhat'] - lg['y'], color='red', lw=0.8, label='fused − true')
    ax.set_ylim(-0.5, 0.5)
    ax.set_ylabel('error (m)')
    ax.set_title('(c) Lateral filter components')
    ax.legend(fontsize=8)
    ax = axes[1, 1]
    shade(ax)
    ax.plot(lg['t'], lg['ztof'] - lg['z'], '.', color='c', ms=1.0, label='accepted ToF − true')
    ax.plot(lg['t'], lg['zdr'] - lg['z'], color='orange', lw=1.0,
            label='pure baro/IMU dead reckoning − true')
    ax.plot(lg['t'], lg['zhat'] - lg['z'], color='red', lw=0.8, label='fused − true')
    ax.set_ylim(-0.4, 0.4)
    ax.set_ylabel('error (m)')
    ax.set_title('(d) Altitude filter components')
    ax.legend(fontsize=8)
    ax = axes[2, 0]
    shade(ax)
    ax.plot(lg['t'], lg['clr'], color='tab:blue', lw=0.8, label='3D edge clearance (all obstacles)')
    ax.axhline(D_SAFE, color='k', ls='--', lw=1, label=f'd_safe = {D_SAFE} m')
    ax.set_ylim(0, 1.0)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('clearance (m)')
    ax.set_title('(e) 3D clearance to rows, wires, walls and floor')
    ax.legend(fontsize=8)
    ax = axes[2, 1]
    shade(ax)
    el = np.abs(lg['yhat'] - lg['y'])
    ea = np.abs(lg['zhat'] - lg['z'])
    win = int(round(2.0 / DT))                        # 2 s moving average for legibility
    ker = np.ones(win) / win
    for e, c, lab in ((np.hypot(el, ea), 'purple', 'E_3D'), (el, 'tab:blue', 'E_lat'),
                      (ea, 'green', 'E_alt')):
        ax.plot(lg['t'], np.convolve(e, ker, mode='same'), color=c, lw=1.1, label=f'{lab} (2 s mean)')
    ax.set_ylim(1e-3, 0.5)
    ax.set_yscale('log')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('error (m, log)')
    ax.set_title('(f) Estimation errors')
    ax.legend(fontsize=8)
    for a in axes.flat:
        a.grid(alpha=0.3)
    fig.suptitle(f'S064 3D analysis — full-length rows, aisles at z = 1.0 m, seed {PLOT_SEED} '
                 f'(gold = aisle changes)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'analysis.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_comparison(out, out_dir):
    fig, axes = plt.subplots(2, 3, figsize=(17, 9))
    keys = [k for k, *_ in MAIN]
    labs = [lab for _, lab, *_ in MAIN]
    for ax, (f, lab, logy) in zip(axes[0], (('mean_lat', 'mean |ŷ − y| (m)', True),
                                            ('max_lat', 'peak |ŷ − y| (m)', True),
                                            ('time', 'mission time (s)', False))):
        data = [[r[f] for r in out['main'][k]] for k in keys]
        bp = ax.boxplot(data, patch_artist=True, widths=0.6)
        for patch, k in zip(bp['boxes'], keys):
            patch.set_facecolor(COLORS[k])
            patch.set_alpha(0.7)
        ax.set_xticks([1, 2, 3])
        ax.set_xticklabels([lb.replace(' (', '\n(').replace(', ', ',\n') for lb in labs],
                           fontsize=8)
        if logy:
            ax.set_yscale('log')
        ax.set_title(f'{lab}, {N_SEEDS} seeds')
        ax.grid(alpha=0.3)
    sw = out['sweep']
    ax = axes[1, 0]
    zs = sorted(sw['z_aisle'])
    ax.plot(zs, [100 * np.mean([r['fix_frac'] for r in sw['z_aisle'][v]]) for v in zs], 'o-',
            color='tab:blue', label='steps with a lateral fix (%)')
    ax.set_xlabel('aisle altitude (m)')
    ax.set_ylabel('lateral fix (%)', color='tab:blue')
    ax2 = ax.twinx()
    ax2.plot(zs, [np.mean([r['mean_lat'] for r in sw['z_aisle'][v]]) for v in zs], 's--',
             color='red', label='mean |ŷ − y|')
    ax2.set_yscale('log')
    ax2.set_ylabel('mean |ŷ − y| (m)', color='red')
    ax.axvspan(H_LO, H_HI, color='green', alpha=0.1)
    ax.set_title('Aisle altitude sweep (green = canopy height range)')
    ax.grid(alpha=0.3)
    ax = axes[1, 1]
    zc = sorted(sw['z_cross'])
    ax.plot(zc, [np.min([r['min_row'] for r in sw['z_cross'][v]]) for v in zc], 'o-',
            color='tab:blue', label='worst clearance to the rows')
    ax.axhline(D_SAFE, color='k', ls='--', lw=1, label='d_safe')
    ax.set_xlabel('crossing altitude z_cross (m)')
    ax.set_ylabel('clearance (m)')
    ax2 = ax.twinx()
    ax2.plot(zc, [np.mean([r['time'] for r in sw['z_cross'][v]]) for v in zc], 's--',
             color='grey', label='mission time')
    ax2.set_ylabel('mission time (s)', color='grey')
    ax.legend(fontsize=8, loc='lower right')
    ax.set_title('Crossing altitude sweep (card 1.9 m)')
    ax.grid(alpha=0.3)
    ax = axes[1, 2]
    bs = sorted(sw['beta'])
    ax.plot(bs, [np.mean([r['mean_alt'] for r in sw['beta'][v]]) for v in bs], 'o-',
            color='green', label='mean |ẑ − z|')
    ax.plot(bs, [np.mean([r['max_alt'] for r in sw['beta'][v]]) for v in bs], 's--',
            color='green', label='peak |ẑ − z|')
    ax.set_yscale('log')
    ax.set_xlabel('altitude filter β (1.0 = no ToF)')
    ax.set_ylabel('altitude error (m)')
    ax.set_title(f'Altitude filter sweep ({len(SWEEP_SEEDS)} seeds)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.suptitle('S064 3D comparison (common random numbers)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'comparison.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(out, out_dir):
    r = out['main']['3d'][PLOT_SEED]
    lg, world = r['log'], r['world']
    n = len(lg['t'])
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    fig, ax = plt.subplots(figsize=(12, 4.2))
    for ri, rw in enumerate(ROW_Y):
        ax.add_patch(Rectangle((world.x0, rw - ROW_THICK / 2), world.x1 - world.x0, ROW_THICK,
                               color='grey', alpha=0.45))
    ax.add_patch(Rectangle((0, 0), GH_X, GH_Y, fill=False, ec='k', lw=1.5))
    ax.set_xlim(-0.5, GH_X + 0.5)
    ax.set_ylim(-0.5, GH_Y + 0.5)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    tr, = ax.plot([], [], color='tab:blue', lw=1.0, label='true path')
    te, = ax.plot([], [], color='red', ls='--', lw=0.8, label='estimate')
    sc = ax.scatter([lg['x'][0]], [lg['y'][0]], c=[lg['z'][0]], cmap='coolwarm', vmin=0.3,
                    vmax=2.1, s=60, edgecolor='k', zorder=5)
    cb = fig.colorbar(sc, ax=ax, fraction=0.03, pad=0.01)
    cb.set_label('drone altitude z (m)')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.18), ncol=2, fontsize=8)
    fig.subplots_adjust(bottom=0.25, top=0.88)

    def update(fi):
        i = min(fi * ANIM_STEP, n - 1)
        tr.set_data(lg['x'][:i + 1], lg['y'][:i + 1])
        te.set_data(lg['x'][:i + 1], lg['yhat'][:i + 1])
        sc.set_offsets([[lg['x'][i], lg['y'][i]]])
        sc.set_array(np.array([lg['z'][i]]))
        ax.set_title(f"S064 3D top view — t = {lg['t'][i]:.1f} s, z = {lg['z'][i]:.2f} m, "
                     f"|ŷ − y| = {abs(lg['yhat'][i] - lg['y'][i]) * 100:.1f} cm (seed {PLOT_SEED})")
        return tr, te, sc

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12)
    plt.close(fig)
    print(f'Saved: {path}  ({n_frames} frames)')


# ── Report ────────────────────────────────────────────────────────────────────

def _ms(v, fmt='.4f'):
    v = np.asarray(v, dtype=float)
    return f'{v.mean():{fmt}} ± {v.std():{fmt}}'


def report(out):
    M = out['main']
    print('\n=== S064 3D Greenhouse Interior Navigation — Results ===')
    print(f"  Aisle centres y: {', '.join(f'{v:.2f}' for v in AISLE_Y)} m; canopy segments "
          f"{SEG_LEN:.0f} m, h ~ U({H_LO}, {H_HI}) m")
    print('  Card arithmetic checks:')
    print(f'    card cruise band: lower bound z_t2 + rho = {WIRE_Z[1] + RHO_CARD:.2f} m, upper bound '
          f'min(h) - d_safe <= {H_HI - D_SAFE:.2f} m -> empty for every canopy draw')
    print(f'    card z_cross 1.9 m over a 1.6 m canopy: edge gap {1.9 - HALF[2] - H_HI:.3f} m '
          f'(< d_safe {D_SAFE}); needed z_cross >= {H_HI + HALF[2] + D_SAFE:.3f} m')
    print(f'    half-diagonal of the 0.3 x 0.3 m body = {math.hypot(0.15, 0.15):.3f} m (card 0.18)')
    print(f'    plantable row length: headland layout {LAYOUT_SPAN["headland"][1] - LAYOUT_SPAN["headland"][0]:.0f} m, '
          f'full-length {GH_X:.0f} m per row -> '
          f'+{100 * (GH_X / (LAYOUT_SPAN["headland"][1] - LAYOUT_SPAN["headland"][0]) - 1):.1f} %')
    za_card = M['card'][PLOT_SEED]['z_aisle']
    allz = np.array([r['z_aisle'] for r in M['card']])
    print(f"    card altitude rule, seed {PLOT_SEED}: aisles at {', '.join(f'{v:.3f}' for v in za_card)} m; "
          f'over {N_SEEDS} seeds {allz.min():.3f}-{allz.max():.3f} m')
    for key, lab, *_ in MAIN:
        R = M[key]
        g = lambda f: [r[f] for r in R]          # noqa: E731
        print(f'  [{lab}]')
        print(f"    Missions complete   : {sum(g('complete'))}/{N_SEEDS} (crashes {sum(g('crash'))}); "
              f"row-ends {_ms(g('row_ends'), '.2f')} of 5")
        done = [r['time'] for r in R if r['complete']]
        if done:
            print(f'    Mission time        : {np.mean(done):.1f} ± {np.std(done):.1f} s; '
                  f"aisle changes {_ms([r['turn_time'] for r in R if r['complete']], '.1f')} s")
        print(f"    Mean |y_hat - y|    : {_ms(g('mean_lat'))} m; peak {_ms(g('max_lat'))} m "
              f"(worst {np.max(g('max_lat')):.4f} m)")
        print(f"    Mean |z_hat - z|    : {_ms(g('mean_alt'))} m; peak {_ms(g('max_alt'))} m "
              f"(worst {np.max(g('max_alt')):.4f} m)")
        print(f"    Mean E_3D           : {_ms(g('mean_3d'))} m")
        print(f"    Lateral fix steps   : {100 * np.mean(g('fix_frac')):.1f} %; ToF fix steps "
              f"{100 * np.mean(g('tof_frac')):.1f} %")
        print(f"    Peak |y_hat - y| inside an aisle change in {sum(g('peak_in_turn'))}/{N_SEEDS} runs "
              f"(aisle changes = {100 * np.mean(g('turn_frac')):.1f} % of flight time)")
        print(f"    Min 3D clearance    : {_ms(g('min_clear'), '.3f')} m (worst {np.min(g('min_clear')):.3f} m); "
              f"rows worst {np.min(g('min_row')):.3f} m, wires worst {np.min(g('min_wire')):.3f} m, "
              f"walls/floor worst {np.min(g('min_wall')):.3f} m")
        print(f"    Steps below d_safe  : {_ms(g('viol'), '.1f')}; runs with zero "
              f"{sum(v == 0 for v in g('viol'))}/{N_SEEDS}")
        print(f"    Max commanded speed : {np.max(g('v_peak')):.2f} m/s (limit {V_MAX})")
    R = M['2d_full']
    nr = len(R)
    print(f"  [Base strategy (z = 1.0 m, sideways turns) in the full-length greenhouse, {nr} seeds]")
    print(f"    Missions complete {sum(r['complete'] for r in R)}/{nr}, collisions "
          f"{sum(r['crash'] for r in R)}/{nr}, row-ends {_ms([r['row_ends'] for r in R], '.2f')}, "
          f"worst row clearance {np.min([r['min_row'] for r in R]):.3f} m, steps below d_safe "
          f"{_ms([r['viol'] for r in R], '.1f')} of {int(round(T_MAX / DT))}")
    a2, a3, ac = (np.array([r['mean_lat'] for r in M[k]]) for k in ('2d', '3d', 'card'))
    p2, p3 = (np.array([r['max_lat'] for r in M[k]]) for k in ('2d', '3d'))
    t2, t3 = (np.array([r['time'] for r in M[k]]) for k in ('2d', '3d'))
    print('  Head-to-head (per seed, common random numbers):')
    print(f'    3D vs 2D base: mean lateral error lower in {int(np.sum(a3 < a2))}/{N_SEEDS}, '
          f'ratio of means {a3.mean() / a2.mean():.3f}; peak lower in {int(np.sum(p3 < p2))}/{N_SEEDS}, '
          f'ratio {p3.mean() / p2.mean():.3f}')
    print(f'    3D vs 2D base: mission time +{(t3 - t2).mean():.1f} ± {(t3 - t2).std():.1f} s '
          f'(+{100 * ((t3 / t2).mean() - 1):.1f} %), longer in {int(np.sum(t3 > t2))}/{N_SEEDS}')
    print(f'    Card rule vs 3D z=1.0: mean lateral error higher in {int(np.sum(ac > a3))}/{N_SEEDS}, '
          f'ratio of means {ac.mean() / a3.mean():.3f}')
    nh = len(HEAD_SEEDS)
    print(f'  Headland width of the 2D arm ({nh} seeds; turn line at the headland centre): width -> row '
          'length, leg length (turn line to turn line), complete, crashes, runs with steps < d_safe, mean '
          'steps < d_safe, worst 3D clearance; base-metric clearance: runs with steps < d_safe, worst; '
          'mission time, mean / peak |y_hat - y|, full-length gain')
    safe = []
    t20 = np.mean([r['time'] for r in out['head'][HEAD_W]])
    for hw in HEAD_SWEEP:
        R = out['head'][hw]
        L = GH_X - 2 * hw
        ok = all(r['complete'] for r in R) and all(r['viol'] == 0 for r in R)
        if ok:
            safe.append(hw)
        print(f"    {hw:.3f} m: {L:.2f} m, {GH_X - hw:.3f} m, {sum(r['complete'] for r in R)}/{nh}, "
              f"{sum(r['crash'] for r in R)}, {sum(r['viol'] > 0 for r in R)}/{nh}, "
              f"{np.mean([r['viol'] for r in R]):.1f}, {np.min([r['min_clear'] for r in R]):.3f} m; "
              f"{sum(r['viol_base'] > 0 for r in R)}/{nh}, {np.min([r['min_base'] for r in R]):.3f} m; "
              f"{np.mean([r['time'] for r in R]):.1f} s ({np.mean([r['time'] for r in R]) - t20:+.1f} s vs "
              f"{HEAD_W} m), {np.mean([r['mean_lat'] for r in R]):.4f} / "
              f"{np.mean([r['max_lat'] for r in R]):.4f} m, +{100 * (GH_X / L - 1):.1f} %"
              f"{'' if ok else '  (unsafe: d_safe violated or incomplete)'}")
    print(f'  Rare-event check on fresh seeds {HEAD_SEEDS_EXTRA[0]}-{HEAD_SEEDS_EXTRA[-1]}: width -> runs '
          'with steps < d_safe, worst clearance, steps below d_safe in the violating runs; all seeds 0-299 combined')
    for hw_ in HEAD_EXTRA:
        Rx = out['headx'][hw_]
        bad = [(sd, r['viol'], r['min_clear']) for sd, r in zip(HEAD_SEEDS_EXTRA, Rx) if r['viol'] > 0]
        tot = sum(r['viol'] > 0 for r in out['head'][hw_]) + len(bad)
        print(f"    {hw_:.3f} m: {len(bad)}/{len(Rx)}, {np.min([r['min_clear'] for r in Rx]):.3f} m, "
              + (', '.join(f'seed {sd}: {v} steps (min {c:.3f} m)' for sd, v, c in bad) or 'none')
              + f"; seeds 0-299: {tot}/{len(Rx) + len(out['head'][hw_])} runs violate")
    print('  Turn-line geometry probes (30 seeds): width, turn line x -> runs with steps < d_safe, worst '
          'clearance, mission time')
    for (hw_, xt), R in out['turnp'].items():
        print(f"    {hw_:.3f} m, x = {xt:.2f} m: {sum(r['viol'] > 0 for r in R)}/{N_SEEDS}, "
              f"{np.min([r['min_clear'] for r in R]):.3f} m, {np.mean([r['time'] for r in R]):.1f} s")
    hw = min(safe)
    R = out['head'][hw][:N_SEEDS]
    tn = np.array([r['time'] for r in R])
    an = np.array([r['mean_lat'] for r in R])
    pn = np.array([r['max_lat'] for r in R])
    print(f'  Narrowest safe headland in the sweep: {hw:.3f} m -> full-length gain '
          f'+{100 * (GH_X / (GH_X - 2 * hw) - 1):.1f} % row length; 3D vs this 2D arm (seeds 0-29): mission '
          f'time {(t3 - tn).mean():+.1f} ± {(t3 - tn).std():.1f} s, longer in {int(np.sum(t3 > tn))}/{N_SEEDS} '
          f'(against the {HEAD_W} m arm it is {(t3 - t2).mean():+.1f} s: the narrow arm\'s longer legs make it '
          f'{np.mean(tn - t2):+.1f} s slower, which shrinks the gap); '
          f'mean lateral error lower in {int(np.sum(a3 < an))}/{N_SEEDS} (ratio {a3.mean() / an.mean():.3f}); '
          f'peak lower in {int(np.sum(p3 < pn))}/{N_SEEDS} (ratio {p3.mean() / pn.mean():.3f})')
    ea = np.array([r['mean_lat'] for r in out['equiv']['arm']])
    eb = np.array([r['mean_lat'] for r in out['equiv']['base']])
    pa = np.array([r['max_lat'] for r in out['equiv']['arm']])
    pb = np.array([r['max_lat'] for r in out['equiv']['base']])
    print(f"  Equivalence with the 2D S064 base: 2D arm with the base's K_alt 1.2, start z 1.0 m and 1.0 m/s "
          f"turns (z still estimated): mean |y_hat - y| {ea.mean():.4f} vs base {eb.mean():.4f} m, "
          f"max per-seed difference {np.max(np.abs(ea - eb)):.4f} m; peak {pa.mean():.4f} vs {pb.mean():.4f} m; "
          f"time {np.mean([r['time'] for r in out['equiv']['arm']]):.1f} vs "
          f"{np.mean([r['time'] for r in out['equiv']['base']]):.1f} s; complete "
          f"{sum(r['complete'] for r in out['equiv']['arm'])} / {sum(r['complete'] for r in out['equiv']['base'])}"
          f" of {N_SEEDS}")
    sw = out['sweep']
    ns = len(SWEEP_SEEDS)
    print(f'  Sweep: aisle altitude (full-length rows, {ns} seeds): z -> lateral fix %, mean / peak '
          '|y_hat - y|, complete, steps < d_safe')
    for v in sorted(sw['z_aisle']):
        R = sw['z_aisle'][v]
        print(f"    {v:.2f} m: {100 * np.mean([r['fix_frac'] for r in R]):.1f} %, "
              f"{np.mean([r['mean_lat'] for r in R]):.4f} / {np.mean([r['max_lat'] for r in R]):.4f} m, "
              f"{sum(r['complete'] for r in R)}/{ns}, {np.mean([r['viol'] for r in R]):.1f}")
    print(f'  Sweep: crossing altitude ({ns} seeds): z_cross -> worst row clearance, steps < d_safe, '
          'mission time, complete')
    for v in sorted(sw['z_cross']):
        R = sw['z_cross'][v]
        print(f"    {v:.2f} m: {np.min([r['min_row'] for r in R]):.3f} m, "
              f"{np.mean([r['viol'] for r in R]):.1f}, {np.mean([r['time'] for r in R]):.1f} s, "
              f"{sum(r['complete'] for r in R)}/{ns}")
    print(f'  Sweep: altitude filter beta ({ns} seeds; 1.0 = no ToF): mean / peak |z_hat - z|, '
          'worst row clearance, steps < d_safe')
    for v in sorted(sw['beta']):
        R = sw['beta'][v]
        print(f"    {v:.2f}: {np.mean([r['mean_alt'] for r in R]):.4f} / "
              f"{np.mean([r['max_alt'] for r in R]):.4f} m, {np.min([r['min_row'] for r in R]):.3f} m, "
              f"{np.mean([r['viol'] for r in R]):.1f}")
    print(f'  Sweep: ToF gate ({ns} seeds): gate -> mean / peak |z_hat - z|, ToF fix %, complete, crashes')
    for v in sorted(sw['gate_z']):
        R = sw['gate_z'][v]
        print(f"    {v:.2f} m: {np.mean([r['mean_alt'] for r in R]):.4f} / "
              f"{np.mean([r['max_alt'] for r in R]):.4f} m, {100 * np.mean([r['tof_frac'] for r in R]):.1f} %, "
              f"{sum(r['complete'] for r in R)}/{ns}, {sum(r['crash'] for r in R)}")
    print(f'  Sweep: aisle-change lateral speed ({ns} seeds): layout, v -> mission time, aisle-change '
          'time, peak |y_hat - y|, peak inside an aisle change')
    for (lay, v) in sorted(sw['v_turn']):
        R = sw['v_turn'][(lay, v)]
        print(f"    {lay:8s} {v:.1f} m/s: {np.mean([r['time'] for r in R]):.1f} s, "
              f"{np.mean([r['turn_time'] for r in R]):.1f} s, {np.mean([r['max_lat'] for r in R]):.4f} m, "
              f"{sum(r['peak_in_turn'] for r in R)}/{ns}; complete {sum(r['complete'] for r in R)}/{ns}")
    for v in (0.5, 1.0):
        th = np.array([r['time'] for r in sw['v_turn'][('headland', v)]])
        tf = np.array([r['time'] for r in sw['v_turn'][('full', v)]])
        print(f'    at {v:.1f} m/s the full-length layout takes +{(tf - th).mean():.1f} s')
    print(f'  Sweep: canopy segment length ({ns} seeds; 40 m = one height per row as in the card): '
          'seg -> lateral fix % / mean |y_hat - y| for z=1.0 vs the card rule')
    for sl in (1.0, 2.0, 4.0, 40.0):
        Rf, Rc = sw['seg'][(sl, 'fixed')], sw['seg'][(sl, 'card')]
        ef = np.array([r['mean_lat'] for r in Rf])
        ec = np.array([r['mean_lat'] for r in Rc])
        print(f"    {sl:4.0f} m: z=1.0 {100 * np.mean([r['fix_frac'] for r in Rf]):.1f} % / {ef.mean():.4f} m; "
              f"card {100 * np.mean([r['fix_frac'] for r in Rc]):.1f} % / {ec.mean():.4f} m; "
              f"card worse in {int(np.sum(ec > ef))}/{ns}; card aisles "
              f"{np.min([min(r['z_aisle']) for r in Rc]):.3f}-{np.max([max(r['z_aisle']) for r in Rc]):.3f} m")
    r3 = M['3d'][PLOT_SEED]
    print(f"  Seed {PLOT_SEED} 3D: row-ends at {', '.join(f'{t:.1f}' for t in r3['reach'])} s; "
          f"mean lat {r3['mean_lat']:.4f} m, peak {r3['max_lat']:.4f} m; mean alt {r3['mean_alt']:.4f} m, "
          f"peak {r3['max_alt']:.4f} m; min clearance {r3['min_clear']:.3f} m; "
          f"z range {r3['log']['z'].min():.3f}-{r3['log']['z'].max():.3f} m")
    r2 = M['2d'][PLOT_SEED]
    print(f"  Seed {PLOT_SEED} 2D base: mean lat {r2['mean_lat']:.4f} m, peak {r2['max_lat']:.4f} m, "
          f"time {r2['time']:.1f} s, min clearance {r2['min_clear']:.3f} m")


if __name__ == '__main__':
    res = run_simulation()
    report(res)
    od = os.path.normpath(OUTPUT_DIR)
    os.makedirs(od, exist_ok=True)
    plot_trajectory(res, od)
    plot_altitude(res, od)
    plot_analysis(res, od)
    plot_comparison(res, od)
    save_animation(res, od)
