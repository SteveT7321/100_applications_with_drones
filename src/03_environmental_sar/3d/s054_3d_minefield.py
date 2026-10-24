"""
S054 3D Minefield Detection
===========================
The S054 GPR survey (200 x 200 m field, 1 m grid, risk map, least-risk escape path,
false alarms per 8 x 8 m block and pass, 95 % coverage stop) now runs over uneven
ground (DEM relief 5 m) with an altitude-sensitive GPR:

    r_gpr(z) = r0 exp(-z / H_ref),  G(z) = exp(-z^2 / (2 H_ref^2)),
    P_d(rho, z) = (1 - exp(-r_gpr(z)^2 / (2 rho^2))) G(z)   (0 beyond r_gpr horizontally)

with rho the slant range to the mine's surface point and z the drone's AGL.
The drone's altitude is a controlled state: z_dot = clip((z_cmd - z) / tau_z, +-1.5 m/s).

Strategies (card), on the same fields, terrain and false-alarm draws per seed:
  1. Fixed world altitude (2D baseline): the S054 boustrophedon at one world-frame z,
     3 m above the highest ground (the 2D altitude)       -> 'fixed3'
     added: the same at 1 m above the highest ground       -> 'fixed1'
  2. Terrain following at z*_AGL, survey only              -> 'tf'
  3. Terrain following at z*_AGL + confirmation detours (descend to 1.5 m AGL over every
     alarm that left the footprint unconfirmed, hover 2 s)  -> 'tfc'
z*_AGL is the maximiser of the card's efficiency eta(z) = integral of P_d over the footprint.

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s054_3d_minefield.py
"""

import os
import math
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.optimize import minimize_scalar

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR   = os.path.join(PROJECT_ROOT, 'outputs', '03_environmental_sar', '3d', 's054_3d_minefield')

_spec = importlib.util.spec_from_file_location('s054_2d', os.path.join(SCRIPT_DIR, '..', 's054_minefield.py'))
s2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(s2d)

# -- Parameters (card) ------------------------------------------------------------------
R0         = 5.0          # m  GPR radius at ground level
H_REF      = 4.0          # m  altitude decay constant
Z_AGL_MIN, Z_AGL_MAX = 1.0, 6.0
Z_LO       = 1.5          # m  confirmation altitude (AGL)
T_HOVER    = 2.0          # s
VZ_MAX     = 1.5          # m/s
TAU_Z      = 0.5          # s
RELIEF     = 5.0          # m  terrain relief
V          = s2d.DRONE_SPEED          # 3 m/s horizontal
DT         = s2d.DT                   # 0.2 s
Z_2D       = s2d.ALTITUDE             # 3 m (2D altitude)
N_CELLS    = s2d.N_CELLS
XC         = s2d.XC
THETA_ALARM, THETA_CONFIRM = s2d.THETA_ALARM, s2d.THETA_CONFIRM
SCAN_PD    = s2d.SCAN_PD
FA_BLOCK   = s2d.FA_BLOCK
P_FA       = s2d.P_FA

# -- Parameters (added) -----------------------------------------------------------------
N_BUMPS    = 8            # Gaussian bumps in the DEM
BUMP_SIG   = (15.0, 40.0) # m
RELIEF_SWEEP = (0.0, 2.5, 10.0)       # m (main 5 m), fixed1 vs tf
Z_SWEEP    = (1.5, 2.0, 2.5, 3.0)     # m survey AGL for tfc (main = z*)
N_MC       = 10
MAIN_SEED  = s2d.MAIN_SEED            # 42 (2D showcase field)
N_WORKERS  = min(6, os.cpu_count() or 1)
ANIM_FRAMES = 90

STRATS = ['fixed3', 'fixed1', 'tf', 'tfc']
LABELS = {'fixed3': 'Fixed z = max h + 3 m (2D baseline)', 'fixed1': 'Fixed z = max h + 1 m',
          'tf': 'Terrain-following, survey only', 'tfc': 'Terrain-following + confirmation'}
COLORS = {'fixed3': '#7F7F7F', 'fixed1': '#BCBD22', 'tf': '#1F77B4', 'tfc': '#D62728'}


# -- GPR model --------------------------------------------------------------------------

def r_gpr(z):
    return R0 * np.exp(-np.asarray(z, float) / H_REF)


def gain(z):
    return np.exp(-np.asarray(z, float) ** 2 / (2 * H_REF ** 2))


def pd3(horiz, dz, z_agl):
    """Single-step P_d at horizontal distance horiz and vertical separation dz (slant range),
    for a drone at AGL z_agl; zero beyond the footprint radius r_gpr(z_agl) (2D cutoff)."""
    r = r_gpr(z_agl)
    rho2 = np.maximum(np.asarray(horiz, float) ** 2 + np.asarray(dz, float) ** 2, 1e-6)
    p = (1.0 - np.exp(-r ** 2 / (2.0 * rho2))) * gain(z_agl)
    return np.where(np.asarray(horiz) <= r, p, 0.0)


def eta(z):
    """Card efficiency: integral of P_d over the footprint disc (flat ground), m^2."""
    r = float(r_gpr(z))
    s = np.linspace(0.0, r, 2001)
    f = pd3(s, z, z) * 2 * np.pi * s
    return float(np.sum((f[1:] + f[:-1]) / 2 * np.diff(s)))


def alarm_radius(z):
    """Largest horizontal distance with single-step P_d > theta_alarm at AGL z (flat ground)."""
    s = np.linspace(0.0, float(r_gpr(z)), 20001)
    ok = pd3(s, z, z) > THETA_ALARM
    return float(s[ok].max()) if ok.any() else 0.0


def optimal_altitude():
    zs = np.linspace(Z_AGL_MIN, Z_AGL_MAX, 501)
    e = np.array([eta(z) for z in zs])
    res = minimize_scalar(lambda z: -eta(z), bounds=(Z_AGL_MIN, Z_AGL_MAX), method='bounded',
                          options={'xatol': 1e-6})
    z_grid = float(zs[int(np.argmax(e))])
    return z_grid, float(res.x), zs, e


# -- Terrain ----------------------------------------------------------------------------

class Terrain:
    def __init__(self, seed, relief=RELIEF):
        rng = np.random.default_rng([seed, 7])
        self.b = [(rng.uniform(0, 200), rng.uniform(0, 200), rng.uniform(-1, 1),
                   rng.uniform(*BUMP_SIG)) for _ in range(N_BUMPS)]
        raw = self._raw(XC[None, :], XC[:, None])
        self.lo, self.hi = float(raw.min()), float(raw.max())
        self.relief = relief
        self.H = self.h(XC[None, :], XC[:, None])        # [y, x] at cell centres

    def _raw(self, x, y):
        out = 0.0
        for bx, by, a, s in self.b:
            out = out + a * np.exp(-((x - bx) ** 2 + (y - by) ** 2) / (2 * s * s))
        return out

    def h(self, x, y):
        if self.relief == 0:
            return np.zeros(np.broadcast(np.asarray(x), np.asarray(y)).shape)
        return self.relief * (self._raw(x, y) - self.lo) / (self.hi - self.lo)

    def slope_deg(self):
        gy, gx = np.gradient(self.H, 1.0)
        return np.degrees(np.arctan(np.hypot(gx, gy)))


# -- 3D mission (extends the 2D mission) --------------------------------------------------

class Mission3D(s2d.Mission):
    def __init__(self, field, terrain, mode, z_agl, confirm=False, record=False, strip_width=s2d.STRIP_WIDTH):
        super().__init__(field, 'fixed', record=record, strip_width=strip_width)
        self.ter = terrain
        self.mode = mode                     # 'fixed' (world z) or 'tf'
        self.z_agl_cmd = z_agl
        self.z_fixed = float(terrain.H.max()) + z_agl if mode == 'fixed' else None
        self.confirm = confirm
        self.mine_h = terrain.h(self.mines[:, 0], self.mines[:, 1])
        self.override_agl = None             # confirmation descent target (AGL)
        self.z = self.z_cmd_at(self.pos)
        self.agl_log = []                    # (t, agl, agl_cmd, z, zg)
        self.traj3 = [np.array([*self.pos, self.z])]
        self.visited = set()
        self.queue = []
        self.n_detours = 0
        self.t_detour = 0.0
        self.detour_t = []
        self.detour_xy = []
        self.max_pd_step = np.zeros(len(self.mines))

    def z_cmd_at(self, p):
        zg = float(self.ter.h(p[0], p[1]))
        if self.override_agl is not None:
            return zg + self.override_agl
        return self.z_fixed if self.mode == 'fixed' else zg + self.z_agl_cmd

    def agl(self):
        return self.z - float(self.ter.h(self.pos[0], self.pos[1]))

    # -- sensing (3D model) -------------------------------------------------------------
    def sense(self):
        px, py = self.pos
        za = self.agl()
        assert za > 0.0, f'drone below ground (AGL {za:.2f} m)'
        rg = float(r_gpr(za))
        changed = False
        i0, i1 = max(0, int(px - rg) - 1), min(N_CELLS, int(px + rg) + 2)
        j0, j1 = max(0, int(py - rg) - 1), min(N_CELLS, int(py + rg) + 2)
        r = np.hypot(XC[i0:i1][None, :] - px, XC[j0:j1][:, None] - py)
        dz = self.z - self.ter.H[j0:j1, i0:i1]
        win = self.cov[j0:j1, i0:i1]
        before = win > SCAN_PD
        win[:] = 1.0 - (1.0 - win) * (1.0 - pd3(r, dz, za))
        self.n_scanned += int(np.sum((win > SCAN_PD) & ~before))
        rm = np.hypot(self.mines[:, 0] - px, self.mines[:, 1] - py)
        np.minimum(self.min_r, rm, out=self.min_r)
        if self.surveying:
            dx = self.mines[:, 0] - px
            row = np.abs(self.mines[:, 1] - py) <= V * DT + 1e-9
            self.pass_w |= row & (dx >= 0) & (dx <= self.strip_width)
            self.pass_e |= row & (dx <= 0) & (-dx <= self.strip_width)
        pd = pd3(rm, self.z - self.mine_h, za)
        np.maximum(self.max_pd_step, pd, out=self.max_pd_step)
        self.cum_pd = 1.0 - (1.0 - self.cum_pd) * (1.0 - pd)
        new_al = (~self.alarmed) & (pd > THETA_ALARM)
        new_cf = (~self.confirmed) & (self.cum_pd > THETA_CONFIRM)
        for k in np.where(new_al)[0]:
            self.queue.append(('m', int(k)))
        if new_al.any() or new_cf.any():
            changed = True
        self.alarmed |= new_al
        self.confirmed |= new_cf
        nb = self.f['nb']
        bi = np.clip(np.arange(int(px // FA_BLOCK) - 1, int(px // FA_BLOCK) + 2), 0, nb - 1)
        bj = np.clip(np.arange(int(py // FA_BLOCK) - 1, int(py // FA_BLOCK) + 2), 0, nb - 1)
        near = np.unique((bj[:, None] * nb + bi[None, :]).ravel())
        db = np.hypot(self.block_xy[near, 0] - px, self.block_xy[near, 1] - py)
        inside = np.zeros(nb * nb, dtype=bool)
        inside[near] = db <= rg
        for b in np.where(inside & ~self.in_block)[0]:
            k = self.block_pass[b]
            self.block_pass[b] += 1
            if self.f['fa_u'][b, k % 64] < P_FA:
                xy = self.block_xy[b] - FA_BLOCK / 2 + self.f['fa_xy'][b, k % 64]
                self.fa_xy = np.vstack([self.fa_xy, xy])
                self.fa_cum = np.append(self.fa_cum, 0.0)
                self.fa_active = np.append(self.fa_active, True)
                self.fa_left = np.append(self.fa_left, False)
                self.queue.append(('f', len(self.fa_cum) - 1))
                changed = True
        self.in_block = inside
        if self.fa_active.any():
            d = np.hypot(self.fa_xy[:, 0] - px, self.fa_xy[:, 1] - py)
            fh = self.ter.h(self.fa_xy[:, 0], self.fa_xy[:, 1])
            p = pd3(d, self.z - fh, za)
            act = self.fa_active
            self.fa_cum[act] = 1.0 - (1.0 - self.fa_cum[act]) * (1.0 - p[act])
            self.fa_left |= act & (d > rg)
            clear = act & self.fa_left & (p > THETA_ALARM)
            if clear.any():
                self.fa_active[clear] = False
                changed = True
        if not changed and self.t - self.last_reweight >= s2d.REWEIGHT_EVERY:
            w = self.source_weights()
            if any(abs(w.get(k, 0) - v) > 0.01 for k, v in w.items()) or \
                    len(w) != len(self.weights_at_update):
                changed = True
            self.last_reweight = self.t
        if changed:
            self.update_risk()
        return changed

    # -- motion (horizontal 2D step + vertical tracker) ---------------------------------
    def _vertical(self):
        zc = self.z_cmd_at(self.pos)
        rate = min(max((zc - self.z) / TAU_Z, -VZ_MAX), VZ_MAX)
        self.z += rate * DT
        assert abs(rate) <= VZ_MAX + 1e-12

    def step_towards(self, target):
        d = target - self.pos
        dist = np.linalg.norm(d)
        stp = V * DT
        if dist <= stp:
            new, arrived = target.astype(float).copy(), True
        else:
            new, arrived = self.pos + d / dist * stp, False
        moved = np.linalg.norm(new - self.pos)
        assert moved <= stp + 1e-9, 'speed limit exceeded'
        if self.risk_at(self.pos) >= s2d.R_CAUTION:
            self.danger_dist += moved
        z0 = self.z
        self.pos = new
        self._vertical()
        self.dist += math.hypot(moved, self.z - z0)
        self.t += DT
        self._log3()
        self.sense()
        self.log()
        return arrived

    def hold(self, n_steps):
        """Hover / vertical-only steps over the current point (sensing every step)."""
        for _ in range(n_steps):
            z0 = self.z
            self._vertical()
            self.dist += abs(self.z - z0)
            self.t += DT
            self._log3()
            self.sense()
            self.log()

    def _log3(self):
        zg = float(self.ter.h(self.pos[0], self.pos[1]))
        cmd = self.override_agl if self.override_agl is not None else \
            (self.z_fixed - zg if self.mode == 'fixed' else self.z_agl_cmd)
        self.agl_log.append((self.t, self.z - zg, cmd, self.z, zg, self.override_agl is not None))
        if self.record:
            self.traj3.append(np.array([*self.pos, self.z]))

    # -- confirmation detours -----------------------------------------------------------
    def pending(self):
        """Alarms that have left the footprint without being confirmed or cleared."""
        rg = float(r_gpr(max(self.agl(), 1e-3)))
        out = []
        for key in self.queue:
            if key in self.visited:
                continue
            if key[0] == 'm':
                if self.confirmed[key[1]]:
                    continue
                xy = self.mines[key[1]]
            else:
                if not self.fa_active[key[1]]:
                    continue
                xy = self.fa_xy[key[1]]
            if np.hypot(*(xy - self.pos)) > rg:
                out.append((key, np.asarray(xy, float)))
        return out

    def detour(self, key, xy):
        t0 = self.t
        back = self.pos.copy()
        self.visited.add(key)
        self.detour_t.append(t0)
        self.detour_xy.append(xy.copy())
        while not self.step_towards(xy):
            pass
        self.override_agl = Z_LO
        n_v = int(math.ceil(abs(Z_LO - self.agl()) / (VZ_MAX * DT))) + int(math.ceil(3 * TAU_Z / DT))
        self.hold(n_v)
        self.hold(int(round(T_HOVER / DT)))
        self.override_agl = None
        self.hold(n_v)
        while not self.step_towards(back):
            pass
        self.n_detours += 1
        self.t_detour += self.t - t0

    def run_survey(self):
        wps = self.waypoints()
        i = 0
        while i < len(wps) and self.coverage() < s2d.C_TARGET and not self.abort:
            if self.step_towards(wps[i]):
                i += 1
            if self.confirm:
                for key, xy in self.pending():
                    self.detour(key, xy)
        if self.confirm:
            for key, xy in self.pending():
                self.detour(key, xy)

    def run(self):
        self.sense()
        self.log()
        self.n_reclass_visits = 0
        self.run_survey()
        self.t_survey = self.t
        self.cov_survey = self.coverage()
        self.surveying = False
        self.return_home()
        el = np.array(self.esc_log)
        al = np.array(self.agl_log)
        surv = ~al[:, 5].astype(bool)
        detected = self.alarmed | self.confirmed
        err = np.abs(al[surv, 1] - al[surv, 2])
        return {
            'time': self.t, 't_survey': self.t_survey, 'dist': self.dist, 'coverage': self.cov_survey,
            'n_mines': len(self.mines), 'detected': int(detected.sum()),
            'confirmed': int(self.confirmed.sum()), 'missed': int((~detected).sum()),
            'd_confirm': float(self.confirmed.mean()),
            'fa_raised': len(self.fa_xy), 'fa_unresolved': int(self.fa_active.sum()),
            'detours': self.n_detours, 't_detour': self.t_detour,
            'agl_min': float(al[:, 1].min()), 'agl_max': float(al[:, 1].max()),
            'agl_err_mean': float(err.mean()), 'agl_err_max': float(err.max()),
            'esc_mean_max': float(el[:, 1].max()), 'frac_esc_unsafe': float(np.mean(el[:, 1] >= s2d.R_SAFE)),
            'max_pd_step': float(self.max_pd_step.max()),
            'cum_pd': self.cum_pd.copy(), 'mine_status': np.where(self.confirmed, 2, np.where(detected, 1, 0)),
        }


def run_2d(fld):
    """The 2D S054 fixed boustrophedon (flat ground, 2D GPR model) on the same field."""
    r = s2d.Mission(fld, 'fixed').run()
    r['d_confirm'] = r['confirmed'] / r['n_mines']
    return r


def run_field(args):
    """All missions on one field (same mines, terrain and false-alarm draws)."""
    seed, z_star, n_mines = args
    fld = s2d.make_field(seed, n_mines=n_mines)
    ter = Terrain(seed)
    out = {}
    for s_, (mode, za, cf) in strategy_specs(z_star).items():
        out[s_] = Mission3D(fld, ter, mode, za, cf).run()
    out['tf_narrow'] = Mission3D(fld, ter, 'tf', z_star, False, strip_width=2 * alarm_radius(z_star)).run()
    out['2d'] = run_2d(fld)
    for za in Z_SWEEP:
        out[('tfc', za)] = Mission3D(fld, ter, 'tf', za, True).run()
        out[('tf', za)] = Mission3D(fld, ter, 'tf', za, False).run()
    for rl in RELIEF_SWEEP:
        t2 = Terrain(seed, rl)
        out[('fixed1', 'relief', rl)] = Mission3D(fld, t2, 'fixed', z_star, False).run()
        out[('tf', 'relief', rl)] = Mission3D(fld, t2, 'tf', z_star, False).run()
    assert len(out) == len(STRATS) + 2 + 2 * len(Z_SWEEP) + 2 * len(RELIEF_SWEEP), 'result key collision'
    sl = ter.slope_deg()
    for v in out.values():
        v.pop('cum_pd', None)
        v.pop('mine_status', None)
    out['terrain'] = {'slope_max': float(sl.max()), 'slope_p95': float(np.percentile(sl, 95)),
                      'h_max': float(ter.H.max())}
    return seed, out


def strategy_specs(z_star):
    return {'fixed3': ('fixed', Z_2D, False), 'fixed1': ('fixed', z_star, False),
            'tf': ('tf', z_star, False), 'tfc': ('tf', z_star, True)}


def run_simulation():
    z_grid, z_opt, zs, e = optimal_altitude()
    z_star = round(z_opt, 3)
    field = s2d.make_field(MAIN_SEED, n_mines=s2d.N_MINE_DEFAULT)
    ter = Terrain(MAIN_SEED)
    show = {}
    for s_, (mode, za, cf) in strategy_specs(z_star).items():
        m = Mission3D(field, ter, mode, za, cf, record=True)
        r = m.run()
        show[s_] = (m, r)
    args = [(sd, z_star, None) for sd in range(N_MC)]
    if N_WORKERS > 1:
        import multiprocessing as mp
        with mp.get_context('spawn').Pool(N_WORKERS) as pool:
            mc = pool.map(run_field, args, chunksize=1)
    else:
        mc = [run_field(a) for a in args]
    show['2d'] = run_2d(field)
    return {'z_grid': z_grid, 'z_star': z_star, 'zs': zs, 'eta': e, 'field': field, 'ter': ter,
            'show': show, 'mc': [o for _, o in mc]}


# -- Report -----------------------------------------------------------------------------

def _arr(mc, key, field):
    return np.array([o[key][field] for o in mc], dtype=float)


def report(res):
    zs = res['z_star']
    mc = res['mc']
    print('=== S054 3D Minefield Detection ===')
    print(f'GPR model: r_gpr(z) = {R0} exp(-z/{H_REF}), G(z) = exp(-z^2/(2*{H_REF}^2)); '
          f'r_gpr = 4 m (2D value) at z = {H_REF * math.log(R0 / 4.0):.3f} m AGL')
    for z in (1.0, 1.5, 2.0, 2.5, 3.0, 6.0):
        print(f'  z = {z:.1f} m: r_gpr {float(r_gpr(z)):.3f} m, G {float(gain(z)):.3f}, P_d directly below '
              f'{float(pd3(0.0, z, z)):.3f}, alarm radius {alarm_radius(z):.3f} m, eta {eta(z):.3f} m^2')
    print(f'Optimal scan altitude: grid argmax {res["z_grid"]:.3f} m, bounded search {zs:.3f} m '
          f'(card: ~2.5 m); eta(z*) / eta(2.5) = {eta(zs) / eta(2.5):.2f}')
    print(f'  no alarm is possible above {alarm_altitude_limit():.3f} m AGL (P_d directly below <= {THETA_ALARM})')
    print(f'  alarm radius at z* = {alarm_radius(zs):.3f} m -> strip width for no seams 2 x {alarm_radius(zs):.3f} = '
          f'{2 * alarm_radius(zs):.2f} m (card strip 8 m)')
    print(f'  card AGL-error bound vz_max*tau/cos(15 deg) = {VZ_MAX * TAU_Z / math.cos(math.radians(15)):.3f} m; '
          f'steady lag on a 15 deg slope tau*v*tan(15 deg) = {TAU_Z * V * math.tan(math.radians(15)):.3f} m')
    ter = res['ter']
    print(f'Showcase field (seed {MAIN_SEED}, 15 mines): terrain 0-{ter.H.max():.2f} m, max slope '
          f'{ter.slope_deg().max():.2f} deg')
    keys = [('time', 'mission time (s)', '{:.0f}'), ('t_survey', 'survey end (s)', '{:.0f}'),
            ('dist', '3D distance (m)', '{:.0f}'), ('coverage', 'coverage C (%)', 'p'),
            ('detected', 'mines detected', '{:.0f}'), ('confirmed', 'mines confirmed', '{:.0f}'),
            ('missed', 'mines missed', '{:.0f}'), ('d_confirm', 'D_confirm (%)', 'p'),
            ('fa_raised', 'false alarms raised', '{:.0f}'), ('fa_unresolved', 'false alarms unresolved', '{:.0f}'),
            ('detours', 'confirmation detours', '{:.0f}'), ('t_detour', 'detour time (s)', '{:.0f}'),
            ('agl_min', 'AGL min (m)', '{:.3f}'), ('agl_max', 'AGL max (m)', '{:.3f}'),
            ('agl_err_mean', 'mean |AGL error| survey (m)', '{:.3f}'),
            ('agl_err_max', 'max |AGL error| survey (m)', '{:.3f}'),
            ('max_pd_step', 'max single-step P_d on a mine', '{:.3f}'),
            ('esc_mean_max', 'max mean escape-path R', '{:.3f}'),
            ('frac_esc_unsafe', 'escape samples mean R >= 0.1 (%)', 'p')]
    print('  metric | ' + ' | '.join(STRATS) + ' | 2D S054 fixed (flat, 2D GPR)')
    for k, lab, fmt in keys:
        row = []
        for s_ in STRATS + ['2d']:
            r = res['show'][s_][1] if s_ != '2d' else res['show']['2d']
            v = r.get(k)
            row.append('-' if v is None else (f'{100 * v:.2f}' if fmt == 'p' else fmt.format(v)))
        print(f'  {lab} | ' + ' | '.join(row))
    m_tfc = res['show']['tfc'][0]
    n_m = int(sum(1 for k in m_tfc.visited if k[0] == 'm'))
    print(f'  tfc detours: {len(m_tfc.visited)} ({n_m} to real mines, {len(m_tfc.visited) - n_m} to false alarms)')

    print(f'\nMonte Carlo ({N_MC} fields, N_mine ~ U{{8, 20}}, own terrain per field; same draws for every strategy):')
    print(f'  mines per field: mean {np.mean(_arr(mc, "tf", "n_mines")):.1f}; terrain max slope '
          f'{np.mean([o["terrain"]["slope_max"] for o in mc]):.2f} deg mean, {max(o["terrain"]["slope_max"] for o in mc):.2f} max')
    cols = STRATS + ['tf_narrow', '2d']
    for k, lab, fmt in keys:
        if k in ('max_pd_step',):
            continue
        parts = []
        for s_ in cols:
            if k not in mc[0][s_]:
                continue
            v = _arr(mc, s_, k)
            sc = 100 if fmt == 'p' else 1
            parts.append(f'{s_} {sc * v.mean():.2f} +- {sc * v.std():.2f}')
        print(f'  {lab}: ' + '; '.join(parts))
    for a, b in (('tf', 'fixed3'), ('tf', 'fixed1'), ('tfc', 'tf'), ('tf_narrow', 'tf'), ('tf', '2d')):
        dm = _arr(mc, a, 'missed') - _arr(mc, b, 'missed')
        print(f'  missed mines {a} vs {b}: fewer {int(np.sum(dm < 0))} / equal {int(np.sum(dm == 0))} / more '
              f'{int(np.sum(dm > 0))}; mean diff {dm.mean():+.2f}')
    for a, b in (('tfc', 'tf'),):
        du = _arr(mc, a, 'fa_unresolved') - _arr(mc, b, 'fa_unresolved')
        dt = _arr(mc, a, 'time') - _arr(mc, b, 'time')
        dc = _arr(mc, a, 'confirmed') - _arr(mc, b, 'confirmed')
        print(f'  {a} vs {b}: unresolved false alarms mean {du.mean():+.2f} (fewer in {int(np.sum(du < 0))}/{N_MC}); '
              f'confirmed mines mean {dc.mean():+.2f} (more in {int(np.sum(dc > 0))}/{N_MC}); time {dt.mean():+.0f} s '
              f'(range {dt.min():+.0f} to {dt.max():+.0f})')
    ok = [(np.mean(_arr(mc, s_, 'coverage') >= 0.95 - 1e-9), np.mean(_arr(mc, s_, 'd_confirm') >= 0.9))
          for s_ in cols]
    print('  fields meeting C >= 0.95 / D_confirm >= 0.90: ' + '; '.join(
        f'{s_} {int(round(a * N_MC))}/{N_MC} / {int(round(b * N_MC))}/{N_MC}' for s_, (a, b) in zip(cols, ok)))
    print(f'Survey-altitude sweep (terrain following, {N_MC} fields):')
    for za in (zs,) + Z_SWEEP:
        for s_ in ('tf', 'tfc'):
            key = s_ if za == zs else (s_, za)
            mi, cv, tm = _arr(mc, key, 'missed'), _arr(mc, key, 'coverage'), _arr(mc, key, 'time')
            dc = _arr(mc, key, 'd_confirm')
            print(f'  {s_} at {za:.2f} m AGL: missed {mi.mean():.2f}, D_confirm {100 * dc.mean():.1f}%, coverage '
                  f'{100 * cv.mean():.2f}%, time {tm.mean():.0f} s, unresolved FA {_arr(mc, key, "fa_unresolved").mean():.1f}')
    print(f'Terrain-relief sweep (fixed world z = max h + {zs:.1f} m vs terrain following at {zs:.1f} m AGL):')
    for rl in (0.0, 2.5, RELIEF, 10.0):
        parts = []
        for s_ in ('fixed1', 'tf'):
            key = s_ if rl == RELIEF else (s_, 'relief', rl)
            parts.append(f'{s_}: missed {_arr(mc, key, "missed").mean():.2f}, coverage '
                         f'{100 * _arr(mc, key, "coverage").mean():.2f}%, time {_arr(mc, key, "time").mean():.0f} s')
        print(f'  relief {rl:4.1f} m: ' + ' | '.join(parts))


# -- Plotting ---------------------------------------------------------------------------

def _save(fig, out, name):
    p = os.path.join(out, name)
    fig.savefig(p, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {p}')


def plot_trajectory_3d(res, out):
    m, r = res['show']['tfc']
    ter = res['ter']
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    g = XC[::5]
    X, Y = np.meshgrid(g, g)
    ax.plot_surface(X, Y, ter.H[::5, ::5], cmap='terrain', alpha=0.35, linewidth=0, zorder=0)
    tr = np.array(m.traj3)
    al = np.array(m.agl_log)
    agl = np.concatenate([[tr[0, 2] - ter.h(*tr[0, :2])], al[:, 1]])
    sc = ax.scatter(tr[::3, 0], tr[::3, 1], tr[::3, 2], c=agl[::3], cmap='viridis', s=1, vmin=0.8, vmax=1.6,
                    depthshade=False, zorder=2)
    fig.colorbar(sc, ax=ax, shrink=0.55, label='AGL (m)')
    det = np.concatenate([[False], al[:, 5].astype(bool)])
    ax.scatter(tr[det, 0], tr[det, 1], tr[det, 2], color='orange', s=4, depthshade=False, zorder=3,
               label='confirmation descents / hovers')
    st = r['mine_status']
    for code, col, mk, lab in ((2, 'red', 'o', 'confirmed mine'), (1, 'orange', '^', 'alarmed only'),
                               (0, 'black', 'x', 'missed mine')):
        k = st == code
        if k.any():
            ax.scatter(m.mines[k, 0], m.mines[k, 1], m.mine_h[k] + 0.05, color=col, marker=mk, s=40,
                       depthshade=False, zorder=4, label=lab)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_box_aspect((1, 1, 0.25)); ax.view_init(elev=30, azim=-60)
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title(f'S054 3D terrain-following + confirmation (seed {MAIN_SEED}): path coloured by AGL', fontsize=10)
    _save(fig, out, 'trajectory_3d.png')


def plot_agl_timeseries(res, out):
    fig, ax = plt.subplots(figsize=(11, 4.8))
    for s_ in STRATS:
        al = np.array(res['show'][s_][0].agl_log)
        ax.plot(al[:, 0], al[:, 1], color=COLORS[s_], lw=0.6, label=LABELS[s_])
    ax.axhline(res['z_star'], color='k', ls='--', lw=1, label=f'z* = {res["z_star"]:.1f} m')
    ax.axhline(Z_LO, color='orange', ls=':', lw=1, label=f'confirmation {Z_LO} m')
    ax.axhline(alarm_altitude_limit(), color='red', ls='-.', lw=1,
               label=f'no alarm possible above {alarm_altitude_limit():.2f} m')
    ax.set_xlabel('time (s)'); ax.set_ylabel('AGL (m)'); ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=2, loc='upper right')
    ax.set_title(f'S054 3D AGL over time, seed {MAIN_SEED} (terrain relief 0-{res["ter"].H.max():.0f} m)')
    _save(fig, out, 'agl_timeseries.png')


def alarm_altitude_limit():
    """Highest AGL at which a mine directly below can still raise an alarm."""
    zz = np.linspace(Z_AGL_MIN, Z_AGL_MAX, 50001)
    ok = pd3(0.0, zz, zz) > THETA_ALARM
    return float(zz[ok].max())


def plot_optimal_altitude(res, out):
    zs, e = res['zs'], res['eta']
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.5))
    a1.plot(zs, e, color='tab:blue')
    a1.axvline(res['z_star'], color='k', ls='--', label=f'z* = {res["z_star"]:.2f} m (bound)')
    a1.axvline(2.5, color='red', ls=':', label=f'card z* ~ 2.5 m (eta {eta(2.5):.2f} m^2)')
    a1.set_xlabel('AGL z (m)'); a1.set_ylabel('eta(z) = integral of P_d over footprint (m^2)')
    a1.set_title(f'Card efficiency eta(z): maximum {eta(res["z_star"]):.2f} m^2 at z = {res["z_star"]:.1f} m')
    a1.legend(fontsize=8); a1.grid(alpha=0.3)
    a2.plot(zs, r_gpr(zs), label='footprint radius r_gpr (m)')
    a2.plot(zs, [alarm_radius(z) for z in zs], label='alarm radius (single-step P_d > 0.5) (m)')
    a2b = a2.twinx()
    a2b.plot(zs, pd3(0.0, zs, zs), color='tab:red', label='P_d directly below')
    a2b.axhline(THETA_ALARM, color='tab:red', ls=':', lw=1)
    a2b.set_ylabel('P_d directly below', color='tab:red')
    a2.set_xlabel('AGL z (m)'); a2.set_ylabel('radius (m)'); a2.grid(alpha=0.3)
    a2.legend(fontsize=8, loc='upper right')
    a2.set_title(f'No alarm is possible above {alarm_altitude_limit():.2f} m AGL')
    _save(fig, out, 'optimal_altitude.png')


def plot_agl_error_hist(res, out):
    al = np.array(res['show']['tf'][0].agl_log)
    err = np.abs(al[:, 1] - al[:, 2])
    bound = VZ_MAX * TAU_Z / math.cos(math.radians(15))
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.hist(err, bins=60, color=COLORS['tf'])
    ax.axvline(bound, color='red', ls='--', label=f'card bound {bound:.2f} m')
    ax.axvline(err.max(), color='k', ls=':', label=f'max {err.max():.3f} m')
    ax.set_yscale('log'); ax.set_xlabel('|AGL - AGL_cmd| (m)'); ax.set_ylabel('steps')
    ax.set_title(f'Terrain-following AGL error, seed {MAIN_SEED} (mean {err.mean():.3f} m)')
    ax.legend(fontsize=8)
    _save(fig, out, 'agl_error_hist.png')


def plot_risk_maps(res, out):
    fig, axes = plt.subplots(1, 4, figsize=(22, 5.8))
    for ax, s_ in zip(axes, STRATS):
        m, r = res['show'][s_]
        ax.imshow(m.risk, origin='lower', extent=[0, 200, 0, 200], cmap=s2d.RISK_CMAP, vmin=0, vmax=1)
        tr = np.array(m.traj3)
        ax.plot(tr[:, 0], tr[:, 1], color='tab:blue', lw=0.4)
        st = r['mine_status']
        for code, col, mk in ((2, 'red', 'o'), (1, 'orange', '^'), (0, 'black', 'x')):
            k = st == code
            ax.plot(m.mines[k, 0], m.mines[k, 1], mk, color=col, ms=7, mfc='none' if code == 2 else col, mew=1.5)
        if m.detour_xy:
            d = np.array(m.detour_xy)
            ax.plot(d[:, 0], d[:, 1], '*', color='orange', ms=8, mec='k', mew=0.5)
        if len(m.fa_xy):
            ax.plot(m.fa_xy[m.fa_active, 0], m.fa_xy[m.fa_active, 1], '+', color='gold', ms=7)
        ax.set_title(f'{LABELS[s_]}\nC = {100 * r["coverage"]:.1f}%, confirmed {r["confirmed"]}/{r["n_mines"]}, '
                     f'missed {r["missed"]}, unresolved FA {r["fa_unresolved"]}', fontsize=9)
        ax.set_xlim(0, 200); ax.set_ylim(0, 200); ax.set_aspect('equal'); ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    fig.suptitle('Final risk maps: red o = confirmed, orange ^ = alarmed only, black x = missed, '
                 'gold + = unresolved false alarm, orange * = confirmation detour targets', fontsize=10)
    _save(fig, out, 'risk_maps.png')


def plot_coverage_time(res, out):
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for s_ in STRATS:
        m = res['show'][s_][0]
        cl = np.array(m.cov_log)
        ax.plot(cl[:, 0], 100 * cl[:, 1], color=COLORS[s_], label=LABELS[s_])
        for t in m.detour_t:
            ax.axvline(t, ymin=0, ymax=0.04, color=COLORS[s_], lw=0.8)
    ax.axhline(95, color='k', ls='--', lw=1)
    ax.set_xlabel('time (s)'); ax.set_ylabel('coverage completeness C (%)'); ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title('Coverage vs time (ticks at the bottom = confirmation detours)')
    _save(fig, out, 'coverage_time.png')


def plot_confirmation(res, out):
    m, r = res['show']['tfc']
    fig, ax = plt.subplots(figsize=(10, 4.2))
    cols = {2: 'red', 1: 'orange', 0: 'black'}
    k = np.arange(len(r['cum_pd']))
    ax.bar(k, r['cum_pd'], color=[cols[c] for c in r['mine_status']])
    ax.axhline(THETA_CONFIRM, color='k', ls='--', lw=1)
    ax.set_xticks(k, [f'M{i + 1}' for i in k], fontsize=8)
    ax.set_ylabel('cumulative P_d at mission end')
    ax.set_title(f'Terrain-following + confirmation, seed {MAIN_SEED}: red confirmed, black missed '
                 f'({r["detours"]} detours, {r["t_detour"]:.0f} s)')
    _save(fig, out, 'confirmation_panel.png')


def plot_mc(res, out):
    mc = res['mc']
    cols = STRATS + ['tf_narrow']
    fig, axes = plt.subplots(1, 4, figsize=(20, 4.5))
    for ax, (k, lab, sc) in zip(axes, (('missed', 'mines missed per field', 1), ('coverage', 'coverage C (%)', 100),
                                       ('time', 'mission time (s)', 1), ('fa_unresolved', 'unresolved false alarms', 1))):
        for i, s_ in enumerate(cols):
            v = sc * _arr(mc, s_, k)
            ax.bar(i, v.mean(), color=COLORS.get(s_, '#17BECF'), alpha=0.8)
            ax.scatter(np.full(len(v), i), v, color='k', s=8, zorder=3)
        ax.set_xticks(range(len(cols)), ['fixed\n+3 m', 'fixed\n+1 m', 'TF', 'TF +\nconfirm', 'TF\nnarrow'])
        ax.set_title(lab); ax.grid(axis='y', alpha=0.3)
    fig.suptitle(f'S054 3D Monte Carlo ({N_MC} fields; bars = mean, dots = fields)')
    _save(fig, out, 'mc_comparison.png')


def plot_sweeps(res, out):
    mc, zs = res['mc'], res['z_star']
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.5))
    alts = (zs,) + Z_SWEEP
    for s_, col in (('tf', COLORS['tf']), ('tfc', COLORS['tfc'])):
        mi = [_arr(mc, s_ if za == zs else (s_, za), 'missed').mean() for za in alts]
        a1.plot(alts, mi, 'o-', color=col, label=LABELS[s_])
    a1.axvline(alarm_altitude_limit(), color='red', ls=':', label='alarm limit')
    a1.set_xlabel('survey AGL (m)'); a1.set_ylabel('mines missed per field (mean)'); a1.grid(alpha=0.3)
    a1.legend(fontsize=8); a1.set_title('Survey altitude sweep')
    rels = (0.0, 2.5, RELIEF, 10.0)
    for s_ in ('fixed1', 'tf'):
        mi = [_arr(mc, s_ if rl == RELIEF else (s_, 'relief', rl), 'missed').mean() for rl in rels]
        a2.plot(rels, mi, 'o-', color=COLORS[s_], label=LABELS[s_])
    a2.set_xlabel('terrain relief (m)'); a2.set_ylabel('mines missed per field (mean)'); a2.grid(alpha=0.3)
    a2.legend(fontsize=8); a2.set_title('Terrain-relief sweep (z* = 1 m)')
    _save(fig, out, 'sweeps.png')


def save_animation(res, out):
    m, r = res['show']['tfc']
    ter = res['ter']
    tr = np.array(m.traj3)
    n = len(tr)
    step = int(np.ceil((n - 1) / (ANIM_FRAMES - 1)))
    n_frames = int(np.ceil((n - 1) / step)) + 1
    idx = [min(f * step, n - 1) for f in range(n_frames)]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 5.2), gridspec_kw={'width_ratios': [1, 1.3]})
    a1.imshow(ter.H, origin='lower', extent=[0, 200, 0, 200], cmap='Greys', alpha=0.5)
    a1.plot(m.mines[:, 0], m.mines[:, 1], 'x', color='k', ms=5)
    (ln,) = a1.plot([], [], color='tab:blue', lw=0.5)
    (dot,) = a1.plot([], [], 'o', color='red', ms=6)
    a1.set_xlim(0, 200); a1.set_ylim(0, 200); a1.set_aspect('equal')
    a2.set_xlim(0, 200); a2.set_ylim(-0.2, ter.H.max() + 3)
    a2.set_xlabel('y along strip (m)'); a2.set_ylabel('z (m)')
    (ground,) = a2.plot([], [], color='saddlebrown')
    (cmd,) = a2.plot([], [], 'k--', lw=0.8, label='z_cmd')
    (act,) = a2.plot([], [], color='tab:blue', lw=1.2, label='z')
    (d2,) = a2.plot([], [], 'o', color='red', ms=5)
    a2.legend(fontsize=8, loc='upper right')
    al = np.array(m.agl_log)
    ttl = fig.suptitle('')

    def update(f):
        k = idx[f]
        ln.set_data(tr[:k + 1, 0], tr[:k + 1, 1]); dot.set_data([tr[k, 0]], [tr[k, 1]])
        x = tr[k, 0]
        yy = np.linspace(0, 200, 201)
        ground.set_data(yy, ter.h(np.full_like(yy, x), yy))
        lo = max(0, k - 400)
        if k > 0:
            seg = al[lo:k]
            ys = tr[lo + 1:k + 1, 1]
            act.set_data(ys, seg[:, 3]); cmd.set_data(ys, seg[:, 4] + seg[:, 2])
        d2.set_data([tr[k, 1]], [tr[k, 2]])
        cl = np.array(m.cov_log)
        c = cl[np.searchsorted(cl[:, 0], k * DT, side='right') - 1, 1] if len(cl) else 0
        ttl.set_text(f'S054 3D TF + confirmation: t = {k * DT:.0f} s, C = {100 * c:.1f}%, '
                     f'side view at x = {x:.0f} m (last 80 s)')
        return ln, dot, ground, act, cmd, d2

    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    p = os.path.join(out, 'animation.gif')
    anim.save(p, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {p} ({n_frames} frames, every {step} steps of {DT} s)')


if __name__ == '__main__':
    res = run_simulation()
    report(res)
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_trajectory_3d(res, out)
    plot_agl_timeseries(res, out)
    plot_optimal_altitude(res, out)
    plot_agl_error_hist(res, out)
    plot_risk_maps(res, out)
    plot_coverage_time(res, out)
    plot_confirmation(res, out)
    plot_mc(res, out)
    plot_sweeps(res, out)
    save_animation(res, out)
