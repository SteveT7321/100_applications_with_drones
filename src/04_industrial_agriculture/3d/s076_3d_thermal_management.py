"""
S076 3D Thermal Management in a Hot, Height-Stratified Environment
==================================================================
The 2D S076 flies the 100 x 100 m lawnmower (20 strips, 5 m apart) at a fixed height in a
uniform 45 degC ambient. Here the ambient is a function of altitude: a superheated
surface layer (55 degC at the ground, -5 degC/m up to z_bl = 2 m) and a weak lapse rate
(0.0065 degC/m) above it. Altitude is a planned state in [1, 6] m: climbs and descents cost
extra motor power k_z v_z^2, and the motor and battery temperatures follow first-order
models driven by T_amb(z(t)).

Every strategy uses the 2D base's fastest safe speed law (above T_warn, throttle to the
speed whose steady state is T_max at the current ambient; cooling pause at T_max), so the
strategies differ only in their altitude plan:
  fixed_2m   fly at 2.0 m (the 2D baseline)
  fixed_1m   fly at 1.0 m (low inspection pass, inside the hot layer)       (added)
  alt_opt    card: climb once to z* = argmin T_m,inf(v0, z)
  mpc        card: before each strip choose the altitude that minimises
             strip time + w (peak T_m - T_warn)+ by simulating the strip ahead
The card case is run, then sweeps of the surface temperature and the boundary-layer depth
show when altitude planning changes the mission.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/3d/s076_3d_thermal_management.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, 'outputs', '04_industrial_agriculture', '3d',
                          's076_3d_thermal_management')

# ── Parameters (card) ─────────────────────────────────────────────────────────
FIELD = 100.0
STRIP_D = 5.0
V0 = 1.0
Z_MIN, Z_MAX = 1.0, 6.0
T_SURF = 55.0
G_BL = -5.0                      # degC/m inside the surface layer
Z_BL = 2.0
G_ATM = 0.0065                   # degC/m above the layer (standard lapse rate)
T_MAX, T_WARN = 85.0, 80.0
R_TH = 0.8
TAU = 40.0
TAU_COOL = 20.0
P_HOVER = 30.0
K_V = 15.0
K_Z = 20.0
VZ_MAX = 1.5
R_BAT = 0.3
TAU_BAT = 120.0
BETA_BAT = 0.005
T_BAT_NOM = 25.0
Q_MIN_RATIO = 0.80
ETA_ESC = 0.92
T_PAUSE = 30.0
DT = 0.1
# Kept from the 2D base
T_TURN = 3.0
V_MIN = 0.05
Z_2D = 2.0                       # the 2D base's height (its 45 degC ambient = T_amb(2 m))

# ── Added parameters (see README Deviations) ──────────────────────────────────
Z_LOW = 1.0                      # low inspection pass
Z_GRID = np.round(np.arange(Z_MIN, Z_MAX + 1e-9, 0.5), 3)   # MPC candidates
W_PEAK = 10.0                    # s per degC above T_warn in the MPC cost
W_SWEEP = [0.0, 10.0, 100.0, 1000.0]
W_CASES = [(55.0, 2.0), (60.0, 3.0), (58.0, 4.0), (63.0, 3.0)]   # (T_surf, z_bl) for the weight sweep
T_SURF_SWEEP = np.arange(55.0, 65.01, 1.0)
Z_BL_SWEEP = [2.0, 3.0, 4.0]
ANIM_DT = 30.0                   # s of mission per GIF frame

T_BAT_MAX = T_BAT_NOM + (1 - Q_MIN_RATIO) / BETA_BAT        # 65 degC
STRATS = ['fixed_2m', 'fixed_1m', 'alt_opt', 'mpc']
LABELS = {'fixed_2m': 'Fixed 2 m (2D baseline)', 'fixed_1m': 'Fixed 1 m (low pass)',
          'alt_opt': 'Altitude-optimised (z*)', 'mpc': 'Strip-ahead MPC'}
COLORS = {'fixed_2m': 'tab:grey', 'fixed_1m': 'tab:red', 'alt_opt': 'tab:blue', 'mpc': 'tab:green'}


# ── Environment and thermal model ─────────────────────────────────────────────
def t_amb(z, t_surf=T_SURF, z_bl=Z_BL):
    t_bl = t_surf + G_BL * z_bl
    return t_surf + G_BL * z if z < z_bl else t_bl - G_ATM * (z - z_bl)


def p_motor(vh, vz=0.0):
    return P_HOVER + K_V * vh ** 2 + K_Z * vz ** 2


def tm_inf(vh, z, env):
    return t_amb(z, *env) + R_TH * p_motor(vh)


def tb_inf(vh, z, env):
    return t_amb(z, *env) + R_BAT * p_motor(vh) / ETA_ESC


def v_safe(z, env):
    """Speed whose steady state is T_max at altitude z (the base's throttle-to-T_max law)."""
    return float(np.sqrt(max(0.0, (T_MAX - t_amb(z, *env)) / (R_TH * K_V) - P_HOVER / K_V)))


def heat(T, T_inf, h, tau):
    return T_inf + (T - T_inf) * np.exp(-h / tau)


def z_star(env, vh=V0):
    """Card: altitude in [z_min, z_max] that minimises T_m,inf (ties: lowest)."""
    zz = np.round(np.arange(Z_MIN, Z_MAX + 1e-9, 0.01), 3)
    vals = np.array([tm_inf(vh, z, env) for z in zz])
    return float(zz[int(np.argmin(vals))])


def z_safe_lo(env, vh=V0):
    zz = np.round(np.arange(Z_MIN, Z_MAX + 1e-9, 0.01), 3)
    ok = [z for z in zz if tm_inf(vh, z, env) <= T_MAX]
    return ok[0] if ok else np.nan


# ── Flight engine ─────────────────────────────────────────────────────────────
class Flight:
    def __init__(self, env, record=True):
        self.env = env
        self.record = record
        self.x, self.y, self.z = 0.0, 0.5 * STRIP_D, Z_2D
        self.tm = self.tb = t_amb(Z_2D, *env)
        self.clock = 0.0
        self.st = dict(n_pause=0, t_over=0.0, t_bat_over=0.0, t_throttled=0.0, t_climb=0.0,
                       n_climb=0, climb_m=0.0, energy=0.0, peak_tm=self.tm, peak_tb=self.tb)
        self.log = dict(t=[0.0], x=[self.x], y=[self.y], z=[self.z], tm=[self.tm], tb=[self.tb],
                        v=[0.0], mode=['start'])

    def copy(self):
        f = Flight.__new__(Flight)
        f.env, f.record = self.env, False
        f.x, f.y, f.z, f.tm, f.tb, f.clock = self.x, self.y, self.z, self.tm, self.tb, self.clock
        f.st = dict(self.st)
        f.log = None
        return f

    def _step(self, h, P, z_mid, mode, v, cool=False):
        ta = t_amb(z_mid, *self.env)
        if cool:
            self.tm = heat(self.tm, ta, h, TAU_COOL)
            self.tb = heat(self.tb, ta, h, TAU_BAT)
        else:
            self.tm = heat(self.tm, ta + R_TH * P, h, TAU)
            self.tb = heat(self.tb, ta + R_BAT * P / ETA_ESC, h, TAU_BAT)
            self.st['energy'] += P / ETA_ESC * h
        self.clock += h
        if self.tm > T_MAX + 1e-9:
            self.st['t_over'] += h
        if self.tb > T_BAT_MAX + 1e-9:
            self.st['t_bat_over'] += h
        self.st['peak_tm'] = max(self.st['peak_tm'], self.tm)
        self.st['peak_tb'] = max(self.st['peak_tb'], self.tb)
        assert Z_MIN - 1e-9 <= self.z <= Z_MAX + 1e-9, 'altitude bounds'
        if self.record:
            for k, val in (('t', self.clock), ('x', self.x), ('y', self.y), ('z', self.z),
                           ('tm', self.tm), ('tb', self.tb), ('v', v), ('mode', mode)):
                self.log[k].append(val)

    def pause(self):
        self.st['n_pause'] += 1
        for _ in range(int(round(T_PAUSE / DT))):
            self._step(DT, 0.0, self.z, 'pause', 0.0, cool=True)

    def climb(self, z_to):
        """Card f_trap (accelerate, then decelerate) with peak speed VZ_MAX: T = 2|dz|/v_max."""
        dz = z_to - self.z
        if abs(dz) < 1e-9:
            return
        T = 2 * abs(dz) / VZ_MAX
        n = int(np.ceil(T / DT - 1e-9))
        z0 = self.z
        f = lambda t: 0.5 * (t / (T / 2)) ** 2 if t <= T / 2 else 1 - 0.5 * ((T - t) / (T / 2)) ** 2
        t = 0.0
        self.st['n_climb'] += 1
        self.st['climb_m'] += abs(dz)
        for i in range(n):
            h = min(DT, T - t)
            z_new = z0 + dz * f(t + h)
            vz = (z_new - self.z) / h
            assert abs(vz) <= VZ_MAX + 1e-6, 'climb speed'
            zm = 0.5 * (self.z + z_new)
            self.z = z_new
            self._step(h, p_motor(0.0, vz), zm, 'climb', 0.0)
            t += h
        self.z = z_to
        self.st['t_climb'] += T

    def turn(self):
        for _ in range(int(round(T_TURN / DT))):
            self._step(DT, P_HOVER, self.z, 'turn', 0.0)

    def line(self, x1, y1):
        x0, y0 = self.x, self.y
        L = np.hypot(x1 - x0, y1 - y0)
        ux, uy = (x1 - x0) / L, (y1 - y0) / L
        done = 0.0
        while L - done > 1e-9:
            if self.tm >= T_MAX:
                self.pause()
                continue
            v = V0
            if self.tm >= T_WARN:
                v = min(V0, max(V_MIN, v_safe(self.z, self.env)))
            ds = min(v * DT, L - done)
            h = ds / v
            done += ds
            self.x, self.y = x0 + ux * done, y0 + uy * done
            if v < V0 - 1e-12:
                self.st['t_throttled'] += h
            self._step(h, p_motor(v), self.z, 'cruise', v)


def strip_ends(i):
    y = (i + 0.5) * STRIP_D
    return ((0.0, y), (FIELD, y)) if i % 2 == 0 else ((FIELD, y), (0.0, y))


N_STRIPS = int(np.ceil(FIELD / STRIP_D))


def fly_strip(f, i, z):
    f.climb(z)
    (_, _), (xe, ye) = strip_ends(i)
    f.line(xe, ye)
    if i + 1 < N_STRIPS:
        f.line(xe, ye + STRIP_D)
        f.turn()


def simulate(strategy, env=(T_SURF, Z_BL), w_peak=W_PEAK, record=True, z_fix=None):
    f = Flight(env, record)
    zs = []
    for i in range(N_STRIPS):
        if strategy == 'fixed_2m':
            z = Z_2D
        elif strategy == 'fixed_z':
            z = z_fix
        elif strategy == 'fixed_1m':
            z = Z_LOW
        elif strategy == 'alt_opt':
            z = z_star(env)
        else:
            best = None
            for zc in Z_GRID:
                g = f.copy()
                t0, pk0 = g.clock, g.tm
                g.st['peak_tm'] = g.tm
                fly_strip(g, i, float(zc))
                J = (g.clock - t0) + w_peak * max(0.0, g.st['peak_tm'] - T_WARN)
                key = (round(J, 9), abs(zc - f.z))
                if best is None or key < best[0]:
                    best = (key, float(zc))
            z = best[1]
        zs.append(z)
        fly_strip(f, i, z)
    out = dict(f.st, mission=f.clock, z_plan=zs,
               q_min=1 - BETA_BAT * max(0.0, f.st['peak_tb'] - T_BAT_NOM))
    if record:
        L = {k: np.array(v) for k, v in f.log.items()}
        assert np.all(np.diff(L['t']) > 0), 'time must advance'
        step = np.hypot(np.diff(L['x']), np.diff(L['y']))
        assert np.all(step <= V0 * np.diff(L['t']) + 1e-9), 'teleport / over-speed'
        assert np.all(L['tm'] >= min(t_amb(z, *env) for z in Z_GRID) - 1e-9), 'motor below ambient'
        out['log'] = L
    return out


def run_simulation():
    env = (T_SURF, Z_BL)
    res = dict(card={s: simulate(s, env) for s in STRATS})
    res['w_sweep'] = {(c, w): simulate('mpc', c, w_peak=w, record=False) for c in W_CASES for w in W_SWEEP}
    # is the deeper-layer gain altitude variation or just a different constant altitude?
    res['const_probe'] = {(ts, zb): {z: simulate('fixed_z', (ts, zb), record=False, z_fix=z)
                                     for z in (zb, Z_MAX)}
                          for ts, zb in ((63.0, 3.0), (63.0, 4.0))}
    hot = (T_SURF_SWEEP[-3], Z_BL)
    res['hot_env'] = hot
    res['hot'] = {s: simulate(s, hot) for s in STRATS}
    res['sweep'] = {(zb, float(ts)): {s: simulate(s, (float(ts), zb), record=False) for s in STRATS}
                    for zb in Z_BL_SWEEP for ts in T_SURF_SWEEP}
    return res


# ── Plots ─────────────────────────────────────────────────────────────────────
def plot_trajectories_3d(res, out_dir):
    fig = plt.figure(figsize=(15, 4.6))
    for j, s in enumerate(['fixed_2m', 'fixed_1m', 'alt_opt', 'mpc']):
        ax = fig.add_subplot(1, 4, j + 1, projection='3d')
        L = res['hot'][s]['log']
        k = slice(None, None, 50)
        ax.plot(L['x'], L['y'], L['z'], color='0.6', lw=0.4)
        sc = ax.scatter(L['x'][k], L['y'][k], L['z'][k], c=L['tm'][k], cmap='hot_r', vmin=45, vmax=86, s=3)
        xx, yy = np.meshgrid([0, FIELD], [0, FIELD])
        ax.plot_surface(xx, yy, np.full_like(xx, Z_BL, dtype=float), alpha=0.15, color='orange')
        ax.set_zlim(0, Z_MAX)
        ax.set_xlabel('x (m)', fontsize=7)
        ax.set_ylabel('y (m)', fontsize=7)
        ax.set_zlabel('z (m)', fontsize=7)
        ax.tick_params(labelsize=6)
        ax.view_init(elev=22, azim=-60)
        ax.set_title(f"{LABELS[s]}\n{res['hot'][s]['mission'] / 60:.2f} min, peak {res['hot'][s]['peak_tm']:.2f} °C",
                     fontsize=8)
    fig.colorbar(sc, ax=fig.axes, shrink=0.6, label='motor temperature (°C)')
    fig.suptitle(f'Hot case (T_surf = {res["hot_env"][0]:.0f} °C): 3D paths (dots coloured by motor temperature); '
                 'orange plane = top of the hot surface layer', fontsize=10, y=0.98)
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_time_series(res, out_dir):
    fig, axes = plt.subplots(3, 2, figsize=(15, 9), sharex='col')
    for c, case in enumerate(('card', 'hot')):
        for s in STRATS:
            L = res[case][s]['log']
            axes[0, c].plot(L['t'] / 60, L['z'], color=COLORS[s], lw=1.2, label=LABELS[s])
            axes[1, c].plot(L['t'] / 60, L['tm'], color=COLORS[s], lw=0.9)
            axes[2, c].plot(L['t'] / 60, L['tb'], color=COLORS[s], lw=1.2)
        axes[1, c].axhline(T_MAX, color='k', ls='--', lw=1)
        axes[1, c].axhline(T_WARN, color='orange', ls=':', lw=1.2)
        axes[2, c].axhline(T_BAT_MAX, color='k', ls='--', lw=1)
        ts = T_SURF if case == 'card' else res['hot_env'][0]
        axes[0, c].set_title(f'{"Card case" if case == "card" else "Hot case"}: T_surf = {ts:.0f} °C, z_bl = {Z_BL:.0f} m')
        axes[2, c].set_xlabel('time (min)')
    axes[0, 0].set_ylabel('altitude (m)')
    axes[1, 0].set_ylabel('motor T (°C)')
    axes[2, 0].set_ylabel('battery T (°C)')
    axes[0, 0].legend(fontsize=8, loc='center right')
    for ax in axes.ravel():
        ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'time_series.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_corridor(res, out_dir):
    zz = np.linspace(Z_MIN, Z_MAX, 501)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), sharey=True)
    for ax, zb in zip(axes, Z_BL_SWEEP):
        for ts, c in ((T_SURF, 'tab:blue'), (res['hot_env'][0], 'tab:red')):
            env = (ts, zb)
            ax.plot([tm_inf(V0, z, env) for z in zz], zz, color=c, lw=1.8, label=f'T_m,inf, T_surf {ts:.0f} °C')
            ax.plot([tb_inf(V0, z, env) for z in zz], zz, color=c, lw=1.2, ls='--', label=f'T_bat,inf, T_surf {ts:.0f} °C')
            ax.plot([t_amb(z, *env) for z in zz], zz, color=c, lw=0.8, ls=':', label=f'T_amb, T_surf {ts:.0f} °C')
        ax.axvline(T_MAX, color='k', ls='--', lw=1)
        ax.axvline(T_BAT_MAX, color='k', ls=':', lw=1)
        ax.axhline(Z_2D, color='tab:grey', lw=3, alpha=0.4)
        ax.set_xlabel('steady-state temperature at v0 (°C)')
        ax.set_title(f'surface layer depth z_bl = {zb:.0f} m')
        ax.grid(alpha=0.3)
    axes[0].set_ylabel('altitude (m)')
    axes[0].legend(fontsize=7, loc='upper right')
    plt.tight_layout()
    path = os.path.join(out_dir, 'safe_corridor.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_sweep(res, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6), sharey=True)
    for ax, zb in zip(axes, Z_BL_SWEEP):
        for s in STRATS:
            m = [res['sweep'][(zb, float(ts))][s]['mission'] / 60 for ts in T_SURF_SWEEP]
            ax.plot(T_SURF_SWEEP, m, 'o-', color=COLORS[s], ms=3, label=LABELS[s])
        ax.set_title(f'z_bl = {zb:.0f} m')
        ax.set_xlabel('surface temperature T_surf (°C)')
        ax.grid(alpha=0.3)
    axes[0].set_ylabel('mission time (min)')
    axes[0].legend(fontsize=8)
    fig.suptitle('Mission time vs surface temperature and hot-layer depth (all strategies never exceed T_max)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'sweep.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics(res, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(16, 4.2))
    x = np.arange(len(STRATS))
    for c, case in enumerate(('card', 'hot')):
        off = (c - 0.5) * 0.38
        vals = [[res[case][s]['mission'] / 60 for s in STRATS], [res[case][s]['t_throttled'] / 60 for s in STRATS],
                [res[case][s]['peak_tm'] for s in STRATS], [res[case][s]['q_min'] for s in STRATS]]
        for ax, v in zip(axes, vals):
            ax.bar(x + off, v, 0.38, color=[COLORS[s] for s in STRATS], alpha=0.9 if c == 0 else 0.5,
                   edgecolor='k', label='card case' if c == 0 else 'hot case')
    for ax, t in zip(axes, ('mission time (min)', 'time throttled below v0 (min)', 'peak motor T (°C)',
                            'min Q_avail / Q_nom')):
        ax.set_title(t, fontsize=10)
        ax.set_xticks(x)
        ax.set_xticklabels(['fixed 2 m', 'fixed 1 m', 'alt-opt', 'MPC'], fontsize=8)
    axes[2].set_ylim(75, 88)
    axes[3].set_ylim(0.75, 0.9)
    axes[3].axhline(Q_MIN_RATIO, color='k', ls='--', lw=1)
    axes[0].legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'metrics.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    logs = {s: res['hot'][s]['log'] for s in STRATS}
    t_end = max(L['t'][-1] for L in logs.values())
    n_frames = int(np.ceil(t_end / ANIM_DT)) + 1
    fig = plt.figure(figsize=(8, 5.5))
    ax = fig.add_subplot(111, projection='3d')
    xx, yy = np.meshgrid([0, FIELD], [0, FIELD])
    ax.plot_surface(xx, yy, np.full_like(xx, Z_BL, dtype=float), alpha=0.12, color='orange')
    lines = {s: ax.plot([], [], [], color=COLORS[s], lw=1, label=LABELS[s])[0] for s in STRATS}
    dots = {s: ax.plot([], [], [], 'o', color=COLORS[s], ms=5)[0] for s in STRATS}
    ax.set_xlim(0, FIELD)
    ax.set_ylim(0, FIELD)
    ax.set_zlim(0, Z_MAX)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.legend(fontsize=7, loc='upper left')

    def update(fr):
        t = min(fr * ANIM_DT, t_end)
        parts = []
        for s, L in logs.items():
            k = min(int(np.searchsorted(L['t'], t, side='right')) - 1, len(L['t']) - 1)
            sl = slice(0, k + 1, 10)
            lines[s].set_data(L['x'][sl], L['y'][sl])
            lines[s].set_3d_properties(L['z'][sl])
            dots[s].set_data([L['x'][k]], [L['y'][k]])
            dots[s].set_3d_properties([L['z'][k]])
            parts.append(f'{s} {L["tm"][k]:.0f}°C')
        ax.set_title(f'hot case, t = {t / 60:5.1f} min | ' + ', '.join(parts), fontsize=8)
        return list(lines.values()) + list(dots.values())

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=65)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Report ────────────────────────────────────────────────────────────────────
def report(res):
    env = (T_SURF, Z_BL)
    print('\n=== S076 3D Thermal Management — Results ===')
    print(f'  T_amb: {t_amb(0, *env):.2f} degC at 0 m, {t_amb(1, *env):.2f} at 1 m, {t_amb(2, *env):.2f} at 2 m, '
          f'{t_amb(6, *env):.4f} at 6 m (1 -> 6 m: {t_amb(1, *env) - t_amb(6, *env):.3f} degC; '
          f'2 -> 6 m: {t_amb(2, *env) - t_amb(6, *env):.3f} degC); dry adiabatic lapse rate g/c_p = '
          f'{9.81 / 1005:.4f} degC/m vs the card\'s {G_ATM} degC/m')
    print(f'  Steady state at v0: T_m,inf {tm_inf(V0, 1, env):.2f} (1 m), {tm_inf(V0, 2, env):.2f} (2 m), '
          f'{tm_inf(V0, 6, env):.3f} (6 m) degC; T_bat,inf {tb_inf(V0, 1, env):.2f} (1 m), '
          f'{tb_inf(V0, 2, env):.2f} (2 m), {tb_inf(V0, 6, env):.3f} (6 m) degC; T_bat,max {T_BAT_MAX:.1f} degC')
    print(f'  z_safe,lo = {z_safe_lo(env):.2f} m; z* = {z_star(env):.2f} m; v_safe at 1 m {v_safe(1, env):.3f} m/s, '
          f'at 2 m {v_safe(2, env):.3f} m/s')
    print(f'  Climb 2 -> 6 m: T = 2 dz / v_max = {2 * 4 / VZ_MAX:.3f} s, peak power {p_motor(0, VZ_MAX):.0f} W '
          f'(card t_climb = dz / v_max = {4 / VZ_MAX:.3f} s would need a peak speed of {2 * VZ_MAX:.1f} m/s)')
    for case in ('card', 'hot'):
        e = env if case == 'card' else res['hot_env']
        print(f'  [{case} case: T_surf {e[0]:.0f} degC, z_bl {e[1]:.0f} m; T_amb(2 m) {t_amb(2, *e):.1f} degC; '
              f'z_safe,lo {z_safe_lo(e):.2f} m]')
        print('    strategy                     mission (s)   min     pauses  peak T_m   time>T_max  peak T_bat  '
              'min Q/Qnom  throttled (s)  climbs (m)  energy (kJ)  strip altitudes')
        for s in STRATS:
            r = res[case][s]
            zs = sorted(set(r['z_plan']))
            print(f"    {LABELS[s]:<28} {r['mission']:9.1f}  {r['mission'] / 60:6.2f}  {r['n_pause']:5d}   "
                  f"{r['peak_tm']:6.2f}     {r['t_over']:6.1f}     {r['peak_tb']:6.2f}      {r['q_min']:.4f}"
                  f"      {r['t_throttled']:7.1f}      {r['climb_m']:6.2f}      {r['energy'] / 1000:7.2f}     "
                  + ', '.join(f'{z:.1f}' for z in zs))
        b = res[case]['fixed_2m']
        for s in ('fixed_1m', 'alt_opt', 'mpc'):
            r = res[case][s]
            print(f"    {s} vs 2D baseline: mission {r['mission'] - b['mission']:+.1f} s, peak T_m "
                  f"{r['peak_tm'] - b['peak_tm']:+.2f} degC, peak T_bat {r['peak_tb'] - b['peak_tb']:+.2f} degC")
    print(f'  [MPC weight sweep: mission, peak motor temperature and the strip altitudes chosen]')
    for c in W_CASES:
        for w in W_SWEEP:
            r = res['w_sweep'][(c, w)]
            print(f"    T_surf {c[0]:.0f} degC, z_bl {c[1]:.0f} m, w = {w:6.1f} s/degC: mission {r['mission']:.1f} s, "
                  f"peak T_m {r['peak_tm']:.2f} degC, altitudes " + ', '.join(f'{z:.1f}' for z in sorted(set(r['z_plan']))))
    print('  [Constant-altitude probe at T_surf 63 degC]')
    for (ts, zb), d in res['const_probe'].items():
        m = res['sweep'][(zb, ts)]['mpc']
        print(f"    z_bl {zb:.0f} m: fixed {zb:.1f} m {d[zb]['mission']:.1f} s, fixed {Z_MAX:.1f} m {d[Z_MAX]['mission']:.1f} s, "
              f"MPC {m['mission']:.1f} s (altitudes " + ', '.join(f'{z:.1f}' for z in sorted(set(m['z_plan']))) + ')')
    print('  [Sweep: mission time (min) by surface temperature; all runs: time above T_max]')
    t_over = max(r['t_over'] for d in res['sweep'].values() for r in d.values())
    for zb in Z_BL_SWEEP:
        print(f'    z_bl = {zb:.0f} m   T_surf: ' + ' '.join(f'{ts:6.0f}' for ts in T_SURF_SWEEP))
        for s in STRATS:
            print(f'      {s:<9}        ' + ' '.join(f"{res['sweep'][(zb, float(ts))][s]['mission'] / 60:6.2f}"
                                               for ts in T_SURF_SWEEP))
        gain = [res['sweep'][(zb, float(ts))]['fixed_2m']['mission'] - min(
            res['sweep'][(zb, float(ts))][s]['mission'] for s in ('alt_opt', 'mpc')) for ts in T_SURF_SWEEP]
        print(f'      best 3D gain vs 2D baseline (s): ' + ' '.join(f'{g:6.1f}' for g in gain))
        qb = [res['sweep'][(zb, float(ts))]['fixed_2m']['q_min'] for ts in T_SURF_SWEEP]
        qa = [res['sweep'][(zb, float(ts))]['alt_opt']['q_min'] for ts in T_SURF_SWEEP]
        print(f'      min Q/Qnom fixed 2 m:       ' + ' '.join(f'{q:6.3f}' for q in qb))
        print(f'      min Q/Qnom alt-opt:         ' + ' '.join(f'{q:6.3f}' for q in qa))
        nb = sum(q < Q_MIN_RATIO - 1e-12 for q in qb)
        na = sum(q < Q_MIN_RATIO - 1e-12 for q in qa)
        print(f'      battery-derating violations (Q < {Q_MIN_RATIO}): fixed 2 m {nb}/{len(qb)}, alt-opt {na}/{len(qa)}')
    print(f'  Longest time above T_max in any sweep run: {t_over:.1f} s')


if __name__ == '__main__':
    res = run_simulation()
    report(res)
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_trajectories_3d(res, out)
    plot_time_series(res, out)
    plot_corridor(res, out)
    plot_sweep(res, out)
    plot_metrics(res, out)
    save_animation(res, out)
