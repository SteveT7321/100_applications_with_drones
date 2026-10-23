"""
S012 3D Upgrade — Relay Pursuit
================================
Three battery-limited pursuers relay the chase of one evader.  The active
drone flies 3D pure pursuit; standby drones fly horizontally toward the
predicted evader position at handoff time while holding their home altitude
tier (ż = k_hold·(z_home − z)).  When the active drone drops below 20 % of E0
the standby with the best score w_E·E/E0 − w_d·d_3D/d_max takes over.

Evader (card: "exploit handoff moments by climbing or diving to an altitude
tier that is far from the next active pursuer"): flies a fixed horizontal
heading; at t = 0 and at every handoff it picks whichever of z = 1.0 m /
z = 5.5 m is farther from the newly active pursuer and climbs or dives there.

Comparison (card): altitude-tiered relay (tiers 1.0 / 2.5 / 4.5 m) vs flat
relay (all tiers 2.0 m), main geometry from the card plus a 100-seed Monte
Carlo over evader heading and start altitude (common random numbers).

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s012_3d_relay_pursuit.py
"""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from matplotlib.animation import FuncAnimation, PillowWriter
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'outputs', '01_pursuit_evasion', '3d', 's012_3d_relay_pursuit')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ───────────────────────────────────────────────
N_PURSUERS    = 3
E0            = 60.0    # J each (card)
P_HOVER       = 10.0    # W   hover power (not in card, see README)
K_E           = 0.4     # W·s²/m²  k (card)
K_CLIMB       = 0.6     # W·s²/m²  1.5·k on the climbing component (card ×1.5)
STANDBY_FRAC  = 0.5     # standby drones draw half power (card pseudocode)
HANDOFF_FRAC  = 0.20    # handoff below 20 % of E0
V_ACTIVE      = 5.0     # m/s
V_STANDBY     = 4.0     # m/s
V_EVADER      = 3.5     # m/s
VZ_EVADER_MAX = 2.0     # m/s  max climb/dive rate of the evader
K_EZ          = 2.0     # 1/s  evader altitude gain
Z_EVADE       = (1.0, 5.5)            # evader's dive / climb altitudes
Z_TIERS       = [1.0, 2.5, 4.5]       # tiered relay home altitudes (card)
Z_FLAT        = [2.0, 2.0, 2.0]       # flat relay (card)
Z_MIN, Z_MAX  = 0.5, 6.0
K_HOLD        = 2.0     # 1/s altitude-hold gain (card)
CAPTURE_R     = 0.15    # m (card pseudocode)
DT            = 0.02    # s (card)
MAX_TIME      = 40.0    # s (card pseudocode)
W_ENERGY      = 0.6
W_DIST        = 0.4
D_MAX         = 15.0    # m (card)
T_H_MIN       = 0.5     # s floor on the handoff-time estimate (2D fix)

# Card start geometry
INIT_P_XY  = np.array([[-4.0, 0.0], [0.0, 0.0], [4.0, 0.0]])
INIT_E     = np.array([0.0, 3.0, 2.5])
HEADING_E  = np.array([0.5, 1.0]) / np.linalg.norm([0.5, 1.0])   # card vel_e (xy part)

N_MC        = 100
MC_HEAD_DEG = 45.0       # heading offset ~ U(−45°, 45°)
MC_Z0       = (1.0, 5.0) # evader start altitude ~ U(1, 5) m

P_COLORS = ['crimson', 'darkorange', 'purple']
P_LEVEL  = P_HOVER + K_E * V_ACTIVE ** 2    # level-flight power at V_ACTIVE = 20 W


# ── Models ───────────────────────────────────────────────────

def power(vel, standby=False):
    """P = P_hover + k|v|² + (k_climb − k)·max(v_z, 0)²  (×0.5 on standby)."""
    p = P_HOVER + K_E * float(vel @ vel) + (K_CLIMB - K_E) * max(vel[2], 0.0) ** 2
    return p * (STANDBY_FRAC if standby else 1.0)


def handoff_score(energy_k, pos_k, pos_intercept):
    d = np.linalg.norm(pos_intercept - pos_k)
    return W_ENERGY * (energy_k / E0) - W_DIST * (d / D_MAX)


def evader_velocity(pos_e, heading, z_goal):
    vz = float(np.clip(K_EZ * (z_goal - pos_e[2]), -VZ_EVADER_MAX, VZ_EVADER_MAX))
    vxy = np.sqrt(V_EVADER ** 2 - vz ** 2)
    return np.array([vxy * heading[0], vxy * heading[1], vz])


def evade_altitude(z_active):
    """Card: climb/dive to the altitude far from the next active pursuer."""
    return max(Z_EVADE, key=lambda z: abs(z - z_active))


def standby_velocity(pos_k, z_home, target_xy):
    vz = float(np.clip(K_HOLD * (z_home - pos_k[2]), -V_STANDBY, V_STANDBY))
    dxy = target_xy - pos_k[:2]
    dn = np.linalg.norm(dxy)
    vxy_max = np.sqrt(max(V_STANDBY ** 2 - vz ** 2, 0.0))
    vxy = dxy / dn * min(vxy_max, dn / DT) if dn > 1e-9 else np.zeros(2)
    return np.array([vxy[0], vxy[1], vz])


# ── Simulation ───────────────────────────────────────────────

def run_simulation(tiers=Z_TIERS, heading=HEADING_E, z0_e=INIT_E[2], evader='card', log=True):
    """evader='card': climb/dive away from each new active drone;
    evader='level': holds its start altitude (control case)."""
    tiers = list(tiers)
    pos_p = np.column_stack([INIT_P_XY, tiers]).astype(float)
    pos_e = np.array([INIT_E[0], INIT_E[1], z0_e])
    energy = np.full(N_PURSUERS, E0)
    e_thr = HANDOFF_FRAC * E0
    active = 0                                     # card: drone 0 starts active
    pick_z = (lambda z_act: evade_altitude(z_act)) if evader == 'card' else (lambda z_act: z0_e)
    z_goal = pick_z(pos_p[active, 2])

    traj_e, traj_p, en_log, act_log, t_hist = [pos_e.copy()], [pos_p.copy()], [energy.copy()], [active], [0.0]
    handoffs = []   # (t, from, to, 3D gap, |dz| gap, evader z, new active z)
    tier_dev = 0.0  # max |z − z_home| of standby drones that have never been active
    been_active = {active}
    z_peak, max_drop = pos_e[2], 0.0   # largest realised evader descent
    outcome, end_t = 'timeout', MAX_TIME

    for step in range(1, int(round(MAX_TIME / DT)) + 1):
        t = step * DT
        v_e = evader_velocity(pos_e, heading, z_goal)
        T_h = max((energy[active] - e_thr) / P_LEVEL, T_H_MIN)
        p_int = pos_e + v_e * T_h

        # handoff (2D rule: only standbys at/above threshold; else wait until empty)
        if energy[active] < e_thr:
            cand = [k for k in range(N_PURSUERS) if k != active and energy[k] >= e_thr]
            if not cand and energy[active] <= 0:
                cand = [k for k in range(N_PURSUERS) if k != active and energy[k] > 0]
            if cand:
                new = max(cand, key=lambda k: (handoff_score(energy[k], pos_p[k], p_int), -k))
                gap = pos_e - pos_p[new]
                handoffs.append((t, active, new, float(np.linalg.norm(gap)), float(abs(gap[2])),
                                 float(pos_e[2]), float(pos_p[new, 2])))
                active = new
                been_active.add(active)
                z_goal = pick_z(pos_p[active, 2])
                v_e = evader_velocity(pos_e, heading, z_goal)
            elif energy[active] <= 0:
                outcome, end_t = 'all batteries empty', t
                break

        new_pos = pos_p.copy()
        for k in range(N_PURSUERS):
            if energy[k] <= 0:
                continue
            if k == active:
                d = pos_e - pos_p[k]
                dn = np.linalg.norm(d)
                vel = d / dn * min(V_ACTIVE, dn / DT) if dn > 1e-9 else np.zeros(3)
                standby = False
            else:
                vel = standby_velocity(pos_p[k], tiers[k], p_int[:2])
                standby = True
            de = power(vel, standby) * DT
            if de > energy[k]:                 # fly the fraction of the step the battery allows
                vel = vel * energy[k] / de
                de = energy[k]
            energy[k] -= de
            new_pos[k] = pos_p[k] + vel * DT
            new_pos[k, 2] = np.clip(new_pos[k, 2], Z_MIN, Z_MAX)
        pos_p = new_pos
        pos_e = pos_e + v_e * DT
        pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)

        for k in range(N_PURSUERS):
            if k not in been_active and energy[k] > 0:
                tier_dev = max(tier_dev, abs(pos_p[k, 2] - tiers[k]))
        z_peak = max(z_peak, pos_e[2])
        max_drop = max(max_drop, z_peak - pos_e[2])
        if log:
            traj_e.append(pos_e.copy()); traj_p.append(pos_p.copy())
            en_log.append(energy.copy()); act_log.append(active); t_hist.append(t)

        if np.linalg.norm(pos_p[active] - pos_e) < CAPTURE_R:
            outcome, end_t = 'captured', t
            break

    traj_e = np.array(traj_e)
    return dict(traj_e=traj_e, traj_p=np.array(traj_p), en_log=np.array(en_log),
                act_log=np.array(act_log), t_hist=np.array(t_hist), handoffs=handoffs,
                outcome=outcome, end_t=end_t, energy=energy.copy(), tiers=tiers,
                final_gap=float(np.linalg.norm(pos_p[active] - pos_e)),
                z_range=(float(traj_e[:, 2].min()), float(traj_e[:, 2].max())) if log else None,
                tier_dev=tier_dev, max_drop=float(max_drop))


EVADERS = ['card', 'level']
RELAYS = [('tiered', Z_TIERS), ('flat', Z_FLAT)]


def run_monte_carlo(n=N_MC):
    rng = np.random.default_rng(12)
    draws = [(rng.uniform(-MC_HEAD_DEG, MC_HEAD_DEG), rng.uniform(*MC_Z0)) for _ in range(n)]
    res = {(ev, name): [] for ev in EVADERS for name, _ in RELAYS}
    base = np.arctan2(HEADING_E[1], HEADING_E[0])
    for off, z0 in draws:     # common random numbers: same draw for every relay / evader
        a = base + np.radians(off)
        h = np.array([np.cos(a), np.sin(a)])
        for ev in EVADERS:
            for name, tiers in RELAYS:
                res[(ev, name)].append(run_simulation(tiers, h, z0, ev, log=False))
    return draws, res


# ── Plots ────────────────────────────────────────────────────

def _label(res):
    return (f"captured @ {res['end_t']:.2f} s" if res['outcome'] == 'captured'
            else f"{res['outcome']} @ {res['end_t']:.2f} s, {res['final_gap']:.2f} m short")


def plot_trajectories_3d(runs, names, out_dir):
    fig = plt.figure(figsize=(15, 7))
    for j, (res, name) in enumerate(zip(runs, names)):
        ax = fig.add_subplot(1, 2, j + 1, projection='3d')
        ax.view_init(elev=22, azim=-60)
        tp, te, act = res['traj_p'], res['traj_e'], res['act_log']
        for k in range(N_PURSUERS):
            ax.plot(tp[:, k, 0], tp[:, k, 1], tp[:, k, 2], color=P_COLORS[k], lw=1.3, alpha=0.85,
                    label=f'D{k} (tier {res["tiers"][k]} m)')
            ax.scatter(*tp[0, k], color=P_COLORS[k], s=40)
        # evader path coloured by the active pursuer (card output)
        for i in range(len(te) - 1):
            ax.plot(te[i:i+2, 0], te[i:i+2, 1], te[i:i+2, 2], color=P_COLORS[act[i + 1]], lw=3.0, alpha=0.5)
        ax.plot(te[:, 0], te[:, 1], te[:, 2], color='royalblue', lw=1.2, label='Evader (band = active drone)')
        ax.scatter(*te[0], color='blue', s=50)
        for h in res['handoffs']:
            i = int(round(h[0] / DT))
            ax.scatter(*tp[min(i, len(tp) - 1), h[2]], color=P_COLORS[h[2]], s=160, marker='*', zorder=8)
        if res['outcome'] == 'captured':
            ax.scatter(*te[-1], color='black', s=120, marker='X', zorder=9, label='Capture')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_zlim(Z_MIN - 0.2, Z_MAX + 0.2)
        ax.set_title(f'{name}: {_label(res)}\n★ = handoff ({len(res["handoffs"])})', fontsize=9)
        ax.legend(fontsize=7, loc='upper left')
    fig.suptitle('S012 3D Relay Pursuit — card geometry', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=110, bbox_inches='tight'); plt.close('all')
    print(f'Saved: {path}')


def _handoff_lines(ax, res):
    for h in res['handoffs']:
        ax.axvline(h[0], color='orange', ls=':', lw=1.5)


def plot_altitude_time(runs, names, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for ax, res, name in zip(axes, runs, names):
        for k in range(N_PURSUERS):
            ax.plot(res['t_hist'], res['traj_p'][:, k, 2], color=P_COLORS[k], lw=1.5,
                    label=f'D{k} (home z={res["tiers"][k]} m)')
            ax.axhline(res['tiers'][k], color=P_COLORS[k], ls=':', lw=0.8)
        ax.plot(res['t_hist'], res['traj_e'][:, 2], color='royalblue', lw=2.0, label='Evader')
        ax.axhline(Z_MIN, color='grey', ls='--', lw=0.8); ax.axhline(Z_MAX, color='grey', ls='--', lw=0.8)
        _handoff_lines(ax, res)
        ax.set_ylabel('z (m)'); ax.grid(True, alpha=0.3); ax.legend(fontsize=8, ncol=4)
        ax.set_title(f'{name}: {_label(res)} (orange = handoff)', fontsize=9)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S012 3D Relay Pursuit — Altitude vs Time', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all')
    print(f'Saved: {path}')


def plot_battery_time(runs, names, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for ax, res, name in zip(axes, runs, names):
        t, act = res['t_hist'], res['act_log']
        for k in range(N_PURSUERS):
            on = act == k
            ax.fill_between(t, 0, E0, where=on, color=P_COLORS[k], alpha=0.10, step='post')
            ax.plot(t, res['en_log'][:, k], color=P_COLORS[k], lw=2.0, ls=['-', '--', '-.'][k],
                    label=f'D{k}')
        ax.axhline(E0 * HANDOFF_FRAC, color='grey', ls='--', lw=1.2, label='20 % threshold')
        _handoff_lines(ax, res)
        ax.set_ylabel('Battery (J)'); ax.set_ylim(0, E0 * 1.05)
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8, ncol=5)
        ax.set_title(f'{name}: {_label(res)} (shading = active drone)', fontsize=9)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S012 3D Relay Pursuit — Battery vs Time', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'battery_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all')
    print(f'Saved: {path}')


def plot_monte_carlo(draws, mc, out_dir):
    fig, axes = plt.subplots(2, 3, figsize=(16, 9.5))
    names, cols = ['tiered', 'flat'], ['crimson', 'grey']
    z0 = np.array([d[1] for d in draws])
    for row, ev in zip(axes, EVADERS):
        ct = {n: np.array([r['end_t'] if r['outcome'] == 'captured' else np.nan for r in mc[(ev, n)]])
              for n in names}
        ax = row[0]
        both = ~np.isnan(ct['flat']) & ~np.isnan(ct['tiered'])
        ax.scatter(ct['flat'][both], ct['tiered'][both], s=18, color='crimson', alpha=0.7)
        hi = np.nanmax(np.r_[ct['flat'], ct['tiered']]) * 1.05
        ax.plot([0, hi], [0, hi], 'k--', lw=1)
        ax.set_xlabel('Flat relay capture time (s)'); ax.set_ylabel('Tiered relay capture time (s)')
        ax.set_title(f'{ev} evader: paired capture times ({both.sum()} seeds where both capture)', fontsize=9)
        ax.grid(True, alpha=0.3)
        ax = row[1]
        for n, c in zip(names, cols):
            gaps = [h[3] for r in mc[(ev, n)] for h in r['handoffs']]
            ax.hist(gaps, bins=np.linspace(0, 12, 25), alpha=0.55, color=c, label=f'{n} ({len(gaps)} handoffs)')
        ax.set_xlabel('3D distance new active → evader at handoff (m)'); ax.set_ylabel('Count')
        ax.set_title(f'{ev} evader: handoff gap (card metric)', fontsize=9)
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
        ax = row[2]
        for n, c, m in zip(names, cols, ['o', 's']):
            cap = ~np.isnan(ct[n])
            ax.scatter(z0[cap], ct[n][cap], color=c, marker=m, s=18, alpha=0.7,
                       label=f'{n}: captured ({cap.sum()})')
            ax.scatter(z0[~cap], np.full((~cap).sum(), 1.0), color=c, marker='x', s=30,
                       label=f'{n}: not captured ({(~cap).sum()}, plotted at 1 s)')
        ax.set_xlabel('Evader start altitude (m)'); ax.set_ylabel('Capture time (s)')
        ax.set_title(f'{ev} evader: capture time vs start altitude', fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    fig.suptitle(f'S012 3D — Flat vs Tiered Relay, {N_MC} seeds (common random numbers); '
                 'top: card evader (climb/dive at handoff), bottom: level evader (control)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo_flat_vs_tiered.png')
    plt.savefig(path, dpi=110, bbox_inches='tight'); plt.close('all')
    print(f'Saved: {path}')


def plot_score_heatmap(out_dir):
    """Card output: handoff score vs altitude offset and horizontal distance."""
    dxy = np.linspace(0, 15, 151)
    dz = np.linspace(0, 5, 101)
    DX, DZ = np.meshgrid(dxy, dz)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    for ax, frac in zip(axes, [1.0, 0.5]):
        S = W_ENERGY * frac - W_DIST * np.sqrt(DX ** 2 + DZ ** 2) / D_MAX
        cs = ax.contourf(DX, DZ, S, levels=20, cmap='viridis')
        c2 = ax.contour(DX, DZ, S, levels=[0.0, 0.2, 0.4], colors='white', linewidths=1)
        ax.clabel(c2, fmt='%.1f', fontsize=8)
        ax.set_xlabel('Horizontal distance to predicted intercept (m)')
        ax.set_title(f'Standby energy E = {frac:.0%} of E0', fontsize=9)
        fig.colorbar(cs, ax=ax, label='score')
    axes[0].set_ylabel('Altitude offset |Δz| (m)')
    fig.suptitle('S012 3D — Handoff Score  w_E·E/E0 − w_d·d_3D/d_max', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'score_heatmap.png')
    plt.savefig(path, dpi=110, bbox_inches='tight'); plt.close('all')
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    te, tp, en, t_hist, act = res['traj_e'], res['traj_p'], res['en_log'], res['t_hist'], res['act_log']
    T = len(t_hist)
    frames = list(range(0, T, 2))
    if frames[-1] != T - 1:
        frames.append(T - 1)
    fig = plt.figure(figsize=(12, 7))
    ax3d = fig.add_axes([0.03, 0.08, 0.62, 0.84], projection='3d'); ax3d.view_init(elev=22, azim=-60)
    ax_bat = fig.add_axes([0.72, 0.15, 0.25, 0.70])
    allp = np.vstack([te] + [tp[:, k] for k in range(N_PURSUERS)])
    lo, hi = allp.min(0) - 0.5, allp.max(0) + 0.5
    ax3d.set_xlim(lo[0], hi[0]); ax3d.set_ylim(lo[1], hi[1]); ax3d.set_zlim(Z_MIN - 0.2, Z_MAX + 0.2)
    ax3d.set_xlabel('X'); ax3d.set_ylabel('Y'); ax3d.set_zlabel('Z')
    lines = [ax3d.plot([], [], [], color=c, lw=1.3)[0] for c in P_COLORS]
    dots = [ax3d.plot([], [], [], '^', color=c, ms=8)[0] for c in P_COLORS]
    le, = ax3d.plot([], [], [], color='royalblue', lw=1.8)
    de, = ax3d.plot([], [], [], 'bs', ms=8)
    ax_bat.set_xlim(-0.5, N_PURSUERS - 0.5); ax_bat.set_ylim(0, E0)
    ax_bat.set_xticks(range(N_PURSUERS)); ax_bat.set_xticklabels([f'D{k}' for k in range(N_PURSUERS)])
    ax_bat.axhline(E0 * HANDOFF_FRAC, color='grey', ls='--', lw=0.8); ax_bat.set_ylabel('Battery (J)')
    bars = ax_bat.bar(range(N_PURSUERS), [E0] * N_PURSUERS, color=P_COLORS, alpha=0.7)
    title = fig.suptitle('', fontsize=10)

    def update(fi):
        si = frames[fi]
        for k in range(N_PURSUERS):
            lines[k].set_data(tp[:si+1, k, 0], tp[:si+1, k, 1]); lines[k].set_3d_properties(tp[:si+1, k, 2])
            dots[k].set_data([tp[si, k, 0]], [tp[si, k, 1]]); dots[k].set_3d_properties([tp[si, k, 2]])
            dots[k].set_markersize(13 if k == act[si] else 7)
            bars[k].set_height(en[si, k]); bars[k].set_edgecolor('black' if k == act[si] else 'none')
            bars[k].set_linewidth(2.0 if k == act[si] else 0)
        le.set_data(te[:si+1, 0], te[:si+1, 1]); le.set_3d_properties(te[:si+1, 2])
        de.set_data([te[si, 0]], [te[si, 1]]); de.set_3d_properties([te[si, 2]])
        title.set_text(f'S012 3D Relay Pursuit (tiered)  t={t_hist[si]:.2f}s  active D{act[si]}'
                       + ('  CAPTURED' if fi == len(frames) - 1 and res['outcome'] == 'captured' else ''))
        return lines + dots + [le, de, title] + list(bars)

    ani = FuncAnimation(fig, update, frames=len(frames), interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=15)); plt.close('all')
    print(f'Saved: {path}  ({len(frames)} frames)')


# ── Main ─────────────────────────────────────────────────────

def _report(name, res):
    print(f"{name}: {_label(res)}; handoffs {len(res['handoffs'])}; final energy "
          + ', '.join(f'D{k} {e:.1f} J' for k, e in enumerate(res['energy']))
          + f"; evader z {res['z_range'][0]:.2f}–{res['z_range'][1]:.2f} m; "
          f"max tier deviation of never-active standbys {res['tier_dev']:.3f} m; "
          f"largest evader descent {res['max_drop']:.2f} m")
    for h in res['handoffs']:
        print(f"    t={h[0]:.2f}s D{h[1]}->D{h[2]}: gap {h[3]:.2f} m (|dz| {h[4]:.2f} m), "
              f"evader z {h[5]:.2f}, new active z {h[6]:.2f}")


def _mc_summary(name, rs):
    cap = [r for r in rs if r['outcome'] == 'captured']
    ct = np.array([r['end_t'] for r in cap])
    gaps = np.array([h[3] for r in rs for h in r['handoffs']])
    dz = np.array([h[4] for r in rs for h in r['handoffs']])
    nh = np.array([len(r['handoffs']) for r in rs])
    empty = sum(r['outcome'] == 'all batteries empty' for r in rs)
    s = (f"{name:>6}: captured {len(cap)}/{len(rs)}, batteries empty {empty}, timeout "
         f"{len(rs) - len(cap) - empty}; capture time {ct.mean():.2f} ± {ct.std():.2f} s"
         if len(cap) else f"{name:>6}: captured 0/{len(rs)}")
    dives = sum(r['max_drop'] >= 1.0 for r in rs)
    solo = [r for r in rs if not r['handoffs']]
    solo_s = (f"; runs without handoff {len(solo)} (" + ', '.join(f"{r['outcome']} @ {r['end_t']:.2f} s" for r in solo)
              + ')') if solo else '; runs without handoff 0'
    print(s + f"; handoffs/run {nh.mean():.2f}; handoff gap 3D {gaps.mean():.2f} m "
          f"(|dz| {dz.mean():.2f} m) over {len(gaps)} handoffs; evader dives >= 1 m in {dives} runs; "
          f"max never-active tier deviation {max(r['tier_dev'] for r in rs):.3f} m; handoffs per run "
          f"{nh.min()}–{nh.max()}" + solo_s)


if __name__ == '__main__':
    print(f'S012 3D Relay Pursuit: E0={E0} J, P_level({V_ACTIVE} m/s)={P_LEVEL:.1f} W -> '
          f'{(1 - HANDOFF_FRAC) * E0 / P_LEVEL:.2f} s of active flight to the 20 % threshold')
    tiered = run_simulation(Z_TIERS)
    flat = run_simulation(Z_FLAT)
    names = ['Tiered relay (1.0/2.5/4.5 m)', 'Flat relay (all 2.0 m)']
    _report(names[0], tiered)
    _report(names[1], flat)

    print(f'\nMonte Carlo: {N_MC} seeds, heading offset U(±{MC_HEAD_DEG:.0f}°), start z U{MC_Z0} m, '
          'same draws for every relay and evader')
    draws, mc = run_monte_carlo()
    for ev in EVADERS:
        print(f' {ev} evader:')
        _mc_summary('tiered', mc[(ev, 'tiered')])
        _mc_summary('flat', mc[(ev, 'flat')])
        ct_t = np.array([r['end_t'] if r['outcome'] == 'captured' else np.inf for r in mc[(ev, 'tiered')]])
        ct_f = np.array([r['end_t'] if r['outcome'] == 'captured' else np.inf for r in mc[(ev, 'flat')]])
        both = np.isfinite(ct_t) & np.isfinite(ct_f)
        print(f'  paired: tiered faster {(ct_t < ct_f).sum()}, flat faster {(ct_f < ct_t).sum()}, '
              f'equal {(ct_t == ct_f).sum()}; only tiered captures {(np.isfinite(ct_t) & ~np.isfinite(ct_f)).sum()}, '
              f'only flat captures {(np.isfinite(ct_f) & ~np.isfinite(ct_t)).sum()}')
        if both.any():
            d = ct_t[both] - ct_f[both]
            print(f'  where both capture ({both.sum()}): tiered − flat = {d.mean():+.2f} ± {d.std():.2f} s')

    plot_trajectories_3d([tiered, flat], names, OUTPUT_DIR)
    plot_altitude_time([tiered, flat], names, OUTPUT_DIR)
    plot_battery_time([tiered, flat], names, OUTPUT_DIR)
    plot_monte_carlo(draws, mc, OUTPUT_DIR)
    plot_score_heatmap(OUTPUT_DIR)
    save_animation(tiered, OUTPUT_DIR)
    print('All outputs saved to:', OUTPUT_DIR)
