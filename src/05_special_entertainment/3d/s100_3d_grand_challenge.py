"""
S100 3D Upgrade — Grand Challenge: Multi-Domain Integration
==========================================================
The fixed S100 mission (Phase 1 aerial ball catching with the S094 predictive catch point,
Phase 2 two couriers with one package per trip and the card's nearest-neighbour split and battery
trigger, Phase 3 Bayesian search by the scout at 8 m, Phase 4 Hungarian assignment + PID with
anti-windup + ORCA-lite star formation; one battery model max(c_hover, c_fly |v|) for every
drone, landing failsafe, 80 m multi-hop links, 300 s budget) is given the 3D card's altitude
changes where altitude decides the outcome:

  * arena ceiling 60 m (card); the star is flown at 55 m: the card's z_show = 60 m is the ceiling
    itself and the PID overshoot leaves the arena (as in S100, the show is put 5 m below the ceiling);
  * Phase 4 altitude-stratified ascent (card): drone k tracks the smoothstep climb profile
    z_k(t) = z_start + (z_show - z_start) Phi(t / tau_k), tau_k = 1.5 dz / v_z,max + 3 k s,
    compared with the S100 direct PID ascent;
  * Phase 2 delivery waypoints in three altitude bands (5, 12, 20 m, card) and the card's
    heterogeneous climb cost c_climb,i = c_fly (m_i g / T_max,i) gamma_climb (gamma = 2.5),
    charged on every metre climbed, in the energy-cost nearest-neighbour split and in the battery
    triggers; the card's 3D formation tolerance (0.20 m) and ORCA radius (1.5 m).

Phase 1 and Phase 3 are flown exactly as in S100 (see the README for why).

2D baseline (same seeds, same throws, survivors and sensor draws; CRN): the S100 geometry (all
waypoints at 5 m, star at a fixed show altitude, direct ascent) with the same 3D energy model and
the same card tolerances, with the show altitude swept over 10-55 m. The exact S100 configuration
is also re-run through this script's mission loop and must reproduce S100 mission by mission.

Monte Carlo: 100 missions (S100: 200), reduced to keep the runtime below 8 min.

Usage:
    conda activate drones
    python src/05_special_entertainment/3d/s100_3d_grand_challenge.py
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
from scipy.optimize import linear_sum_assignment
from scipy.stats import binomtest

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    's100_2d', os.path.join(_HERE, '..', 's100_grand_challenge.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '05_special_entertainment', '3d', 's100_3d_grand_challenge'))

_norm = m2d._norm
DT, K_TICKS, T_HARD, T_TOTAL = m2d.DT, m2d.K_TICKS, m2d.T_HARD, m2d.T_TOTAL
PARKED, GOTO, PID, TRACK = m2d.PARKED, m2d.GOTO, m2d.PID, m2d.TRACK
NAMES, COL = m2d.NAMES, m2d.COL

# ── Card 3D parameters ────────────────────────────────────────────────────────
ARENA_Z3 = 60.0
Z_SHOW3 = 55.0              # card: 60 m = the 60 m ceiling itself; 5 m below it, as S100 did
WPS3 = np.array([[20.0, 70.0, 5.0], [75.0, 60.0, 12.0], [40.0, 40.0, 20.0]])
MASS = np.array([0.8, 1.2, 1.2, 0.9, 0.8])
THRUST = np.array([16.0, 20.0, 20.0, 14.0, 16.0])
G = 9.81
GAMMA_CLIMB = 2.5
CLIMB_F = MASS * G / THRUST * GAMMA_CLIMB            # c_climb,i / c_fly
V_ZMAX, DT_STAGGER = 3.0, 3.0
EPS_FORM3, R_SAFE3 = 0.20, 1.5
# ── Added ─────────────────────────────────────────────────────────────────────
N_SEEDS = 100
KAPPAS3 = [1.0, 1.5, 2.0, 3.0, 5.0]
ZSHOW_SWEEP = [10.0, 15.0, 25.0, 40.0, 55.0]
N_FACT = 50                       # missions per factorial cell (reduced for runtime)
FACT_KAPPAS = [1.25, 1.5, 1.75]
FACT_Z = [10.0, 25.0, 40.0, 55.0]
STRAT_F = 1.5                     # smoothstep peak slope: tau = 1.5 dz / v_z,max keeps v_z <= 3 m/s
GIF_EVERY = 15

CFG_S100 = dict(wps=m2d.DELIVERY_WPS, z_show=m2d.Z_SHOW, eps=m2d.EPS_FORM, r_safe=m2d.R_SAFE,
                strat=False, climb=False, arena_z=m2d.ARENA[2], nn='dist')
CFG_2D = dict(wps=m2d.DELIVERY_WPS, z_show=m2d.Z_SHOW, eps=EPS_FORM3, r_safe=R_SAFE3,
              strat=False, climb=True, arena_z=ARENA_Z3, nn='energy')
CFG_3D = dict(wps=WPS3, z_show=Z_SHOW3, eps=EPS_FORM3, r_safe=R_SAFE3, strat=True, climb=True,
              arena_z=ARENA_Z3, nn='energy')
CFG_3DD = dict(CFG_3D, strat=False)
LBL = {'2d': '2D baseline (S100 geometry, star 25 m)', '3d': '3D card (bands, star 55 m, stratified)',
       '3dd': '3D, direct ascent (bands, star 55 m)'}
VC = {'2d': 'k', '3d': '#d62728', '3dd': '#1f77b4'}


def star_for(z):
    a = 2 * np.pi * np.arange(5) / 5 - np.pi / 2
    return np.stack([50 + m2d.STAR_R * np.cos(a), 50 + m2d.STAR_R * np.sin(a), np.full(5, z)], 1)


def trip_energy(c, w, wps, climb):
    """Energy of one round trip pad -> waypoint -> pad in units of c_fly (card energy model)."""
    d = float(np.linalg.norm(wps[w] - m2d.PADS[c]))
    up = max(0.0, wps[w][2] - m2d.PADS[c][2])
    return 2 * d + (CLIMB_F[c + 1] * up if climb else 0.0)


def nn_plan3(wps, climb, by):
    """Card heuristic, on distance (S100) or on the card's 3D energy cost."""
    remaining = [0, 1, 2]
    plan = ([], [])
    while remaining:
        key = (lambda c, w: trip_energy(c, w, wps, climb)) if by == 'energy' else \
            (lambda c, w: float(np.linalg.norm(wps[w] - m2d.PADS[c])))
        _, w, c = min((key(c, w), w, c) for w in remaining for c in range(2))
        plan[c].append(w)
        remaining.remove(w)
    return plan


def orca_lite(pos, r_safe):
    diff = pos[:, :, None, :] - pos[:, None, :, :]
    d = _norm(diff)
    idx = np.arange(5)
    d[:, idx, idx] = np.inf
    act = d < r_safe
    mag = np.where(act, m2d.K_REP * (1.0 / d - 1.0 / r_safe) / d ** 2, 0.0)
    return np.sum((mag / d)[..., None] * diff, axis=2), act


def formation_step(pos, vel, goal, integ, e_prev, mp, r_safe, ki=m2d.KI):
    """S100 PID with anti-windup + ORCA-lite (radius r_safe), |v| <= 6 m/s, |a| <= 10 m/s^2."""
    err = goal - pos
    integ_new = integ + err * DT
    u = m2d.KP * err + ki * integ_new + m2d.KD * (err - e_prev) / DT
    sat = _norm(u) > m2d.V_FORM
    integ_new = np.where(sat[..., None], integ, integ_new)
    u = m2d.KP * err + ki * integ_new + m2d.KD * (err - e_prev) / DT
    integ = np.where(mp[..., None], integ_new, integ)
    e_prev = np.where(mp[..., None], err, e_prev)
    F, close = orca_lite(pos, r_safe)
    u = u + F
    n = _norm(u)
    u = u * np.where(n > m2d.V_FORM, m2d.V_FORM / np.maximum(n, 1e-12), 1.0)[..., None]
    v_p = m2d.rate_limit(vel, m2d.ground_barrier(pos, u))
    return pos + v_p * DT, v_p, integ, e_prev, close


def slice_p1(p1, ix):
    out = {}
    for k, v in p1.items():
        if isinstance(v, np.ndarray) and v.ndim >= 1 and len(v) == p1['S']:
            out[k] = v[ix]
        elif k == 'tracks_2ms':
            out[k] = [v[i] for i in ix]
        else:
            out[k] = v
    out['S'] = len(ix)
    return out


# ═════════════════════════════════════════════════════════════════════════════
# Mission loop: the S100 loop with the 3D changes marked "3D"
# ═════════════════════════════════════════════════════════════════════════════
def run_simulation(p1, seeds, cfg, kappa=1.0, log=False, phase2_only=False):
    S = len(seeds)
    wps, z_show, eps_form, r_safe = cfg['wps'], cfg['z_show'], cfg['eps'], cfg['r_safe']
    star = star_for(z_show)
    star_c = np.array([50.0, 50.0, z_show])
    plan = nn_plan3(wps, cfg['climb'], cfg['nn'])
    cfly, chov = m2d.C_FLY / kappa, m2d.C_HOVER / kappa
    cclimb = cfly * CLIMB_F if cfg['climb'] else np.zeros(5)           # 3D: climb surcharge
    pos = np.repeat(m2d.START[None], S, axis=0)
    vel = np.zeros((S, 5, 3))
    E = np.ones((S, 5))
    air = np.zeros((S, 5), bool)
    air[:, 4] = True
    mode = np.full((S, 5), PARKED)
    mode[:, 4] = GOTO
    target = pos.copy()
    vmax = np.zeros((S, 5))
    vmax[:, 4] = m2d.V_FORM
    phase = np.full(S, 2 if phase2_only else 1)
    t_end = np.full((S, 4), np.nan)
    if not phase2_only:
        air[:, 0] = True
        mode[:, 0] = TRACK
        k1_end, k_track = p1['k1_end'], p1['k_track']
        pos[:, 0] = p1['track'][:, 0]
    else:
        k1_end, k_track = np.zeros(S, int), np.zeros(S, int)
        t_end[:, 0] = 0.0
    cst = np.where(phase[:, None] == 2, 1, 0) * np.ones((S, 2), int)
    ptr = np.zeros((S, 2), int)
    timer = np.zeros((S, 2), int)
    delivered = np.zeros((S, 3), bool)
    aborted = np.zeros((S, 2), bool)
    deliv_p2 = np.zeros(S, int)
    surv = np.array([m2d.survivors_for(s) for s in seeds]) if not phase2_only else np.zeros((S, 2, 2))
    U = np.array([m2d.sensor_draws(s) for s in seeds]) if not phase2_only else None
    Braw = m2d.padded_priors(S) if not phase2_only else None
    Zb = np.ones((S, 2))
    found = np.zeros((S, 2), bool)
    dst = np.zeros(S, int)
    k3_start = np.zeros(S, int)
    p3_status = np.zeros(S, int)
    p3_pending = np.zeros(S, bool)
    assign = np.full((S, 5), -1)
    have = np.zeros((S, 5), bool)
    pend = np.zeros((S, 5), bool)
    integ = np.zeros((S, 5, 3))
    e_prev = np.zeros((S, 5, 3))
    goal = np.zeros((S, 5, 3))
    failsafe = np.zeros((S, 5), bool)
    land = np.zeros((S, 5), int)
    t_fs = np.full((S, 5), np.nan)
    z_start4 = np.zeros((S, 5))                           # 3D: stratified ascent
    t_rec4 = np.zeros((S, 5))
    E_p4 = np.full((S, 5), np.nan)
    E_p2end = np.full((S, 5), np.nan)
    vz_p4 = np.zeros(S)
    zmax_p4 = np.zeros(S)
    orca_steps = np.zeros(S, int)
    lk = pos.copy()
    minsep = np.full((S, 4), np.inf)
    iu = np.triu_indices(5, 1)
    if log:
        pos_log = np.zeros((S, K_TICKS + 1, 5, 3), np.float32)
        E_log = np.zeros((S, K_TICKS + 1, 5), np.float32)
        ph_log = np.zeros((S, K_TICKS + 1), np.int8)
        pos_log[:, 0], E_log[:, 0], ph_log[:, 0] = pos, E, phase
    for k in range(1, K_TICKS + 1):
        t = k * DT
        act = phase < 5
        if not act.any():
            break
        p_g, v_g = m2d.goto_step(pos, vel, target, vmax)
        new_p, new_v = pos.copy(), vel.copy()
        mg = act[:, None] & air & (mode == GOTO)
        new_p[mg], new_v[mg] = p_g[mg], v_g[mg]
        mp = act[:, None] & (mode == PID)
        if mp.any():
            g_eff = goal.copy()
            if cfg['strat']:                                  # 3D: card's stratified climb profile
                dz = z_show - z_start4
                tau = STRAT_F * np.abs(dz) / V_ZMAX + np.arange(5)[None, :] * DT_STAGGER
                xi = np.clip((t - t_rec4) / np.maximum(tau, 1e-9), 0.0, 1.0)
                g_eff[..., 2] = z_start4 + dz * (3 * xi ** 2 - 2 * xi ** 3)
            p_p, v_p, integ, e_prev, close = formation_step(pos, vel, g_eff, integ, e_prev, mp,
                                                            r_safe, cfg.get('ki', m2d.KI))
            orca_steps += np.any(close & mp[:, :, None], axis=(1, 2))
            new_v[mp], new_p[mp] = v_p[mp], p_p[mp]
            vz_p4 = np.maximum(vz_p4, np.where(mp, np.abs(v_p[..., 2]), 0.0).max(axis=1))
            zmax_p4 = np.maximum(zmax_p4, np.where(mp, p_p[..., 2], 0.0).max(axis=1))
        on_track = act & (k <= k_track) & (not phase2_only)
        if on_track.any():
            new_p[on_track, 0] = p1['track'][on_track, k]
            new_v[on_track, 0] = (p1['track'][on_track, k] - p1['track'][on_track, k - 1]) / DT
        moved = act[:, None] & air & (mode != TRACK)
        spd = _norm(new_v)
        acc = _norm(new_v - vel) / DT
        assert np.all(spd[moved] <= m2d.V_PHYS + 1e-9), 'speed limit'
        assert np.all(acc[moved] <= m2d.A_FLEET * (1 + 1e-6)), 'acceleration limit'
        assert np.all(new_p[..., 2] >= -1e-9) and np.all(new_p[..., 2] <= cfg['arena_z'] + 1e-9)
        assert np.all(new_p[..., :2] >= -1e-9) and np.all(new_p[..., :2] <= 100.0 + 1e-9)
        rate = np.maximum(chov, cfly * spd) + cclimb[None, :] * np.maximum(new_v[..., 2], 0.0)
        drain = act[:, None] & air & (mode != TRACK)
        E = E - np.where(drain, rate * DT, 0.0)
        if on_track.any():
            E[on_track, 0] = 1.0 - p1['e_used'][on_track, k] / kappa
        assert np.all(E >= -1e-12), 'flying on an empty battery'
        pos, vel = new_p, new_v
        hand = act & (k == k_track) & (not phase2_only)
        if hand.any():
            vel[hand, 0] = 0.0
            mode[hand, 0] = GOTO
            target[hand, 0] = pos[hand, 0] * [1.0, 1.0, 0.0]
            vmax[hand, 0] = m2d.V1
            land[hand, 0] = 2
        # Phase 3 sensing (S100, unchanged)
        if not phase2_only:
            sens = act & (dst == 1) & (np.abs(pos[:, 3, 2] - m2d.SCOUT_H) <= m2d.SENSOR_ALT_TOL)
            if sens.any():
                ix = np.where(sens)[0]
                dxy = pos[ix, 3, :2]
                in_true = _norm(dxy[:, None, :] - surv[ix]) <= m2d.SENSOR_R
                u_ = U[ix, k - k3_start[ix] - 1]
                hit = u_ < np.where(in_true, m2d.P_DET, m2d.P_FA)
                r = np.where(hit, m2d.P_DET / m2d.P_FA, (1 - m2d.P_DET) / (1 - m2d.P_FA))
                r = np.where(found[ix], 1.0, r)
                m2d.p3_observe(Braw, Zb, ix, dxy, r)
                new_hit = hit & in_true & ~found[ix]
                V = Braw[:, :, m2d.PADC:m2d.PADC + m2d.GRID_N, m2d.PADC:m2d.PADC + m2d.GRID_N]
                cmax = np.max(V, axis=(2, 3))[ix] / Zb[ix]
                cand = (cmax >= m2d.DET_THRESH / 9) & ~found[ix] & ~new_hit
                for jj, s in enumerate(ix):
                    for v_ in range(2):
                        if new_hit[jj, v_]:
                            found[s, v_] = True
                        elif cand[jj, v_]:
                            if m2d.box3(V[s, v_] / Zb[s, v_]).max() >= m2d.DET_THRESH:
                                found[s, v_] = True
                if k % 50 == 0:
                    for s in ix:
                        Braw[s] /= Zb[s][:, None, None]
                    Zb[ix] = 1.0
        arrived = (_norm(target - pos) <= m2d.ARRIVE_TOL) & (_norm(vel) <= m2d.ARRIVE_TOL) & (mode == GOTO)
        m_ = act[:, None] & (land == 2) & arrived
        air[m_], mode[m_], land[m_] = False, PARKED, 0
        vel[m_] = 0.0
        m_ = act[:, None] & (land == 1) & arrived
        target[m_, 2] = 0.0
        land[m_] = 2
        for c in range(2):
            d = c + 1
            n_trips = len(plan[c])
            m1 = act & (cst[:, c] == 1)
            if m1.any():
                more = ptr[:, c] < n_trips
                w = np.array([plan[c][min(int(x), n_trips - 1)] for x in ptr[:, c]]) if n_trips else np.zeros(S, int)
                d_out = _norm(wps[w] - pos[:, d])
                up = np.maximum(wps[w][:, 2] - pos[:, d, 2], 0.0)
                need = cfly * 2 * d_out + chov * m2d.TAU_DELIVER + cclimb[d] * up   # 3D
                go = m1 & more & (E[:, d] - need >= m2d.E_RESERVE)
                stop = m1 & ~go
                aborted[stop & more, c] = True
                cst[stop, c] = 6
                cst[go, c] = 2
                air[go, d], mode[go, d] = True, GOTO
                target[go, d] = wps[w[go]]
                vmax[go, d] = m2d.V_COURIER
            m2 = act & (cst[:, c] == 2) & arrived[:, d]
            cst[m2, c] = 3
            timer[m2, c] = int(round(m2d.TAU_DELIVER / DT))
            m3 = act & (cst[:, c] == 3)
            timer[m3, c] -= 1
            m3d = m3 & (timer[:, c] <= 0)
            if m3d.any():
                for s in np.where(m3d)[0]:
                    delivered[s, plan[c][ptr[s, c]]] = True
                ptr[m3d, c] += 1
                cst[m3d, c] = 4
                target[m3d, d] = m2d.PADS[c]
                arrived[m3d, d] = False
            m4 = act & (cst[:, c] == 4) & arrived[:, d]
            if m4.any():
                air[m4, d], mode[m4, d] = False, PARKED
                vel[m4, d] = 0.0
                again = m4 & (ptr[:, c] < n_trips) & (phase == 2)
                cst[again, c] = 5
                timer[again, c] = int(round(m2d.T_LOAD / DT))
                cst[m4 & ~again, c] = 6
            m5 = act & (cst[:, c] == 5)
            timer[m5, c] -= 1
            cst[m5 & (timer[:, c] <= 0), c] = 1
        if not phase2_only:
            ms = act & (dst == 1)
            if ms.any():
                ix = np.where(ms)[0]
                done_both = found[ix].all(axis=1)
                d_far = _norm(pos[ix, 3][:, None, :] - star[None]).max(axis=1)
                up = np.maximum(z_show - pos[ix, 3, 2], 0.0)
                low = E[ix, 3] - cfly * (d_far + z_show) - cclimb[3] * up < m2d.E_RESERVE   # 3D
                late = np.full(len(ix), t >= m2d.T_DEADLINE[2])
                stop = done_both | low | late
                st = ix[stop]
                p3_status[st] = np.where(done_both[stop], 1, np.where(low[stop], 2, 3))
                p3_pending[st] = True
                dst[st] = 2
                target[st, 3] = star_c
                go = ix[~stop]
                if len(go):
                    alpha = (~found).astype(float)
                    txy = m2d.p3_target(Braw, Zb, alpha, go, pos[go, 3, :2])
                    target[go, 3, :2] = txy
                    target[go, 3, 2] = m2d.SCOUT_H
        fs = (act[:, None] & air & ~failsafe & (mode != TRACK) & (E < cfly * pos[..., 2] + m2d.FAILSAFE_MARGIN))
        if fs.any():
            failsafe |= fs
            t_fs[fs] = t
            mode[fs] = GOTO
            target[fs] = pos[fs]
            vmax[fs] = 2.0
            land[fs] = 1
            have[fs] = False
            for s, d in zip(*np.where(fs)):
                if d in (1, 2):
                    cst[s, d - 1] = 6
                if d == 3 and dst[s] == 1:
                    p3_status[s], p3_pending[s] = 2, True
                if d == 3:
                    dst[s] = 4
        conn, dmat = m2d.connectivity(pos)
        lk = np.where(conn[..., None], pos, lk)
        pair_air = (air[:, :, None] | air[:, None, :])[:, iu[0], iu[1]]
        dpair = np.where(pair_air, dmat[:, iu[0], iu[1]], np.inf).min(axis=1)
        for p_ in range(4):
            mm = act & (phase == p_ + 1)
            minsep[mm, p_] = np.minimum(minsep[mm, p_], dpair[mm])
        if not phase2_only:
            m = act & (phase == 1) & (k == k1_end)
            phase[m] = 2
            t_end[m, 0] = t
            cst[m] = 1
        m = act & (phase == 2)
        done2 = m & np.all(cst == 6, axis=1) & conn[:, 1] & conn[:, 2]
        dl2 = m & ~done2 & (t >= m2d.T_DEADLINE[1])
        for mm in (done2, dl2):
            if mm.any():
                deliv_p2[mm] = delivered[mm].sum(axis=1)
                t_end[mm, 1] = t
                phase[mm] = 3 if not phase2_only else 5
                E_p2end[mm] = E[mm]
        if dl2.any():
            for c in range(2):
                fly = dl2 & np.isin(cst[:, c], [2, 3])
                cst[fly, c] = 4
                target[fly, c + 1] = m2d.PADS[c]
                cst[dl2 & np.isin(cst[:, c], [1, 5]), c] = 6
        if phase2_only:
            if log:
                pos_log[:, k], E_log[:, k], ph_log[:, k] = pos, E, phase
            continue
        st3 = done2 | dl2
        if st3.any():
            dst[st3] = 1
            k3_start[st3] = k
            air[st3, 3], mode[st3, 3] = True, GOTO
            vmax[st3, 3] = m2d.V_SCOUT
            ix3 = np.where(st3)[0]
            target[ix3, 3, :2] = m2d.p3_target(Braw, Zb, np.ones((S, 2)), ix3, pos[ix3, 3, :2])
            target[ix3, 3, 2] = m2d.SCOUT_H
        m = act & (phase == 3)
        ev = m & p3_pending & conn[:, 3]
        dl3 = m & ~ev & (t >= m2d.T_DEADLINE[2])
        st4 = ev | dl3
        if st4.any():
            t_end[st4, 2] = t
            phase[st4] = 4
            for s in np.where(st4)[0]:
                r_, c_ = linear_sum_assignment(_norm(lk[s][:, None, :] - star[None]))
                assign[s, r_] = c_
            goal[st4] = star[assign[st4]]
            pend[st4] = ~failsafe[st4]
        rec = act[:, None] & pend & conn
        if rec.any():
            pend &= ~rec
            have |= rec
            mode[rec], air[rec], land[rec] = PID, True, 0
            integ[rec] = 0.0
            e_prev[rec] = goal[rec] - pos[rec]
            z_start4[rec] = pos[rec][:, 2]
            t_rec4[rec] = t
            E_p4[rec] = E[rec]
            dst[rec[:, 3]] = 3
        m = act & (phase == 4)
        err4 = _norm(goal - pos).max(axis=1)
        conv = m & have.all(axis=1) & (err4 <= eps_form)
        dl4 = m & ~conv & (t >= T_HARD - 1e-9)
        for mm in (conv, dl4):
            t_end[mm, 3] = t
            phase[mm] = 5
        if log:
            pos_log[:, k], E_log[:, k], ph_log[:, k] = pos, E, phase
    assert np.all(phase == 5), 'mission loop did not terminate'
    t4 = t_end[:, 3]
    s1 = (p1['success'] & (t_end[:, 0] <= m2d.T_DEADLINE[0] + 1e-9)) if not phase2_only else np.zeros(S, bool)
    sc2 = np.where(t_end[:, 1] <= m2d.T_DEADLINE[1] + 1e-9, deliv_p2, 0)
    s3 = found.all(axis=1) & (p3_status == 1) & (t_end[:, 2] <= m2d.T_DEADLINE[2] + 1e-9)
    s4 = (t4 <= m2d.T_DEADLINE[3] + 1e-9) & ~np.isnan(t4)
    if not phase2_only:
        s4 = s4 & (_norm(goal - pos).max(axis=1) <= eps_form)
    score = (m2d.WEIGHTS[0] * s1 + sc2 * (m2d.WEIGHTS[1] / 3.0) + m2d.WEIGHTS[2] * s3
             + m2d.WEIGHTS[3] * s4 - m2d.OT_PENALTY * np.maximum(0.0, np.nan_to_num(t4, nan=T_HARD) - T_TOTAL))
    tau_all = STRAT_F * np.abs(z_show - z_start4) / V_ZMAX + np.arange(5)[None, :] * DT_STAGGER
    ramp_end = (t_rec4 + tau_all).max(axis=1) - t_end[:, 2]      # stratified profile ends
    out = dict(t_end=t_end, s1=s1, ramp_end=ramp_end, sc2=sc2, s3=s3, s4=s4, score=score, delivered=delivered,
               aborted=aborted, p3_status=p3_status, failsafe=failsafe, t_fs=t_fs, E_final=E,
               E_p4=E_p4, E_p2end=E_p2end, minsep=minsep, vz_p4=vz_p4, zmax_p4=zmax_p4, orca_steps=orca_steps,
               plan=plan)
    if log:
        out.update(pos_log=pos_log, E_log=E_log, ph_log=ph_log)
    return out


# ── Report helpers ────────────────────────────────────────────────────────────

def ms(x):
    x = np.asarray(x, float)
    return f'{np.nanmean(x):.2f} +- {np.nanstd(x, ddof=1):.2f}'


def mline(name, r):
    S = len(r['score'])
    first_fs = []
    for s in range(S):
        f = np.where(r['failsafe'][s])[0]
        if len(f):
            first_fs.append(NAMES[f[np.argmin(r['t_fs'][s, f])]])
    who = {n: first_fs.count(n) for n in NAMES if first_fs.count(n)}
    return (f'  {name:<46} P1 {100 * r["s1"].mean():5.1f}%  P2 3/3 {100 * np.mean(r["sc2"] == 3):5.1f}%  '
            f'P3 {100 * r["s3"].mean():5.1f}%  P4 {100 * r["s4"].mean():5.1f}%  perfect '
            f'{int(np.sum(np.abs(r["score"] - 10) < 1e-9)):3d}/{S}  score {ms(r["score"])}; first failsafe '
            f'landing: {who if who else "none"}')


def paired(a, b, na, nb):
    w, l = int(np.sum(a > b + 1e-9)), int(np.sum(b > a + 1e-9))
    p = binomtest(w, w + l, 0.5).pvalue if w + l else 1.0
    return f'{na} vs {nb}: {na} better {w}, ties {len(a) - w - l}, {nb} better {l} (sign test p = {p:.2e})'


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_phase2(p2, out_dir):
    fig = plt.figure(figsize=(18, 5.5))
    ax = fig.add_subplot(1, 3, 1, projection='3d')
    for c, ls in ((0, '-'), (1, '--')):
        for w in p2['bands']['plan'][c]:
            P = np.vstack([m2d.PADS[c], WPS3[w]])
            ax.plot(P[:, 0], P[:, 1], P[:, 2], ls, color=COL[NAMES[c + 1]], lw=2,
                    label=f'courier {NAMES[c + 1]}' if w == p2['bands']['plan'][c][0] else None)
    ax.scatter(WPS3[:, 0], WPS3[:, 1], WPS3[:, 2], c=['#2ca02c', '#1f77b4', '#9467bd'], s=80,
               marker='D', label='waypoints 5 / 12 / 20 m')
    ax.scatter(m2d.PADS[:, 0], m2d.PADS[:, 1], m2d.PADS[:, 2], c='k', marker='s', s=40, label='pads')
    ax.set_xlim(0, 100); ax.set_ylim(0, 100); ax.set_zlim(0, 25)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_title('Phase 2 routes, altitude bands (card)')
    ax.legend(fontsize=7)
    ax = fig.add_subplot(1, 3, 2)
    keys = list(p2)
    x = np.arange(len(keys))
    for j, d in enumerate((1, 2)):
        ax.bar(x + (j - 0.5) * 0.38, [100 * p2[k]['E_p2end'][0, d] for k in keys], 0.38,
               color=COL[NAMES[d]], label=f'{NAMES[d]} battery after Phase 2')
    ax.axhline(100 * m2d.E_RESERVE, color='k', ls=':', label='15 % reserve')
    ax.set_xticks(x); ax.set_xticklabels(keys, fontsize=8)
    ax.set_ylabel('battery (%)'); ax.set_title('Courier battery after Phase 2 (x1)')
    ax.legend(fontsize=7)
    ax = fig.add_subplot(1, 3, 3)
    ax.bar(np.arange(5), CLIMB_F + 1.0, color=[COL[n] for n in NAMES])
    ax.set_xticks(np.arange(5)); ax.set_xticklabels(NAMES)
    ax.set_ylabel('climb cost per metre / c_fly')
    ax.set_title('Card climb cost 1 + (m g / T_max) x 2.5')
    fig.subplots_adjust(wspace=0.35)
    path = os.path.join(out_dir, 'phase2_bands.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude_battery(M, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    ax = axes[0]
    for kap, mk in ((1.0, 'o-'), (3.0, 's--')):
        ax.plot(ZSHOW_SWEEP, [100 * M[('z', z, kap)]['s4'].mean() for z in ZSHOW_SWEEP], mk,
                color='k', label=f'P4 formed, x{kap:g}')
        ax.plot(ZSHOW_SWEEP, [10 * M[('z', z, kap)]['score'].mean() for z in ZSHOW_SWEEP], mk,
                color='#ff7f0e', label=f'mean score x 10, x{kap:g}')
    ax.axvline(25, color='grey', ls=':', label='S100 show altitude')
    ax.axvline(Z_SHOW3, color='#d62728', ls=':', label='3D show altitude')
    ax.set_xlabel('show altitude z_show (m), 2D geometry, direct ascent')
    ax.set_ylabel('% of missions')
    ax.set_title('2D baseline: show-altitude sweep')
    ax.legend(fontsize=7)
    ax = axes[1]
    for key in ('2d', '3d', '3dd'):
        ax.plot(KAPPAS3, [100 * M[(key, k)]['s4'].mean() for k in KAPPAS3], 'o-', color=VC[key],
                label=LBL[key])
    ax.set_xscale('log'); ax.set_xticks(KAPPAS3); ax.set_xticklabels([f'x{k:g}' for k in KAPPAS3])
    ax.minorticks_off()
    ax.set_xlabel('battery capacity multiplier'); ax.set_ylabel('Phase 4 formed (%)')
    ax.set_title('Phase 4 success vs battery')
    ax.legend(fontsize=7)
    ax = axes[2]
    for key in ('2d', '3d', '3dd'):
        ax.plot(KAPPAS3, [M[(key, k)]['score'].mean() for k in KAPPAS3], 'o-', color=VC[key],
                label=LBL[key])
    ax.set_xscale('log'); ax.set_xticks(KAPPAS3); ax.set_xticklabels([f'x{k:g}' for k in KAPPAS3])
    ax.minorticks_off()
    ax.set_xlabel('battery capacity multiplier'); ax.set_ylabel('mean mission score')
    ax.set_title('Mission score vs battery')
    ax.legend(fontsize=7)
    fig.tight_layout()
    path = os.path.join(out_dir, 'altitude_battery.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_phase4(Lg, M, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    for ax, key in ((axes[0], '3d'), (axes[1], '3dd')):
        r = Lg[key]
        ph = r['ph_log'][0]
        t = np.arange(len(ph)) * DT
        m = ph >= 4
        for d in range(5):
            ax.plot(t[m], r['pos_log'][0, m, d, 2], color=COL[NAMES[d]], label=f'drone {NAMES[d]}')
        ax.axhline(Z_SHOW3, color='grey', ls=':')
        ax.set_xlabel('mission time (s)'); ax.set_ylabel('z (m)')
        ax.set_title(f'{LBL[key]}\nmission 0, x5: Phase 4 altitude')
        ax.legend(fontsize=7)
    ax = axes[2]
    data = [M[('3d', 5.0)]['minsep'][:, 3], M[('3dd', 5.0)]['minsep'][:, 3], M[('2d', 5.0)]['minsep'][:, 3]]
    ax.boxplot(data, tick_labels=['3D stratified', '3D direct', '2D baseline'])
    ax.axhline(R_SAFE3, color='r', ls='--', label='ORCA radius 1.5 m')
    ax.set_ylabel('Phase 4 minimum separation (m)')
    ax.set_title(f'Phase 4 minimum separation, x5, {N_SEEDS} missions')
    ax.legend(fontsize=7)
    fig.tight_layout()
    path = os.path.join(out_dir, 'phase4_ascent.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _arena(ax):
    ax.set_xlim(0, 100); ax.set_ylim(0, 100); ax.set_zlim(0, ARENA_Z3)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')


def plot_mission_3d(Lg, out_dir):
    fig = plt.figure(figsize=(15, 7))
    for i, key in enumerate(('2d', '3d')):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        r = Lg[key]
        n = int(np.argmax(r['ph_log'][0] == 5)) or r['pos_log'].shape[1] - 1
        for d in range(5):
            P = r['pos_log'][0, :n + 1, d]
            ax.plot(P[:, 0], P[:, 1], P[:, 2], color=COL[NAMES[d]], lw=1, label=f'drone {NAMES[d]}')
        ax.scatter(WPS3[:, 0] if key == '3d' else m2d.DELIVERY_WPS[:, 0],
                   WPS3[:, 1] if key == '3d' else m2d.DELIVERY_WPS[:, 1],
                   WPS3[:, 2] if key == '3d' else m2d.DELIVERY_WPS[:, 2], c='g', marker='D', s=40,
                   label='waypoints')
        st = star_for(Z_SHOW3 if key == '3d' else m2d.Z_SHOW)
        ax.scatter(st[:, 0], st[:, 1], st[:, 2], c='gold', marker='*', s=120, label='star tips')
        _arena(ax)
        ax.set_title(f'{LBL[key]}, mission 0, battery x5')
        ax.legend(fontsize=7)
    fig.tight_layout()
    path = os.path.join(out_dir, 'mission_3d.png')
    fig.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(r, out_dir):
    ph = r['ph_log'][0]
    n_end = int(np.argmax(ph == 5)) if np.any(ph == 5) else len(ph) - 1
    idx = [min(i * GIF_EVERY, n_end) for i in range(math.ceil(n_end / GIF_EVERY) + 1)]
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')

    def update(fi):
        k = idx[fi]
        ax.cla()
        _arena(ax)
        P = r['pos_log'][0]
        for d in range(5):
            ax.plot(P[:k + 1, d, 0], P[:k + 1, d, 1], P[:k + 1, d, 2], color=COL[NAMES[d]], lw=0.8)
            ax.scatter(*P[k, d], color=COL[NAMES[d]], s=30)
        st = star_for(Z_SHOW3)
        ax.scatter(st[:, 0], st[:, 1], st[:, 2], c='gold', marker='*', s=60)
        E = r['E_log'][0, k]
        ax.set_title(f'S100 3D (card), mission 0, x5: t = {k * DT:5.1f} s, phase {min(int(ph[k]), 4)}\n'
                     'battery ' + ' '.join(f'{NAMES[d]} {100 * E[d]:3.0f}%' for d in range(5)),
                     fontsize=9)
        return []

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=67, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=15, dpi=60)
    plt.close(fig)
    print(f'Saved: {path}  ({len(idx)} frames, every {GIF_EVERY} ticks up to tick {n_end})')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    seeds = list(range(N_SEEDS))
    print('=== S100 3D Grand Challenge ===')
    print('Card climb cost factors (m g / T_max) x 2.5: ' + ', '.join(
        f'{NAMES[i]} {CLIMB_F[i]:.3f}' for i in range(5)) + ' -> climbing costs '
        + ', '.join(f'{1 + CLIMB_F[i]:.3f}' for i in range(5)) + ' x c_fly per metre')
    hv = np.sqrt((THRUST / MASS) ** 2 - G ** 2)
    print('Thrust-to-weight T/(m g): ' + ', '.join(f'{NAMES[i]} {THRUST[i] / (MASS[i] * G):.2f}' for i in range(5))
          + f'; catcher A: T/m = {THRUST[0] / MASS[0]:.1f} m/s^2, horizontal {hv[0]:.1f} m/s^2, up '
          f'{THRUST[0] / MASS[0] - G:.1f} m/s^2')
    # card checks
    print(f'Card ascent: smoothstep peak slope 1.5 -> with tau = dz / v_z,max the peak climb rate is '
          f'1.5 x {V_ZMAX:.0f} = {1.5 * V_ZMAX:.1f} m/s; used tau = 1.5 dz / v_z,max + 3 k s')
    for z in (25.0, Z_SHOW3, 60.0):
        print(f'  energy for B (x1) to climb from 2 m to {z:.0f} m: {100 * m2d.C_FLY * (1 + CLIMB_F[1]) * (z - 2):.1f} % '
              f'with the card climb cost, {100 * m2d.C_FLY * (z - 2):.1f} % without; landing reserve '
              f'c_fly z + 3 % = {100 * (m2d.C_FLY * z + m2d.FAILSAFE_MARGIN):.1f} %')
    rs, kap = 3.0, 0.4
    h = np.linspace(0, 8, 801)
    rr = np.sqrt(np.maximum((rs + kap * h) ** 2 - h ** 2, 0.0))
    print(f'Card Phase 3 cone |x - p_D| <= r_s + kappa |dz|: horizontal footprint radius {rr[0]:.2f} m '
          f'at dz = 0, largest {rr.max():.2f} m at dz = {h[np.argmax(rr)]:.2f} m, zero beyond dz = '
          f'{h[np.where(rr > 0)[0].max()]:.2f} m (the card says flying higher widens the cone)')

    print(f'\nPhase 1 (S094 predictive catch point, as S100), {N_SEEDS} missions ...')
    balls = m2d.run_phase1('pcp', seeds)
    p1 = m2d.assemble_phase1(balls)
    print(f'  Phase 1 success {int(p1["success"].sum())}/{N_SEEDS}, catcher battery used '
          f'{100 * np.mean(p1["e_used"][np.arange(N_SEEDS), p1["k_track"]]):.2f} % (mean)')

    # reproduction: this script's loop with the S100 configuration == S100's loop
    r_ref = m2d.run_mission(p1, seeds, kappa=1.0)
    r_me = run_simulation(p1, seeds, CFG_S100, kappa=1.0)
    same = np.array_equal(r_ref['score'], r_me['score']) and np.array_equal(r_ref['t_end'], r_me['t_end'], equal_nan=True)
    print(f'Reproduction: S100 configuration through this loop vs the S100 loop: scores and phase '
          f'end times identical in all {N_SEEDS} missions: {same}; S100 x1 score {ms(r_ref["score"])}')
    assert same

    for strat in (False, True):
        r60 = run_simulation(p1, seeds, dict(CFG_3D, z_show=60.0, arena_z=1000.0, strat=strat),
                             kappa=5.0)
        print(f'Card show altitude 60 m with the ceiling lifted (x5, '
              f'{"stratified" if strat else "direct"}): highest Phase 4 altitude '
              f'{np.max(r60["zmax_p4"]):.3f} m, above the 60 m ceiling in '
              f'{int(np.sum(r60["zmax_p4"] > ARENA_Z3))}/{N_SEEDS} missions')
    print('\nPhase 2 on its own (x1, deterministic):')
    p2 = {}
    for name, w in (('flat 5 m', m2d.DELIVERY_WPS),
                    ('flat 12 m', np.column_stack([m2d.DELIVERY_WPS[:, :2], np.full(3, 12.0)])),
                    ('flat 20 m', np.column_stack([m2d.DELIVERY_WPS[:, :2], np.full(3, 20.0)])),
                    ('bands', WPS3)):
        cfg = dict(CFG_3D, wps=w)
        p2[name] = run_simulation(None, [0], cfg, kappa=1.0, phase2_only=True)
        r = p2[name]
        print(f'  waypoints {name:9s}: plan B {r["plan"][0]} C {r["plan"][1]} (distance NN plan '
              f'{nn_plan3(w, False, "dist")}); delivered {int(r["sc2"][0])}/3, aborted B/C '
              f'{r["aborted"][0].tolist()}, battery left B {100 * r["E_p2end"][0, 1]:.1f} %, C '
              f'{100 * r["E_p2end"][0, 2]:.1f} %, Phase 2 ends at {r["t_end"][0, 1]:.1f} s')

    print(f'\nMissions ({N_SEEDS} seeds, common random numbers):')
    M = {}
    for key, cfg in (('2d', CFG_2D), ('3d', CFG_3D), ('3dd', CFG_3DD)):
        for k in KAPPAS3:
            M[(key, k)] = run_simulation(p1, seeds, cfg, kappa=k)
            print(mline(f'{LBL[key]}, x{k:g}', M[(key, k)]))
    print('\n2D baseline show-altitude sweep (S100 geometry, direct ascent, 3D energy model):')
    for z in ZSHOW_SWEEP:
        for k in (1.0, 3.0):
            cfg = dict(CFG_2D, z_show=z)
            M[('z', z, k)] = (M[('2d', k)] if z == m2d.Z_SHOW else run_simulation(p1, seeds, cfg, kappa=k))
            r = M[('z', z, k)]
            print(mline(f'z_show {z:4.0f} m, x{k:g}', r))
            print(f'      B battery at Phase 4 start {ms(100 * r["E_p4"][:, 1])} %, at the end '
                  f'{ms(100 * r["E_final"][:, 1])} %; P3 {100 * r["s3"].mean():.1f} %')
    print(f'\nShow altitude x waypoint geometry x battery factorial (direct ascent, first {N_FACT} '
          f'missions, same seeds):')
    fseeds = seeds[:N_FACT]
    p1f = slice_p1(p1, list(range(N_FACT)))
    F = {}
    for geo, wps in (('2D waypoints (5 m)', m2d.DELIVERY_WPS), ('bands (5/12/20 m)', WPS3)):
        for k in FACT_KAPPAS:
            row = []
            for z in FACT_Z:
                r = run_simulation(p1f, fseeds, dict(CFG_3DD, wps=wps, z_show=z), kappa=k)
                F[(geo, k, z)] = r
                row.append(f'z {z:2.0f} m: P4 {int(r["s4"].sum()):3d}/{N_FACT}, P2 3/3 '
                           f'{int(np.sum(r["sc2"] == 3)):3d}, score {np.mean(r["score"]):.2f}')
            print(f'  {geo:19s} x{k:<5g} ' + '; '.join(row))
    print('  lowest battery multiplier (of ' + ', '.join(f'x{k:g}' for k in FACT_KAPPAS)
          + ') with all formations completed: ' + '; '.join(
              f'{geo} z {z:.0f} m -> ' + next((f'x{k:g}' for k in FACT_KAPPAS
                                              if F[(geo, k, z)]['s4'].all()), 'none')
              for geo in ('2D waypoints (5 m)', 'bands (5/12/20 m)') for z in FACT_Z))
    print('\nPaired comparisons (per mission):')
    for k in KAPPAS3:
        a, b, c = M[('3d', k)]['score'], M[('2d', k)]['score'], M[('3dd', k)]['score']
        print(f'  x{k:g}: ' + paired(a, b, '3D card', '2D baseline') + '; ' + paired(a, c, 'stratified', 'direct'))
    print('\nPhase 4 detail (missions where all five drones received the assignment):')
    for key in ('2d', '3d', '3dd'):
        for k in (3.0, 5.0):
            r = M[(key, k)]
            ok = r['s4']
            dur = r['t_end'][:, 3] - r['t_end'][:, 2]
            print(f'  {LBL[key]}, x{k:g}: formed {int(ok.sum())}/{N_SEEDS}, Phase 4 duration '
                  f'{ms(dur[ok]) if ok.any() else "-"} s, min separation {np.min(r["minsep"][:, 3]):.2f} m '
                  f'(mean {np.mean(r["minsep"][:, 3]):.2f}), ORCA ticks {int(r["orca_steps"].sum())}, peak '
                  f'|v_z| {np.max(r["vz_p4"]):.2f} m/s, peak altitude {np.max(r["zmax_p4"]):.2f} m, B battery at Phase 4 start {ms(100 * r["E_p4"][:, 1])} %, '
                  f'B at the end {ms(100 * r["E_final"][:, 1])} %, P3 {100 * r["s3"].mean():.1f} %')
    a, b = M[('3d', 5.0)]['minsep'][:, 3], M[('3dd', 5.0)]['minsep'][:, 3]
    print(f'  x5 stratified vs direct: larger Phase 4 min separation in {int(np.sum(a > b + 1e-9))}/{N_SEEDS}, '
          f'smaller in {int(np.sum(a < b - 1e-9))}; mean difference {np.mean(a - b):+.2f} m')
    d3, dd = (M[('3d', 5.0)]['t_end'][:, 3] - M[('3d', 5.0)]['t_end'][:, 2],
              M[('3dd', 5.0)]['t_end'][:, 3] - M[('3dd', 5.0)]['t_end'][:, 2])
    print(f'  x5 stratified minus direct Phase 4 duration: {np.mean(d3 - dd):+.2f} s (min {np.min(d3 - dd):+.2f}, '
          f'max {np.max(d3 - dd):+.2f}); the stratified climb profiles end {ms(M[("3d", 5.0)]["ramp_end"])} s '
          f'after Phase 4 starts')
    print('  Probe: the same at x5 without the integral term (PD, K_i = 0):')
    for key, cfg in (('3d', CFG_3D), ('3dd', CFG_3DD)):
        r = run_simulation(p1, seeds, dict(cfg, ki=0.0), kappa=5.0)
        dur = r['t_end'][:, 3] - r['t_end'][:, 2]
        print(f'    {LBL[key]}, PD: formed {int(r["s4"].sum())}/{N_SEEDS}, Phase 4 duration '
              f'{ms(dur)} s, peak altitude {np.max(r["zmax_p4"]):.2f} m, score {ms(r["score"])}')

    Lg = {}
    p1_0 = slice_p1(p1, [0])
    for key, cfg in (('2d', CFG_2D), ('3d', CFG_3D), ('3dd', CFG_3DD)):
        Lg[key] = run_simulation(p1_0, [0], cfg, kappa=5.0, log=True)
        assert np.isclose(Lg[key]['score'][0], M[(key, 5.0)]['score'][0])
    print()
    plot_phase2(p2, OUTPUT_DIR)
    plot_altitude_battery(M, OUTPUT_DIR)
    plot_phase4(Lg, M, OUTPUT_DIR)
    plot_mission_3d(Lg, OUTPUT_DIR)
    save_animation(Lg['3d'], OUTPUT_DIR)


if __name__ == '__main__':
    main()
