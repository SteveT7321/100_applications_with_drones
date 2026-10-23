"""
S039 Offshore Platform Exchange
================================
4 drones carry cargo between 5 offshore platforms in a 6 x 6 km sea area
(2-D model). A support vessel sails the card's looping route at 4 m/s and
offers battery swaps on a single moving deck. Wind = steady mean + Dryden gust;
drones fly the crab-angle groundspeed and burn P = P_cruise + k_wind |w|^2.

Energy / range model
  BATTERY_FULL = energy to fly R_MAX = 5000 m eastbound in the mean wind.
  Planning uses the mean wind and the expected gust power
      P_plan = P_cruise + k_wind (|w_mean|^2 + 2 sigma_g^2)
  and keeps a reserve E_res = 15 % of BATTERY_FULL at the end of every
  battery-limited stretch. Execution integrates the actual power with gusts;
  the reported fleet energy is the integral of P dt over all flight/holding,
  including everything consumed before battery swaps.

Strategies (genuinely different code paths)
  greedy  : nearest idle drone takes each pending request, flies direct, and
            diverts to the vessel only on a battery emergency (remaining energy
            minus energy-to-target < E_res). If the emergency fires again right
            after a swap (battery >= 98 %), the request is aborted instead of
            looping.
  relay   : one static plan at t = 0 (requests in deadline order). For every
            drone the planner evaluates direct, swap-after-pickup (with an
            optimised wait at the origin) and swap-before-pickup routes; the
            plan maximises the SoC left at delivery among on-time options
            ("proactive swaps, lower peak battery draw"), else earliest
            delivery. Requests with no feasible route inside T_MAX are rejected.
  rolling : every 30 s re-plans all requests not yet picked up (including
            previously rejected ones) from the actual drone states; picks the
            minimum-energy on-time route (card objective), else earliest.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s039_offshore_platform_exchange.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation

# ── Parameters ─────────────────────────────────────────────────────────────────
N_DRONES    = 4
N_PLATFORMS = 5
V_AIRSPEED  = 12.0       # m/s  drone cruise airspeed
V_VESSEL    = 4.0        # m/s  support vessel speed
R_MAX       = 5000.0     # m    range per full battery (eastbound, mean wind)
D_SAFETY    = 0.15       # –    battery reserve fraction
P_CRUISE    = 80.0       # W    nominal cruise power
K_WIND      = 0.8        # W·s²/m²  wind penalty coefficient
W_MEAN      = np.array([4.0, 1.5])  # m/s  mean wind vector
SIGMA_GUST  = 1.2        # m/s  gust intensity
L_GUST      = 200.0      # m    gust length scale
D_DOCK      = 15.0       # m    moving-deck engagement radius (dock when inside)
R_HOLD      = 30.0       # m    holding orbit radius
T_SWAP      = 5.0        # s    battery swap on the vessel deck
T_HANDLE    = 10.0       # s    load / unload time on a platform pad
DT          = 0.1        # s    simulation timestep
T_MAX       = 600.0      # s    mission horizon
DT_PLAN     = 30.0       # s    rolling-horizon replanning interval
WAIT_GRID   = np.arange(0.0, 400.0, 10.0)   # s  candidate waits before a vessel leg
N_SEEDS     = 5          # gust realisations per strategy (seeds 0..N_SEEDS-1)
SHOW_SEED   = 0          # realisation used for the maps / time plots
GIF_FRAMES  = 150

PLATFORMS = np.array([
    [500.,  500.],
    [2000., 4500.],
    [4000., 1000.],
    [5500., 3500.],
    [3000., 2800.],
])

VESSEL_ROUTE = np.array([
    [1000., 1000.],
    [3000.,  500.],
    [5000., 2000.],
    [4500., 4500.],
    [1500., 3500.],
    [1000., 1000.],
])

# (origin_idx, dest_idx, payload_kg, deadline_s)
REQUESTS = [                          # card instance
    (0, 3, 1.5, 400.0),
    (2, 1, 0.8, 350.0),
    (4, 0, 2.0, 500.0),
    (1, 4, 1.2, 450.0),
]
# Extended instance: card requests + 4 more, so drones must chain tasks and
# choose between direct flights and vessel swaps (all known at t = 0)
REQUESTS_EXT = REQUESTS + [
    (0, 4, 1.0, 450.0),               # R5 P1 -> P5
    (2, 4, 1.0, 400.0),               # R6 P3 -> P5
    (4, 3, 0.8, 550.0),               # R7 P5 -> P4
    (0, 2, 1.0, 550.0),               # R8 P1 -> P3
]
INSTANCES = {'card': REQUESTS, 'extended': REQUESTS_EXT}
SHOW_INSTANCE = 'extended'
DRONE_START = [0, 2, 4, 1]          # platform index where each drone starts

STRATEGIES = ['greedy', 'relay', 'rolling']
STRAT_LABEL = {'greedy': 'Greedy direct', 'relay': 'Vessel-relay scheduled',
               'rolling': 'Rolling-horizon'}
DRONE_COLORS = ['red', 'green', 'purple', 'darkorange']
PLAT_COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's039_offshore_platform_exchange',
)

# ── Wind, power, vessel ────────────────────────────────────────────────────────
_PSI_W_MEAN = float(np.arctan2(W_MEAN[1], W_MEAN[0]))
_W_MEAN_MAG = float(np.linalg.norm(W_MEAN))


def groundspeed(u, wind):
    """Crab-angle groundspeed along unit direction u in wind (card formula)."""
    psi_des = np.arctan2(u[1], u[0])
    psi_w = np.arctan2(wind[1], wind[0])
    w_mag = float(np.hypot(wind[0], wind[1]))
    sin_d = np.clip(w_mag * np.sin(psi_des - psi_w) / V_AIRSPEED, -1.0, 1.0)
    vg = V_AIRSPEED * np.cos(np.arcsin(sin_d)) + w_mag * np.cos(psi_des - psi_w)
    return max(float(vg), 0.5)


def power(wind):
    return P_CRUISE + K_WIND * float(wind[0] ** 2 + wind[1] ** 2)


P_NOM = power(W_MEAN)                                       # mean-wind power
P_PLAN = P_CRUISE + K_WIND * (float(W_MEAN @ W_MEAN) + 2 * SIGMA_GUST ** 2)
BATTERY_FULL = P_NOM * R_MAX / groundspeed(np.array([1.0, 0.0]), W_MEAN)
E_RES = D_SAFETY * BATTERY_FULL

_V_LEGS = np.diff(VESSEL_ROUTE, axis=0)
_V_LEN = np.linalg.norm(_V_LEGS, axis=1)
_V_CUM = np.concatenate([[0.0], np.cumsum(_V_LEN)])


def vessel_position(t):
    d = (V_VESSEL * t) % _V_CUM[-1]
    s = min(int(np.searchsorted(_V_CUM, d, side='right')) - 1, len(VESSEL_ROUTE) - 2)
    return VESSEL_ROUTE[s] + (d - _V_CUM[s]) / _V_LEN[s] * _V_LEGS[s]


def fly_cost(p, q):
    """(time, planned energy) to fly p -> q in the mean wind."""
    d = float(np.hypot(q[0] - p[0], q[1] - p[1]))
    if d < 1e-9:
        return 0.0, 0.0
    t = d / groundspeed((q - p) / d, W_MEAN)
    return t, P_PLAN * t


def intercept(p, t0, n_iter=20):
    """Fixed-point vessel intercept time/point for a drone leaving p at t0."""
    ts = t0
    for _ in range(n_iter):
        q = vessel_position(ts)
        ts = t0 + fly_cost(p, q)[0]
    return ts, vessel_position(ts)


class GustModel:
    """First-order Dryden gust, one per drone, seeded."""

    def __init__(self, rng, sigma=SIGMA_GUST):
        self.rng = rng
        self.xi = np.zeros(2)
        tau = L_GUST / V_AIRSPEED
        self.a = np.exp(-DT / tau)
        self.b = sigma * np.sqrt(max(1.0 - self.a ** 2, 0.0))

    def step(self):
        self.xi = self.a * self.xi + self.b * self.rng.standard_normal(2)
        return self.xi


# ── Planner (shared by relay and rolling) ─────────────────────────────────────
def plan_options(pos, energy, t_free, req_idx, reqs):
    """All feasible routes for one request from a drone state.

    Returns list of dicts: steps, t_deliver, energy_used (planned),
    energy_end, swaps.
    """
    o, d, _, _ = reqs[req_idx]
    po, pd = PLATFORMS[o], PLATFORMS[d]
    opts = []
    # direct
    t1, e1 = fly_cost(pos, po)
    t2, e2 = fly_cost(po, pd)
    if energy - e1 - e2 >= E_RES:
        t_del = t_free + t1 + T_HANDLE + t2
        opts.append(dict(steps=[('fly', o), ('pickup', req_idx), ('fly', d), ('deliver', req_idx)],
                         t_deliver=t_del, e_used=e1 + e2, e_end=energy - e1 - e2, swaps=0))
    # swap after pickup (optionally wait at the origin first)
    t_pick = t_free + t1 + T_HANDLE
    if energy - e1 >= E_RES:
        for w in WAIT_GRID:
            t_dep = t_pick + w
            ts, pv = intercept(po, t_dep)
            _, ev = fly_cost(po, pv)
            tf, ef = fly_cost(pv, pd)
            if energy - e1 - ev >= E_RES and BATTERY_FULL - ef >= E_RES:
                t_del = ts + T_SWAP + tf
                steps = [('fly', o), ('pickup', req_idx)]
                if w > 0:
                    steps.append(('wait', t_dep))
                steps += [('vessel',), ('fly', d), ('deliver', req_idx)]
                opts.append(dict(steps=steps, t_deliver=t_del, e_used=e1 + ev + ef,
                                 e_end=BATTERY_FULL - ef, swaps=1))
    # swap before pickup
    ts, pv = intercept(pos, t_free)
    _, ev = fly_cost(pos, pv)
    ta, ea = fly_cost(pv, po)
    if energy - ev >= E_RES and BATTERY_FULL - ea - e2 >= E_RES:
        t_del = ts + T_SWAP + ta + T_HANDLE + t2
        opts.append(dict(steps=[('vessel',), ('fly', o), ('pickup', req_idx), ('fly', d),
                                ('deliver', req_idx)],
                         t_deliver=t_del, e_used=ev + ea + e2,
                         e_end=BATTERY_FULL - ea - e2, swaps=1))
    return [op for op in opts if op['t_deliver'] <= T_MAX]


def choose(opts, deadline, mode):
    on_time = [op for op in opts if op['t_deliver'] <= deadline]
    if on_time:
        if mode == 'max_soc':
            return max(on_time, key=lambda op: (op['e_end'], -op['t_deliver']))
        return min(on_time, key=lambda op: (op['e_used'], op['t_deliver']))
    return min(opts, key=lambda op: (op['t_deliver'], op['e_used'])) if opts else None


def build_plan(states, req_list, mode, reqs):
    """Assign requests (deadline order) to drones; chain tasks per drone.

    states: {k: (pos, energy, t_free)}; returns ({k: steps}, rejected list).
    """
    states = {k: (np.array(v[0], float), v[1], v[2]) for k, v in states.items()}
    queues = {k: [] for k in states}
    rejected = []
    for r in sorted(req_list, key=lambda r: (reqs[r][3], r)):
        best = None
        for k, (pos, en, tf) in states.items():
            op = choose(plan_options(pos, en, tf, r, reqs), reqs[r][3], mode)
            if op is None:
                continue
            key = (op['t_deliver'] > reqs[r][3], op['t_deliver'], k)
            if best is None or key < best[0]:
                best = (key, k, op)
        if best is None:
            rejected.append(r)
            continue
        _, k, op = best
        queues[k] += op['steps']
        states[k] = (PLATFORMS[reqs[r][1]].copy(), op['e_end'],
                     op['t_deliver'] + T_HANDLE)
    return queues, rejected


# ── Simulation ────────────────────────────────────────────────────────────────
class Drone:
    def __init__(self, k):
        self.k = k
        self.pos = PLATFORMS[DRONE_START[k]].copy()
        self.energy = BATTERY_FULL
        self.e_used = 0.0
        self.queue = []
        self.mode = 'idle'          # idle | fly | hold | land | vessel | vhold | docked | wait
        self.timer = 0.0
        self.cargo = None           # request index carried
        self.assigned = []          # requests in queue (not yet picked up)
        self.swaps = 0
        self.swaps_task = 0         # swaps since the current queue was given
        self.min_soc = 1.0
        self.hold_angle = 0.0
        self.log_pos, self.log_soc, self.log_mode = [], [], []


def run_strategy(strategy, seed, reqs, sigma=SIGMA_GUST):
    drones = [Drone(k) for k in range(N_DRONES)]
    gusts = [GustModel(np.random.default_rng([seed, k]), sigma) for k in range(N_DRONES)]
    n_req = len(reqs)
    status = ['pending'] * n_req       # pending | assigned | picked | delivered | rejected | aborted
    t_deliver = [None] * n_req
    pad_busy = [None] * N_PLATFORMS
    deck_busy = None
    events = []                         # (t, drone, kind, info)
    pad_log = []                        # (platform, drone, t_start, t_end, kind)
    hold_time = 0.0

    def give(dr, steps, r_ids):
        dr.queue = list(steps)
        dr.assigned = list(r_ids)
        dr.swaps_task = 0
        for r in r_ids:
            status[r] = 'assigned'

    def reqs_in(steps):
        return sorted({s[1] for s in steps if s[0] == 'pickup'})

    def platform_at(dr):
        for m in range(N_PLATFORMS):
            if np.hypot(*(dr.pos - PLATFORMS[m])) < 1.0:
                return m
        return None

    # ── planners ──
    def plan_static():
        states = {k: (dr.pos, dr.energy, 0.0) for k, dr in enumerate(drones)}
        queues, rej = build_plan(states, list(range(n_req)), 'max_soc', reqs)
        for k, q in queues.items():
            give(drones[k], q, reqs_in(q))
        for r in rej:
            status[r] = 'rejected'

    def plan_rolling(t):
        # drones that can be re-planned: not carrying cargo and not in the middle
        # of a landing, pad handling or vessel swap
        states = {}
        for k, dr in enumerate(drones):
            if dr.cargo is None and dr.mode in ('idle', 'fly', 'wait'):
                states[k] = (dr.pos, dr.energy, t)
        locked = {r for k, dr in enumerate(drones) if k not in states for r in dr.assigned}
        open_reqs = [r for r in range(n_req)
                     if status[r] in ('pending', 'assigned', 'rejected') and r not in locked]
        if not open_reqs or not states:
            return
        queues, rej = build_plan(states, open_reqs, 'min_energy', reqs)
        for k in states:
            dr = drones[k]
            for r in dr.assigned:
                if status[r] == 'assigned':
                    status[r] = 'pending'
            if queues[k] or dr.mode in ('fly', 'wait'):
                if dr.mode == 'wait':
                    dr.mode = 'idle'
                give(dr, queues[k], reqs_in(queues[k]))
                if dr.mode == 'fly' and not queues[k]:
                    m = int(np.argmin(np.linalg.norm(PLATFORMS - dr.pos, axis=1)))
                    dr.queue = [('fly', m)]
                if dr.mode == 'fly':
                    dr.mode = 'idle'          # re-dispatch from current position
        for r in rej:
            if status[r] != 'assigned':
                status[r] = 'rejected'

    def assign_greedy():
        for r in range(n_req):
            if status[r] != 'pending':
                continue
            idle = [dr for dr in drones if dr.mode == 'idle' and not dr.queue]
            if not idle:
                return
            o, d, _, _ = reqs[r]
            dr = min(idle, key=lambda x: (np.hypot(*(x.pos - PLATFORMS[o])), x.k))
            give(dr, [('fly', o), ('pickup', r), ('fly', d), ('deliver', r)], [r])

    # ── execution helpers ──
    def target_energy_need(dr):
        """Planned energy to reach the current fly target (mean wind)."""
        if dr.mode != 'fly' or not dr.queue or dr.queue[0][0] != 'fly':
            return 0.0
        return fly_cost(dr.pos, PLATFORMS[dr.queue[0][1]])[1]

    def emergency(dr, t):
        if dr.mode != 'fly':
            return
        if dr.energy - target_energy_need(dr) >= E_RES:
            return
        if dr.energy >= 0.98 * BATTERY_FULL and dr.swaps_task >= 1:
            # a full battery cannot do it: abort instead of looping
            r_ab = dr.cargo if dr.cargo is not None else (dr.assigned[0] if dr.assigned else None)
            if r_ab is not None:
                status[r_ab] = 'aborted'
                events.append((t, dr.k, 'abort', r_ab))
            dr.cargo = None
            dr.assigned = []
            m = int(np.argmin([fly_cost(dr.pos, P)[1] for P in PLATFORMS]))
            dr.queue = [('fly', m)]
            return
        events.append((t, dr.k, 'divert', None))
        dr.queue.insert(0, ('vessel',))
        dr.mode = 'idle'   # re-dispatch next step to the vessel

    # ── main loop ──
    n_steps = int(round(T_MAX / DT))
    vessel_log = []
    if strategy == 'relay':
        plan_static()
    for step in range(n_steps + 1):
        t = step * DT
        vp = vessel_position(t)
        vessel_log.append(vp)
        if strategy == 'rolling' and step % int(round(DT_PLAN / DT)) == 0:
            plan_rolling(t)
        if strategy == 'greedy':
            assign_greedy()

        for k, dr in enumerate(drones):
            xi = gusts[k].step()
            wind = W_MEAN + xi
            # dispatch next queued step if idle
            if dr.mode == 'idle' and dr.queue:
                kind = dr.queue[0][0]
                if kind == 'fly':
                    m = dr.queue[0][1]
                    if np.hypot(*(dr.pos - PLATFORMS[m])) < 1.0:
                        dr.queue.pop(0)
                        dr.mode = 'land'
                        dr.land_platform = m
                    else:
                        dr.mode = 'fly'
                elif kind == 'vessel':
                    dr.mode = 'vessel'
                elif kind == 'wait':
                    dr.mode = 'wait'
                elif kind in ('pickup', 'deliver'):
                    dr.queue.pop(0)     # stale (already at platform handled below)
            if dr.mode == 'fly':
                emergency(dr, t)
                if dr.mode == 'idle' and dr.queue and dr.queue[0][0] == 'vessel':
                    dr.mode = 'vessel'

            flying = False
            if dr.mode == 'fly':
                m = dr.queue[0][1]
                tgt = PLATFORMS[m]
                vec = tgt - dr.pos
                dist = float(np.hypot(*vec))
                step_len = groundspeed(vec / dist, wind) * DT
                flying = True
                if dist <= step_len:
                    dr.pos = tgt.copy()
                    dr.queue.pop(0)
                    dr.mode = 'land'
                    dr.land_platform = m
                else:
                    dr.pos = dr.pos + vec / dist * step_len
            elif dr.mode == 'land':
                m = dr.land_platform
                if pad_busy[m] is None:
                    if dr.queue and dr.queue[0][0] in ('pickup', 'deliver') and \
                            reqs[dr.queue[0][1]][0 if dr.queue[0][0] == 'pickup' else 1] == m:
                        pad_busy[m] = dr.k
                        dr.pos = PLATFORMS[m].copy()
                        dr.mode = 'handle'
                        dr.timer = T_HANDLE
                        dr.handle = dr.queue.pop(0)
                        dr.pad_start = t
                        if dr.handle[0] == 'deliver':
                            r = dr.handle[1]
                            t_deliver[r] = t
                            status[r] = 'delivered'
                            dr.cargo = None
                            events.append((t, dr.k, 'deliver', r))
                    else:
                        dr.pos = PLATFORMS[m].copy()
                        dr.mode = 'idle'
                else:
                    dr.mode = 'hold'
                    dr.hold_angle = float(np.arctan2(*(dr.pos - PLATFORMS[m])[::-1]))
                    events.append((t, dr.k, 'hold', m))
            elif dr.mode == 'hold':
                m = dr.land_platform
                dr.hold_angle += V_AIRSPEED / R_HOLD * DT
                dr.pos = PLATFORMS[m] + R_HOLD * np.array([np.cos(dr.hold_angle),
                                                           np.sin(dr.hold_angle)])
                flying = True
                hold_time += DT
                if pad_busy[m] is None:
                    dr.pos = PLATFORMS[m].copy()
                    dr.mode = 'land'
            elif dr.mode == 'handle':
                dr.timer -= DT
                if dr.timer <= 1e-9:
                    m = dr.land_platform
                    pad_busy[m] = None
                    pad_log.append((m, dr.k, dr.pad_start, t, dr.handle[0]))
                    if dr.handle[0] == 'pickup':
                        r = dr.handle[1]
                        dr.cargo = r
                        status[r] = 'picked'
                        if r in dr.assigned:
                            dr.assigned.remove(r)
                    dr.mode = 'idle'
            elif dr.mode == 'wait':
                if t >= dr.queue[0][1] - 1e-9:
                    dr.queue.pop(0)
                    dr.mode = 'idle'
            elif dr.mode in ('vessel', 'vhold'):
                flying = True
                vec = vp - dr.pos
                dist = float(np.hypot(*vec))
                if dist <= D_DOCK and deck_busy is None:
                    deck_busy = dr.k
                    dr.mode = 'docked'
                    dr.timer = T_SWAP
                    dr.pos = vp.copy()
                    flying = False
                    events.append((t, dr.k, 'dock', None))
                elif dist <= D_DOCK:
                    dr.mode = 'vhold'           # deck busy: hover next to the vessel
                    dr.pos = vp + (dr.pos - vp) / max(dist, 1e-6) * D_DOCK
                    hold_time += DT
                else:
                    _, pint = intercept(dr.pos, t, n_iter=6)
                    u = (pint - dr.pos) / max(float(np.hypot(*(pint - dr.pos))), 1e-6)
                    dr.pos = dr.pos + u * min(groundspeed(u, wind) * DT, dist)
            elif dr.mode == 'docked':
                dr.pos = vp.copy()
                dr.timer -= DT
                if dr.timer <= 1e-9:
                    dr.energy = BATTERY_FULL
                    dr.swaps += 1
                    dr.swaps_task += 1
                    deck_busy = None
                    if dr.queue and dr.queue[0][0] == 'vessel':
                        dr.queue.pop(0)
                    dr.mode = 'idle'
                    events.append((t, dr.k, 'swap', None))

            if flying:
                e = min(power(wind) * DT, dr.energy)
                dr.energy -= e
                dr.e_used += e
                if dr.energy <= 1e-9:
                    # battery exhausted in flight: the drone ditches; its cargo is
                    # lost and its not-yet-picked requests go back to the pool
                    dr.energy = 0.0
                    dr.mode = 'ditched'
                    dr.queue = []
                    if dr.cargo is not None:
                        status[dr.cargo] = 'lost'
                        dr.cargo = None
                    for r in dr.assigned:
                        if status[r] == 'assigned':
                            status[r] = 'pending'
                    dr.assigned = []
                    events.append((t, dr.k, 'ditched', None))
            dr.min_soc = min(dr.min_soc, dr.energy / BATTERY_FULL)
            dr.log_pos.append(dr.pos.copy())
            dr.log_soc.append(dr.energy / BATTERY_FULL)
            dr.log_mode.append(dr.mode)

    delivered = [r for r in range(n_req) if status[r] == 'delivered']
    late = [r for r in delivered if t_deliver[r] > reqs[r][3]]
    return dict(strategy=strategy, seed=seed, reqs=reqs, drones=drones, status=status,
                t_deliver=t_deliver, events=events, pad_log=pad_log,
                vessel_log=np.array(vessel_log), hold_time=hold_time,
                energy_kJ=sum(dr.e_used for dr in drones) / 1000.0,
                swaps=sum(dr.swaps for dr in drones), delivered=len(delivered),
                on_time=len(delivered) - len(late), late=len(late),
                min_soc=min(dr.min_soc for dr in drones),
                ditched=sum(1 for e in events if e[2] == 'ditched'))


def run_simulation():
    """runs[instance][strategy] = list of results, one per gust seed."""
    return {name: {s: [run_strategy(s, seed, reqs) for seed in range(N_SEEDS)]
                   for s in STRATEGIES}
            for name, reqs in INSTANCES.items()}


# ── Plots ─────────────────────────────────────────────────────────────────────
def _base_map(ax):
    ax.plot(VESSEL_ROUTE[:, 0], VESSEL_ROUTE[:, 1], '--', color='gray', lw=1.2,
            label='Vessel route (loop)')
    for m, (px, py) in enumerate(PLATFORMS):
        ax.add_patch(plt.Rectangle((px - 100, py - 100), 200, 200,
                                   color=PLAT_COLORS[m], alpha=0.7, zorder=3))
        ax.text(px, py + 170, f'P{m+1}', ha='center', fontsize=9, fontweight='bold')
    ax.annotate('', xy=(5700 + W_MEAN[0] * 80, 300 + W_MEAN[1] * 80), xytext=(5700, 300),
                arrowprops=dict(arrowstyle='->', color='navy', lw=2))
    ax.text(5700, 120, 'wind', ha='center', fontsize=7, color='navy')
    ax.set_xlim(-200, 6300)
    ax.set_ylim(-200, 5200)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)


def plot_sea_area(runs, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(21, 7))
    for ax, s in zip(axes, STRATEGIES):
        res = runs[s][SHOW_SEED]
        _base_map(ax)
        vl = res['vessel_log']
        ax.plot(vl[:, 0], vl[:, 1], '-', color='steelblue', lw=3, alpha=0.5,
                label='Vessel track 0–600 s')
        for dr in res['drones']:
            tr = np.array(dr.log_pos)
            ax.plot(tr[:, 0], tr[:, 1], color=DRONE_COLORS[dr.k], lw=1.2,
                    label=f'Drone {dr.k+1}')
        for (t, k, kind, info) in res['events']:
            idx = int(round(t / DT))
            p = res['drones'][k].log_pos[min(idx, len(res['drones'][k].log_pos) - 1)]
            if kind == 'swap':
                ax.scatter(*p, s=160, marker='*', color=DRONE_COLORS[k], edgecolors='black',
                           zorder=6)
            elif kind == 'abort':
                ax.scatter(*p, s=120, marker='X', color=DRONE_COLORS[k], edgecolors='black',
                           zorder=6)
            elif kind == 'ditched':
                ax.scatter(*p, s=150, marker='v', color='black', edgecolors=DRONE_COLORS[k],
                           linewidths=2, zorder=7)
        st = res['status']
        txt = ', '.join(f'R{r+1}:{st[r]}' for r in range(len(res['reqs'])))
        txt = txt.replace('R5', '\nR5')
        ax.set_title(f'{STRAT_LABEL[s]} (seed {SHOW_SEED})\n{txt}', fontsize=9)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
    h, l = axes[0].get_legend_handles_labels()
    h += [plt.Line2D([], [], marker='*', ls='', color='gray', mec='black', ms=12),
          plt.Line2D([], [], marker='X', ls='', color='gray', mec='black', ms=10),
          plt.Line2D([], [], marker='v', ls='', color='black', ms=10)]
    l += ['Battery swap on vessel', 'Request aborted', 'Battery empty: ditched']
    fig.legend(h, l, loc='lower center', ncol=8, fontsize=9)
    fig.suptitle('S039 Offshore Platform Exchange — Sea Area Map', fontsize=13,
                 fontweight='bold')
    plt.tight_layout(rect=[0, 0.06, 1, 0.95])
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'sea_area_map.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def plot_battery_soc(runs, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    t = np.arange(len(runs['greedy'][SHOW_SEED]['drones'][0].log_soc)) * DT
    ls = {'greedy': '-', 'relay': '--', 'rolling': ':'}
    for k, ax in enumerate(axes.flatten()):
        for s in STRATEGIES:
            dr = runs[s][SHOW_SEED]['drones'][k]
            ax.plot(t, np.array(dr.log_soc) * 100, ls[s], color=DRONE_COLORS[k], lw=1.6,
                    label=f'{STRAT_LABEL[s]} (used {dr.e_used/1000:.1f} kJ, {dr.swaps} swaps)')
        ax.axhline(D_SAFETY * 100, color='red', ls='--', lw=1)
        ax.axhspan(0, D_SAFETY * 100, color='red', alpha=0.08)
        ax.set_ylim(0, 105)
        ax.set_title(f'Drone {k+1}', fontsize=10)
        ax.set_ylabel('SoC (%)')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, loc='lower left')
    for ax in axes[1]:
        ax.set_xlabel('Time (s)')
    fig.suptitle(f'S039 Battery State of Charge (seed {SHOW_SEED}; red band = 15 % reserve; '
                 'jumps to 100 % = vessel swap)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'battery_soc.png')
    plt.savefig(path, dpi=140)
    plt.close()
    print(f'Saved: {path}')


def plot_platform_gantt(runs, out_dir):
    """Per strategy: what each drone does over time (seed SHOW_SEED)."""
    mode_col = {'fly': '#4c72b0', 'vessel': '#55a868', 'vhold': '#c44e52', 'docked': '#ffd92f',
                'hold': '#c44e52', 'handle': '#8172b2', 'wait': '#bbbbbb', 'land': '#4c72b0',
                'ditched': '#000000'}
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for ax, s in zip(axes, STRATEGIES):
        res = runs[s][SHOW_SEED]
        for dr in res['drones']:
            modes = dr.log_mode
            start = 0
            for i in range(1, len(modes) + 1):
                if i == len(modes) or modes[i] != modes[start]:
                    m = modes[start]
                    if m in mode_col:
                        ax.barh(dr.k, (i - start) * DT, left=start * DT, height=0.6,
                                color=mode_col[m], edgecolor='none')
                    start = i
        for (m, k, t0, t1, kind) in res['pad_log']:
            ax.text((t0 + t1) / 2, k + 0.36, f'P{m+1}', ha='center', fontsize=6)
        for r, (o, d, w, dl) in enumerate(res['reqs']):
            ax.axvline(dl, color='black', lw=0.6, ls=':')
            ax.text(dl, N_DRONES - 0.45, f'dl R{r+1}', fontsize=7, rotation=90, va='top')
        ax.set_yticks(range(N_DRONES))
        ax.set_yticklabels([f'Drone {k+1}' for k in range(N_DRONES)])
        ax.set_xlim(0, T_MAX)
        ax.set_title(f'{STRAT_LABEL[s]}: delivered {res["delivered"]}/{len(res["reqs"])} '
                     f'({res["late"]} late), energy {res["energy_kJ"]:.1f} kJ, '
                     f'{res["swaps"]} swaps', fontsize=10)
        ax.grid(True, axis='x', alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    handles = [mpatches.Patch(color=c, label=l) for l, c in
               (('Flying to platform', '#4c72b0'), ('Flying to vessel', '#55a868'),
                ('Holding (pad / deck busy)', '#c44e52'), ('Docked: battery swap', '#ffd92f'),
                ('On pad: load / unload', '#8172b2'), ('Waiting on platform', '#bbbbbb'),
                ('Ditched (battery empty)', '#000000'))]
    fig.legend(handles=handles, loc='lower center', ncol=7, fontsize=8)
    fig.suptitle(f'S039 Drone Activity and Platform Pad Use (seed {SHOW_SEED}; '
                 'white = parked idle)', fontsize=12)
    plt.tight_layout(rect=[0, 0.04, 1, 0.96])
    path = os.path.join(out_dir, 'platform_gantt.png')
    plt.savefig(path, dpi=140)
    plt.close()
    print(f'Saved: {path}')


def plot_strategy_comparison(all_runs, out_dir):
    fig, axes_all = plt.subplots(2, 3, figsize=(16, 9.5))
    x = np.arange(len(STRATEGIES))
    cols = ['#e74c3c', '#3498db', '#2ecc71']
    for row, (inst, runs) in enumerate(all_runs.items()):
        axes = axes_all[row]
        n_req = len(INSTANCES[inst])
        E = [[r['energy_kJ'] for r in runs[s]] for s in STRATEGIES]
        axes[0].bar(x, [np.mean(e) for e in E], yerr=[np.std(e) for e in E], color=cols,
                    edgecolor='black', capsize=5)
        for i, e in enumerate(E):
            axes[0].text(i + 0.12, np.mean(e) + 2, f'{np.mean(e):.1f}', ha='left')
        axes[0].set_ylabel('Total fleet energy used (kJ)')
        axes[0].set_title(f'{inst} instance: integrated fleet energy (mean ± std)')
        on = [np.mean([r['on_time'] for r in runs[s]]) for s in STRATEGIES]
        lt = [np.mean([r['late'] for r in runs[s]]) for s in STRATEGIES]
        nd = [n_req - o - l for o, l in zip(on, lt)]
        axes[1].bar(x, on, color='#2ecc71', edgecolor='black', label='On time')
        axes[1].bar(x, lt, bottom=on, color='#f39c12', edgecolor='black', label='Late')
        axes[1].bar(x, nd, bottom=np.array(on) + np.array(lt), color='#bbbbbb',
                    edgecolor='black', label='Not delivered')
        axes[1].set_ylim(0, n_req + 1.5)
        axes[1].set_ylabel(f'Requests (mean over {N_SEEDS} seeds)')
        axes[1].set_title(f'{inst} instance: request outcome ({n_req} requests)')
        axes[1].legend(fontsize=8, ncol=3, loc='upper center')
        sw = [np.mean([r['swaps'] for r in runs[s]]) for s in STRATEGIES]
        ms = [np.mean([r['min_soc'] for r in runs[s]]) * 100 for s in STRATEGIES]
        axes[2].bar(x - 0.2, sw, 0.4, color='#9b59b6', edgecolor='black',
                    label='Battery swaps')
        ax2 = axes[2].twinx()
        ax2.bar(x + 0.2, ms, 0.4, color='#95a5a6', edgecolor='black',
                label='Lowest SoC of any drone (%)')
        ax2.axhline(D_SAFETY * 100, color='red', ls='--', lw=1)
        ax2.set_ylim(0, 40)
        axes[2].set_ylim(0, max(sw) + 2)
        axes[2].set_ylabel('Swaps per mission')
        ax2.set_ylabel('Lowest SoC (%)')
        axes[2].set_title(f'{inst} instance: swaps and lowest SoC')
        h1, l1 = axes[2].get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        axes[2].legend(h1 + h2, l1 + l2, fontsize=8, loc='upper right')
        for ax in axes:
            ax.set_xticks(x)
            ax.set_xticklabels([STRAT_LABEL[s].replace(' ', '\n', 1) for s in STRATEGIES],
                               fontsize=9)
            ax.grid(True, axis='y', alpha=0.3)
    fig.suptitle(f'S039 Strategy Comparison ({N_SEEDS} gust realisations per strategy; '
                 'red dashed = 15 % reserve)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'strategy_comparison.png')
    plt.savefig(path, dpi=140)
    plt.close()
    print(f'Saved: {path}')


def save_animation(runs, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2))
    n = len(runs['greedy'][SHOW_SEED]['drones'][0].log_pos)
    frames = np.linspace(0, n - 1, GIF_FRAMES).round().astype(int)
    art = []
    for ax, s in zip(axes, STRATEGIES):
        _base_map(ax)
        ax.set_title(STRAT_LABEL[s], fontsize=10)
        ax.tick_params(labelsize=7)
        res = runs[s][SHOW_SEED]
        dots = [ax.plot([], [], 'o', color=DRONE_COLORS[k], ms=6, zorder=6)[0]
                for k in range(N_DRONES)]
        trails = [ax.plot([], [], '-', color=DRONE_COLORS[k], lw=0.8, alpha=0.6)[0]
                  for k in range(N_DRONES)]
        ves = ax.plot([], [], 's', color='steelblue', ms=9, zorder=5)[0]
        art.append((res, dots, trails, ves))
    clock = fig.suptitle('')

    def update(fi):
        i = frames[fi]
        out = []
        for res, dots, trails, ves in art:
            vp = res['vessel_log'][i]
            ves.set_data([vp[0]], [vp[1]])
            for k, dr in enumerate(res['drones']):
                tr = np.array(dr.log_pos[:i + 1])
                dots[k].set_data([tr[-1, 0]], [tr[-1, 1]])
                trails[k].set_data(tr[:, 0], tr[:, 1])
            out += dots + trails + [ves]
        clock.set_text(f'S039 Offshore Platform Exchange — t = {i * DT:.0f} s '
                       f'(seed {SHOW_SEED}; square = vessel)')
        return out + [clock]

    ani = animation.FuncAnimation(fig, update, frames=len(frames), blit=False)
    plt.tight_layout()
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=15, dpi=72)
    plt.close()
    print(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────────────────────
def print_feasibility(reqs):
    for r, (o, d, w, dl) in enumerate(reqs):
        dist = float(np.hypot(*(PLATFORMS[d] - PLATFORMS[o])))
        t, e = fly_cost(PLATFORMS[o], PLATFORMS[d])
        opts = plan_options(PLATFORMS[o], BATTERY_FULL, 0.0, r, reqs)
        best = min(opts, key=lambda op: op['t_deliver']) if opts else None
        bt = (f'earliest feasible delivery {best["t_deliver"]:.0f} s '
              f'({best["swaps"]} swap)') if best else 'no feasible route by T_max'
        print(f'  R{r+1}: P{o+1}->P{d+1} {dist:.0f} m, direct needs {e/BATTERY_FULL*100:.0f} % '
              f'of a battery ({t:.0f} s), deadline {dl:.0f} s; from a full battery at the '
              f'origin at t=0: {bt}')


def print_results(name, runs):
    reqs = INSTANCES[name]
    print(f'\n=== {name} instance: per-run results ({N_SEEDS} gust seeds) ===')
    print(f'{"strategy":<8} {"seed":>4} {"E(kJ)":>7} {"swaps":>5} {"deliv":>5} {"late":>4} '
          f'{"minSoC":>6} {"hold(s)":>7}  request status (delivery time s)')
    for s in STRATEGIES:
        for res in runs[s]:
            st = ' '.join(
                f'R{r+1}:{res["status"][r]}' +
                (f'({res["t_deliver"][r]:.0f})' if res['t_deliver'][r] is not None else '')
                for r in range(len(reqs)))
            print(f'{s:<8} {res["seed"]:>4} {res["energy_kJ"]:>7.2f} {res["swaps"]:>5} '
                  f'{res["delivered"]:>5} {res["late"]:>4} {res["min_soc"]*100:>5.1f}% '
                  f'{res["hold_time"]:>7.1f}  {st}')
    print(f'--- {name} instance: summary (mean over seeds) ---')
    for s in STRATEGIES:
        R = runs[s]
        print(f'{STRAT_LABEL[s]:<24} energy {np.mean([r["energy_kJ"] for r in R]):6.2f} kJ '
              f'(std {np.std([r["energy_kJ"] for r in R]):.2f}), '
              f'delivered {np.mean([r["delivered"] for r in R]):.1f}/{len(reqs)}, '
              f'on time {np.mean([r["on_time"] for r in R]):.1f}, '
              f'swaps {np.mean([r["swaps"] for r in R]):.1f}, '
              f'lowest SoC {np.mean([r["min_soc"] for r in R])*100:.1f} %, '
              f'runs below 15 % reserve {sum(1 for r in R if r["min_soc"] < D_SAFETY)}/{len(R)}, '
              f'drones ditched (battery empty) {sum(r["ditched"] for r in R)}')
    print(f'--- {name} instance: per-drone energy and events, seed {SHOW_SEED} ---')
    for s in STRATEGIES:
        res = runs[s][SHOW_SEED]
        print(f'{s:<8} ' + ', '.join(
            f'D{dr.k+1} {dr.e_used/1000:.2f} kJ / {dr.swaps} swaps / final SoC '
            f'{dr.energy/BATTERY_FULL*100:.0f}%' for dr in res['drones']))
        ev = [(round(t, 1), f'D{k+1}', kind, (f'R{info+1}' if kind in ('abort', 'deliver')
                                              else (f'P{info+1}' if kind == 'hold' else '')))
              for t, k, kind, info in res['events']
              if kind in ('divert', 'abort', 'swap', 'hold', 'deliver', 'ditched')]
        print(f'         events: {ev}')


def print_r3_plan_vs_execution(card_runs):
    """Why executed R3 deliveries scatter around the planned time."""
    r3 = 2
    opts = plan_options(PLATFORMS[REQUESTS[r3][0]], BATTERY_FULL, 0.0, r3, REQUESTS)
    best = min(opts, key=lambda op: op['t_deliver'])
    wait = [s for s in best['steps'] if s[0] == 'wait']
    t_dep = wait[0][1] if wait else T_HANDLE
    ts, pv = intercept(PLATFORMS[REQUESTS[r3][0]], t_dep)
    t_leg, _ = fly_cost(pv, PLATFORMS[REQUESTS[r3][1]])
    print('\n=== R3 (card instance, relay): plan vs execution ===')
    print(f'plan: leave P5 at {t_dep:.1f} s, dock {ts:.1f} s at ({pv[0]:.0f}, {pv[1]:.0f}) m '
          f'({float(np.hypot(*(PLATFORMS[0] - pv))):.0f} m from P1), swap done {ts + T_SWAP:.1f} s, '
          f'leg to P1 {t_leg:.1f} s, delivery {best["t_deliver"]:.1f} s')

    def describe(label, res):
        ev = {kind: t for t, k, kind, info in res['events']
              if k == 2 and kind in ('dock', 'swap')}
        t_del = res['t_deliver'][r3]
        print(f'{label:<26} dock {ev["dock"]:.1f} s, swap done {ev["swap"]:.1f} s, leg to P1 '
              f'{t_del - ev["swap"]:.1f} s, delivery {t_del:.1f} s')

    describe('execution without gusts', run_strategy('relay', 0, REQUESTS, sigma=0.0))
    for res in card_runs['relay']:
        describe(f'execution, gust seed {res["seed"]}', res)


if __name__ == '__main__':
    print('=== Planning feasibility (mean wind + expected gust power, 15 % reserve) ===')
    print(f'BATTERY_FULL = {BATTERY_FULL/1000:.2f} kJ, P_plan = {P_PLAN:.1f} W '
          f'(endurance {BATTERY_FULL/P_PLAN:.0f} s), E_res = {E_RES/1000:.2f} kJ')
    v0, v1 = vessel_position(0.0), vessel_position(T_MAX)
    print(f'Vessel position: t=0 ({v0[0]:.0f}, {v0[1]:.0f}) m, t={T_MAX:.0f} s '
          f'({v1[0]:.0f}, {v1[1]:.0f}) m')
    print_feasibility(REQUESTS_EXT)
    all_runs = run_simulation()
    for name in INSTANCES:
        print_results(name, all_runs[name])
    print_r3_plan_vs_execution(all_runs['card'])

    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    show = all_runs[SHOW_INSTANCE]
    plot_sea_area(show, out_dir)
    plot_battery_soc(show, out_dir)
    plot_platform_gantt(show, out_dir)
    plot_strategy_comparison(all_runs, out_dir)
    save_animation(show, out_dir)
