"""
S080 Underground Pipe CCTV Replacement
======================================
A 0.20 m micro-drone flies through a 50 m sewer pipe (inner diameter 0.50 m) with three
90 deg mitre elbows at s = 15, 30 and 43 m. GPS is denied. Four body-frame ultrasonic
rangers (N, S, E, W) measure skin-to-wall distance by ray casting against the pipe walls
(sigma = 5 mm); the drone advances at 0.2 m/s and is pushed sideways by a slowly varying
air-flow disturbance. Four navigation strategies are compared (common random numbers):
  A. Dead reckoning only: follow the pipe map by the odometry estimate of arc length
  B. Ultrasonic PD centring + map turns triggered by dead-reckoned arc length
  C. Ultrasonic PD centring + card bend rule (|d_k - d_opp| > 6 cm, turn at once)
  D. Ultrasonic PD centring + gated bend rule (reading longer than a straight pipe allows),
     turning after advancing one pipe radius to the elbow centre
Metrics: mission completion, wall-contact time, RMS centring error, cracks passed.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s080_underground_pipe.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters (card) ─────────────────────────────────────────────────────────
PIPE_R      = 0.25       # m  inner radius (D = 0.50 m)
DRONE_R     = 0.10       # m
BUDGET      = PIPE_R - DRONE_R   # 0.15 m lateral budget
PIPE_LEN    = 50.0       # m
ELBOW_S     = [15.0, 30.0, 43.0]
HEADINGS    = [np.array([1.0, 0, 0]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0]),
               np.array([1.0, 0, 0])]
SIGMA_WALL  = 0.005      # m
D_BEND      = 0.06       # m  card bend-asymmetry threshold
D_CONTACT   = 0.02       # m  near-contact (sensor) threshold
KP, KD      = 2.0, 0.40  # centring PD gains
V_ADV       = 0.20       # m/s
SIGMA_V     = 0.002      # m/s odometry noise
N_CRACKS    = 10
I_THRESHOLD = 80
DS_CRACK    = 0.30       # m
T_BLEND     = 0.5        # s
DT          = 0.05       # s

# ── Added parameters ──────────────────────────────────────────────────────────
MAX_RANGE   = 0.60       # m  ultrasonic saturation (skin distance)
GATE        = 2 * PIPE_R - DRONE_R + 0.02   # 0.42 m: longer than any straight-pipe reading
D_DELAY     = PIPE_R     # m  advance after detection before turning (strategy D)
REFRACTORY  = 1.0        # m  no new bend detection within this distance after a turn
SETTLE      = 0.3        # m  partial centring after a turn
U_MAX       = 0.30       # m/s lateral command limit
SIGMA_D     = 0.03       # m/s air-flow disturbance (Ornstein-Uhlenbeck, 3D)
TAU_D       = 2.0        # s   disturbance correlation time
SIGMA_BIAS  = 0.02       # odometry scale bias, per-seed N(0, 2 %)
T_MAX       = 400.0      # s
STUCK_T     = 10.0       # s  stuck if arc progress < 0.05 m in this time
N_MC        = 20
BIAS_SWEEP  = [-0.04, -0.02, 0.0, 0.02, 0.04]
SIGMA_SWEEP = [0.0, 0.01, 0.06]
N_SWEEP     = 8
SEED        = 0
ANIM_STEP   = 25         # steps per GIF frame (1.25 s)

STRATS = ['A', 'B', 'C', 'D']
LABELS = {'A': 'A: dead reckoning only', 'B': 'B: centring + DR-timed turns',
          'C': 'C: centring + card bend rule', 'D': 'D: centring + gated bend rule'}
COLORS = {'A': '#7F7F7F', 'B': '#FF7F0E', 'C': '#1F77B4', 'D': '#D62728'}
N_MAX = int(T_MAX / DT)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '04_industrial_agriculture', 's080_underground_pipe')


# ── Pipe geometry ─────────────────────────────────────────────────────────────

def build_pipe():
    starts = [0.0] + ELBOW_S
    ends = ELBOW_S + [PIPE_LEN]
    segs, p = [], np.zeros(3)
    for i, (s0, s1, h) in enumerate(zip(starts, ends, HEADINGS)):
        L = s1 - s0
        lo = -1.0 if i == 0 else -PIPE_R            # entrance open behind the start
        hi = L + (2.0 if i == len(HEADINGS) - 1 else PIPE_R)   # exit open
        segs.append(dict(A=p.copy(), a=h, L=L, lo=lo, hi=hi, s0=s0))
        p = p + h * L
    return segs


SEGS = build_pipe()
CORNERS = [s['A'] for s in SEGS] + [SEGS[-1]['A'] + SEGS[-1]['a'] * SEGS[-1]['L']]


def centreline(s):
    for sg in SEGS[::-1]:
        if s >= sg['s0']:
            return sg['A'] + sg['a'] * min(s - sg['s0'], sg['L'])
    return SEGS[0]['A'].copy()


def project(p):
    """Arc length and distance of p to the true centreline polyline."""
    best = (math.inf, 0.0)
    for sg in SEGS:
        a, q = _local(sg, p)
        ac = min(max(a, 0.0), sg['L'])
        d = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + (a - ac) ** 2)
        if d < best[0]:
            best = (d, sg['s0'] + ac)
    return best[1], best[0]


def _local(sg, p):
    r = (p[0] - sg['A'][0], p[1] - sg['A'][1], p[2] - sg['A'][2])
    a = r[0] * sg['a'][0] + r[1] * sg['a'][1] + r[2] * sg['a'][2]
    q = (r[0] - a * sg['a'][0], r[1] - a * sg['a'][1], r[2] - a * sg['a'][2])
    return a, q


def valid(p):
    """Drone centre position allowed (drone body inside the pipe)?"""
    for sg in SEGS:
        a, q = _local(sg, p)
        if sg['lo'] + DRONE_R - 1e-9 <= a <= sg['hi'] - DRONE_R + 1e-9 and \
                q[0] * q[0] + q[1] * q[1] + q[2] * q[2] <= BUDGET * BUDGET + 1e-12:
            return True
    return False


def clamp(p):
    """Nearest allowed position (wall contact: the drone slides along the wall)."""
    best, bd = None, np.inf
    for sg in SEGS:
        a, q = _local(sg, p)
        a_c = min(max(a, sg['lo'] + DRONE_R), sg['hi'] - DRONE_R)
        qn = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2)
        f = min(1.0, BUDGET / qn) if qn > 1e-12 else 1.0
        c = sg['A'] + sg['a'] * a_c + np.array(q) * f
        d = float(np.linalg.norm(c - p))
        if d < bd:
            best, bd = c, d
    return best


def _exit_t(sg, o, d):
    """Exit distance from extended cylinder sg along ray o + t d (o assumed inside)."""
    a0, q = _local(sg, o)
    ad = d[0] * sg['a'][0] + d[1] * sg['a'][1] + d[2] * sg['a'][2]
    dq = (d[0] - ad * sg['a'][0], d[1] - ad * sg['a'][1], d[2] - ad * sg['a'][2])
    A = dq[0] ** 2 + dq[1] ** 2 + dq[2] ** 2
    B = 2 * (q[0] * dq[0] + q[1] * dq[1] + q[2] * dq[2])
    C = q[0] ** 2 + q[1] ** 2 + q[2] ** 2 - PIPE_R ** 2
    t_r = (-B + math.sqrt(max(B * B - 4 * A * C, 0.0))) / (2 * A) if A > 1e-12 else math.inf
    if ad > 1e-12:
        t_a = (sg['hi'] - a0) / ad
    elif ad < -1e-12:
        t_a = (sg['lo'] - a0) / ad
    else:
        t_a = math.inf
    return min(t_r, t_a)


def _inside(sg, p):
    a, q = _local(sg, p)
    return sg['lo'] <= a <= sg['hi'] and q[0] ** 2 + q[1] ** 2 + q[2] ** 2 <= PIPE_R ** 2 + 1e-12


def near_segments(p, margin=MAX_RANGE + DRONE_R + 0.05):
    """Segments that a short ray from p can reach."""
    out = []
    for sg in SEGS:
        a, q = _local(sg, p)
        if sg['lo'] - margin <= a <= sg['hi'] + margin and                 q[0] ** 2 + q[1] ** 2 + q[2] ** 2 <= (PIPE_R + margin) ** 2:
            out.append(sg)
    return out


def ray(p, d, rmax=MAX_RANGE + DRONE_R, segs=None):
    """Distance from p along unit d to the wall of the pipe (union of extended cylinders)."""
    segs = SEGS if segs is None else segs
    t = 0.0
    for _ in range(4):
        o = (p[0] + d[0] * (t + 1e-7), p[1] + d[1] * (t + 1e-7), p[2] + d[2] * (t + 1e-7))
        ex = [_exit_t(sg, o, d) for sg in segs if _inside(sg, o)]
        if not ex:
            break
        t = t + 1e-7 + max(ex)
        if t >= rmax:
            return rmax
    return min(t, rmax)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def body_axes(h):
    """Body lateral axes (E = ex, N = ey) perpendicular to heading h."""
    up = (0.0, 0.0, 1.0) if abs(h[2]) <= 0.9 else (1.0, 0.0, 0.0)
    ey = _cross(h, up)
    n = math.sqrt(ey[0] ** 2 + ey[1] ** 2 + ey[2] ** 2)
    ey = (ey[0] / n, ey[1] / n, ey[2] / n)
    ex = _cross(ey, h)
    n = math.sqrt(ex[0] ** 2 + ex[1] ** 2 + ex[2] ** 2)
    return np.array(ex) / n, np.array(ey)


def slerp(h0, h1, f):
    om = math.acos(max(-1.0, min(1.0, float(h0 @ h1))))
    if om < 1e-9:
        return h1.copy()
    return (math.sin((1 - f) * om) * h0 + math.sin(f * om) * h1) / math.sin(om)


# ── Simulation ────────────────────────────────────────────────────────────────

def make_noise(seed, sigma_d=SIGMA_D, bias=None):
    rng = np.random.default_rng(seed)
    b = rng.normal(0.0, SIGMA_BIAS) if bias is None else bias
    phi = math.exp(-DT / TAU_D)
    innov = rng.standard_normal((N_MAX, 3)) * sigma_d * math.sqrt(1 - phi * phi)
    sens = rng.normal(0.0, SIGMA_WALL, (N_MAX, 4))
    odo = rng.normal(0.0, SIGMA_V, N_MAX)
    crack_s = rng.uniform(2.0, 48.0, N_CRACKS)
    crack_i = rng.uniform(20, 60, N_CRACKS)
    w0 = rng.standard_normal(3) * sigma_d
    return dict(b=b, phi=phi, innov=innov, sens=sens, odo=odo, crack_s=crack_s,
                crack_i=crack_i, w0=w0)


def simulate(strategy, seed, sigma_d=SIGMA_D, bias=None, keep=False):
    nz = make_noise(seed, sigma_d, bias)
    p = SEGS[0]['A'].copy()
    h = HEADINGS[0].copy()
    w = nz['w0'].copy()
    s_dr, p_dr = 0.0, p.copy()
    prev_e = None
    phase, n_bend, dist_phase, t_phase, h_old = 'straight', None, 0.0, 0.0, h.copy()
    since_turn = REFRACTORY            # distance since last turn (detection allowed)
    k_map = 0                          # next elbow index for map-based strategies
    contact, n_contact, near = 0, 0, 0
    prev_contact = False
    rho_sq, n_steps = 0.0, 0
    false_trig, turns = 0, 0
    detected = np.zeros(N_CRACKS, bool)
    s_hist = []
    log = dict(p=[], dr=[], rho=[], s=[], d=[], contact=[], event=[]) if keep else None
    status = 'timeout'
    s_true, rho = 0.0, 0.0
    for k in range(N_MAX):
        ex, ey = body_axes(h)
        dirs = (ey, -ey, ex, -ex)                      # N, S, E, W
        if strategy != 'A':
            near_s = near_segments(p)
            d = [ray(p, u, segs=near_s) - DRONE_R + nz['sens'][k, i] for i, u in enumerate(dirs)]
            d = [min(max(x, 0.0), MAX_RANGE) for x in d]
        else:
            d = [np.nan] * 4
        # --- bend handling ---
        event = ''
        if strategy in ('A', 'B'):
            if phase == 'straight' and k_map < len(ELBOW_S) and s_dr >= ELBOW_S[k_map]:
                phase, h_old, n_bend, t_phase = 'turn', h.copy(), HEADINGS[k_map + 1], 0.0
                k_map += 1; turns += 1; event = 'turn'
        elif phase == 'straight' and since_turn >= REFRACTORY:
            dn, ds, de, dw = d
            cand = None
            if strategy == 'C':
                if abs(dn - ds) > D_BEND:
                    cand = ey if dn > ds else -ey
                elif abs(de - dw) > D_BEND:
                    cand = ex if de > dw else -ex
            else:
                mx = int(np.argmax(d))
                if d[mx] > GATE:
                    cand = dirs[mx]
            if cand is not None:
                ax = int(np.argmax(np.abs(cand)))
                n_bend = np.zeros(3); n_bend[ax] = np.sign(cand[ax])
                s_c, _ = project(p)
                real = any(abs(s_c - e) < 1.0 for e in ELBOW_S)
                false_trig += 0 if real else 1
                turns += 1
                h_old = h.copy()
                if strategy == 'C':
                    phase, t_phase = 'turn', 0.0
                else:
                    phase, dist_phase = 'approach', 0.0
                event = 'detect' if real else 'false'
        if phase == 'approach' and dist_phase >= D_DELAY:
            phase, t_phase = 'turn', 0.0
        if phase == 'turn':
            t_phase += DT
            h = slerp(h_old, n_bend, min(1.0, t_phase / T_BLEND))
            h /= np.linalg.norm(h)
            if t_phase >= T_BLEND:
                h = n_bend.copy()
                phase, dist_phase, since_turn = 'settle', 0.0, 0.0
        if phase == 'settle' and dist_phase >= SETTLE:
            phase = 'straight'
        # --- centring command (PD on reading differences) ---
        u = np.zeros(3)
        if strategy != 'A':
            e = np.array([d[0] - d[1], d[2] - d[3]])
            de = (e - prev_e) / DT if prev_e is not None else np.zeros(2)
            prev_e = e
            cmd = np.clip(KP * e + KD * de, -U_MAX, U_MAX)
            if strategy in ('B', 'D'):
                # a reading longer than any straight pipe allows is an opening, not a wall:
                # do not centre on that axis (otherwise the drone is pulled into the elbow)
                cmd[0] *= max(d[0], d[1]) <= GATE
                cmd[1] *= max(d[2], d[3]) <= GATE
            ex, ey = body_axes(h)
            for c_val, axis in ((cmd[0], ey), (cmd[1], ex)):
                # suppress the axis that points into the bend while the opening is beside us
                if phase in ('approach', 'turn') and abs(float(axis @ n_bend)) > 0.5:
                    continue
                if phase == 'turn' and abs(float(axis @ h_old)) > 0.5:
                    continue
                if phase == 'settle' and abs(float(axis @ h_old)) > 0.5 and dist_phase < SETTLE:
                    continue
                u += c_val * axis
        # --- motion with disturbance and wall contact ---
        w = nz['phi'] * w + nz['innov'][k]
        v = V_ADV * h + u + w
        p_new = p + v * DT
        hit = not valid(p_new)
        if hit:
            p_new = clamp(p_new)
        step = float(np.linalg.norm(p_new - p))
        assert step <= (V_ADV + 2 * U_MAX + 1.0) * DT, 'teleport'
        assert valid(p_new), 'drone body outside the pipe'
        p = p_new
        contact += hit
        n_contact += hit and not prev_contact
        prev_contact = hit
        if strategy != 'A':
            near += min(d) < D_CONTACT
        # --- dead reckoning (card: integrate the commanded advance speed) ---
        v_meas = V_ADV * (1.0 + nz['b']) + nz['odo'][k]
        s_dr += v_meas * DT
        p_dr = p_dr + v_meas * DT * h
        dist_phase += V_ADV * DT
        since_turn += V_ADV * DT
        s_true, rho = project(p)
        rho_sq += rho * rho; n_steps += 1
        hit_c = (~detected) & (np.abs(s_true - nz['crack_s']) < DS_CRACK) & (nz['crack_i'] < I_THRESHOLD)
        detected |= hit_c
        s_hist.append(s_true)
        if keep:
            log['p'].append(p.copy()); log['dr'].append(p_dr.copy()); log['rho'].append(rho)
            log['s'].append(s_true); log['d'].append(d); log['contact'].append(hit)
            log['event'].append(event)
        if s_true >= PIPE_LEN - 0.05:
            status = 'complete'
            break
        m = int(STUCK_T / DT)
        if len(s_hist) > m and s_hist[-1] - s_hist[-1 - m] < 0.05:
            status = 'stuck'
            break
    s_reached = max(s_hist)
    dr_err = float(np.linalg.norm(p_dr - p))
    res = dict(strategy=strategy, seed=seed, status=status, s_reached=s_reached,
               contact_frac=contact / n_steps, n_contact=n_contact, near_frac=near / n_steps,
               rms=math.sqrt(rho_sq / n_steps), cracks=int(detected.sum()),
               cracks_reachable=int(np.sum(nz['crack_s'] <= s_reached)), false_trig=false_trig,
               turns=turns, t=n_steps * DT, bias=nz['b'], dr_err=dr_err, s_dr_err=s_dr - s_true)
    if keep:
        res['log'] = {kk: np.array(vv) if kk != 'event' else vv for kk, vv in log.items()}
        res['crack_s'] = nz['crack_s']
        res['detected'] = detected
    return res


def run_simulation():
    single = {st: simulate(st, SEED, keep=True) for st in STRATS}
    mc = {st: [simulate(st, s) for s in range(N_MC)] for st in STRATS}
    bias_sweep = [[simulate('B', s, bias=b) for s in range(N_SWEEP)] for b in BIAS_SWEEP]
    sig_sweep = {st: [[simulate(st, s, sigma_d=sg) for s in range(N_SWEEP)] for sg in SIGMA_SWEEP]
                 for st in ('A', 'B', 'D')}
    return single, mc, bias_sweep, sig_sweep


def summ(runs):
    st = np.array([r['status'] for r in runs])
    return dict(complete=float(np.mean(st == 'complete')), stuck=float(np.mean(st == 'stuck')),
                contact=float(np.mean([r['contact_frac'] for r in runs])),
                contact_sd=float(np.std([r['contact_frac'] for r in runs])),
                rms=float(np.mean([r['rms'] for r in runs])),
                rms_sd=float(np.std([r['rms'] for r in runs])),
                cracks=float(np.mean([r['cracks'] for r in runs])),
                s_reached=float(np.mean([r['s_reached'] for r in runs])),
                false_trig=float(np.mean([r['false_trig'] for r in runs])),
                n_contact=float(np.mean([r['n_contact'] for r in runs])))


# ── Plotting ──────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name, dpi=140):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=dpi, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _tube(ax):
    th = np.linspace(0, 2 * np.pi, 24)
    for sg in SEGS:
        ex, ey = body_axes(sg['a'])
        for a in np.arange(0, sg['L'] + 1e-9, 1.0):
            c = sg['A'] + sg['a'] * a
            ring = c[:, None] + PIPE_R * (np.outer(ex, np.cos(th)) + np.outer(ey, np.sin(th)))
            ax.plot(*ring, color='grey', lw=0.4, alpha=0.35)


def plot_pipe_3d(single, out_dir):
    fig = plt.figure(figsize=(11, 8.5))
    ax = fig.add_subplot(111, projection='3d')
    _tube(ax)
    cl = np.array(CORNERS)
    ax.plot(*cl.T, 'k--', lw=1, label='Centreline')
    r = single['D']
    P, DR = r['log']['p'], r['log']['dr']
    ax.plot(*P.T, color=COLORS['D'], lw=1.5, label='True path (D)')
    ax.plot(*DR.T, ':', color='orange', lw=1.5, label='Dead-reckoning estimate (D)')
    ax.scatter(*cl[1:4].T, color='purple', s=70, label='Elbows')
    for cs, det in zip(r['crack_s'], r['detected']):
        ax.scatter(*centreline(cs), marker='x', s=70, color='green' if det else 'grey')
    ax.scatter([], [], marker='x', color='green', label='Crack passed (flagged)')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_box_aspect((22, 14, 16))
    ax.view_init(elev=22, azim=-55)
    ax.set_title(f"S080 Pipe Navigation, strategy D (seed {SEED}): {r['status']}, "
                 f"RMS centring error {100 * r['rms']:.1f} cm")
    ax.legend(fontsize=8, loc='upper left')
    _save(fig, out_dir, 'pipe_3d.png')


def plot_centering(single, out_dir):
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    for ax, st in zip(axes, STRATS):
        r = single[st]
        L = r['log']
        ax.plot(L['s'], 100 * L['rho'], color=COLORS[st], lw=0.8)
        c = L['contact']
        if c.any():
            ax.fill_between(L['s'], 0, 30, where=c, color='red', alpha=0.25, step='mid',
                            label='wall contact')
        ax.axhline(100 * BUDGET, color='r', ls='--', lw=1, label='15 cm budget')
        ax.axhline(3, color='g', ls='--', lw=1, label='3 cm target')
        for e in ELBOW_S:
            ax.axvline(e, color='purple', ls=':', lw=1)
        ax.set_ylim(0, 30)
        ax.set_ylabel('ρ (cm)')
        ax.set_title(f"{LABELS[st]} — {r['status']} at s = {r['s_reached']:.1f} m, "
                     f"RMS {100 * r['rms']:.1f} cm, contact {100 * r['contact_frac']:.1f}% of time",
                     fontsize=9)
        ax.legend(fontsize=7, loc='upper right')
    axes[-1].set_xlabel('Arc length s (m); dotted = elbows')
    fig.suptitle(f'S080 Distance from the Pipe Centreline (seed {SEED})')
    plt.tight_layout()
    _save(fig, out_dir, 'centering_error.png')


def plot_wall_readings(single, out_dir):
    r = single['D']
    L = r['log']
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5), sharey=True)
    names = ['N', 'S', 'E', 'W']
    for ax, e in zip(axes, ELBOW_S):
        m = (L['s'] > e - 1.2) & (L['s'] < e + 0.8)
        for i in range(4):
            ax.plot(L['s'][m], L['d'][m, i], lw=1, label=names[i])
        ev = [i for i in np.where(m)[0] if L['event'][i] in ('detect', 'false')]
        for i in ev:
            ax.axvline(L['s'][i], color='k', ls='--', lw=1, label='bend detected')
        ax.axhline(GATE, color='red', ls=':', lw=1, label=f'gate {GATE:.2f} m')
        ax.axvline(e, color='purple', ls=':', lw=1, label='elbow centre')
        ax.set_xlabel('Arc length s (m)')
        ax.set_title(f'Elbow at s = {e:.0f} m', fontsize=10)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel('Ultrasonic skin-to-wall reading (m)')
    axes[0].legend(fontsize=7, loc='upper left')
    fig.suptitle(f'S080 Ultrasonic Readings around the Elbows, strategy D (seed {SEED})')
    plt.tight_layout()
    _save(fig, out_dir, 'wall_readings.png')


def plot_elbow_paths(single, out_dir):
    """Paths through the first elbow (x-z plane) for the strategies that use turns."""
    fig, ax = plt.subplots(figsize=(7, 7))
    C = CORNERS[1]
    ax.add_patch(plt.Rectangle((C[0] - 3, -PIPE_R), 3 + PIPE_R, 2 * PIPE_R, fc='#EEEEEE', ec='k'))
    ax.add_patch(plt.Rectangle((C[0] - PIPE_R, -PIPE_R), 2 * PIPE_R, 3 + PIPE_R, fc='#EEEEEE',
                               ec='k'))
    for st in STRATS:
        P = single[st]['log']['p']
        m = (np.abs(P[:, 0] - C[0]) < 3.2) & (np.abs(P[:, 2] - C[2]) < 3.2)
        ax.plot(P[m, 0], P[m, 2], color=COLORS[st], lw=1.5, label=LABELS[st])
        if single[st]['status'] == 'stuck' and abs(P[-1, 0] - C[0]) < 3 and P[-1, 2] < 3:
            ax.plot(P[-1, 0], P[-1, 2], 'x', color=COLORS[st], ms=12, mew=3)
    ax.plot([C[0] - 3, C[0]], [0, 0], 'k--', lw=0.8)
    ax.plot([C[0], C[0]], [0, 3], 'k--', lw=0.8)
    ax.set_xlim(C[0] - 1.5, C[0] + 0.6); ax.set_ylim(-0.6, 1.5)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)'); ax.set_ylabel('z (m)')
    ax.set_title(f'S080 First Elbow (x→z) Seen from the Side (seed {SEED}); × = stuck', fontsize=10)
    ax.legend(fontsize=7, loc='upper left')
    _save(fig, out_dir, 'elbow_paths.png')


def plot_monte_carlo(sm, bias_sweep, sig_sweep, out_dir):
    fig, axes = plt.subplots(2, 3, figsize=(17, 9))
    x = np.arange(len(STRATS))
    cols = [COLORS[s] for s in STRATS]
    short = [s for s in STRATS]
    a = axes[0, 0]
    a.bar(x, [100 * sm[s]['complete'] for s in STRATS], color=cols, edgecolor='k')
    for i, s in enumerate(STRATS):
        a.text(i, 100 * sm[s]['complete'] + 2, f"{100 * sm[s]['complete']:.0f}%", ha='center')
    a.set_ylim(0, 115); a.set_ylabel('% of runs reaching s = 50 m'); a.set_title('Mission completion')
    a = axes[0, 1]
    a.bar(x, [100 * sm[s]['contact'] for s in STRATS],
          yerr=[100 * sm[s]['contact_sd'] for s in STRATS], capsize=5, color=cols, edgecolor='k')
    a.set_ylabel('% of flight time in wall contact'); a.set_title('Wall contact')
    a = axes[0, 2]
    a.bar(x, [100 * sm[s]['rms'] for s in STRATS], yerr=[100 * sm[s]['rms_sd'] for s in STRATS],
          capsize=5, color=cols, edgecolor='k')
    a.axhline(3, color='g', ls='--', lw=1, label='3 cm target')
    a.set_ylabel('RMS centring error (cm)'); a.set_title('Centring error'); a.legend(fontsize=8)
    for a in axes[0]:
        a.set_xticks(x); a.set_xticklabels(short)
        a.grid(alpha=0.3, axis='y')
    a = axes[1, 0]
    sb = [summ(rs) for rs in bias_sweep]
    a.plot(100 * np.array(BIAS_SWEEP), [100 * s['complete'] for s in sb], 'o-', color=COLORS['B'],
           label='completion (%)')
    a.plot(100 * np.array(BIAS_SWEEP), [100 * s['contact'] for s in sb], 's--', color='k',
           label='contact time (%)')
    a.set_xlabel('Odometry scale bias (%)'); a.set_title(f'Strategy B vs odometry bias ({N_SWEEP} seeds)')
    a.legend(fontsize=8); a.grid(alpha=0.3)
    a = axes[1, 1]
    for st in ('A', 'B', 'D'):
        ss = [summ(rs) for rs in sig_sweep[st]]
        a.plot(SIGMA_SWEEP, [100 * s['contact'] for s in ss], 'o-', color=COLORS[st],
               label=f'{LABELS[st]}: contact (%)')
        a.plot(SIGMA_SWEEP, [100 * s['rms'] for s in ss], 's--', color=COLORS[st],
               label=f'{LABELS[st]}: RMS (cm)')
    a.set_xlabel('Air-flow disturbance σ_d (m/s)')
    a.set_title(f'Disturbance sweep ({N_SWEEP} seeds)'); a.legend(fontsize=7); a.grid(alpha=0.3)
    a = axes[1, 2]
    a.bar(x, [sm[s]['cracks'] for s in STRATS], color=cols, edgecolor='k')
    a.bar(x, [sm[s]['false_trig'] for s in STRATS], color='none', edgecolor='k', hatch='//',
          label='false bend triggers / run')
    a.axhline(N_CRACKS, color='k', ls='--', lw=1)
    a.set_xticks(x); a.set_xticklabels(short)
    a.set_title('Cracks passed (flagged) per run, of 10'); a.legend(fontsize=8)
    fig.suptitle(f'S080 Monte Carlo ({N_MC} seeds, same disturbances / noise / bias for every '
                 f'strategy)')
    plt.tight_layout()
    _save(fig, out_dir, 'monte_carlo.png')


def save_animation(single, out_dir):
    r = single['D']
    P, DR = r['log']['p'], r['log']['dr']
    n = len(P)
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    fig = plt.figure(figsize=(8, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    _tube(ax)
    cl = np.array(CORNERS)
    ax.plot(*cl.T, 'k--', lw=0.8)
    ax.scatter(*cl[1:4].T, color='purple', marker='D', s=40)
    cpts = np.array([centreline(cs) for cs in r['crack_s']])
    csc = ax.scatter(*cpts.T, marker='x', s=60, color='grey', depthshade=False)
    trail, = ax.plot([], [], [], color=COLORS['D'], lw=1.2)
    drl, = ax.plot([], [], [], ':', color='orange', lw=1.2)
    dot, = ax.plot([], [], [], 'o', color=COLORS['D'], mec='k', ms=7)
    ax.set_box_aspect((22, 14, 16))
    ax.view_init(elev=22, azim=-55)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    title = ax.set_title('')

    def update(f):
        i = min(f * ANIM_STEP, n - 1)
        trail.set_data(P[:i + 1, 0], P[:i + 1, 1]); trail.set_3d_properties(P[:i + 1, 2])
        drl.set_data(DR[:i + 1, 0], DR[:i + 1, 1]); drl.set_3d_properties(DR[:i + 1, 2])
        dot.set_data([P[i, 0]], [P[i, 1]]); dot.set_3d_properties([P[i, 2]])
        s_now = r['log']['s'][i]
        seen = np.abs(r['crack_s'] - s_now) < DS_CRACK
        passed = r['crack_s'] < s_now
        csc.set_color(['green' if (p_ or q_) else 'grey' for p_, q_ in zip(passed, seen)])
        title.set_text(f'S080 strategy D, t = {(i + 1) * DT:.0f} s, s = {s_now:.1f} m, '
                       f'ρ = {100 * r["log"]["rho"][i]:.1f} cm\n'
                       f'red = true path, orange dotted = dead reckoning')
        return []

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    single, mc, bias_sweep, sig_sweep = run_simulation()
    sm = {st: summ(mc[st]) for st in STRATS}
    print('=== S080 Underground Pipe CCTV Replacement — Results ===')
    p0 = SEGS[0]['A'] + np.array([3.0, 0, 0])
    ex0, ey0 = body_axes(HEADINGS[0])
    print(f'  Centred straight-pipe reading: {ray(p0, ey0) - DRONE_R:.3f} m; '
          f'reading into the first elbow opening from 0.2 m before its centre: '
          f'{min(ray(CORNERS[1] - np.array([0.2, 0, 0]), np.array([0, 0, 1.0])) - DRONE_R, MAX_RANGE):.3f} m')
    h, nb = np.array([1.0, 0, 0]), np.array([0, 0, 1.0])
    print(f'  Card heading update h x n_bend for h = +x, n = +z: {np.cross(h, nb)} '
          f'(the bend direction is +z)')
    print(f'  Dead-reckoning noise only: position std after 250 s = '
          f'{SIGMA_V * math.sqrt(DT * 250):.4f} m (card states sigma_v sqrt(t) = '
          f'{SIGMA_V * math.sqrt(250):.3f} m)')
    for st in STRATS:
        r = single[st]
        print(f'  Seed {SEED} [{LABELS[st]}]: {r["status"]} at s = {r["s_reached"]:.2f} m after '
              f'{r["t"]:.1f} s; RMS rho {100 * r["rms"]:.2f} cm; contact {100 * r["contact_frac"]:.1f}% '
              f'({r["n_contact"]} events); near-contact {100 * r["near_frac"]:.1f}%; cracks '
              f'{r["cracks"]}/{N_CRACKS}; false triggers {r["false_trig"]}; odometry bias '
              f'{100 * r["bias"]:+.2f}%; final DR error {r["dr_err"]:.2f} m')
    print(f'  --- Monte Carlo, {N_MC} seeds (bias ~ N(0, {100 * SIGMA_BIAS:.0f}%), '
          f'sigma_d = {SIGMA_D} m/s) ---')
    for st in STRATS:
        s = sm[st]
        print(f'  [{LABELS[st]}] complete {100 * s["complete"]:.0f}%, stuck {100 * s["stuck"]:.0f}%, '
              f'mean s reached {s["s_reached"]:.1f} m, contact {100 * s["contact"]:.2f} ± '
              f'{100 * s["contact_sd"]:.2f}% of time ({s["n_contact"]:.1f} events), RMS rho '
              f'{100 * s["rms"]:.2f} ± {100 * s["rms_sd"]:.2f} cm, cracks {s["cracks"]:.1f}/{N_CRACKS}, '
              f'false triggers {s["false_trig"]:.2f}')
    cd = np.array([r['status'] == 'complete' for r in mc['D']])
    cb = np.array([r['status'] == 'complete' for r in mc['B']])
    print(f'  Paired completion D vs B: both {int(np.sum(cd & cb))}, only D {int(np.sum(cd & ~cb))}, '
          f'only B {int(np.sum(~cd & cb))}, neither {int(np.sum(~cd & ~cb))}')
    bstat = [(r['bias'], r['status']) for r in mc['B']]
    print('  Strategy B outcome by seed bias: ' + ', '.join(
        f'{100 * b:+.1f}%:{s[0]}' for b, s in sorted(bstat)))
    print('  Bias sweep, strategy B (bias: complete %, contact %):')
    for b, rs in zip(BIAS_SWEEP, bias_sweep):
        s = summ(rs)
        print(f'    {100 * b:+.0f}%: {100 * s["complete"]:.0f}%, {100 * s["contact"]:.2f}%')
    print('  Disturbance sweep (sigma_d: contact % / RMS cm / complete % per strategy):')
    for i, sg in enumerate(SIGMA_SWEEP):
        parts = []
        for st in ('A', 'B', 'D'):
            s = summ(sig_sweep[st][i])
            parts.append(f'{st} {100 * s["contact"]:.2f}% / {100 * s["rms"]:.2f} cm / '
                         f'{100 * s["complete"]:.0f}%')
        print(f'    {sg:.2f} m/s: ' + '; '.join(parts))

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_pipe_3d(single, out)
    plot_centering(single, out)
    plot_wall_readings(single, out)
    plot_elbow_paths(single, out)
    plot_monte_carlo(sm, bias_sweep, sig_sweep, out)
    save_animation(single, out)
