"""
S044 3D Upgrade — Wall Crack Inspection
=======================================
The S044 camera (60 deg FOV, 2048 px, 1.5 m standoff, per-frame detection
P = Phi((w - w_min(r)) / 0.05 mm)) and standoff PD loop (Kp = 16, Kd = 8,
0.05 m range noise) are applied to two 3D structures:

  A  cylindrical chimney  R_c = 4 m, H = 20 m, orbit radius 5.5 m
       A1 card helix       : 15 turns, pitch 1.386 m, from z = 0.5 m, clipped at 19.5 m
       A2 bounded helix    : level orbit at 0.5 m, 14 turns of pitch 19/14 m, level orbit at 19.5 m
  B  square building     20 x 20 m footprint, four 20 x 20 m faces
       B1 card             : every face scanned bottom-up (restart at z = 0.5 m after each corner)
       B2 alternating      : faces scanned up, down, up, down (no descent at the corners)

Motion: 3D double integrator.  The reference point moves along the path at 2 m/s.  Along
the local outward surface normal n_hat (from the nearest surface point) the card PD acts on
the standoff error e_n = d_ref - d_meas (d_ref = the reference point's own distance to the
surface: 1.5 m on the faces / orbit, up to 2.0 m at a corner waypoint; d_meas has 5 cm
noise).  In the tangent plane a PD with the same omega_n tracks the reference point.  On
the cylinder only the normal PD can supply the centripetal acceleration v^2/r, so it needs
a standing error v^2/(r Kp); a variant with -v_t^2/r as feed-forward is also run.
The card's "flat-wall controller on a cylinder" (standoff measured along a fixed x axis)
is run for one orbit as a check of the card's claim.

Camera: pinhole with boresight -n_hat; a surface cell / crack is in the frame when its
horizontal and vertical image angles are both <= 30 deg (on a flat wall at 1.5 m this is
the S044 square |dy|, |dz| <= 0.866 m).  The card's viewing-angle gate additionally needs
cos(psi) = (p - s_c).n_c / r >= cos 30 deg.  Detection draws one uniform number per
(frame, crack) and uses it for both the gated and the ungated rule (common random numbers).

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s044_3d_wall_crack.py
"""

import os
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.special import ndtr

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('s044_2d', os.path.join(_HERE, '..', 's044_wall_crack.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '03_environmental_sar', '3d', 's044_3d_wall_crack'))

# ── Parameters ──────────────────────────────────────────────────────────────
R_C        = 4.0                    # m — cylinder radius
H          = 20.0                   # m — structure height
HALF_B     = 10.0                   # m — building half width (20 x 20 m footprint)
D_STANDOFF = m2d.D_STANDOFF         # 1.5 m
R_ORBIT    = R_C + D_STANDOFF       # 5.5 m
D_CORNER   = D_STANDOFF + 0.5       # m — corner waypoint offset along the bisector
V_CRUISE   = m2d.V_CRUISE           # 2.0 m/s
ALPHA      = m2d.ALPHA              # 30 deg half FOV
TAN_A      = np.tan(ALPHA)
H_STRIP    = m2d.H_STRIP            # 0.866 m
DELTA_Z    = m2d.DELTA_Z            # 1.386 m
N_PASS     = m2d.N_PASS             # 15
Z_MIN, Z_MAX = m2d.Z_MIN, m2d.Z_MAX # 0.5, 19.5 m
KP, KD     = m2d.KP_X, m2d.KD_X     # 16, 8
SIGMA_X    = m2d.SIGMA_X            # 0.05 m range noise
K_TRACK_P  = 16.0                   # 1/s^2 — tangent-plane path tracking (added, same omega_n = 4 rad/s)
K_TRACK_D  = 8.0                    # 1/s
DT         = m2d.DT                 # 0.05 s
N_CRACKS   = m2d.N_CRACKS           # 40 per structure
CAM_RES_PX = m2d.CAM_RES_PX
SIGMA_W    = m2d.SIGMA_NOISE        # 0.05 mm
DPHI_DEG   = 0.5                    # coverage grid (cylinder)
DZ_GRID    = 0.1
DU_GRID    = 0.1                    # coverage grid (faces)
N_SEEDS    = 20                     # crack layouts for the detection statistics
SEED_DYN   = 0                      # range-noise seed
N_FRAMES   = 120
BINS       = [0.0, 0.5, 1.0, 1.5, 2.0]
R_NEAR     = 2.5                    # m — "near" views (slant range); farther in-frame views are grazing

# Building faces: outward normal, start corner, unit direction along the face (u)
FACES = [
    (np.array([1.0, 0.0]),  np.array([HALF_B,  HALF_B]),  np.array([0.0, -1.0])),
    (np.array([0.0, -1.0]), np.array([HALF_B, -HALF_B]),  np.array([-1.0, 0.0])),
    (np.array([-1.0, 0.0]), np.array([-HALF_B, -HALF_B]), np.array([0.0, 1.0])),
    (np.array([0.0, 1.0]),  np.array([-HALF_B, HALF_B]),  np.array([1.0, 0.0])),
]
FACE_COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:purple']


# ── Surface geometry ────────────────────────────────────────────────────────

def nearest_cyl(p):
    r = np.hypot(p[0], p[1])
    n = np.array([p[0] / r, p[1] / r, 0.0])
    return r - R_C, n


def nearest_box(p):
    q = np.clip(p[:2], -HALF_B, HALF_B)
    d = p[:2] - q
    dist = np.linalg.norm(d)
    return dist, np.array([d[0] / dist, d[1] / dist, 0.0])


# ── Paths (polylines) ───────────────────────────────────────────────────────

def helix_path(kind):
    pts = []
    if kind == 'card':
        th = np.arange(0, 2 * np.pi * N_PASS + 1e-9, np.deg2rad(1.0))
        z = np.clip(Z_MIN + DELTA_Z * th / (2 * np.pi), Z_MIN, Z_MAX)
        pts = np.column_stack([R_ORBIT * np.cos(th), R_ORBIT * np.sin(th), z])
    else:
        n_turn = int(np.ceil((Z_MAX - Z_MIN) / DELTA_Z))
        pitch = (Z_MAX - Z_MIN) / n_turn
        th = np.arange(0, 2 * np.pi * (n_turn + 2) + 1e-9, np.deg2rad(1.0))
        z = np.clip(Z_MIN + pitch * (th - 2 * np.pi) / (2 * np.pi), Z_MIN, Z_MAX)
        pts = np.column_stack([R_ORBIT * np.cos(th), R_ORBIT * np.sin(th), z])
    return pts


def face_point(i, u, z, off=D_STANDOFF):
    n, c0, du = FACES[i]
    xy = c0 + u * du + off * n
    return np.array([xy[0], xy[1], z])


def building_path(kind):
    heights = np.linspace(Z_MIN, Z_MAX, N_PASS)
    pts, corner_idx = [], []
    for i in range(4):
        hs = heights if (kind == 'card' or i % 2 == 0) else heights[::-1]
        for k, z in enumerate(hs):
            us = (0.0, 2 * HALF_B) if k % 2 == 0 else (2 * HALF_B, 0.0)
            for u in us:
                pts.append(face_point(i, u, z))
        if i < 3:
            n0, c1 = FACES[i][0], FACES[i + 1][1]
            b = n0 + FACES[i + 1][0]
            b = b / np.linalg.norm(b)
            corner_idx.append(len(pts))
            pts.append(np.array([c1[0] + D_CORNER * b[0], c1[1] + D_CORNER * b[1], pts[-1][2]]))
    return np.array(pts), corner_idx


def resample(pts):
    """Arc-length parameterisation of a polyline."""
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = np.concatenate([[True], seg > 1e-12])
    pts = pts[keep]
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    return pts, s


def ref_at(pts, s, sq):
    return np.array([np.interp(sq, s, pts[:, j]) for j in range(3)])


# ── Flight ──────────────────────────────────────────────────────────────────

def fly(structure, path_pts, ff=False, fixed_axis=False, t_limit=None, seed=SEED_DYN, dt=DT, sigma=SIGMA_X):
    """Fly the reference path; return the drone trajectory and standoff errors."""
    rng = np.random.default_rng(seed)
    near = nearest_cyl if structure == 'cyl' else nearest_box
    pts, s = resample(path_pts)
    L = s[-1]
    T = L / V_CRUISE if t_limit is None else t_limit
    n_steps = int(np.ceil(T / dt))
    p = pts[0].copy()
    v = np.zeros(3)
    out_p = np.zeros((n_steps + 1, 3)); out_ref = np.zeros((n_steps + 1, 3))
    e_n = np.zeros(n_steps + 1); d_true = np.zeros(n_steps + 1); d_ref = np.zeros(n_steps + 1)
    speed = np.zeros(n_steps + 1)
    n_clip = 0
    out_p[0] = p; out_ref[0] = p
    d0, _ = near(p)
    d_true[0] = d_ref[0] = d0
    for k in range(1, n_steps + 1):
        t = k * dt
        sq = min(V_CRUISE * t, L)
        pr = ref_at(pts, s, sq)
        pr_next = ref_at(pts, s, min(V_CRUISE * (t + dt), L))
        v_ref = (pr_next - pr) / dt
        dist, n = near(p)
        if fixed_axis:                                  # "flat-wall" law: standoff along a fixed x axis
            n = np.array([1.0, 0.0, 0.0])
            dist = p[0] - R_C
            dref = D_STANDOFF
        else:
            dref, _ = near(pr)
        d_meas = dist + rng.normal(0.0, sigma)
        # card PD along the surface normal
        a = (KP * (dref - d_meas) - KD * np.dot(v, n)) * n
        if ff and structure == 'cyl':                    # centripetal feed-forward -|v_t|^2 / r (inward)
            v_t = v - np.dot(v, n) * n
            a += -np.dot(v_t, v_t) / np.hypot(p[0], p[1]) * n
        # path tracking in the tangent plane (same omega_n = 4 rad/s)
        a_t = K_TRACK_P * (pr - p) + K_TRACK_D * (v_ref - v)
        a += a_t - np.dot(a_t, n) * n
        v = v + a * dt
        p = p + v * dt
        if p[2] < Z_MIN or p[2] > Z_MAX:                 # altitude limiter (as in S044): stop at the band edge
            n_clip += 1
            p[2] = np.clip(p[2], Z_MIN, Z_MAX)
            v[2] = 0.0
        speed[k] = np.linalg.norm(v)
        out_p[k] = p; out_ref[k] = pr
        dt_, _ = near(p)
        d_true[k] = dt_ if not fixed_axis else nearest_cyl(p)[0]
        d_ref[k] = dref
        e_n[k] = dref - d_true[k]
    if not fixed_axis:
        assert d_true.min() > 0.5, 'standoff collapsed'
    return dict(p=out_p, ref=out_ref, e_n=e_n, d_true=d_true, d_ref=d_ref, speed=speed, n_clip=n_clip,
                t=np.arange(n_steps + 1) * dt, length=float(np.sum(np.linalg.norm(np.diff(out_p, axis=0), axis=1))),
                ref_length=L, T=n_steps * dt)


# ── Camera model ────────────────────────────────────────────────────────────

def camera_axes(p, structure):
    _, n = nearest_cyl(p) if structure == 'cyl' else nearest_box(p)
    b = -n
    h = np.cross(b, np.array([0.0, 0.0, 1.0]))
    h /= np.linalg.norm(h)
    return b, h


def view_test(p, b, h, c, nc):
    """Pinhole in-frame test and viewing-angle gate for surface points c (M,3) with normals nc."""
    v = c - p
    depth = v @ b
    lat = v @ h
    vert = v[:, 2]
    r = np.linalg.norm(v, axis=1)
    facing = np.einsum('ij,ij->i', -v, nc) > 0
    inframe = facing & (depth > 0) & (np.abs(lat) <= depth * TAN_A) & (np.abs(vert) <= depth * TAN_A)
    cospsi = np.einsum('ij,ij->i', -v, nc) / np.maximum(r, 1e-9)
    gate = cospsi >= np.cos(ALPHA)
    return inframe, gate, r


def cyl_cells():
    ph = np.deg2rad(np.arange(DPHI_DEG / 2, 360, DPHI_DEG))
    zz = np.arange(DZ_GRID / 2, H, DZ_GRID)
    PH, ZZ = np.meshgrid(ph, zz, indexing='ij')
    c = np.column_stack([R_C * np.cos(PH.ravel()), R_C * np.sin(PH.ravel()), ZZ.ravel()])
    n = np.column_stack([np.cos(PH.ravel()), np.sin(PH.ravel()), np.zeros(PH.size)])
    return c, n, PH.shape


def box_cells():
    uu = np.arange(DU_GRID / 2, 2 * HALF_B, DU_GRID)
    zz = np.arange(DZ_GRID / 2, H, DZ_GRID)
    U, Z = np.meshgrid(uu, zz, indexing='ij')
    cs, ns = [], []
    for i in range(4):
        n, c0, du = FACES[i]
        xy = c0[None, :] + U.ravel()[:, None] * du[None, :]
        cs.append(np.column_stack([xy, Z.ravel()]))
        ns.append(np.tile([n[0], n[1], 0.0], (U.size, 1)))
    return np.vstack(cs), np.vstack(ns), (4,) + U.shape


def _candidate_cells(p, structure, shape):
    """Flat indices of the surface cells that can possibly be in frame from p."""
    if structure == 'cyl':
        nph, nz = shape
        iph0 = int(np.degrees(np.arctan2(p[1], p[0])) % 360 / DPHI_DEG)
        iph = np.arange(iph0 - 60, iph0 + 61) % nph                      # +-30 deg
        iz = np.arange(max(int((p[2] - 2.5) / DZ_GRID), 0), min(int((p[2] + 2.5) / DZ_GRID) + 1, nz))
        return (iph[:, None] * nz + iz[None, :]).ravel()
    _, nu, nz = shape
    out = []
    iz = np.arange(max(int((p[2] - 2.5) / DZ_GRID), 0), min(int((p[2] + 2.5) / DZ_GRID) + 1, nz))
    for i in range(4):
        n, c0, du = FACES[i]
        if np.dot(p[:2] - c0, n) <= 0:
            continue                                                     # behind this face's plane
        u = np.dot(p[:2] - c0, du)
        iu = np.arange(max(int((u - 8.0) / DU_GRID), 0), min(int((u + 8.0) / DU_GRID) + 1, nu))
        if len(iu):
            out.append((i * nu * nz + iu[:, None] * nz + iz[None, :]).ravel())
    return np.concatenate(out) if out else np.zeros(0, int)


def coverage(fl, structure, every=2):
    """Fraction of surface cells inside at least one frame (ungated / gated)."""
    c, nc, shape = cyl_cells() if structure == 'cyl' else box_cells()
    cov = np.zeros(len(c), dtype=bool); cov_g = np.zeros(len(c), dtype=bool)
    for p in fl['p'][::every]:
        sel = _candidate_cells(p, structure, shape)
        if len(sel) == 0:
            continue
        b, h = camera_axes(p, structure)
        inf, g, _ = view_test(p, b, h, c[sel], nc[sel])
        cov[sel[inf]] = True
        cov_g[sel[inf & g]] = True
    return cov.mean(), cov_g.mean(), cov.reshape(shape), cov_g.reshape(shape)


def make_cracks(structure, seed):
    rng = np.random.default_rng([seed, 42])
    z = rng.uniform(0.5, H - 0.5, N_CRACKS)
    w = rng.uniform(0.1, 2.0, N_CRACKS)
    if structure == 'cyl':
        ph = rng.uniform(0, 2 * np.pi, N_CRACKS)
        c = np.column_stack([R_C * np.cos(ph), R_C * np.sin(ph), z])
        n = np.column_stack([np.cos(ph), np.sin(ph), np.zeros(N_CRACKS)])
        aux = ph
    else:
        f = rng.integers(0, 4, N_CRACKS)
        u = rng.uniform(0.5, 2 * HALF_B - 0.5, N_CRACKS)
        c = np.array([[*(FACES[fi][1] + ui * FACES[fi][2]), zi] for fi, ui, zi in zip(f, u, z)])
        n = np.array([[FACES[fi][0][0], FACES[fi][0][1], 0.0] for fi in f])
        aux = f
    return c, n, w, aux


def w_min(r):
    return 2.0 * r * TAN_A / CAM_RES_PX * 1000.0


def frame_axes(fl, structure):
    """Camera boresight and horizontal image axis for every frame (cached in fl)."""
    if 'B' not in fl:
        ax = [camera_axes(p, structure) for p in fl['p'][1:]]
        fl['B'] = np.array([a[0] for a in ax]); fl['Hx'] = np.array([a[1] for a in ax])
    return fl['p'][1:], fl['B'], fl['Hx']


def detect(fl, structure, cracks, seed):
    """Per-frame detection with and without the viewing-angle gate (same uniforms),
    vectorised over frames (F) and cracks (N)."""
    c, nc, w, _ = cracks
    P, Bf, Hf = frame_axes(fl, structure)
    rng = np.random.default_rng([seed, 7])
    U = rng.random((len(P), len(w)))
    v = c[None, :, :] - P[:, None, :]                       # (F, N, 3)
    depth = np.einsum('fnj,fj->fn', v, Bf)
    lat = np.einsum('fnj,fj->fn', v, Hf)
    vert = v[:, :, 2]
    r = np.linalg.norm(v, axis=2)
    facing_dot = np.einsum('fnj,nj->fn', -v, nc)
    inf = (facing_dot > 0) & (depth > 0) & (np.abs(lat) <= depth * TAN_A) & (np.abs(vert) <= depth * TAN_A)
    g = facing_dot / np.maximum(r, 1e-9) >= np.cos(ALPHA)
    pd = ndtr((w[None, :] - w_min(r)) / SIGMA_W)
    hit = inf & (U < pd)
    det = hit.any(axis=0)
    det_g = (hit & g).any(axis=0)
    near_ = r <= R_NEAR
    return (det, det_g, inf.sum(axis=0), (inf & ~g).sum(axis=0),
            (inf & near_).sum(axis=0), (inf & near_ & ~g).sum(axis=0))


def gate_profile(structure, n=401):
    """Fraction of the in-frame image that the gate rejects, vs lateral offset on the surface
    (drone on the nominal standoff, crack at the drone's height)."""
    if structure == 'cyl':
        p = np.array([R_ORBIT, 0.0, 10.0])
        dphi = np.linspace(-0.3, 0.3, n)
        c = np.column_stack([R_C * np.cos(dphi), R_C * np.sin(dphi), np.full(n, 10.0)])
        nc = np.column_stack([np.cos(dphi), np.sin(dphi), np.zeros(n)])
        off = R_C * dphi
    else:
        p = np.array([HALF_B + D_STANDOFF, 0.0, 10.0])
        y = np.linspace(-1.2, 1.2, n)
        c = np.column_stack([np.full(n, HALF_B), y, np.full(n, 10.0)])
        nc = np.tile([1.0, 0.0, 0.0], (n, 1))
        off = y
    b, h = camera_axes(p, structure)
    inf, g, r = view_test(p, b, h, c, nc)
    return off, inf, g


# ── Run ─────────────────────────────────────────────────────────────────────

def summarize_flight(name, fl, extra=''):
    seg = fl['e_n'][1:]
    print(f'{name:28s}: ref path {fl["ref_length"]:.1f} m, flown {fl["length"]:.1f} m, time {fl["T"]:.1f} s, '
          f'e_n RMS {100 * np.sqrt(np.mean(seg ** 2)):.2f} cm, mean {100 * seg.mean():+.2f} cm, '
          f'max |e_n| {100 * np.abs(seg).max():.2f} cm, min standoff {fl["d_true"].min():.3f} m, '
          f'max speed {fl["speed"].max():.2f} m/s, altitude-limiter steps {fl["n_clip"]}{extra}')


def run_simulation():
    print(f'h_strip = 1.5 tan30 = {H_STRIP:.3f} m, delta_z = {DELTA_Z:.3f} m, N_pass = {N_PASS}')
    print(f'Card helix: 15 turns x {DELTA_Z:.3f} m from z = 0.5 m would end at {Z_MIN + N_PASS * DELTA_Z:.2f} m '
          f'(> z_max = {Z_MAX} m); L_helix (card formula) = '
          f'{2 * np.pi * N_PASS * np.sqrt(R_ORBIT ** 2 + (DELTA_Z / (2 * np.pi)) ** 2):.1f} m')
    n_turn = int(np.ceil((Z_MAX - Z_MIN) / DELTA_Z))
    print(f'Bounded helix: {n_turn} turns of pitch {(Z_MAX - Z_MIN) / n_turn:.4f} m between two level orbits')
    print(f'Centripetal acceleration on the orbit v^2/r = {V_CRUISE ** 2 / R_ORBIT:.4f} m/s^2 -> PD-only '
          f'steady-state standoff offset {100 * V_CRUISE ** 2 / R_ORBIT / KP:.2f} cm (outward)')
    flights = {}
    flights['A1'] = fly('cyl', helix_path('card'))
    flights['A2'] = fly('cyl', helix_path('bounded'))
    flights['A2ff'] = fly('cyl', helix_path('bounded'), ff=True)
    bp1, ci1 = building_path('card')
    bp2, ci2 = building_path('alt')
    flights['B1'] = fly('box', bp1)
    flights['B2'] = fly('box', bp2)
    flights['A_fixed'] = fly('cyl', helix_path('bounded'), fixed_axis=True, t_limit=2 * np.pi * R_ORBIT / V_CRUISE)
    names = {'A1': 'A1 cylinder, card helix', 'A2': 'A2 cylinder, bounded helix',
             'A2ff': 'A2 + centripetal feed-forward', 'B1': 'B1 building, card (all up)',
             'B2': 'B2 building, alternating', 'A_fixed': 'Fixed-x-axis PD on cylinder'}
    print('=== Flights ===')
    for k in ['A1', 'A2', 'A2ff', 'B1', 'B2']:
        summarize_flight(names[k], flights[k])
    # orbit-only standoff bias (A2, middle of the mission, away from the start transient)
    fl = flights['A2']; m = (fl['t'] > 20) & (fl['t'] < fl['T'] - 5)
    fl2 = flights['A2ff']
    print(f'A2 steady orbit (t > 20 s): mean e_n {100 * fl["e_n"][m].mean():+.2f} cm, RMS '
          f'{100 * np.sqrt(np.mean(fl["e_n"][m] ** 2)):.2f} cm; with feed-forward: mean '
          f'{100 * fl2["e_n"][m].mean():+.2f} cm, RMS {100 * np.sqrt(np.mean(fl2["e_n"][m] ** 2)):.2f} cm')
    print('Integration-step check (noiseless, first 3000 deg of the A2 helix, mean e_n for t > 20 s):')
    for dt_chk in (0.05, 0.01, 0.005):
        vals = []
        for ff in (False, True):
            fc = fly('cyl', helix_path('bounded')[:3000], ff=ff, dt=dt_chk, sigma=0.0)
            vals.append(100 * fc['e_n'][fc['t'] > 20].mean())
        print(f'   dt = {dt_chk:.3f} s: PD only {vals[0]:+.2f} cm, with feed-forward {vals[1]:+.2f} cm')
    fa = flights['A_fixed']
    bad = np.nonzero(np.abs(fa['d_true'] - D_STANDOFF) > 0.5)[0]
    print(f'Fixed-x-axis standoff law on the cylinder (one orbit, {fa["T"]:.1f} s): true standoff '
          f'{fa["d_true"].min():.2f} .. {fa["d_true"].max():.2f} m, first |error| > 0.5 m at '
          f't = {fa["t"][bad[0]] if len(bad) else float("nan"):.2f} s')
    # Corner transitions (B2, noiseless re-fly): error peak and settling after each corner waypoint
    corners = []
    flights['B2_clean'] = fb = fly('box', bp2, sigma=0.0)
    pts, s = resample(bp2)
    s_corner = [np.interp(i, np.arange(len(bp2)), np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(bp2, axis=0), axis=1))])) for i in ci2]
    for j, sc in enumerate(s_corner):
        t_c = sc / V_CRUISE
        win = (fb['t'] >= t_c - 3) & (fb['t'] <= t_c + 6)
        e = fb['e_n'][win]; tt = fb['t'][win]
        late = np.nonzero(np.abs(e) > 0.01)[0]
        first = np.nonzero(np.abs(e) > 0.01)[0]
        corners.append((t_c, np.abs(e).max(), tt[np.argmax(np.abs(e))] - t_c, fb['d_true'][win].min()))
        print(f'Corner {j + 1} (B2, noiseless) at t = {t_c:.1f} s: max |e_n| {100 * np.abs(e).max():.2f} cm at '
              f't_c {tt[np.argmax(np.abs(e))] - t_c:+.2f} s, min standoff {fb["d_true"][win].min():.3f} m, '
              f'|e_n| > 1 cm from t_c {tt[first[0]] - t_c if len(first) else float("nan"):+.2f} s '
              f'to t_c {tt[late[-1]] - t_c if len(late) else float("nan"):+.2f} s')
    print(f'Building transitions: B1 flown {flights["B1"]["length"]:.1f} m / {flights["B1"]["T"]:.1f} s, '
          f'B2 {flights["B2"]["length"]:.1f} m / {flights["B2"]["T"]:.1f} s, saving '
          f'{flights["B1"]["length"] - flights["B2"]["length"]:.1f} m')
    print('=== Coverage (cells inside at least one frame; gated = also cos psi >= cos 30) ===')
    cov = {}
    for k, st in [('A1', 'cyl'), ('A2', 'cyl'), ('B1', 'box'), ('B2', 'box')]:
        cov[k] = coverage(flights[k], st)
        print(f'{names[k]:28s}: coverage {100 * cov[k][0]:.2f} %, gated {100 * cov[k][1]:.2f} %')
    # card arc-length criterion on the cylinder
    for k in ('A1', 'A2'):
        p = flights[k]['p']
        phd = np.arctan2(p[:, 1], p[:, 0])
        ph = np.deg2rad(np.arange(DPHI_DEG / 2, 360, DPHI_DEG))
        zz = np.arange(DZ_GRID / 2, H, DZ_GRID)
        covc = np.zeros((len(ph), len(zz)), bool)
        for pp, a in zip(p[::2], phd[::2]):
            ds = R_C * np.abs(np.angle(np.exp(1j * (ph - a))))
            covc[np.ix_(ds <= H_STRIP, np.abs(zz - pp[2]) <= H_STRIP)] = True
        print(f'{names[k]:28s}: card arc-length coverage {100 * covc.mean():.2f} %')
    print('=== Viewing-angle gate on the nominal standoff (crack at drone height) ===')
    gp = {}
    for st in ('cyl', 'box'):
        off, inf, g = gate_profile(st)
        gp[st] = (off, inf, g)
        print(f'{st}: in-frame lateral half-width {np.abs(off[inf]).max():.3f} m, gate passes up to '
              f'{np.abs(off[inf & g]).max():.3f} m')
    print(f'=== Crack detection: {N_SEEDS} crack layouts x {N_CRACKS} cracks per structure ===')
    det = {}
    for k, st in [('A2', 'cyl'), ('B2', 'box')]:
        rows = []
        for sd in range(N_SEEDS):
            cr = make_cracks(st, sd)
            d, dg, fin, frej, fnear, frejn = detect(flights[k], st, cr, sd)
            rows.append((cr, d, dg, fin, frej, fnear, frejn))
        det[k] = rows
        w = np.concatenate([r[0][2] for r in rows])
        d = np.concatenate([r[1] for r in rows]); dg = np.concatenate([r[2] for r in rows])
        fin = np.concatenate([r[3] for r in rows]); frej = np.concatenate([r[4] for r in rows])
        fnear = np.concatenate([r[5] for r in rows]); frejn = np.concatenate([r[6] for r in rows])
        per = np.array([r[1].sum() for r in rows]); perg = np.array([r[2].sum() for r in rows])
        print(f'{names[k]}: detected without gate {d.sum()}/{len(w)} ({100 * d.mean():.1f} %, per layout '
              f'{per.mean():.1f} +- {per.std():.1f}), with gate {dg.sum()}/{len(w)} ({100 * dg.mean():.1f} %, '
              f'{perg.mean():.1f} +- {perg.std():.1f}); cracks never in frame {int((fin == 0).sum())}; '
              f'layouts where the gate costs detections {int(np.sum(per > perg))}/{N_SEEDS}')
        print(f'   in-frame crack views {fin.sum()}: gate rejects {100 * frej.sum() / max(fin.sum(), 1):.1f} %; '
              f'near views (slant range <= {R_NEAR} m) {fnear.sum()}, gate rejects '
              f'{100 * frejn.sum() / max(fnear.sum(), 1):.1f} % of them; grazing views (> {R_NEAR} m) '
              f'{fin.sum() - fnear.sum()}, gate rejects {100 * (frej.sum() - frejn.sum()) / max(fin.sum() - fnear.sum(), 1):.1f} %; '
              f'median near views per crack {np.median(fnear):.0f}, gate-passed near views per crack (median) '
              f'{np.median(fnear - frejn):.0f}')
        for lo, hi in zip(BINS[:-1], BINS[1:]):
            sel = (w >= lo) & (w < hi)
            print(f'   width {lo:.1f}-{hi:.1f} mm: {d[sel].sum()}/{sel.sum()} ungated, {dg[sel].sum()}/{sel.sum()} gated')
    return dict(flights=flights, cov=cov, gp=gp, det=det, bp=(bp1, bp2), ci=(ci1, ci2),
                corners=corners, names=names)


# ── Plots ───────────────────────────────────────────────────────────────────

def _cyl_wire(ax):
    ph = np.linspace(0, 2 * np.pi, 49)
    for z in np.linspace(0, H, 6):
        ax.plot(R_C * np.cos(ph), R_C * np.sin(ph), z, color='grey', lw=0.5)
    for a in ph[::4]:
        ax.plot([R_C * np.cos(a)] * 2, [R_C * np.sin(a)] * 2, [0, H], color='grey', lw=0.5)


def plot_helix_3d(res, out_dir):
    fl = res['flights']['A2']
    cr, d, dg = res['det']['A2'][0][:3]
    fig = plt.figure(figsize=(9, 9))
    ax = fig.add_subplot(111, projection='3d')
    _cyl_wire(ax)
    p = fl['p'][::4]
    ax.scatter(p[:, 0], p[:, 1], p[:, 2], c=np.linspace(0, 1, len(p)), cmap='coolwarm', s=1.5)
    c = cr[0]
    ax.scatter(*c[dg].T, marker='*', s=90, color='gold', edgecolors='k', label=f'detected (gated) {dg.sum()}')
    ax.scatter(*c[~dg].T, marker='o', s=25, color='grey', label=f'missed {(~dg).sum()}')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_box_aspect((1, 1, 1.6))
    ax.legend(loc='upper left', fontsize=9)
    ax.set_title('A2 bounded helix around the chimney (blue start -> red end), crack layout 0')
    plt.tight_layout()
    path = os.path.join(out_dir, 'helix_3d.png')
    plt.savefig(path, dpi=120); plt.close()
    print(f'Saved: {path}')


def plot_altitude_azimuth(res, out_dir):
    fig, ax = plt.subplots(figsize=(11, 4.5))
    for k, col in (('A1', 'tab:red'), ('A2', 'tab:blue')):
        p = res['flights'][k]['p']
        phi = np.unwrap(np.arctan2(p[:, 1], p[:, 0]))
        ax.plot(np.degrees(phi) / 360.0, p[:, 2], color=col, lw=1.2, label=res['names'][k])
    ax.axhline(Z_MIN, color='k', ls=':', lw=0.8); ax.axhline(Z_MAX, color='k', ls=':', lw=0.8)
    ax.set_xlabel('Azimuth travelled (turns)'); ax.set_ylabel('z (m)')
    ax.set_title('Altitude vs azimuth: card helix (ends clipped at 19.5 m) vs bounded helix with level end orbits')
    ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_vs_azimuth.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_cylinder_coverage(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5))
    cr, _, dg = res['det']['A2'][0][:3]
    phc = np.degrees(np.arctan2(cr[0][:, 1], cr[0][:, 0])) % 360
    for ax, k in zip(axes, ('A1', 'A2')):
        _, _, cv, cg = res['cov'][k]
        img = np.where(cg, 2, np.where(cv, 1, 0)).T
        ax.imshow(img, origin='lower', extent=[0, 360, 0, H], aspect='auto',
                  cmap=matplotlib.colors.ListedColormap(['white', '#f4a261', '#2a9d8f']), vmin=0, vmax=2)
        ax.plot(phc[dg], cr[0][dg, 2], '*', color='gold', mec='k', ms=9)
        ax.plot(phc[~dg], cr[0][~dg, 2], 'o', color='grey', mec='k', ms=5)
        ax.set_title(f'{res["names"][k]}: in frame {100 * res["cov"][k][0]:.2f} %, gated {100 * res["cov"][k][1]:.2f} %',
                     fontsize=10)
        ax.set_xlabel('azimuth (deg)'); ax.set_ylabel('z (m)')
    axes[1].legend(handles=[plt.Rectangle((0, 0), 1, 1, fc='#2a9d8f', label='in frame + gate'),
                            plt.Rectangle((0, 0), 1, 1, fc='#f4a261', label='in frame only'),
                            plt.Rectangle((0, 0), 1, 1, fc='white', ec='k', label='never imaged'),
                            Line2D([], [], marker='*', color='gold', mec='k', ls='', label='crack detected (A2, layout 0)'),
                            Line2D([], [], marker='o', color='grey', mec='k', ls='', label='crack missed')],
                   fontsize=8, loc='upper right')
    plt.tight_layout()
    path = os.path.join(out_dir, 'cylinder_coverage.png')
    plt.savefig(path, dpi=120); plt.close()
    print(f'Saved: {path}')


def plot_building(res, out_dir):
    fig = plt.figure(figsize=(16, 7))
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    sq = np.array([[-HALF_B, -HALF_B], [HALF_B, -HALF_B], [HALF_B, HALF_B], [-HALF_B, HALF_B], [-HALF_B, -HALF_B]])
    ax1.fill(sq[:, 0], sq[:, 1], color='lightgrey')
    bp2, ci2 = res['bp'][1], res['ci'][1]
    fl = res['flights']['B2']
    ax1.plot(fl['p'][:, 0], fl['p'][:, 1], color='k', lw=0.8)
    for j in ci2:
        ax1.plot(*bp2[j][:2], 'r^', ms=10)
    for i in range(4):
        n, c0, du = FACES[i]
        a, b = c0, c0 + 2 * HALF_B * du
        ax1.plot([a[0], b[0]], [a[1], b[1]], color=FACE_COLORS[i], lw=4, label=f'face {i + 1}')
    ax1.set_aspect('equal'); ax1.set_xlim(-14, 14); ax1.set_ylim(-14, 14)
    ax1.set_title('Top view: B2 path, corner waypoints (red)'); ax1.legend(fontsize=8, loc='center')
    for k, col in (('B1', 'tab:red'), ('B2', 'tab:blue')):
        p = res['flights'][k]['p']
        ax2.plot(p[:, 0], p[:, 1], p[:, 2], color=col, lw=0.6, label=f'{res["names"][k]} ({res["flights"][k]["length"]:.0f} m)')
    for i in range(4):
        n, c0, du = FACES[i]
        a, b = c0, c0 + 2 * HALF_B * du
        X = np.array([[a[0], b[0]], [a[0], b[0]]]); Y = np.array([[a[1], b[1]], [a[1], b[1]]])
        Z = np.array([[0, 0], [H, H]])
        ax2.plot_surface(X, Y, Z, color=FACE_COLORS[i], alpha=0.25)
    ax2.set_xlabel('x'); ax2.set_ylabel('y'); ax2.set_zlabel('z')
    ax2.legend(fontsize=8); ax2.set_title('Isometric: card (red) vs alternating (blue)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'building_path.png')
    plt.savefig(path, dpi=110); plt.close()
    print(f'Saved: {path}')


def plot_standoff(res, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.5))
    fb = res['flights']['B2_clean']
    for j, (tc, emax, tpk, dmin) in enumerate(res['corners']):
        w = (fb['t'] >= tc - 3) & (fb['t'] <= tc + 6)
        axes[0].plot(fb['t'][w] - tc, 100 * fb['e_n'][w], lw=1, label=f'corner {j + 1}')
    axes[0].set_xlabel('time from corner waypoint (s)'); axes[0].set_ylabel('$e_n$ (cm)')
    axes[0].set_title('B2 corner transitions (noiseless): standoff error'); axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=8)
    for k, col in (('A2', 'tab:blue'), ('A2ff', 'tab:green')):
        f = res['flights'][k]
        axes[1].plot(f['t'], 100 * f['e_n'], color=col, lw=0.4, alpha=0.8, label=res['names'][k])
    axes[1].axhline(-100 * V_CRUISE ** 2 / R_ORBIT / KP, color='k', ls='--', lw=1, label='$-v^2/(r K_p)$')
    axes[1].set_xlabel('time (s)'); axes[1].set_ylabel('$e_n$ (cm)')
    axes[1].set_title('Cylinder: curvature bias of the PD standoff loop'); axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    fa = res['flights']['A_fixed']
    axes[2].plot(fa['t'], fa['d_true'], color='tab:red')
    axes[2].axhline(D_STANDOFF, color='k', ls='--', lw=1)
    axes[2].set_xlabel('time (s)'); axes[2].set_ylabel('true standoff (m)')
    axes[2].set_title('Fixed-x-axis (flat-wall) PD on the cylinder, one orbit'); axes[2].grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'standoff_error.png')
    plt.savefig(path, dpi=120); plt.close()
    print(f'Saved: {path}')


def plot_detection(res, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.8))
    for ax, k in zip(axes[:2], ('A2', 'B2')):
        rows = res['det'][k]
        w = np.concatenate([r[0][2] for r in rows])
        d = np.concatenate([r[1] for r in rows]); dg = np.concatenate([r[2] for r in rows])
        xs = np.arange(len(BINS) - 1)
        tot = [((w >= lo) & (w < hi)).sum() for lo, hi in zip(BINS[:-1], BINS[1:])]
        nd = [d[(w >= lo) & (w < hi)].sum() for lo, hi in zip(BINS[:-1], BINS[1:])]
        ng = [dg[(w >= lo) & (w < hi)].sum() for lo, hi in zip(BINS[:-1], BINS[1:])]
        ax.bar(xs - 0.2, tot, 0.4, color='lightgrey', label='cracks')
        ax.bar(xs - 0.2, nd, 0.4, color='tab:blue', label='detected, no gate')
        ax.bar(xs + 0.2, tot, 0.4, color='lightgrey')
        ax.bar(xs + 0.2, ng, 0.4, color='tab:green', label='detected, with gate')
        ax.set_xticks(xs, [f'{lo:.1f}-{hi:.1f}' for lo, hi in zip(BINS[:-1], BINS[1:])])
        ax.set_xlabel('crack width (mm)'); ax.set_ylabel(f'cracks ({N_SEEDS} layouts x {N_CRACKS})')
        ax.set_title(res['names'][k]); ax.legend(fontsize=8)
    for st, col in (('cyl', 'tab:red'), ('box', 'tab:blue')):
        off, inf, g = res['gp'][st]
        axes[2].plot(off, inf.astype(float) + (0.02 if st == 'cyl' else 0), color=col, ls=':', label=f'{st}: in frame')
        axes[2].plot(off, (inf & g).astype(float) * 0.9, color=col, label=f'{st}: in frame and gate passed')
    axes[2].set_xlabel('lateral offset on the surface (m)'); axes[2].set_yticks([0, 0.9, 1], ['no', 'gated', 'yes'])
    axes[2].set_title('Viewing-angle gate at the drone height'); axes[2].legend(fontsize=7)
    plt.tight_layout()
    path = os.path.join(out_dir, 'detection_by_width.png')
    plt.savefig(path, dpi=120); plt.close()
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    fl = res['flights']['A2']
    n = len(fl['p'])
    step = int(np.ceil((n - 1) / (N_FRAMES - 1)))
    n_fr = int(np.ceil((n - 1) / step)) + 1
    fig = plt.figure(figsize=(6.5, 7))
    ax = fig.add_subplot(111, projection='3d')

    def update(i):
        k = min(i * step, n - 1)
        ax.clear()
        _cyl_wire(ax)
        p = fl['p'][:k + 1:4]
        ax.plot(p[:, 0], p[:, 1], p[:, 2], color='tab:blue', lw=0.8)
        q = fl['p'][k]
        ax.scatter(*q, color='red', s=30)
        b, _ = camera_axes(q, 'cyl')
        ax.plot([q[0], q[0] + 1.5 * b[0]], [q[1], q[1] + 1.5 * b[1]], [q[2], q[2]], color='red', lw=1)
        ax.set_xlim(-6, 6); ax.set_ylim(-6, 6); ax.set_zlim(0, 20)
        ax.set_box_aspect((1, 1, 1.6))
        ax.set_title(f'A2 helix inspection  t = {fl["t"][k]:.0f} s, z = {q[2]:.1f} m', fontsize=10)
        return []

    ani = FuncAnimation(fig, update, frames=n_fr, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=10), dpi=70)
    plt.close()
    print(f'Saved: {path}  ({n_fr} frames)')


if __name__ == '__main__':
    print('Running S044 3D Wall Crack Inspection simulation...')
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_helix_3d(data, OUTPUT_DIR)
    plot_altitude_azimuth(data, OUTPUT_DIR)
    plot_cylinder_coverage(data, OUTPUT_DIR)
    plot_building(data, OUTPUT_DIR)
    plot_standoff(data, OUTPUT_DIR)
    plot_detection(data, OUTPUT_DIR)
    save_animation(data, OUTPUT_DIR)
    print('Done.')
