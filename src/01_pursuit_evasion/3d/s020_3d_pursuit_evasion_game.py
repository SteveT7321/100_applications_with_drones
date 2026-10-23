"""
S020 3D Upgrade — Pursuit-Evasion Game (RL Capstone)
====================================================
The card's 3D game: a 5 m/s pursuer against a 3.5 m/s evader with 3D unit-sphere
velocity commands, capture radius 0.15 m, 30 s episodes and observation noise
sigma = 0.1 on every observed position and velocity component.

1. PPO training (stable-baselines3, the S020 2D pipeline) in the card arena with
   the card's 3D rewards (distance shaping, altitude-advantage / altitude-escape
   terms, effort and altitude-limit penalties) and the card's three-phase
   curriculum at a reduced budget:
     Phase 1  pursuer vs a random 3D evader (random spawn altitude)
     Phase 2  evader vs the frozen Phase-1 pursuer
     Phase 3  self-play, alternating rounds against frozen snapshots
2. Benchmark grid, every pursuer vs every evader, 500 seeded episodes per cell:
   Pursuers (always exactly v_P):
     RL      trained PPO pursuer (deterministic)
     PP      pure pursuit at the observed evader position
     PN      3D proportional navigation, N = 3, constant speed, launched along
             the initial line of sight (the S016 / S020 2D fix)
     APF     attraction to the evader + repulsion from every arena face closer than d0
     Lead    pure pursuit at the predicted position e + v_e * 0.3 s
     AltAdv  pure pursuit with a climb bias until 1 m above the evader
   Evaders (always exactly v_E):
     RL             trained PPO evader (deterministic)
     straight       fixed random 3D heading (elevation within +-30 deg)
     perpendicular  horizontal, perpendicular to the line of sight (fixed side)
     spiral         helix: heading rotating at 0.8 rad/s, vertical 0.5 sin(0.8 t + phase)
     alt_escape     the card's altitude escape: away horizontally while climbing /
                    diving to the altitude limit on the far side of the pursuer
   Two arenas with identical spawns, evader parameters and noise (common random numbers):
     card  x, y in [-5, 5] m, z in [0.5, 5.5] m (card; RL is trained here)
     open  no x-y walls, same z limits (separates wall cornering from pursuit)
3. RL pursuer policy quiver plots at the card's z slices (1.0, 2.5, 4.5 m).

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s020_3d_pursuit_evasion_game.py
"""
import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK', 'TRUE')   # avoid libomp/libiomp conflict on Windows
import copy
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

import torch
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's020_3d_pursuit_evasion_game'))

# ── Parameters ────────────────────────────────────────────────────────────────
V_PURSUER   = 5.0
V_EVADER    = 3.5
CAPTURE_R   = 0.15
DT          = 0.02
T_MAX       = 30.0
MAX_STEPS   = int(round(T_MAX / DT))
XY_HALF     = 5.0            # card arena: x, y in [-5, 5]
Z_MIN, Z_MAX = 0.5, 5.5      # card z range
Z_MID       = 0.5 * (Z_MIN + Z_MAX)
NOISE_SIGMA = 0.1
PN_GAIN     = 3.0
APF_D0      = 1.0            # m, wall influence distance
APF_ETA     = 1.0 / 48.0     # repulsion = attraction at 0.25 m from a face
T_LEAD      = 0.3
ALT_ADV_DZ  = 1.0
STRAIGHT_EL_MAX = np.radians(30.0)
SPIRAL_OMEGA = 0.8
SPIRAL_AZ    = 0.5
SPAWN_MIN_SEP = 5.0
N_EPISODES  = 500
SEED        = 2020
CONTACT_TOL = 1e-6
N_RECORD    = 2

# Card rewards
R_CAPTURE   = 100.0
W_CLOSE     = 0.01           # distance shaping (per metre)
W_ALT_ADV   = 0.02           # pursuer altitude-advantage term
W_EFFORT    = 0.1            # pursuer effort penalty * |a| / v_max
W_ALT_ESC   = 0.05           # evader altitude-escape term * |z_E - z_mid|
R_SURVIVE   = 0.1            # evader survival bonus per step
R_ZLIMIT    = 1.0            # penalty per step on (or beyond) an altitude limit

# PPO (card) and training budget (reduced; card: 5000 / 5000 / 10000 episodes)
LEARNING_RATE = 3e-4
CLIP_RANGE    = 0.2
BATCH_SIZE    = 256
NET_ARCH      = [256, 256]
N_STEPS_PPO   = 512
PHASE1_TIMESTEPS = 90_000
PHASE2_TIMESTEPS = 90_000
PHASE3_ROUNDS    = 4
PHASE3_STEPS_PER_ROUND = 12_000      # per agent per round
TRAIN_SEED  = 0

PURSUERS = ['RL', 'PP', 'PN', 'APF', 'Lead', 'AltAdv']
EVADERS  = ['RL', 'straight', 'perpendicular', 'spiral', 'alt_escape']
ARENAS   = ['card', 'open']
P_COLORS = {'RL': '#2a9d8f', 'PP': '#e63946', 'PN': '#9d0208', 'APF': '#f77f00', 'Lead': '#ff70a6',
            'AltAdv': '#6a040f'}
OBS_SCALE = np.array([XY_HALF] * 3 + [V_PURSUER] * 3 + [XY_HALF] * 3 + [V_PURSUER] * 3 + [2 * XY_HALF],
                     dtype=np.float32)
QUIVER_Z = [1.0, 2.5, 4.5]


def unit(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-9)


def clip_arena(x, arena):
    x = x.copy()
    if arena == 'card':
        x[:, :2] = np.clip(x[:, :2], -XY_HALF, XY_HALF)
    x[:, 2] = np.clip(x[:, 2], Z_MIN, Z_MAX)
    return x


def obs13(Po, Vpo, Eo, Veo):
    """Card state [p_P, v_P, p_E, v_E, d] built from the (noisy) observation."""
    return np.concatenate([Po, Vpo, Eo, Veo, np.linalg.norm(Eo - Po, axis=1, keepdims=True)], axis=1)


def closest_in_step(P, E, Pn, En):
    r0, d = E - P, (En - Pn) - (E - P)
    a = np.sum(d * d, axis=1)
    s = np.clip(-np.sum(r0 * d, axis=1) / np.maximum(a, 1e-15), 0.0, 1.0)
    return np.linalg.norm(r0 + s[:, None] * d, axis=1)


# ── Hand-crafted pursuers ────────────────────────────────────────────────────

class Pursuer:
    def __init__(self, kind, arena):
        self.kind, self.arena = kind, arena

    def reset(self, Po, Eo):
        self.v = unit(Eo - Po) * V_PURSUER          # PN launch: along the initial LOS

    def act(self, Po, Vpo, Eo, Veo, t):
        k = self.kind
        if k == 'PP':
            return unit(Eo - Po) * V_PURSUER
        if k == 'Lead':
            return unit(Eo + Veo * T_LEAD - Po) * V_PURSUER
        if k == 'AltAdv':
            v = unit(Eo - Po) * V_PURSUER
            dalt = Po[:, 2] - Eo[:, 2]
            v[:, 2] += np.where(dalt < ALT_ADV_DZ, np.minimum(2.0, (ALT_ADV_DZ - dalt) * 1.5), 0.0)
            return unit(v) * V_PURSUER
        if k == 'APF':
            f = unit(Eo - Po)
            faces = [(2, Z_MIN, +1.0), (2, Z_MAX, -1.0)]
            if self.arena == 'card':
                faces += [(0, -XY_HALF, +1.0), (0, XY_HALF, -1.0), (1, -XY_HALF, +1.0), (1, XY_HALF, -1.0)]
            for ax, pos, normal in faces:
                d = np.maximum(np.abs(Po[:, ax] - pos), 0.01)
                rep = np.where(d < APF_D0, APF_ETA * (1.0 / d - 1.0 / APF_D0) / d ** 2, 0.0)
                f[:, ax] += normal * rep
            return unit(f) * V_PURSUER
        # PN: a = N v_c lambda_dot, velocity rescaled to v_P
        los = Eo - Po
        r = np.maximum(np.linalg.norm(los, axis=1, keepdims=True), 1e-9)
        lh = los / r
        vrel = Veo - self.v
        vr = np.sum(vrel * lh, axis=1, keepdims=True)
        ldot = (vrel - vr * lh) / r
        self.v = unit(self.v + PN_GAIN * (-vr) * ldot * DT) * V_PURSUER
        return self.v


# ── Hand-crafted evaders ─────────────────────────────────────────────────────

class Evader:
    def __init__(self, kind, rng, n):
        self.kind = kind
        az = rng.uniform(0, 2 * np.pi, n)
        el = rng.uniform(-STRAIGHT_EL_MAX, STRAIGHT_EL_MAX, n)
        self.heading = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        self.side = np.where(rng.random(n) < 0.5, -1.0, 1.0)
        self.phase = rng.uniform(0, 2 * np.pi, n)
        self.phase_z = rng.uniform(0, 2 * np.pi, n)

    def reset(self, Po, Eo):
        pass

    def act(self, Po, Vpo, Eo, Veo, t):
        k, n = self.kind, len(Po)
        if k == 'straight':
            return self.heading * V_EVADER
        if k == 'perpendicular':
            los = Eo - Po
            perp = np.column_stack([-los[:, 1], los[:, 0], np.zeros(n)]) * self.side[:, None]
            small = np.linalg.norm(perp, axis=1) < 1e-6
            perp[small] = [1.0, 0.0, 0.0]
            return unit(perp) * V_EVADER
        if k == 'spiral':
            a = SPIRAL_OMEGA * t + self.phase
            v = np.column_stack([np.cos(a), np.sin(a),
                                 SPIRAL_AZ * np.sin(SPIRAL_OMEGA * t + self.phase_z)])
            return unit(v) * V_EVADER
        # alt_escape: away horizontally, vertical toward the limit on the far side of the pursuer
        away = Eo[:, :2] - Po[:, :2]
        hn = np.linalg.norm(away, axis=1, keepdims=True)
        h = np.where(hn > 1e-6, away / np.maximum(hn, 1e-9), np.array([[1.0, 0.0]]))
        vz = np.where(Po[:, 2] <= Eo[:, 2], 1.0, -1.0)
        return unit(np.column_stack([h, vz])) * V_EVADER


class RandomEvader:
    """Phase-1 opponent: a fresh random 3D direction every step."""
    def __init__(self, rng):
        self.rng = rng

    def reset(self, Po, Eo):
        pass

    def act(self, Po, Vpo, Eo, Veo, t):
        return unit(self.rng.standard_normal((len(Po), 3))) * V_EVADER


class RLAgent:
    """Deterministic frozen PPO policy acting on the scaled noisy 13-D observation."""
    def __init__(self, policy, role):
        self.policy = policy
        self.policy.set_training_mode(False)
        self.speed = V_PURSUER if role == 'pursuer' else V_EVADER

    def reset(self, Po, Eo):
        pass

    def act(self, Po, Vpo, Eo, Veo, t):
        with torch.no_grad():
            x = torch.as_tensor((obs13(Po, Vpo, Eo, Veo) / OBS_SCALE).astype(np.float32))
            a = self.policy._predict(x, deterministic=True).numpy().astype(float)
        return unit(np.clip(a, -1.0, 1.0)) * self.speed


# ── Spawns ───────────────────────────────────────────────────────────────────

def spawn_one(rng):
    while True:
        p = np.array([rng.uniform(-4.5, 4.5), rng.uniform(-4.5, 4.5), rng.uniform(1.0, 5.0)])
        e = np.array([rng.uniform(-4.5, 4.5), rng.uniform(-4.5, 4.5), rng.uniform(1.0, 5.0)])
        if np.linalg.norm(p - e) >= SPAWN_MIN_SEP:
            return p, e


def spawns(n, seed):
    rng = np.random.default_rng(seed)
    P = np.zeros((n, 3)); E = np.zeros((n, 3))
    for k in range(n):
        P[k], E[k] = spawn_one(rng)
    return P, E


# ── Card rewards ─────────────────────────────────────────────────────────────

def on_z_limit(z):
    return (z <= Z_MIN + CONTACT_TOL) | (z >= Z_MAX - CONTACT_TOL)


def pursuer_reward(d_prev, d, a, P, E, VP, captured):
    dz = P[2] - E[2]
    alt_tac = max(0.0, abs(dz) - 1.0) * np.sign(dz) * np.sign(VP[2])
    return (W_CLOSE * (d_prev - d) + W_ALT_ADV * alt_tac - W_EFFORT * np.linalg.norm(a) / V_PURSUER
            - R_ZLIMIT * float(on_z_limit(P[2])) + (R_CAPTURE if captured else 0.0))


def evader_reward(d_prev, d, E, captured):
    return (R_SURVIVE + W_CLOSE * (d - d_prev) + W_ALT_ESC * abs(E[2] - Z_MID)
            - R_ZLIMIT * float(on_z_limit(E[2])) - (R_CAPTURE if captured else 0.0))


# ── Gymnasium environment (card arena) ───────────────────────────────────────

class PursuitEvasion3DEnv(gym.Env):
    """One learning agent ('pursuer' or 'evader') vs a fixed opponent, card arena."""
    metadata = {'render_modes': []}

    def __init__(self, role, opponent, rng_seed):
        super().__init__()
        self.role, self.opponent = role, opponent
        self.rng = np.random.default_rng(rng_seed)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(13,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)

    def _noisy(self):
        n = lambda: self.rng.normal(0, NOISE_SIGMA, (1, 3))
        return self.P + n(), self.VP + n(), self.E + n(), self.VE + n()

    def _own_obs(self):
        return (obs13(*self._noisy())[0] / OBS_SCALE).astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        p, e = spawn_one(self.rng)
        self.P, self.E = p[None].copy(), e[None].copy()
        self.VP, self.VE = np.zeros((1, 3)), np.zeros((1, 3))
        self.step_i = 0
        self.d_prev = float(np.linalg.norm(self.P - self.E))
        Po, _, Eo, _ = self._noisy()
        self.opponent.reset(Po, Eo)
        return self._own_obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        own_speed = V_PURSUER if self.role == 'pursuer' else V_EVADER
        v_own = unit(a[None]) * own_speed
        v_opp = self.opponent.act(*self._noisy(), self.step_i * DT)
        vp, ve = (v_own, v_opp) if self.role == 'pursuer' else (v_opp, v_own)
        Pn, En = clip_arena(self.P + vp * DT, 'card'), clip_arena(self.E + ve * DT, 'card')
        captured = bool(closest_in_step(self.P, self.E, Pn, En)[0] < CAPTURE_R)
        self.VP, self.VE = (Pn - self.P) / DT, (En - self.E) / DT
        self.P, self.E = Pn, En
        self.step_i += 1
        d = float(np.linalg.norm(self.P - self.E))
        if self.role == 'pursuer':
            r = pursuer_reward(self.d_prev, d, a, self.P[0], self.E[0], self.VP[0], captured)
        else:
            r = evader_reward(self.d_prev, d, self.E[0], captured)
        self.d_prev = d
        return self._own_obs(), float(r), captured, self.step_i >= MAX_STEPS, {}


class EpisodeLogger(BaseCallback):
    def __init__(self):
        super().__init__()
        self.returns, self.captured, self._cur = [], [], 0.0

    def _on_step(self):
        self._cur += float(self.locals['rewards'][0])
        if self.locals['dones'][0]:
            info = self.locals['infos'][0]
            self.returns.append(self._cur)
            self.captured.append(not info.get('TimeLimit.truncated', False))
            self._cur = 0.0
        return True


def make_ppo(env, seed):
    return PPO('MlpPolicy', env, verbose=0, learning_rate=LEARNING_RATE, clip_range=CLIP_RANGE,
               batch_size=BATCH_SIZE, n_steps=N_STEPS_PPO, policy_kwargs=dict(net_arch=NET_ARCH),
               seed=seed, device='cpu')


def frozen(model, role):
    pol = model.policy
    clone = type(pol)(**pol._get_constructor_parameters())
    clone.load_state_dict(copy.deepcopy(pol.state_dict()))
    return RLAgent(clone, role)


def train():
    torch.manual_seed(TRAIN_SEED)
    torch.set_num_threads(1)
    curves = {}

    def learn(model, env, steps, key):
        model.set_env(env)
        cb = EpisodeLogger()
        n0 = model.num_timesteps
        model.learn(total_timesteps=steps, callback=cb, reset_num_timesteps=False)
        c = curves.setdefault(key, {'returns': [], 'captured': [], 'steps': 0})
        c['steps'] += model.num_timesteps - n0
        c['returns'] += cb.returns
        c['captured'] += cb.captured

    print('PPO training in the card arena')
    print('  Phase 1: pursuer vs random 3D evader ...')
    env1 = PursuitEvasion3DEnv('pursuer', RandomEvader(np.random.default_rng(31)), 11)
    pursuer = make_ppo(env1, seed=TRAIN_SEED + 1)
    learn(pursuer, env1, PHASE1_TIMESTEPS, 'Phase 1: pursuer')
    print('  Phase 2: evader vs frozen Phase-1 pursuer ...')
    env2 = PursuitEvasion3DEnv('evader', frozen(pursuer, 'pursuer'), 12)
    evader = make_ppo(env2, seed=TRAIN_SEED + 2)
    learn(evader, env2, PHASE2_TIMESTEPS, 'Phase 2: evader')
    print(f'  Phase 3: self-play, {PHASE3_ROUNDS} alternating rounds ...')
    for k in range(PHASE3_ROUNDS):
        learn(pursuer, PursuitEvasion3DEnv('pursuer', frozen(evader, 'evader'), 100 + k),
              PHASE3_STEPS_PER_ROUND, 'Phase 3: pursuer')
        learn(evader, PursuitEvasion3DEnv('evader', frozen(pursuer, 'pursuer'), 200 + k),
              PHASE3_STEPS_PER_ROUND, 'Phase 3: evader')
    print(f'  model.num_timesteps: pursuer {pursuer.num_timesteps}, evader {evader.num_timesteps}')
    print('  Training summary (timesteps, episodes; captured and mean return in first / last 20% of episodes):')
    for key, c in curves.items():
        cap = np.array(c['captured'], float)
        n = len(cap); q = max(1, n // 5)
        print(f'    {key:17s} {c["steps"]:6d} timesteps, {n:3d} episodes | captured {cap[:q].mean() * 100:5.1f}% '
              f'-> {cap[-q:].mean() * 100:5.1f}% | return {np.mean(c["returns"][:q]):8.2f} -> '
              f'{np.mean(c["returns"][-q:]):8.2f}')
    return frozen(pursuer, 'pursuer'), frozen(evader, 'evader'), curves


# ── Benchmark episodes (vectorised) ──────────────────────────────────────────

def make_pursuer(pk, arena, agents):
    return agents['RL pursuer'] if pk == 'RL' else Pursuer(pk, arena)


def simulate(pk, ek, arena, agents, n=N_EPISODES, seed=SEED, n_record=0, sigma=NOISE_SIGMA):
    P, E = spawns(n, seed)
    rng = np.random.default_rng(seed + 1)          # evader parameters (same for every cell)
    hand = Evader('straight' if ek == 'RL' else ek, rng, n)
    ev = agents['RL evader'] if ek == 'RL' else hand
    noise = np.random.default_rng(seed + 2)        # observation noise (same stream for every cell)
    pu = make_pursuer(pk, arena, agents)
    VP, VE = np.zeros((n, 3)), np.zeros((n, 3))
    active = np.ones(n, bool)
    t_cap = np.full(n, np.nan)
    wall_xy = np.zeros(n, bool); wall_z = np.zeros(n, bool)
    min_d = np.linalg.norm(P - E, axis=1)
    zp_hist, ze_hist = np.zeros(10), np.zeros(10)
    z_edges = np.linspace(Z_MIN, Z_MAX, 11)
    rec_p, rec_e = [P[:n_record].copy()], [E[:n_record].copy()]
    pu.reset(P + noise.normal(0, sigma, (n, 3)), E + noise.normal(0, sigma, (n, 3)))
    for step in range(MAX_STEPS):
        Po = P + noise.normal(0, sigma, (n, 3)); Eo = E + noise.normal(0, sigma, (n, 3))
        Vpo = VP + noise.normal(0, sigma, (n, 3)); Veo = VE + noise.normal(0, sigma, (n, 3))
        vp = pu.act(Po, Vpo, Eo, Veo, step * DT)
        ve = ev.act(Po, Vpo, Eo, Veo, step * DT)
        Pn = clip_arena(P + vp * DT, arena); En = clip_arena(E + ve * DT, arena)
        dist = closest_in_step(P, E, Pn, En)
        act = active.copy()
        VP[act], VE[act] = ((Pn - P) / DT)[act], ((En - E) / DT)[act]   # actual (wall-clipped) velocity
        P[act], E[act] = Pn[act], En[act]
        min_d[act] = np.minimum(min_d[act], dist[act])
        if arena == 'card':
            wall_xy[act] |= np.any(np.abs(E[act, :2]) >= XY_HALF - CONTACT_TOL, axis=1)
        wall_z[act] |= on_z_limit(E[act, 2])
        zp_hist += np.histogram(P[act, 2], z_edges)[0]; ze_hist += np.histogram(E[act, 2], z_edges)[0]
        cap = act & (dist < CAPTURE_R)
        t_cap[cap] = (step + 1) * DT
        active &= ~cap
        if n_record:
            rec_p.append(P[:n_record].copy()); rec_e.append(E[:n_record].copy())
        if not active.any():
            break
    return {'t_cap': t_cap, 'wall_xy': wall_xy, 'wall_z': wall_z, 'min_d': min_d,
            'zp_hist': zp_hist, 'ze_hist': ze_hist, 'rec_p': np.array(rec_p), 'rec_e': np.array(rec_e),
            'P_end': P.copy(), 'E_end': E.copy()}


def summarise(r):
    t = r['t_cap']
    cap = ~np.isnan(t)
    out = {'rate': cap.mean(), 'mean': np.nanmean(t) if cap.any() else np.nan,
           'median': np.median(np.where(cap, t, np.inf)), 'n': len(t)}
    for key, mask in [('wall_xy', r['wall_xy']), ('wall_z', r['wall_z'])]:
        out[key + '_frac'] = mask[cap].mean() if cap.any() else np.nan
    anyw = r['wall_xy'] | r['wall_z']
    out['no_wall_t'] = np.nanmean(t[cap & ~anyw]) if (cap & ~anyw).any() else np.nan
    out['no_wall_n'] = int((cap & ~anyw).sum())
    out['esc_min_d'] = np.median(r['min_d'][~cap]) if (~cap).any() else np.nan
    return out


def compute_quiver(rl_p, n_grid=11):
    """RL pursuer command with the evader at rest at (0, 0, 3), pursuer at rest on z slices, no noise."""
    xs = np.linspace(-4.5, 4.5, n_grid)
    X, Y = np.meshgrid(xs, xs)
    e = np.array([0.0, 0.0, Z_MID])
    out = {}
    print(f'\nRL pursuer policy probe (evader at rest at (0, 0, {Z_MID}), pursuer at rest on an '
          f'{n_grid}x{n_grid} grid, no noise):')
    for z in QUIVER_Z:
        Pq = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, z)])
        Eq = np.tile(e, (len(Pq), 1))
        zero = np.zeros_like(Pq)
        v = rl_p.act(Pq, zero, Eq, zero, 0.0)
        ang = np.degrees(np.arccos(np.clip(np.sum(unit(v) * unit(Eq - Pq), axis=1), -1, 1)))
        print(f'  z = {z:.1f} m: mean angle to LOS {ang.mean():5.1f} deg, mean v_z {np.mean(v[:, 2]):+.2f} m/s, '
              f'mean |v_z|/v_P {np.mean(np.abs(v[:, 2])) / V_PURSUER:.3f}')
        out[z] = (X, Y, v[:, 0].reshape(X.shape), v[:, 1].reshape(X.shape), ang.reshape(X.shape),
                  v[:, 2].reshape(X.shape))
    return out


def run_simulation():
    print('S020 3D Pursuit-Evasion Game (RL Capstone)')
    print('=' * 70)
    rl_p, rl_e, curves = train()
    agents = {'RL pursuer': rl_p, 'RL evader': rl_e}
    print(f'\nBenchmark: {N_EPISODES} seeded episodes per cell (seed {SEED}), same spawns, evader '
          f'parameters and noise in every cell; spawn separation >= {SPAWN_MIN_SEP} m')
    res, summ = {}, {}
    for arena in ARENAS:
        print(f'\nArena "{arena}"' + (' (x, y in [-5, 5], z in [0.5, 5.5])' if arena == 'card'
                                     else ' (no x-y walls, z in [0.5, 5.5])'))
        print('  pursuer | evader        | capture | mean t | median | captures with evader on x-y wall / '
              'z limit | mean t no contact (n) | escapes: median closest approach')
        for pk in PURSUERS:
            for ek in EVADERS:
                r = simulate(pk, ek, arena, agents, n_record=N_RECORD)
                s = summarise(r)
                res[(arena, pk, ek)], summ[(arena, pk, ek)] = r, s
                med = 'inf' if np.isinf(s['median']) else f'{s["median"]:.2f}'
                print(f'  {pk:7s} | {ek:13s} | {100 * s["rate"]:5.1f}% | {s["mean"]:5.2f}s | {med:>6s} | '
                      f'{100 * s["wall_xy_frac"]:5.1f}% / {100 * s["wall_z_frac"]:5.1f}%'
                      f'{"":21s} | {s["no_wall_t"]:5.2f}s ({s["no_wall_n"]:3d})'
                      f'{"":11s} | {s["esc_min_d"]:.2f} m')
    for arena in ARENAS:
        print(f'\nMean over the hand-crafted evaders by pursuer, arena {arena} (captured episodes):')
        for pk in PURSUERS:
            ts = np.concatenate([res[(arena, pk, ek)]['t_cap'] for ek in EVADERS if ek != 'RL'])
            print(f'  {pk:7s}: capture {100 * np.mean(~np.isnan(ts)):5.1f}%, mean {np.nanmean(ts):.2f} s')

    print('\nAPF escapes and evader contact (escaped episodes):')
    for arena in ARENAS:
        for ek in ['straight', 'alt_escape']:
            r = res[(arena, 'APF', ek)]
            esc = np.isnan(r['t_cap'])
            md = f'{np.median(r["min_d"][esc]):.2f} m' if esc.any() else '-'
            print(f'  {arena:4s} {ek:10s}: escapes {int(esc.sum()):3d}, with x-y wall contact '
                  f'{int((r["wall_xy"] & esc).sum()):3d}, with z-limit contact {int((r["wall_z"] & esc).sum()):3d}, '
                  f'median closest approach {md}')
    print('\nEvader altitude usage in the card arena vs PP (share of evader steps in the bottom / top 0.5 m):')
    for ek in EVADERS:
        h = res[('card', 'PP', ek)]['ze_hist']
        print(f'  {ek:13s}: {100 * h[0] / h.sum():5.1f}% / {100 * h[-1] / h.sum():5.1f}% '
              f'(together {100 * (h[0] + h[-1]) / h.sum():5.1f}%)')
    r = res[('open', 'PN', 'alt_escape')]
    if np.isnan(r['t_cap'][0]):
        print(f'\nOpen arena, PN vs alt_escape, episode 0 (no capture): pursuer ends '
              f'{np.linalg.norm(r["P_end"][0]):.1f} m from the origin and '
              f'{np.linalg.norm(r["P_end"][0] - r["E_end"][0]):.1f} m from the evader')

    print('\nWithout observation noise (sigma = 0, same spawns), capture rate in the card arena / open arena:')
    print('  pursuer | ' + ' | '.join(f'{e:>13s}' for e in EVADERS))
    for pk in PURSUERS:
        row = []
        for ek in EVADERS:
            rc = summarise(simulate(pk, ek, 'card', agents, sigma=0.0))['rate']
            ro = summarise(simulate(pk, ek, 'open', agents, sigma=0.0))['rate']
            row.append(f'{100 * rc:5.1f} /{100 * ro:5.1f}')
        print(f'  {pk:7s} | ' + ' | '.join(f'{x:>13s}' for x in row))
    quiver = compute_quiver(rl_p)
    return {'res': res, 'summ': summ, 'curves': curves, 'quiver': quiver}


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_learning_curves(curves, out_dir):
    fig, axes = plt.subplots(1, len(curves), figsize=(5 * len(curves), 4.5))
    colors = ['steelblue', 'tomato', 'seagreen', 'darkorange']
    for ax, (key, c), col in zip(axes, curves.items(), colors):
        r = np.array(c['returns'])
        ax.plot(np.arange(1, len(r) + 1), r, color=col, alpha=0.3, lw=0.7)
        w = max(1, len(r) // 15)
        ax.plot(np.arange(w, len(r) + 1), np.convolve(r, np.ones(w) / w, 'valid'), color=col, lw=2,
                label=f'moving mean (w={w})')
        cap = np.array(c['captured'], float)
        ax2 = ax.twinx()
        ax2.plot(np.arange(w, len(cap) + 1), np.convolve(cap, np.ones(w) / w, 'valid') * 100,
                 color='black', ls=':', lw=1.2, label='capture % (moving)')
        ax2.set_ylim(-5, 105); ax2.set_ylabel('Capture % (moving)', fontsize=8)
        ax.set_title(f'{key}\n{c["steps"]} timesteps, {len(r)} episodes', fontsize=9)
        ax.set_xlabel('Episode'); ax.set_ylabel('Episode return'); ax.grid(True, alpha=0.3)
        h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=7, loc='best')
    fig.suptitle('S020 3D — PPO learning curves, card arena (Phase 3 = alternating self-play rounds)',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'learning_curves.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_policy_quiver(quiver, out_dir):
    fig, axes = plt.subplots(1, len(QUIVER_Z), figsize=(6 * len(QUIVER_Z), 5.5))
    for ax, z in zip(axes, QUIVER_Z):
        X, Y, U, V, ang, W = quiver[z]
        q = ax.quiver(X, Y, U, V, ang, cmap='viridis', clim=(0, 180), scale=80, width=0.005)
        ax.scatter(0, 0, c='blue', s=150, marker='^', zorder=5, label=f'evader (0, 0, {Z_MID}), at rest')
        ax.add_patch(plt.Rectangle((-XY_HALF, -XY_HALF), 2 * XY_HALF, 2 * XY_HALF, fill=False, ls='--',
                                   color='grey'))
        ax.set_xlim(-5.5, 5.5); ax.set_ylim(-5.5, 5.5); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_title(f'pursuer at z = {z} m: mean angle to LOS {ang.mean():.0f} deg, mean v_z {W.mean():+.2f} m/s',
                     fontsize=9)
        ax.set_xlabel('Pursuer X (m)'); ax.set_ylabel('Pursuer Y (m)'); ax.legend(fontsize=7, loc='upper right')
    plt.colorbar(q, ax=axes, label='Angle between RL command and line of sight (deg)', fraction=0.02)
    fig.suptitle('S020 3D — RL pursuer command (xy part) by pursuer position at three z slices '
                 '(0 deg = straight at the evader)', fontsize=11)
    path = os.path.join(out_dir, 'policy_quiver.png')
    plt.savefig(path, dpi=110, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def plot_capture_matrix(summ, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(16, 11))
    for row, arena in enumerate(ARENAS):
        rate = np.array([[summ[(arena, p, e)]['rate'] * 100 for e in EVADERS] for p in PURSUERS])
        mt = np.array([[summ[(arena, p, e)]['mean'] for e in EVADERS] for p in PURSUERS])
        for col, (M, title, cmap, fmt) in enumerate([(rate, 'capture rate (%)', 'Greens', '{:.1f}%'),
                                                     (mt, 'mean capture time of captured episodes (s)',
                                                      'Reds', '{:.2f} s')]):
            ax = axes[row, col]
            im = ax.imshow(np.nan_to_num(M, nan=0.0), cmap=cmap, aspect='auto')
            for i in range(len(PURSUERS)):
                for j in range(len(EVADERS)):
                    ax.text(j, i, 'no capture' if np.isnan(M[i, j]) else fmt.format(M[i, j]),
                            ha='center', va='center', fontsize=9)
            ax.set_xticks(range(len(EVADERS))); ax.set_xticklabels(EVADERS)
            ax.set_yticks(range(len(PURSUERS))); ax.set_yticklabels(PURSUERS)
            ax.set_title(f'{arena} arena — {title}', fontsize=10)
            plt.colorbar(im, ax=ax, fraction=0.04)
    fig.suptitle(f'S020 3D — pursuer x evader benchmark, {N_EPISODES} episodes per cell', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_matrix.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_wall_contact(summ, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(18, 5.5))
    x = np.arange(len(EVADERS)); w = 0.13
    off = (len(PURSUERS) - 1) / 2
    for k, pk in enumerate(PURSUERS):
        f = [100 * np.nan_to_num(summ[('card', pk, e)]['wall_xy_frac']) for e in EVADERS]
        axes[0].bar(x + (k - off) * w, f, w, color=P_COLORS[pk], label=pk, edgecolor='black')
        tc = [summ[('card', pk, e)]['mean'] for e in EVADERS]
        to = [summ[('open', pk, e)]['mean'] for e in EVADERS]
        axes[1].bar(x + (k - off) * w, np.nan_to_num(tc), w, color=P_COLORS[pk], edgecolor='black', label=pk)
        axes[1].scatter(x + (k - off) * w, to, color='white', edgecolors='black', zorder=5, s=30,
                        label='open arena (same spawns)' if k == 0 else None)
    axes[0].set_xticks(x); axes[0].set_xticklabels(EVADERS)
    axes[0].set_ylabel('% of captures with the evader on an x-y wall before capture')
    axes[0].set_title('Card arena: wall contact before capture', fontsize=10)
    axes[0].legend(fontsize=8); axes[0].grid(True, alpha=0.3, axis='y')
    axes[1].set_xticks(x); axes[1].set_xticklabels(EVADERS)
    axes[1].set_ylabel('Mean capture time of captured episodes (s)')
    axes[1].set_title('Bars: card arena (colours as left); dots: open arena; no bar = no capture', fontsize=10)
    axes[1].legend(fontsize=8); axes[1].grid(True, alpha=0.3, axis='y')
    fig.suptitle('S020 3D — how much do the walls of the card arena decide the result?', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'wall_contact.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_altitude_usage(res, out_dir):
    fig, axes = plt.subplots(1, len(EVADERS), figsize=(21, 4.5), sharey=True)
    edges = np.linspace(Z_MIN, Z_MAX, 11)
    centers = 0.5 * (edges[1:] + edges[:-1])
    for ax, ek in zip(axes, EVADERS):
        r = res[('card', 'PP', ek)]
        ax.barh(centers, r['ze_hist'] / r['ze_hist'].sum(), height=0.45, color='#0077b6', alpha=0.7,
                label='evader')
        ax.plot(r['zp_hist'] / r['zp_hist'].sum(), centers, 'o-', color='#e63946', label='pursuer (PP)')
        ax.set_title(f'{ek} evader', fontsize=10); ax.set_xlabel('Fraction of agent-steps')
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel('z (m)'); axes[0].legend(fontsize=8)
    fig.suptitle(f'S020 3D — altitude usage over {N_EPISODES} episodes, card arena, PP pursuer', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_usage.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_capture_cdf(res, out_dir):
    fig, axes = plt.subplots(1, len(EVADERS), figsize=(22, 4.8), sharey=True)
    tt = np.linspace(0, T_MAX, 301)
    for ax, ek in zip(axes, EVADERS):
        for pk in PURSUERS:
            for arena, ls in [('card', '-'), ('open', '--')]:
                t = res[(arena, pk, ek)]['t_cap']
                ax.plot(tt, [np.mean(t <= x) for x in tt], ls=ls, color=P_COLORS[pk],
                        label=f'{pk} ({arena})')
        ax.set_xscale('log'); ax.set_xlim(0.3, T_MAX); ax.set_title(f'{ek} evader', fontsize=10)
        ax.set_xlabel('Time (s)'); ax.grid(True, alpha=0.3, which='both')
    axes[0].set_ylabel('Fraction captured'); axes[-1].legend(fontsize=7, loc='lower right', ncol=2)
    fig.suptitle('S020 3D — capture-time CDF (solid = card arena, dashed = open arena)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_cdf.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


TRAJ_EVADER = 'alt_escape'


def draw_box(ax, arena):
    if arena == 'card':
        ax.set_xlim(-5, 5); ax.set_ylim(-5, 5)
    ax.set_zlim(0, 6)


def plot_trajectories_3d(res, out_dir):
    fig = plt.figure(figsize=(26, 9))
    npk = len(PURSUERS)
    for row, arena in enumerate(ARENAS):
        for k, pk in enumerate(PURSUERS):
            r = res[(arena, pk, TRAJ_EVADER)]
            ax = fig.add_subplot(2, npk, row * npk + k + 1, projection='3d')
            tc = r['t_cap'][0]
            n = int(round(tc / DT)) + 1 if not np.isnan(tc) else len(r['rec_p'])
            p, e = r['rec_p'][:n, 0], r['rec_e'][:n, 0]
            ax.plot(p[:, 0], p[:, 1], p[:, 2], color='red', lw=1.6, label=f'pursuer {pk}')
            ax.plot(e[:, 0], e[:, 1], e[:, 2], color='blue', lw=1.6, ls='--', label=f'evader {TRAJ_EVADER}')
            ax.scatter(*p[0], color='red', marker='D', s=30); ax.scatter(*e[0], color='blue', marker='^', s=30)
            if not np.isnan(tc):
                ax.scatter(*e[-1], color='gold', marker='*', s=120, edgecolors='black')
            draw_box(ax, arena)
            ax.set_title(f'{arena}: {pk} — ' + ('timeout' if np.isnan(tc) else f'capture {tc:.2f} s'),
                         fontsize=9)
            ax.tick_params(labelsize=6); ax.legend(fontsize=6, loc='upper left')
    fig.suptitle('S020 3D — episode 0, every pursuer vs the altitude-escape evader '
                 '(top: card arena, bottom: open arena; star = capture)', fontsize=11)
    plt.tight_layout(rect=(0, 0, 1, 0.96))
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=100); plt.close(); print(f'Saved: {path}')


ANIM_CASES = [('RL', 'spiral'), ('PP', 'spiral'), ('PN', 'RL'), ('PP', 'RL')]


def save_animation(res, out_dir):
    runs = []
    for pk, ek in ANIM_CASES:
        r = res[('card', pk, ek)]
        tc = r['t_cap'][0]
        n = int(round(tc / DT)) + 1 if not np.isnan(tc) else len(r['rec_p'])
        runs.append((pk, ek, r['rec_p'][:n, 0], r['rec_e'][:n, 0], tc))
    max_len = max(len(p) for _, _, p, _, _ in runs)
    step = max(4, int(np.ceil((max_len - 1) / 150)))
    frames = int(np.ceil((max_len - 1) / step)) + 1      # last frame = final sample, no duplicate
    fig = plt.figure(figsize=(12, 10))
    arts = []
    for k, (pk, ek, p, e, tc) in enumerate(runs):
        ax = fig.add_subplot(2, 2, k + 1, projection='3d')
        draw_box(ax, 'card')
        pl, = ax.plot([], [], [], color='red', lw=1.4)
        el, = ax.plot([], [], [], color='blue', lw=1.4, ls='--')
        pd, = ax.plot([], [], [], 'o', color='red', mec='black', ms=6)
        ed, = ax.plot([], [], [], '^', color='blue', mec='black', ms=7)
        arts.append((ax, pk, ek, p, e, tc, pl, el, pd, ed))

    def update(f):
        out = []
        for ax, pk, ek, p, e, tc, pl, el, pd, ed in arts:
            n = min(f * step + 1, len(p))
            pl.set_data_3d(p[:n, 0], p[:n, 1], p[:n, 2]); el.set_data_3d(e[:n, 0], e[:n, 1], e[:n, 2])
            pd.set_data_3d([p[n - 1, 0]], [p[n - 1, 1]], [p[n - 1, 2]])
            ed.set_data_3d([e[n - 1, 0]], [e[n - 1, 1]], [e[n - 1, 2]])
            done = n == len(p) and not np.isnan(tc)
            ax.set_title(f'{pk} pursuer vs {ek} evader, card arena   t = {(n - 1) * DT:.2f} s'
                         + ('  CAPTURED' if done else ''), fontsize=9)
            out += [pl, el, pd, ed]
        return out

    ani = FuncAnimation(fig, update, frames=frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=60)
    plt.close(); print(f'Saved: {path}  ({frames} frames)')


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_learning_curves(data['curves'], OUTPUT_DIR)
    plot_policy_quiver(data['quiver'], OUTPUT_DIR)
    plot_capture_matrix(data['summ'], OUTPUT_DIR)
    plot_wall_contact(data['summ'], OUTPUT_DIR)
    plot_altitude_usage(data['res'], OUTPUT_DIR)
    plot_capture_cdf(data['res'], OUTPUT_DIR)
    plot_trajectories_3d(data['res'], OUTPUT_DIR)
    save_animation(data['res'], OUTPUT_DIR)
    print('Done.')
