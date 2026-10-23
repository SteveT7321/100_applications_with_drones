"""
S020 Pursuit-Evasion Game (RL Capstone)
========================================
Capstone of the pursuit-evasion domain.  A pursuer and an evader are trained
with Proximal Policy Optimisation (PPO, stable-baselines3) in a three-phase
curriculum and then benchmarked against hand-crafted strategies:

  Phase 1: pursuer trains against a random evader.
  Phase 2: evader trains against the frozen Phase-1 RL pursuer.
  Phase 3: self-play — pursuer and evader train in alternating rounds, each
           against a frozen snapshot of the other's latest policy.

Benchmark (every pursuer x every evader, TEST_EPISODES seeded episodes each,
same spawn positions for every matchup):
  pursuers: RL, pure pursuit (PP), proportional navigation (PN), artificial
            potential field (APF: attraction to the evader + repulsion from the
            arena walls)
  evaders : RL, straight, perpendicular-to-LOS, spiral

All agents observe the 13-D state with the same Gaussian noise (sigma = 0.1)
that the RL agents were trained with.  Both agents act simultaneously; capture
is detected continuously within each step.  Benchmark episodes are simulated in
a vectorised batch.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s020_pursuit_evasion_game.py
"""

import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK', 'TRUE')   # avoid libomp/libiomp conflict on Windows
import copy
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import torch
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback

# ── Parameters ──────────────────────────────────────────────────────────────
V_PURSUER    = 5.0          # m/s  pursuer speed
V_EVADER     = 3.5          # m/s  evader speed
ARENA        = 20.0         # m    arena half-extent (cube [-20, 20]^3)
CAPTURE_R    = 0.3          # m    capture radius
DT           = 0.05         # s    timestep
T_MAX        = 30.0         # s    episode length
MAX_STEPS    = int(round(T_MAX / DT))   # 600
INIT_R       = ARENA * 0.7  # m    spawn radius; pursuer and evader on opposite sides (28 m apart)
NOISE_SIGMA  = 0.1          # observation noise (m, m/s, s) on every observation component
PN_GAIN      = 3.0          # proportional navigation gain N
APF_D0       = 2.0          # m    APF wall influence distance
APF_ETA      = 1.0 / 6.0    # APF repulsion gain: repulsion = attraction at 0.5 m from a wall
SPIRAL_OMEGA = 0.3          # rad/s spiral evader heading rate

# PPO hyper-parameters (scenario card)
LEARNING_RATE = 3e-4
CLIP_RANGE    = 0.2
BATCH_SIZE    = 256
NET_ARCH      = [256, 256]
N_STEPS_PPO   = 512         # rollout length per update

# Training budget (timesteps).  Card: 5000 / 5000 / 10000 episodes (up to
# 3.0 M / 3.0 M / 6.0 M steps); reduced to keep the script under ~5 min.
PHASE1_TIMESTEPS = 60_000
PHASE2_TIMESTEPS = 60_000
PHASE3_ROUNDS    = 4
PHASE3_STEPS_PER_ROUND = 10_000   # per agent per round -> 80 000 steps in Phase 3

TEST_EPISODES = 1000
BENCH_SEED    = 2024
SEED          = 0

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's020_pursuit_evasion_game',
)

OBS_SCALE = np.array([ARENA] * 3 + [V_PURSUER] * 3 + [ARENA] * 3 + [V_PURSUER] * 3 + [T_MAX],
                     dtype=np.float32)


# ── Dynamics shared by training env and benchmark ───────────────────────────

def unit(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return np.where(n > 1e-9, v / np.maximum(n, 1e-9), 0.0)


def raw_obs(p, pv, e, ev, step):
    """13-D observation [p_P, v_P, p_E, v_E, t_remaining] (batched)."""
    t_rem = np.full((len(p), 1), (MAX_STEPS - step) * DT)
    return np.concatenate([p, pv, e, ev, t_rem], axis=1)


def world_step(p, e, vp, ve):
    """Move both agents for one step; return new positions and capture flags
    (closest approach of the straight-line relative motion within the step)."""
    p_new = np.clip(p + vp * DT, -ARENA, ARENA)
    e_new = np.clip(e + ve * DT, -ARENA, ARENA)
    r0 = e - p
    d = (e_new - e) - (p_new - p)
    dd = np.sum(d * d, axis=1)
    s = np.where(dd > 1e-12, np.clip(-np.sum(r0 * d, axis=1) / np.maximum(dd, 1e-12), 0, 1), 0.0)
    dmin = np.linalg.norm(r0 + s[:, None] * d, axis=1)
    return p_new, e_new, dmin < CAPTURE_R


# ── Agents (batched; obs = noisy 13-D observation) ──────────────────────────

class Agent:
    speed = V_PURSUER
    def reset(self, n, rng, p, e):
        pass
    def act(self, obs, step):
        raise NotImplementedError


class PurePursuit(Agent):
    name = 'Pure pursuit'
    def act(self, obs, step):
        return unit(obs[:, 6:9] - obs[:, 0:3]) * V_PURSUER


class PropNav(Agent):
    """Constant-speed PN: launched along the initial LOS; a = N v_c dlambda/dt turns the velocity."""
    name = 'Prop. nav.'
    def reset(self, n, rng, p, e):
        self.v = unit(e - p) * V_PURSUER
    def act(self, obs, step):
        los = obs[:, 6:9] - obs[:, 0:3]
        r = np.maximum(np.linalg.norm(los, axis=1, keepdims=True), 1e-6)
        lh = los / r
        v_rel = obs[:, 9:12] - self.v
        lam_dot = (v_rel - np.sum(v_rel * lh, axis=1, keepdims=True) * lh) / r
        v_c = -np.sum(v_rel * lh, axis=1, keepdims=True)
        v_new = self.v + PN_GAIN * v_c * lam_dot * DT
        self.v = unit(v_new) * V_PURSUER
        return self.v


class APF(Agent):
    """Attraction toward the evader + repulsion from the six arena walls (Khatib)."""
    name = 'APF'
    def act(self, obs, step):
        p = obs[:, 0:3]
        f = unit(obs[:, 6:9] - p)
        for sign in (-1.0, 1.0):
            dist = np.maximum(ARENA - sign * p, 1e-3)          # distance to the wall at sign*ARENA
            rep = np.where(dist < APF_D0, APF_ETA * (1 / dist - 1 / APF_D0) / dist ** 2, 0.0)
            f = f - sign * rep                                   # push away from that wall
        return unit(f) * V_PURSUER


class RandomEvader(Agent):
    name = 'Random'
    speed = V_EVADER
    def reset(self, n, rng, p, e):
        self.rng = rng
    def act(self, obs, step):
        d = self.rng.standard_normal((len(obs), 3)); d[:, 2] = 0.0
        return unit(d) * V_EVADER


class StraightEvader(Agent):
    name = 'Straight'
    speed = V_EVADER
    def reset(self, n, rng, p, e):
        a = rng.uniform(0, 2 * np.pi, n)
        self.d = np.column_stack([np.cos(a), np.sin(a), np.zeros(n)])
    def act(self, obs, step):
        return self.d * V_EVADER


class PerpEvader(Agent):
    name = 'Perpendicular'
    speed = V_EVADER
    def act(self, obs, step):
        diff = obs[:, 6:8] - obs[:, 0:2]
        d2 = unit(diff)
        return np.column_stack([-d2[:, 1], d2[:, 0], np.zeros(len(obs))]) * V_EVADER


class SpiralEvader(Agent):
    name = 'Spiral'
    speed = V_EVADER
    def act(self, obs, step):
        phi = SPIRAL_OMEGA * step * DT
        n = len(obs)
        return np.tile([np.cos(phi) * V_EVADER, np.sin(phi) * V_EVADER, 0.0], (n, 1))


class RLAgent(Agent):
    """Deterministic PPO policy (frozen copy) acting on the scaled noisy observation."""
    def __init__(self, policy, role):
        self.policy = policy
        self.policy.set_training_mode(False)
        self.speed = V_PURSUER if role == 'pursuer' else V_EVADER
        self.name = f'RL {role}'
    def act(self, obs, step):
        with torch.no_grad():
            x = torch.as_tensor((obs / OBS_SCALE).astype(np.float32))
            a = self.policy._predict(x, deterministic=True).numpy().astype(float)
        a = np.clip(a, -1.0, 1.0)
        return unit(a) * self.speed


# ── Gymnasium environment ───────────────────────────────────────────────────

class PursuitEvasionEnv(gym.Env):
    """
    One learning agent ('pursuer' or 'evader') vs a fixed opponent Agent.
    Obs (13-D, noisy, scaled): [p_P, v_P, p_E, v_E, t_remaining].
    Act (3-D) in [-1, 1]^3 -> velocity = v_max * a / |a|.
    Rewards (card): pursuer +100 capture, +0.01*(d_prev - d), -0.1*|a|/v_max per step;
                    evader  -100 capture, +0.1 per step, +0.01*(d - d_prev).
    """
    metadata = {'render_modes': []}

    def __init__(self, role, opponent, rng_seed):
        super().__init__()
        self.role, self.opponent = role, opponent
        self.rng = np.random.default_rng(rng_seed)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(13,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)

    def _noisy(self):
        o = raw_obs(self.p, self.pv, self.e, self.ev, self.step_i)
        return o + self.rng.normal(0, NOISE_SIGMA, o.shape)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        a = self.rng.uniform(0, 2 * np.pi)
        self.p = np.array([[INIT_R * np.cos(a), INIT_R * np.sin(a), 0.0]])
        self.e = -self.p.copy()
        self.pv = np.zeros((1, 3)); self.ev = np.zeros((1, 3))
        self.step_i = 0
        self.d_prev = float(np.linalg.norm(self.p - self.e))
        self.opponent.reset(1, self.rng, self.p, self.e)
        self.obs = self._noisy()
        return (self.obs[0] / OBS_SCALE).astype(np.float32), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        v_own = unit(a[None]) * (V_PURSUER if self.role == 'pursuer' else V_EVADER)
        v_opp = self.opponent.act(self._noisy(), self.step_i)   # opponent's own noisy view
        vp, ve = (v_own, v_opp) if self.role == 'pursuer' else (v_opp, v_own)
        self.p, self.e, cap = world_step(self.p, self.e, vp, ve)
        self.pv, self.ev = vp, ve
        self.step_i += 1
        d = float(np.linalg.norm(self.p - self.e))
        captured = bool(cap[0])
        if self.role == 'pursuer':
            r = 0.01 * (self.d_prev - d) - 0.1 * np.linalg.norm(a) / V_PURSUER + (100.0 if captured else 0.0)
        else:
            r = 0.1 + 0.01 * (d - self.d_prev) - (100.0 if captured else 0.0)
        self.d_prev = d
        self.obs = self._noisy()
        return ((self.obs[0] / OBS_SCALE).astype(np.float32), float(r), captured,
                self.step_i >= MAX_STEPS, {})


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
    """Independent copy of the current policy (same constructor, copied weights)."""
    pol = model.policy
    clone = type(pol)(**pol._get_constructor_parameters())
    clone.load_state_dict(copy.deepcopy(pol.state_dict()))
    return RLAgent(clone, role)


def train():
    torch.manual_seed(SEED)
    torch.set_num_threads(1)
    curves = {}

    def learn(model, env, steps, key):
        model.set_env(env)
        cb = EpisodeLogger()
        model.learn(total_timesteps=steps, callback=cb, reset_num_timesteps=False)
        curves.setdefault(key, {'returns': [], 'captured': []})
        curves[key]['returns'] += cb.returns
        curves[key]['captured'] += cb.captured
        return cb

    print('Phase 1: pursuer vs random evader ...')
    pursuer = make_ppo(PursuitEvasionEnv('pursuer', RandomEvader(), 11), seed=SEED + 1)
    learn(pursuer, PursuitEvasionEnv('pursuer', RandomEvader(), 11), PHASE1_TIMESTEPS, 'Phase 1: pursuer')

    print('Phase 2: evader vs frozen Phase-1 pursuer ...')
    env2 = PursuitEvasionEnv('evader', frozen(pursuer, 'pursuer'), 12)
    evader = make_ppo(env2, seed=SEED + 2)
    learn(evader, env2, PHASE2_TIMESTEPS, 'Phase 2: evader')

    print(f'Phase 3: self-play, {PHASE3_ROUNDS} alternating rounds ...')
    for k in range(PHASE3_ROUNDS):
        learn(pursuer, PursuitEvasionEnv('pursuer', frozen(evader, 'evader'), 100 + k),
              PHASE3_STEPS_PER_ROUND, 'Phase 3: pursuer')
        learn(evader, PursuitEvasionEnv('evader', frozen(pursuer, 'pursuer'), 200 + k),
              PHASE3_STEPS_PER_ROUND, 'Phase 3: evader')

    print('\nTraining summary (episodes, capture fraction in first / last 20% of episodes):')
    for key, c in curves.items():
        cap = np.array(c['captured'], dtype=float)
        n = len(cap); q = max(1, n // 5)
        print(f'  {key:18s} {n:4d} episodes | captured {cap[:q].mean() * 100:5.1f}% -> '
              f'{cap[-q:].mean() * 100:5.1f}% | mean return first/last 20%: '
              f'{np.mean(c["returns"][:q]):8.2f} / {np.mean(c["returns"][-q:]):8.2f}')
    return frozen(pursuer, 'pursuer'), frozen(evader, 'evader'), curves


# ── Benchmark ───────────────────────────────────────────────────────────────

def simulate(pursuer, evader, n, seed, n_record=0, noise=None, diag=None):
    """
    Vectorised episodes.  Returns capture times (nan = no capture), mean angle
    between pursuer command and LOS, trajectories of the first n_record episodes.
    If diag is a dict it receives per-episode closest approach and final positions.
    """
    sigma = NOISE_SIGMA if noise is None else noise
    rng = np.random.default_rng(seed)
    a = rng.uniform(0, 2 * np.pi, n)
    p = np.column_stack([INIT_R * np.cos(a), INIT_R * np.sin(a), np.zeros(n)])
    e = -p.copy()
    pv = np.zeros((n, 3)); ev = np.zeros((n, 3))
    pursuer.reset(n, rng, p, e); evader.reset(n, rng, p, e)
    active = np.ones(n, bool)
    t_cap = np.full(n, np.nan)
    ang_sum, ang_cnt = 0.0, 0
    traj = [np.stack([p[:n_record], e[:n_record]], 1)] if n_record else None
    closest = np.linalg.norm(p - e, axis=1)
    for step in range(MAX_STEPS):
        base = raw_obs(p, pv, e, ev, step)
        vp = pursuer.act(base + rng.normal(0, sigma, base.shape), step)
        ve = evader.act(base + rng.normal(0, sigma, base.shape), step)
        los = unit(e - p)
        cosang = np.clip(np.sum(unit(vp) * los, axis=1), -1, 1)
        ang_sum += float(np.degrees(np.arccos(cosang[active])).sum()); ang_cnt += int(active.sum())
        vp[~active] = 0.0; ve[~active] = 0.0
        p_new, e_new, cap = world_step(p, e, vp, ve)
        p, e, pv, ev = p_new, e_new, vp, ve
        closest = np.where(active, np.minimum(closest, np.linalg.norm(p - e, axis=1)), closest)
        newly = active & cap
        t_cap[newly] = (step + 1) * DT
        active &= ~cap
        if n_record:
            traj.append(np.stack([p[:n_record], e[:n_record]], 1))
        if not active.any():
            break
    if diag is not None:
        diag.update(closest=closest, p=p, e=e)
    return t_cap, ang_sum / max(ang_cnt, 1), (np.array(traj) if n_record else None)


def benchmark(rl_p, rl_e):
    pursuers = [rl_p, PurePursuit(), PropNav(), APF()]
    evaders = [rl_e, StraightEvader(), PerpEvader(), SpiralEvader()]
    res = {}
    print(f'\nBenchmark: {TEST_EPISODES} episodes per matchup (seed {BENCH_SEED}, same spawns), '
          f'noise sigma {NOISE_SIGMA}')
    print(f'  {"pursuer":13s} {"evader":13s} {"capture":>8s} {"mean t (captured)":>18s} '
          f'{"median t":>9s} {"mean |angle(v_P, LOS)|":>23s}')
    fails = []
    for pu in pursuers:
        for evd in evaders:
            dg = {}
            t, ang, _ = simulate(pu, evd, TEST_EPISODES, BENCH_SEED, diag=dg)
            c = ~np.isnan(t)
            if 0 < (~c).sum() and pu.name != 'RL pursuer':
                f = ~c
                fails.append((pu.name, evd.name, int(f.sum()), np.median(dg['closest'][f]),
                              np.median(np.linalg.norm(dg['p'][f] - dg['e'][f], axis=1)),
                              np.mean(np.max(np.abs(dg['e'][f][:, :2]), axis=1) > ARENA - 1e-6)))
            rate = c.mean()
            med = np.median(np.where(c, t, np.inf))
            res[(pu.name, evd.name)] = {'times': t, 'rate': rate, 'angle': ang,
                                        'mean': float(np.mean(t[c])) if c.any() else np.nan,
                                        'median': med}
            mean_s = f'{np.mean(t[c]):.2f} s' if c.any() else 'no capture'
            med_s = f'{med:.2f} s' if np.isfinite(med) else '> T_max'
            print(f'  {pu.name:13s} {evd.name:13s} {rate * 100:7.1f}% {mean_s:>18s} {med_s:>9s} '
                  f'{ang:20.1f} deg')
    print('\nEscapes against the hand-crafted pursuers (median over the escaped episodes):')
    print(f'  {"pursuer":13s} {"evader":13s} {"escapes":>7s} {"closest approach":>17s} '
          f'{"final distance":>15s} {"evader on a wall":>17s}')
    for a, b, k, cl, fd, wall in fails:
        print(f'  {a:13s} {b:13s} {k:7d} {cl:15.2f} m {fd:13.2f} m {wall * 100:15.0f}%')

    print('\nSame hand-crafted matchups without observation noise (capture rate):')
    for pu in pursuers[1:]:
        row = []
        for evd in evaders[1:]:
            t, _, _ = simulate(pu, evd, TEST_EPISODES, BENCH_SEED, noise=0.0)
            row.append(f'{evd.name} {np.mean(~np.isnan(t)) * 100:5.1f}%')
        print(f'  {pu.name:13s} ' + ' | '.join(row))
    return res, pursuers, evaders


def compute_quiver(rl_p, n_grid=13):
    xs = np.linspace(-ARENA * 0.9, ARENA * 0.9, n_grid)
    X, Y = np.meshgrid(xs, xs)
    p = np.column_stack([X.ravel(), Y.ravel(), np.zeros(X.size)])
    z = np.zeros_like(p)
    v = rl_p.act(raw_obs(p, z, z, z, 0), 0)
    los = unit(-p)
    ang = np.degrees(np.arccos(np.clip(np.sum(unit(v) * los, axis=1), -1, 1)))
    print(f'\nPolicy probe (evader at rest at the origin, pursuer at rest on a {n_grid}x{n_grid} grid, '
          f'no noise): mean angle between RL command and LOS {ang.mean():.1f} deg, '
          f'max {ang.max():.1f} deg, |v_z|/|v| mean {np.mean(np.abs(v[:, 2])) / V_PURSUER:.3f}')
    return X, Y, v[:, 0].reshape(X.shape), v[:, 1].reshape(X.shape), ang.reshape(X.shape)


def run_simulation():
    rl_p, rl_e, curves = train()
    res, pursuers, evaders = benchmark(rl_p, rl_e)
    quiver = compute_quiver(rl_p)
    return {'curves': curves, 'bench': res, 'quiver': quiver, 'rl_p': rl_p, 'rl_e': rl_e,
            'pursuers': pursuers, 'evaders': evaders}


# ── Plots ───────────────────────────────────────────────────────────────────

def plot_learning_curves(curves, out_dir):
    fig, axes = plt.subplots(1, len(curves), figsize=(5 * len(curves), 4.5))
    colors = ['steelblue', 'tomato', 'seagreen', 'darkorange']
    for ax, (key, c), col in zip(axes, curves.items(), colors):
        r = np.array(c['returns'])
        ax.plot(np.arange(1, len(r) + 1), r, color=col, alpha=0.3, lw=0.7)
        w = max(1, len(r) // 15)
        if len(r) >= w:
            ax.plot(np.arange(w, len(r) + 1), np.convolve(r, np.ones(w) / w, 'valid'), color=col, lw=2,
                    label=f'moving mean (w={w})')
        cap = np.array(c['captured'], float)
        ax2 = ax.twinx()
        ax2.plot(np.arange(w, len(cap) + 1), np.convolve(cap, np.ones(w) / w, 'valid') * 100,
                 color='black', ls=':', lw=1.2, label='capture % (moving)')
        ax2.set_ylim(-5, 105); ax2.set_ylabel('Capture % (moving)', fontsize=8)
        ax.set_title(f'{key} ({len(r)} episodes)', fontsize=9)
        ax.set_xlabel('Episode'); ax.set_ylabel('Episode return'); ax.grid(True, alpha=0.3)
        h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=7, loc='best')
    fig.suptitle('S020 PPO learning curves (Phase 3 = alternating self-play rounds)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'learning_curves.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def plot_quiver(quiver, out_dir):
    X, Y, U, V, ang = quiver
    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    q = ax.quiver(X, Y, U, V, ang, cmap='viridis', clim=(0, 180), scale=110, width=0.004)
    plt.colorbar(q, ax=ax, label='Angle between RL command and line of sight (deg)')
    ax.scatter(0, 0, c='royalblue', s=200, marker='^', zorder=5, label='Evader (origin, at rest)')
    ax.add_patch(plt.Rectangle((-ARENA, -ARENA), 2 * ARENA, 2 * ARENA, fill=False, ls='--', color='grey'))
    ax.set_xlim(-ARENA * 1.05, ARENA * 1.05); ax.set_ylim(-ARENA * 1.05, ARENA * 1.05)
    ax.set_xlabel('Pursuer X (m)'); ax.set_ylabel('Pursuer Y (m)')
    ax.set_title('RL pursuer: commanded direction (xy part) by pursuer position\n'
                 '(0 deg = straight at the evader, i.e. pure pursuit)', fontsize=10)
    ax.legend(loc='upper right', fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    path = os.path.join(out_dir, 'policy_quiver.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def plot_performance_comparison(res, pursuers, evaders, out_dir):
    P = [p.name for p in pursuers]; E = [e.name for e in evaders]
    rate = np.array([[res[(p, e)]['rate'] * 100 for e in E] for p in P])
    mean = np.array([[res[(p, e)]['mean'] for e in E] for p in P])
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5))
    for ax, M_, title, fmt, cmap in [(axes[0], rate, 'Capture rate (%)', '{:.1f}%', 'Greens'),
                                     (axes[1], mean, 'Mean capture time of captured episodes (s)',
                                      '{:.2f} s', 'Oranges')]:
        im = ax.imshow(np.nan_to_num(M_, nan=0.0), cmap=cmap, aspect='auto')
        for i in range(len(P)):
            for j in range(len(E)):
                v = M_[i, j]
                ax.text(j, i, 'no capture' if np.isnan(v) else fmt.format(v), ha='center', va='center',
                        fontsize=10, fontweight='bold')
        ax.set_xticks(range(len(E))); ax.set_xticklabels([f'vs {e}' for e in E])
        ax.set_yticks(range(len(P))); ax.set_yticklabels(P)
        ax.set_title(title, fontsize=10, fontweight='bold')
        plt.colorbar(im, ax=ax)
    fig.suptitle(f'S020 — every pursuer vs every evader, {TEST_EPISODES} episodes each '
                 f'(T_max = {T_MAX:.0f} s)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'performance_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def plot_capture_cdf(res, pursuers, evaders, out_dir):
    fig, axes = plt.subplots(1, len(evaders), figsize=(5 * len(evaders), 4.5), sharey=True)
    colors = {'RL pursuer': 'seagreen', 'Pure pursuit': 'crimson', 'Prop. nav.': 'darkorange', 'APF': 'grey'}
    for ax, ev in zip(axes, evaders):
        for pu in pursuers:
            t = res[(pu.name, ev.name)]['times']
            ts = np.sort(t[~np.isnan(t)])
            ax.step(np.concatenate([[0], ts, [T_MAX]]),
                    np.concatenate([[0], np.arange(1, len(ts) + 1), [len(ts)]]) / len(t),
                    where='post', color=colors[pu.name], lw=1.8, label=pu.name)
        ax.set_title(f'vs {ev.name}' + ('' if 'evader' in ev.name else ' evader'), fontsize=10)
        ax.set_xlabel('Time (s)'); ax.set_xlim(0, T_MAX); ax.set_ylim(0, 1.02); ax.grid(True, alpha=0.3)
    axes[0].set_ylabel('Fraction captured by time t'); axes[0].legend(fontsize=8, loc='upper left')
    fig.suptitle(f'S020 — capture-time CDF ({TEST_EPISODES} episodes per curve; '
                 'curve ends below 1 = escapes)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_cdf.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def save_animation(data, out_dir):
    """Episode 0 of four matchups (same spawn), top-down."""
    ev = {e.name: e for e in data['evaders']}
    pu = {p.name: p for p in data['pursuers']}
    configs = [('RL pursuer', 'RL evader'), ('RL pursuer', 'Straight'),
               ('Pure pursuit', 'Straight'), ('Prop. nav.', 'RL evader')]
    trajs = []
    for a, b in configs:
        t, _, tr = simulate(pu[a], ev[b], TEST_EPISODES, BENCH_SEED, n_record=1)
        trajs.append((tr[:, 0], t[0]))
    max_len = max(len(tr[0]) for tr in trajs)
    step = max(1, int(np.ceil(max_len / 150)))
    frames = list(range(0, max_len, step)) + [max_len - 1]
    fig, axes = plt.subplots(2, 2, figsize=(10, 10))
    arts = []
    for ax, (a, b), (tr, tc) in zip(axes.flat, configs, trajs):
        ax.set_xlim(-ARENA * 1.05, ARENA * 1.05); ax.set_ylim(-ARENA * 1.05, ARENA * 1.05)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.25)
        ax.add_patch(plt.Rectangle((-ARENA, -ARENA), 2 * ARENA, 2 * ARENA, fill=False, ls='--', color='grey'))
        ax.scatter(*tr[0, 0, :2], c='crimson', marker='D', s=40); ax.scatter(*tr[0, 1, :2], c='royalblue', marker='^', s=40)
        pl, = ax.plot([], [], color='crimson', lw=1.3); el, = ax.plot([], [], color='royalblue', lw=1.3, ls='--')
        pd, = ax.plot([], [], 'o', color='crimson', ms=7); ed, = ax.plot([], [], '^', color='royalblue', ms=7)
        arts.append((ax, a, b, tr, tc, pl, el, pd, ed))

    def update(f):
        out = []
        for ax, a, b, tr, tc, pl, el, pd, ed in arts:
            k = min(f, len(tr) - 1)
            pl.set_data(tr[:k + 1, 0, 0], tr[:k + 1, 0, 1]); el.set_data(tr[:k + 1, 1, 0], tr[:k + 1, 1, 1])
            pd.set_data([tr[k, 0, 0]], [tr[k, 0, 1]]); ed.set_data([tr[k, 1, 0]], [tr[k, 1, 1]])
            end = 'captured at %.2f s' % tc if not np.isnan(tc) else 'no capture in 30 s'
            ax.set_title(f'{a} vs {b}' + ('' if 'evader' in b else ' evader') +
                         f'\nt = {k * DT:.2f} s ({end})', fontsize=9)
            out += [pl, el, pd, ed]
        return out

    ani = animation.FuncAnimation(fig, update, frames=frames, interval=50, blit=False)
    fig.suptitle('S020 — benchmark episode 0 (red = pursuer, blue = evader)', fontsize=11)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=70)
    plt.close(); print(f'Saved: {path}  ({len(frames)} frames)')
    for (a, b), (tr, tc) in zip(configs, trajs):
        print(f'  animation: {a} vs {b}: ' + (f'captured at {tc:.2f} s' if not np.isnan(tc) else 'no capture'))


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    print('=' * 78)
    print('S020 Pursuit-Evasion Game (RL Capstone)')
    print('=' * 78)
    data = run_simulation()
    plot_learning_curves(data['curves'], out_dir)
    plot_quiver(data['quiver'], out_dir)
    plot_performance_comparison(data['bench'], data['pursuers'], data['evaders'], out_dir)
    plot_capture_cdf(data['bench'], data['pursuers'], data['evaders'], out_dir)
    save_animation(data, out_dir)
