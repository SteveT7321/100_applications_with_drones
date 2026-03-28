"""
S020 Pursuit-Evasion Game (RL Capstone)
========================================
Capstone of the pursuit-evasion domain. Both pursuer and evader are trained
with Proximal Policy Optimisation (PPO). The trained RL policies are benchmarked
against hand-crafted strategies (pure pursuit, proportional navigation, APF;
straight, perpendicular and spiral evasion). Three training phases:
  Phase 1: pursuer trains against a random evader.
  Phase 2: evader trains against a frozen pure-pursuit pursuer (approximates
            the frozen Phase-1 pursuer described in the scenario card).
  Phase 3: pursuer continues training against the Phase-2 RL evader
            (self-play refinement).
A fast, faithful simulation of all three curriculum phases plus 300-episode
benchmark suite and CDF analysis.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s020_pursuit_evasion_game.py
"""

import sys, os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK', 'TRUE')   # avoid libomp/libiomp conflict on Windows
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ──────────────────────────────────────────────────────────────
V_PURSUER    = 5.0          # m/s  pursuer max speed
V_EVADER     = 3.5          # m/s  evader max speed
ARENA        = 5.0          # m    arena half-extent ([-5,5]^3)
CAPTURE_R    = 0.15         # m    capture radius
DT           = 0.05         # s    timestep
T_MAX        = 30.0         # s    episode length
NOISE_SIGMA  = 0.1          # m    observation noise
EPS_VEL      = 1e-8         # safety epsilon

# PPO hyper-parameters (from scenario card)
LEARNING_RATE = 3e-4
CLIP_RANGE    = 0.2
BATCH_SIZE    = 256
NET_ARCH      = [256, 256]
N_STEPS_PPO   = 512         # rollout buffer size (shorter = faster iteration)

# Training scale (scenario card: 5000/5000/10000 episodes; scaled ~1:40 for demo)
PHASE1_TIMESTEPS = 80_000   # ~133 episodes
PHASE2_TIMESTEPS = 80_000
PHASE3_TIMESTEPS = 120_000
MAX_STEPS        = int(T_MAX / DT)   # 600 steps / episode

TEST_EPISODES    = 300

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's020_pursuit_evasion_game',
)

RNG = np.random.default_rng(0)


# ── Hand-crafted strategies ──────────────────────────────────────────────────

def _normalise(v):
    n = np.linalg.norm(v)
    return v / (n + EPS_VEL) if n > EPS_VEL else v


def strategy_pure_pursuit(p_pos, e_pos):
    """Return velocity command for pure-pursuit pursuer."""
    return _normalise(e_pos - p_pos) * V_PURSUER


def strategy_prop_nav(p_pos, p_vel, e_pos, e_vel, N_gain=3.0):
    """Proportional Navigation pursuer velocity command."""
    r_vec = e_pos - p_pos
    r_dist = np.linalg.norm(r_vec) + EPS_VEL
    los = r_vec / r_dist
    rel_vel = e_vel - p_vel
    los_rate = (rel_vel - np.dot(rel_vel, los) * los) / r_dist
    accel = N_gain * V_PURSUER * los_rate
    new_vel = p_vel + accel * DT
    spd = np.linalg.norm(new_vel)
    if spd > V_PURSUER:
        new_vel = new_vel / spd * V_PURSUER
    return new_vel


def strategy_apf(p_pos, e_pos):
    """APF pursuer velocity command (attractive field)."""
    return _normalise(e_pos - p_pos) * V_PURSUER


def evader_random(rng):
    """Random velocity command for evader."""
    d = rng.standard_normal(3); d[2] = 0.0
    return _normalise(d) * V_EVADER


def evader_straight(direction):
    """Constant-direction evader velocity."""
    return _normalise(direction) * V_EVADER


def evader_perpendicular(p_pos, e_pos):
    """Perpendicular-to-LOS evader."""
    diff = e_pos - p_pos
    if np.linalg.norm(diff[:2]) < EPS_VEL:
        return np.array([V_EVADER, 0.0, 0.0])
    d2 = diff[:2] / np.linalg.norm(diff[:2])
    perp = np.array([-d2[1], d2[0], 0.0])
    return perp * V_EVADER


def evader_spiral(t):
    """Spiral evader."""
    omega = 0.3
    phi = omega * t
    return np.array([np.cos(phi), np.sin(phi), 0.0]) * V_EVADER


# ── Fast episode runner (no model.predict inside) ────────────────────────────

def run_episode(pursuer_fn, evader_fn, seed):
    """
    Run one episode using callable policy functions.
    pursuer_fn(p_pos, p_vel, e_pos, e_vel, t) -> v_cmd (3,)
    evader_fn (p_pos, p_vel, e_pos, e_vel, t) -> v_cmd (3,)
    Returns capture_time or None (escaped).
    """
    rng = np.random.default_rng(seed)
    p_pos = rng.uniform(-ARENA * 0.8, ARENA * 0.8, 3); p_pos[2] = 0.0
    e_pos = rng.uniform(-ARENA * 0.8, ARENA * 0.8, 3); e_pos[2] = 0.0
    p_vel = np.zeros(3)
    e_vel = np.zeros(3)
    for step in range(MAX_STEPS):
        dist = np.linalg.norm(p_pos - e_pos)
        if dist < CAPTURE_R:
            return step * DT
        t = step * DT
        p_vel = pursuer_fn(p_pos, p_vel, e_pos, e_vel, t)
        e_vel = evader_fn(p_pos, p_vel, e_pos, e_vel, t)
        p_pos = np.clip(p_pos + p_vel * DT, -ARENA, ARENA)
        e_pos = np.clip(e_pos + e_vel * DT, -ARENA, ARENA)
    return None


# ── Gymnasium Environment ────────────────────────────────────────────────────

class PursuitEvasionEnv(gym.Env):
    """
    Single-pursuer single-evader environment.
    Obs (13-D): [p_P(3), v_P(3), p_E(3), v_E(3), t_remaining(1)]
    Act (3-D): normalised velocity command in [-1,1]^3.
    opponent_fn(p_pos, p_vel, e_pos, e_vel, t) -> v_cmd  (hand-crafted; fast)
    """
    metadata = {'render_modes': []}

    def __init__(self, role='pursuer', opponent_fn=None,
                 noise_sigma=NOISE_SIGMA, rng_seed=None):
        super().__init__()
        self.role = role
        self.opponent_fn = opponent_fn
        self.noise_sigma = noise_sigma
        self._rng = np.random.default_rng(rng_seed)

        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(13,), dtype=np.float32)
        self.action_space      = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)

        self.pursuer_pos = None; self.pursuer_vel = None
        self.evader_pos  = None; self.evader_vel  = None
        self.t_step = 0; self._prev_dist = None

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        rng = self._rng
        self.pursuer_pos = rng.uniform(-ARENA * 0.8, ARENA * 0.8, 3); self.pursuer_pos[2] = 0.0
        self.evader_pos  = rng.uniform(-ARENA * 0.8, ARENA * 0.8, 3); self.evader_pos[2]  = 0.0
        self.pursuer_vel = np.zeros(3)
        self.evader_vel  = np.zeros(3)
        self.t_step = 0
        self._prev_dist = np.linalg.norm(self.pursuer_pos - self.evader_pos)
        return self._obs(), {}

    def _obs(self):
        t_rem = (MAX_STEPS - self.t_step) * DT
        raw = np.concatenate([
            self.pursuer_pos, self.pursuer_vel,
            self.evader_pos, self.evader_vel, [t_rem]
        ]).astype(np.float32)
        raw += self._rng.normal(0, self.noise_sigma, size=13).astype(np.float32)
        return raw

    def step(self, action):
        action = np.asarray(action, dtype=float)
        t = self.t_step * DT
        if self.role == 'pursuer':
            v_p = V_PURSUER * action / (np.linalg.norm(action) + EPS_VEL)
            self.pursuer_vel = v_p
            self.pursuer_pos = np.clip(self.pursuer_pos + v_p * DT, -ARENA, ARENA)
            # opponent: hand-crafted evader
            if self.opponent_fn is not None:
                v_e = self.opponent_fn(self.pursuer_pos, self.pursuer_vel,
                                       self.evader_pos, self.evader_vel, t)
            else:
                v_e = evader_random(self._rng)
            self.evader_vel = v_e
            self.evader_pos = np.clip(self.evader_pos + v_e * DT, -ARENA, ARENA)
        else:  # evader
            v_e = V_EVADER * action / (np.linalg.norm(action) + EPS_VEL)
            self.evader_vel = v_e
            self.evader_pos = np.clip(self.evader_pos + v_e * DT, -ARENA, ARENA)
            if self.opponent_fn is not None:
                v_p = self.opponent_fn(self.pursuer_pos, self.pursuer_vel,
                                       self.evader_pos, self.evader_vel, t)
            else:
                v_p = _normalise(self.evader_pos - self.pursuer_pos) * V_PURSUER
            self.pursuer_vel = v_p
            self.pursuer_pos = np.clip(self.pursuer_pos + v_p * DT, -ARENA, ARENA)

        dist = np.linalg.norm(self.pursuer_pos - self.evader_pos)
        captured = dist < CAPTURE_R
        self.t_step += 1

        if self.role == 'pursuer':
            reward = 0.01 * (self._prev_dist - dist)
            reward -= 0.1 * np.linalg.norm(action) / (V_PURSUER + EPS_VEL)
            if captured: reward += 100.0
        else:
            reward = 0.1
            reward += 0.01 * (dist - self._prev_dist)
            if captured: reward -= 100.0

        self._prev_dist = dist
        terminated = captured
        truncated  = (self.t_step >= MAX_STEPS)
        return self._obs(), float(reward), terminated, truncated, {}


# ── Reward logger ────────────────────────────────────────────────────────────

class RewardLogger(BaseCallback):
    def __init__(self):
        super().__init__()
        self.episode_rewards = []
        self._cur = 0.0

    def _on_step(self):
        self._cur += self.locals['rewards'][0]
        if self.locals['dones'][0]:
            self.episode_rewards.append(self._cur)
            self._cur = 0.0
        return True


# ── Training ─────────────────────────────────────────────────────────────────

def _make_ppo(env, seed):
    return PPO(
        "MlpPolicy", env, verbose=0,
        learning_rate=LEARNING_RATE,
        clip_range=CLIP_RANGE,
        batch_size=BATCH_SIZE,
        n_steps=N_STEPS_PPO,
        policy_kwargs=dict(net_arch=NET_ARCH),
        seed=seed,
    )


def run_simulation():
    """3-phase PPO training + benchmark."""

    # ── Phase 1: pursuer vs random evader ───────────────────────────────────
    print('Phase 1: pursuer trains vs random evader ...')
    env1 = PursuitEvasionEnv(role='pursuer', opponent_fn=None, rng_seed=1)
    cb1  = RewardLogger()
    model_pursuer = _make_ppo(env1, seed=1)
    model_pursuer.learn(total_timesteps=PHASE1_TIMESTEPS, callback=cb1)

    # ── Phase 2: evader vs pure-pursuit pursuer ──────────────────────────────
    print('Phase 2: evader trains vs pure-pursuit pursuer ...')
    def _pp_opponent(p_pos, p_vel, e_pos, e_vel, t):
        return strategy_pure_pursuit(p_pos, e_pos)

    env2 = PursuitEvasionEnv(role='evader', opponent_fn=_pp_opponent, rng_seed=2)
    cb2  = RewardLogger()
    model_evader = _make_ppo(env2, seed=2)
    model_evader.learn(total_timesteps=PHASE2_TIMESTEPS, callback=cb2)

    # ── Phase 3: pursuer self-play vs RL evader ──────────────────────────────
    print('Phase 3: pursuer self-play vs RL evader ...')

    # Extract RL evader policy as a fast numpy callable
    import torch
    evader_policy_net = model_evader.policy
    evader_policy_net.set_training_mode(False)

    def _rl_evader_opponent(p_pos, p_vel, e_pos, e_vel, t):
        t_rem = (MAX_STEPS - int(t / DT)) * DT
        obs = np.concatenate([p_pos, p_vel, e_pos, e_vel, [t_rem]]).astype(np.float32)
        with torch.no_grad():
            obs_t = torch.as_tensor(obs[None])
            action = evader_policy_net._predict(obs_t, deterministic=True).numpy()[0]
        return V_EVADER * action / (np.linalg.norm(action) + EPS_VEL)

    env3 = PursuitEvasionEnv(role='pursuer', opponent_fn=_rl_evader_opponent, rng_seed=3)
    cb3  = RewardLogger()
    # Fine-tune pursuer from Phase-1 weights
    model_pursuer.set_env(env3)
    model_pursuer.learn(total_timesteps=PHASE3_TIMESTEPS, callback=cb3, reset_num_timesteps=False)

    # ── Assemble learning curves ─────────────────────────────────────────────
    learning_curves = {
        'Phase 1 — Pursuer vs Random':    cb1.episode_rewards,
        'Phase 2 — Evader vs Pure-Pursuit': cb2.episode_rewards,
        'Phase 3 — Pursuer vs RL Evader':  cb3.episode_rewards,
    }

    # ── Benchmark ───────────────────────────────────────────────────────────
    print('Benchmarking ...')
    # Build fast numpy functions for RL pursuer & evader
    pursuer_policy_net = model_pursuer.policy
    pursuer_policy_net.set_training_mode(False)
    import torch

    def _rl_pursuer_fn(p_pos, p_vel, e_pos, e_vel, t):
        t_rem = (MAX_STEPS - int(t / DT)) * DT
        obs = np.concatenate([p_pos, p_vel, e_pos, e_vel, [t_rem]]).astype(np.float32)
        with torch.no_grad():
            obs_t = torch.as_tensor(obs[None])
            action = pursuer_policy_net._predict(obs_t, deterministic=True).numpy()[0]
        return V_PURSUER * action / (np.linalg.norm(action) + EPS_VEL)

    def _rl_evader_fn(p_pos, p_vel, e_pos, e_vel, t):
        t_rem = (MAX_STEPS - int(t / DT)) * DT
        obs = np.concatenate([p_pos, p_vel, e_pos, e_vel, [t_rem]]).astype(np.float32)
        with torch.no_grad():
            obs_t = torch.as_tensor(obs[None])
            action = evader_policy_net._predict(obs_t, deterministic=True).numpy()[0]
        return V_EVADER * action / (np.linalg.norm(action) + EPS_VEL)

    bench_results = benchmark(
        _rl_pursuer_fn, _rl_evader_fn,
        lambda seed: np.random.default_rng(seed),
    )

    # ── Quiver ───────────────────────────────────────────────────────────────
    quiver_data = compute_quiver(_rl_pursuer_fn)

    return learning_curves, bench_results, quiver_data, _rl_pursuer_fn, _rl_evader_fn


# ── Benchmark ────────────────────────────────────────────────────────────────

def benchmark(rl_pursuer_fn, rl_evader_fn, rng_factory):
    """Run TEST_EPISODES episodes for each strategy pair."""
    results = {}
    n = TEST_EPISODES

    def _straight_evader_fn(p_pos, p_vel, e_pos, e_vel, t, direction):
        return evader_straight(direction)

    def _perp_evader_fn(p_pos, p_vel, e_pos, e_vel, t):
        return evader_perpendicular(p_pos, e_pos)

    def _spiral_evader_fn(p_pos, p_vel, e_pos, e_vel, t):
        return evader_spiral(t)

    def _pp_pursuer_fn(p_pos, p_vel, e_pos, e_vel, t):
        return strategy_pure_pursuit(p_pos, e_pos)

    def _pn_pursuer_fn(p_pos, p_vel, e_pos, e_vel, t):
        return strategy_prop_nav(p_pos, p_vel, e_pos, e_vel)

    def _apf_pursuer_fn(p_pos, p_vel, e_pos, e_vel, t):
        return strategy_apf(p_pos, e_pos)

    # Pair RL pursuer vs hand-crafted evaders
    for ev_name, ev_fn in [('straight', None), ('perpendicular', _perp_evader_fn),
                            ('spiral', _spiral_evader_fn)]:
        times = []
        for i in range(n):
            rng_ep = np.random.default_rng(i)
            d_str = _normalise(rng_ep.standard_normal(3)) * V_EVADER
            d_str[2] = 0.0
            if ev_name == 'straight':
                fn = lambda pp, pv, ep, ev, t, d=d_str: evader_straight(d)
            else:
                fn = ev_fn
            ct = run_episode(rl_pursuer_fn, fn, seed=i)
            times.append(ct)
        cap = [t for t in times if t is not None]
        results[f'RL pursuer vs {ev_name}'] = {
            'capture_rate': len(cap) / n,
            'mean_time': float(np.mean(cap)) if cap else T_MAX,
            'times': times,
        }

    # RL pursuer vs RL evader
    times = [run_episode(rl_pursuer_fn, rl_evader_fn, seed=i) for i in range(n)]
    cap = [t for t in times if t is not None]
    results['RL pursuer vs RL evader'] = {
        'capture_rate': len(cap) / n,
        'mean_time': float(np.mean(cap)) if cap else T_MAX,
        'times': times,
    }

    # Hand-crafted pursuers vs straight evader
    for pu_name, pu_fn in [('pure pursuit', _pp_pursuer_fn),
                            ('prop nav', _pn_pursuer_fn),
                            ('APF', _apf_pursuer_fn)]:
        times = []
        for i in range(n):
            rng_ep = np.random.default_rng(i)
            d_str = _normalise(rng_ep.standard_normal(3)) * V_EVADER
            d_str[2] = 0.0
            fn = lambda pp, pv, ep, ev, t, d=d_str: evader_straight(d)
            ct = run_episode(pu_fn, fn, seed=i)
            times.append(ct)
        cap = [t for t in times if t is not None]
        results[f'{pu_name} vs straight'] = {
            'capture_rate': len(cap) / n,
            'mean_time': float(np.mean(cap)) if cap else T_MAX,
            'times': times,
        }

    # Hand-crafted pursuers vs RL evader
    for pu_name, pu_fn in [('pure pursuit', _pp_pursuer_fn),
                            ('prop nav', _pn_pursuer_fn)]:
        times = [run_episode(pu_fn, rl_evader_fn, seed=i) for i in range(n)]
        cap = [t for t in times if t is not None]
        results[f'{pu_name} vs RL evader'] = {
            'capture_rate': len(cap) / n,
            'mean_time': float(np.mean(cap)) if cap else T_MAX,
            'times': times,
        }

    return results


# ── Quiver ───────────────────────────────────────────────────────────────────

def compute_quiver(rl_pursuer_fn, n_grid=12):
    xs = np.linspace(-ARENA * 0.8, ARENA * 0.8, n_grid)
    ys = np.linspace(-ARENA * 0.8, ARENA * 0.8, n_grid)
    U = np.zeros((n_grid, n_grid))
    V = np.zeros((n_grid, n_grid))
    e_pos = np.zeros(3); e_vel = np.zeros(3)
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            p_pos = np.array([x, y, 0.0])
            v = rl_pursuer_fn(p_pos, np.zeros(3), e_pos, e_vel, 0.0)
            U[j, i] = v[0]
            V[j, i] = v[1]
    return xs, ys, U, V


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_learning_curves(learning_curves, out_dir):
    """Learning curves for all training phases."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    colors = ['steelblue', 'tomato', 'seagreen']

    for ax, (label, rewards), color in zip(axes, learning_curves.items(), colors):
        if len(rewards) == 0:
            ax.text(0.5, 0.5, 'No data', transform=ax.transAxes, ha='center')
            continue
        episodes = np.arange(1, len(rewards) + 1)
        ax.plot(episodes, rewards, color=color, alpha=0.3, lw=0.7)
        win = max(1, len(rewards) // 15)
        smoothed = np.convolve(rewards, np.ones(win) / win, mode='valid')
        ax.plot(np.arange(win, len(rewards) + 1), smoothed, color=color, lw=2.0,
                label=f'Smoothed (w={win})')
        ax.set_title(label, fontsize=9)
        ax.set_xlabel('Episode', fontsize=9)
        ax.set_ylabel('Episode Return', fontsize=9)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    fig.suptitle('S020 PPO Learning Curves — 3-Phase Curriculum', fontsize=12)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'learning_curves.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_quiver(quiver_data, out_dir):
    """RL pursuer policy direction over XY plane."""
    xs, ys, U, V = quiver_data
    XX, YY = np.meshgrid(xs, ys)
    mag = np.sqrt(U**2 + V**2)

    fig, ax = plt.subplots(figsize=(7, 6))
    q = ax.quiver(XX, YY, U, V, mag, cmap='plasma', scale=20, width=0.004)
    plt.colorbar(q, ax=ax, label='Action magnitude')
    ax.scatter(0, 0, c='limegreen', s=200, marker='*', zorder=5, label='Evader (origin)')
    ax.set_xlabel('Pursuer X (m)'); ax.set_ylabel('Pursuer Y (m)')
    ax.set_title('RL Pursuer Policy: Action Direction (evader at origin)', fontsize=10)
    ax.legend(); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'policy_quiver.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_performance_comparison(bench_results, out_dir):
    """Bar chart: capture rate and mean time for all strategy pairs."""
    keys = list(bench_results.keys())
    labels = [k.replace(' vs ', '\nvs\n') for k in keys]
    cap_rates  = [bench_results[k]['capture_rate'] * 100 for k in keys]
    mean_times = [bench_results[k]['mean_time'] for k in keys]

    def _color(k):
        if k.startswith('RL pursuer'):
            return 'seagreen'
        if k.endswith('RL evader'):
            return 'tomato'
        return 'steelblue'
    colors = [_color(k) for k in keys]

    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    x = np.arange(len(keys))
    for ax, vals, ylabel, title, fmt in [
        (axes[0], cap_rates,  'Capture Rate (%)',     'Capture Rate',     '{:.0f}%'),
        (axes[1], mean_times, 'Mean Capture Time (s)', 'Mean Capture Time', '{:.1f}s'),
    ]:
        bars = ax.bar(x, vals, color=colors, edgecolor='black', lw=0.7)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.3,
                    fmt.format(v), ha='center', va='bottom', fontsize=7, fontweight='bold')
        ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=6)
        ax.set_ylabel(ylabel, fontsize=10); ax.set_title(title, fontsize=10)
        ax.grid(True, axis='y', alpha=0.3)

    axes[0].set_ylim(0, 118)
    axes[1].set_ylim(0, T_MAX * 1.2)

    legend_patches = [
        mpatches.Patch(color='seagreen',  label='RL pursuer'),
        mpatches.Patch(color='tomato',    label='vs RL evader'),
        mpatches.Patch(color='steelblue', label='Baseline'),
    ]
    fig.legend(handles=legend_patches, loc='upper right', fontsize=9)
    fig.suptitle('S020 — Performance Comparison', fontsize=12)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'performance_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_capture_cdf(bench_results, out_dir):
    """CDF of capture times for key strategy pairs."""
    plot_keys = [
        ('RL pursuer vs straight',     'RL pursuer vs Straight',      'seagreen',       '-'),
        ('RL pursuer vs perpendicular', 'RL pursuer vs Perpendicular', 'mediumseagreen', '--'),
        ('RL pursuer vs spiral',        'RL pursuer vs Spiral',        'limegreen',      ':'),
        ('RL pursuer vs RL evader',     'RL pursuer vs RL evader',     'darkgreen',      '-.'),
        ('pure pursuit vs straight',    'Pure pursuit vs Straight',    'steelblue',      '-'),
        ('pure pursuit vs RL evader',   'Pure pursuit vs RL evader',   'tomato',         '--'),
        ('prop nav vs straight',        'Prop nav vs Straight',        'darkorange',     ':'),
    ]

    fig, ax = plt.subplots(figsize=(9, 6))
    for key, label, color, ls in plot_keys:
        if key not in bench_results:
            continue
        times = sorted(t if t is not None else T_MAX for t in bench_results[key]['times'])
        n = len(times)
        ax.step(times, np.arange(1, n + 1) / n, color=color, lw=1.8,
                linestyle=ls, label=label, where='post')

    ax.axvline(T_MAX, color='gray', lw=1.0, linestyle=':', alpha=0.6, label=f'T_MAX={T_MAX}s')
    ax.set_xlabel('Capture Time (s)', fontsize=11)
    ax.set_ylabel('Cumulative Probability', fontsize=11)
    ax.set_title(f'S020 — Capture Time CDF ({TEST_EPISODES} test episodes each)', fontsize=11)
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(True, alpha=0.35)
    ax.set_xlim(0, T_MAX + 0.5); ax.set_ylim(0, 1.05)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'capture_cdf.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(rl_pursuer_fn, rl_evader_fn, out_dir):
    """Animate 4 representative episodes in XY plane."""
    import matplotlib.animation as animation

    configs = [
        ('RL Pursuer vs\nRL Evader',    rl_pursuer_fn,       rl_evader_fn),
        ('RL Pursuer vs\nStraight',     rl_pursuer_fn,       None),
        ('Pure Pursuit vs\nStraight',   None,                None),
        ('RL Pursuer vs\nPerpendicular', rl_pursuer_fn,      None),
    ]

    def _collect(pu_fn, ev_fn_type, seed=42):
        rng = np.random.default_rng(seed)
        # Start pursuer and evader on opposite sides of the arena
        angle = rng.uniform(0, 2 * np.pi)
        r = ARENA * 0.75
        p_pos = np.array([ r * np.cos(angle),  r * np.sin(angle),  0.0])
        e_pos = np.array([-r * np.cos(angle), -r * np.sin(angle),  0.0])
        p_vel = np.zeros(3); e_vel = np.zeros(3)
        d_str = _normalise(rng.standard_normal(3)); d_str[2] = 0.0
        p_traj = [p_pos.copy()]; e_traj = [e_pos.copy()]
        for step in range(MAX_STEPS):
            dist = np.linalg.norm(p_pos - e_pos)
            if dist < CAPTURE_R: break
            t = step * DT
            if pu_fn is None:
                p_vel = strategy_pure_pursuit(p_pos, e_pos)
            else:
                p_vel = pu_fn(p_pos, p_vel, e_pos, e_vel, t)
            if ev_fn_type == 'rl':
                e_vel = rl_evader_fn(p_pos, p_vel, e_pos, e_vel, t)
            elif ev_fn_type == 'perp':
                e_vel = evader_perpendicular(p_pos, e_pos)
            else:
                e_vel = evader_straight(d_str)
            p_pos = np.clip(p_pos + p_vel * DT, -ARENA, ARENA)
            e_pos = np.clip(e_pos + e_vel * DT, -ARENA, ARENA)
            p_traj.append(p_pos.copy()); e_traj.append(e_pos.copy())
        return np.array(p_traj), np.array(e_traj)

    ev_types = ['rl', 'straight', 'straight', 'perp']
    all_p, all_e = [], []
    for (title, pu_fn, _), ev_type in zip(configs, ev_types):
        p_t, e_t = _collect(pu_fn, ev_type)
        all_p.append(p_t); all_e.append(e_t)

    max_frames = max(len(t) for t in all_p)
    step_dec   = max(1, max_frames // 300)

    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    axes_flat  = axes.flatten()
    p_lines, e_lines, p_dots, e_dots = [], [], [], []

    for ax, (title, _, _), p_t, e_t in zip(axes_flat, configs, all_p, all_e):
        ax.set_xlim(-ARENA * 1.05, ARENA * 1.05); ax.set_ylim(-ARENA * 1.05, ARENA * 1.05)
        ax.set_aspect('equal'); ax.set_title(title, fontsize=9)
        ax.set_xlabel('X (m)', fontsize=7); ax.set_ylabel('Y (m)', fontsize=7)
        ax.tick_params(labelsize=6)
        rect = plt.Rectangle((-ARENA, -ARENA), 2 * ARENA, 2 * ARENA,
                              fill=False, edgecolor='gray', lw=0.8, ls='--')
        ax.add_patch(rect)
        ax.scatter(*p_t[0, :2], c='crimson', s=40, marker='D', zorder=5)
        ax.scatter(*e_t[0, :2], c='royalblue', s=40, marker='^', zorder=5)
        pl, = ax.plot([], [], 'crimson', lw=1.2, alpha=0.7)
        el, = ax.plot([], [], 'royalblue', lw=1.2, alpha=0.7, ls='--')
        pd, = ax.plot([], [], 'o', color='crimson', ms=7)
        ed, = ax.plot([], [], '^', color='royalblue', ms=7)
        p_lines.append(pl); e_lines.append(el)
        p_dots.append(pd); e_dots.append(ed)

    all_artists = p_lines + e_lines + p_dots + e_dots

    def init():
        for a in all_artists: a.set_data([], [])
        return all_artists

    def update(frame):
        for i in range(4):
            sub_p = all_p[i][::step_dec]; sub_e = all_e[i][::step_dec]
            fp = min(frame, len(sub_p) - 1); fe = min(frame, len(sub_e) - 1)
            p_lines[i].set_data(sub_p[:fp+1, 0], sub_p[:fp+1, 1])
            e_lines[i].set_data(sub_e[:fe+1, 0], sub_e[:fe+1, 1])
            p_dots[i].set_data([sub_p[fp, 0]], [sub_p[fp, 1]])
            e_dots[i].set_data([sub_e[fe, 0]], [sub_e[fe, 1]])
        return all_artists

    n_frames = max_frames // step_dec + 1
    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  init_func=init, blit=True, interval=50)
    fig.suptitle('S020 Pursuit-Evasion Game — RL vs Baselines', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    learning_curves, bench_results, quiver_data, rl_pursuer_fn, rl_evader_fn = run_simulation()

    print('=' * 70)
    print('S020 Pursuit-Evasion Game (RL Capstone) — Results')
    print('=' * 70)
    for key, v in bench_results.items():
        print(f'  {key:<35s}  cap={v["capture_rate"]*100:.1f}%  '
              f'mean_time={v["mean_time"]:.2f}s')
    print('=' * 70)

    rl_vs_str  = bench_results.get('RL pursuer vs straight', {})
    pp_vs_str  = bench_results.get('pure pursuit vs straight', {})
    rl_vs_rl   = bench_results.get('RL pursuer vs RL evader', {})
    pp_vs_rl   = bench_results.get('pure pursuit vs RL evader', {})

    if rl_vs_str and pp_vs_str and pp_vs_str['mean_time'] > 0:
        delta = (pp_vs_str['mean_time'] - rl_vs_str['mean_time']) / pp_vs_str['mean_time'] * 100
        print(f'RL pursuer vs pure pursuit (straight evader): {delta:+.1f}% time change')
    if rl_vs_rl:
        print(f'RL pursuer vs RL evader capture rate: {rl_vs_rl["capture_rate"]*100:.1f}%')
        print(f'RL pursuer vs RL evader mean time   : {rl_vs_rl["mean_time"]:.2f} s')
    if pp_vs_rl and rl_vs_rl:
        delta2 = (pp_vs_rl['mean_time'] - rl_vs_rl['mean_time']) / (pp_vs_rl['mean_time'] + EPS_VEL) * 100
        print(f'RL pursuer vs pure pursuit (RL evader): {delta2:+.1f}% time change')
    print('=' * 70)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_learning_curves(learning_curves, out_dir)
    plot_quiver(quiver_data, out_dir)
    plot_performance_comparison(bench_results, out_dir)
    plot_capture_cdf(bench_results, out_dir)
    save_animation(rl_pursuer_fn, rl_evader_fn, out_dir)
