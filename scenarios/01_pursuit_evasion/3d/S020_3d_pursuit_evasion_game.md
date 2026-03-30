# S020 3D Upgrade — Pursuit-Evasion Game (RL Capstone)

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[x]` Complete
**Based on**: [S020 original](../S020_pursuit_evasion_game.md)

---

## What Changes in 3D

The original fixes $z = 0$ throughout and uses a **2D action space** $[\cos\theta, \sin\theta]$. In the full 3D upgrade, the state space grows to 13 dimensions (adding $p_z$ and $v_z$ for both agents), the action space becomes a **3D unit-sphere direction** $[a_x, a_y, a_z]$, and new reward terms shape altitude strategy. The 3D policy is significantly harder to train — an emergent altitude strategy (dive-and-intercept for the pursuer; vertical escape for the evader) must be discovered through the curriculum. Visualisation of the learned policy requires both a 2D quiver plot at fixed z and 3D streamline plots.

---

## Problem Definition

**Setup**: Capstone of the 3D pursuit-evasion domain. Both pursuer and evader are trained with PPO in full 3D space. The arena is $[-5, 5]^3$ m with altitude bounds enforced as soft constraints via reward shaping. A three-phase curriculum identical to the original is used, now in 3D.

**Training curriculum**:
1. **Phase 1** (5 000 episodes): pursuer trains against random 3D evader
2. **Phase 2** (5 000 episodes): evader trains against frozen Phase-1 pursuer
3. **Phase 3** (10 000 episodes): both train simultaneously (self-play)

---

## Mathematical Model

### 3D State Space

Each agent observes a 13-dimensional state:

$$\mathbf{s} = [\underbrace{p_x^P, p_y^P, p_z^P}_{\text{pursuer pos}},\; \underbrace{v_x^P, v_y^P, v_z^P}_{\text{pursuer vel}},\; \underbrace{p_x^E, p_y^E, p_z^E}_{\text{evader pos}},\; \underbrace{v_x^E, v_y^E, v_z^E}_{\text{evader vel}},\; \underbrace{d}_{\text{3D dist}}] \in \mathbb{R}^{13}$$

where $d = \|\mathbf{p}_P - \mathbf{p}_E\|_2$ is the current 3D Euclidean distance.

### 3D Action Space

Continuous 3D velocity direction on the unit sphere:

$$\mathbf{a} \in [-1, 1]^3, \quad \mathbf{v}_{cmd} = v_{max} \cdot \frac{\mathbf{a}}{\|\mathbf{a}\| + \varepsilon}$$

The action must now specify altitude change direction — the policy must learn to use the $a_z$ dimension.

### Reward Functions

**Pursuer reward**:

$$r_P = \begin{cases}
+100 & \text{on capture (}d < r_{cap}\text{)} \\
+0.01 \cdot (d_{prev} - d_{curr}) & \text{distance-closing shaping} \\
+0.02 \cdot \Delta z_{tactical} & \text{altitude advantage shaping} \\
-0.1 \cdot \|\mathbf{a}\|/v_{max} & \text{effort penalty} \\
-1.0 \cdot \mathbb{1}[z_P \notin [z_{min}, z_{max}]] & \text{altitude boundary penalty}
\end{cases}$$

where the altitude advantage term rewards the pursuer for gaining a z-advantage over the evader:

$$\Delta z_{tactical} = \max(0,\; |z_P - z_E| - 1.0) \cdot \text{sign}(z_P - z_E) \cdot \text{sign}(v_{z,P})$$

This rewards the pursuer for positioning itself above the evader (potential dive-boost advantage).

**Evader reward**:

$$r_E = \begin{cases}
-100 & \text{on capture} \\
+0.1 & \text{survival bonus per step} \\
+0.01 \cdot (d_{curr} - d_{prev}) & \text{distance-increasing shaping} \\
+0.05 \cdot |z_E - \bar{z}_{arena}| & \text{altitude-escape shaping (use full z range)} \\
-1.0 \cdot \mathbb{1}[z_E \notin [z_{min}, z_{max}]] & \text{altitude boundary penalty}
\end{cases}$$

The altitude-escape term rewards the evader for using extreme altitude positions (high or low) to complicate the pursuer's geometry.

### Episode Termination

$$\|\mathbf{p}_P - \mathbf{p}_E\|_2 < r_{capture} = 0.15 \text{ m} \quad \text{or} \quad t \geq T_{max} = 30 \text{ s}$$

### Altitude Strategy

The RL agent must discover the following altitude strategies from the reward function alone:
- **Pursuer**: learn to position above the evader and execute dive-boost intercepts ($\Delta z_{tactical}$ term)
- **Evader**: learn to exploit corners of the 3D arena, including ceiling and floor, to break LOS geometry

To bootstrap altitude learning, Phase 1 includes a curriculum stage where the evader is initialised at a random altitude (uniformly in $[z_{min}, z_{max}]$) — this forces the pursuer to learn 3D interception from the start.

---

## Key 3D Additions

- **Altitude strategy**: altitude advantage reward term for pursuer; altitude-escape term for evader; altitude boundary soft-constraint penalty; Phase 1 curriculum initialises evader at random z
- **3D guidance law**: action space is 3D unit sphere; policy network must learn $a_z$ component; 3D velocity integration with clip to arena bounds
- **Vertical evasion / geometry**: 3D arena $[-5,5]^3$; 3D distance as state observation; 3D quiver/streamline policy visualisation at multiple z slices

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer max speed | 5 m/s |
| Evader max speed | 3.5 m/s |
| Arena | [−5, 5]³ m |
| z range | 0.5 – 5.5 m |
| Episode length | 30 s max |
| Observation noise sigma | 0.1 m |
| PPO clip range | 0.2 |
| Learning rate | 3e-4 |
| Batch size | 256 |
| Network architecture | 2-layer MLP, 256 units each |
| Capture radius | 0.15 m |
| Altitude advantage weight | 0.02 |
| Altitude escape weight | 0.05 |
| Phase 1 episodes | 5 000 |
| Phase 2 episodes | 5 000 |
| Phase 3 episodes | 10 000 |

---

## Implementation

```python
# Requires: pip install stable-baselines3 gymnasium numpy

import numpy as np
import gymnasium as gym
from stable_baselines3 import PPO

V_PURSUER = 5.0
V_EVADER  = 3.5
R_CAPTURE = 0.15
T_MAX     = 30.0
DT        = 0.05
Z_MIN, Z_MAX = 0.5, 5.5
ARENA_BOUND  = 5.0

class PursuitEvasion3DEnv(gym.Env):
    """Full 3D pursuit-evasion RL environment."""

    def __init__(self, evader_policy="random", pursuer_policy=None):
        super().__init__()
        # 13-dim state: [px,py,pz,vx,vy,vz,ex,ey,ez,evx,evy,evz, dist]
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(13,), dtype=np.float32)
        # 3D action: direction on unit sphere
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        self.evader_policy = evader_policy
        self.pursuer_policy = pursuer_policy

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.t = 0.0
        # Random 3D initialisation
        self.pos_p = np.random.uniform(-4, 4, 3)
        self.pos_p[2] = np.random.uniform(Z_MIN + 0.5, Z_MAX - 0.5)
        self.pos_e = np.random.uniform(-4, 4, 3)
        self.pos_e[2] = np.random.uniform(Z_MIN + 0.5, Z_MAX - 0.5)
        self.vel_p = np.zeros(3)
        self.vel_e = np.zeros(3)
        self.prev_dist = np.linalg.norm(self.pos_p - self.pos_e)
        return self._obs(), {}

    def _obs(self):
        d = np.linalg.norm(self.pos_p - self.pos_e)
        obs = np.concatenate([
            self.pos_p, self.vel_p,
            self.pos_e, self.vel_e,
            [d]
        ]).astype(np.float32)
        obs += np.random.normal(0, 0.1, 13).astype(np.float32)
        return obs

    def _evader_action(self):
        if self.evader_policy == "random":
            a = np.random.randn(3)
        elif self.evader_policy == "straight_escape":
            a = self.pos_e - self.pos_p
            a[2] += 0.5 * np.sign(self.pos_e[2] - self.pos_p[2])
        elif self.evader_policy == "ppo" and self.pursuer_policy is not None:
            obs_e = np.concatenate([
                self.pos_e, self.vel_e,
                self.pos_p, self.vel_p,
                [np.linalg.norm(self.pos_p - self.pos_e)]
            ]).astype(np.float32)
            a, _ = self.evader_policy.predict(obs_e, deterministic=True)
        else:
            a = self.pos_e - self.pos_p
        return a / (np.linalg.norm(a) + 1e-8)

    def step(self, action_pursuer):
        # Pursuer moves
        v_p = V_PURSUER * action_pursuer / (np.linalg.norm(action_pursuer) + 1e-8)
        self.pos_p = self.pos_p + v_p * DT
        self.pos_p = np.clip(self.pos_p, -ARENA_BOUND, ARENA_BOUND)
        self.pos_p[2] = np.clip(self.pos_p[2], Z_MIN, Z_MAX)
        self.vel_p = v_p

        # Evader moves
        a_e = self._evader_action()
        v_e = V_EVADER * a_e
        self.pos_e = self.pos_e + v_e * DT
        self.pos_e = np.clip(self.pos_e, -ARENA_BOUND, ARENA_BOUND)
        self.pos_e[2] = np.clip(self.pos_e[2], Z_MIN, Z_MAX)
        self.vel_e = v_e

        self.t += DT
        curr_dist = np.linalg.norm(self.pos_p - self.pos_e)
        captured = curr_dist < R_CAPTURE
        timeout  = self.t >= T_MAX

        # Pursuer reward
        reward = 0.01 * (self.prev_dist - curr_dist)
        reward -= 0.1 * np.linalg.norm(action_pursuer) / V_PURSUER

        # Altitude advantage reward
        dz = abs(self.pos_p[2] - self.pos_e[2])
        if dz > 1.0:
            adv = (dz - 1.0) * np.sign(self.pos_p[2] - self.pos_e[2])
            reward += 0.02 * adv * np.sign(v_p[2])

        # Altitude boundary penalty (soft constraint)
        if self.pos_p[2] <= Z_MIN + 0.1 or self.pos_p[2] >= Z_MAX - 0.1:
            reward -= 1.0

        if captured:
            reward += 100.0
        self.prev_dist = curr_dist

        done = captured or timeout
        return self._obs(), float(reward), done, False, {}

class EvaderEnv(PursuitEvasion3DEnv):
    """Swaps roles: action is evader, pursuer uses frozen policy."""

    def __init__(self, pursuer_model):
        super().__init__()
        self.frozen_pursuer = pursuer_model

    def step(self, action_evader):
        # Pursuer uses frozen model
        obs_p = np.concatenate([
            self.pos_p, self.vel_p,
            self.pos_e, self.vel_e,
            [np.linalg.norm(self.pos_p - self.pos_e)]
        ]).astype(np.float32)
        a_p, _ = self.frozen_pursuer.predict(obs_p, deterministic=True)
        v_p = V_PURSUER * a_p / (np.linalg.norm(a_p) + 1e-8)
        self.pos_p = np.clip(self.pos_p + v_p * DT, -ARENA_BOUND, ARENA_BOUND)
        self.pos_p[2] = np.clip(self.pos_p[2], Z_MIN, Z_MAX)
        self.vel_p = v_p

        # Evader action
        v_e = V_EVADER * action_evader / (np.linalg.norm(action_evader) + 1e-8)
        self.pos_e = np.clip(self.pos_e + v_e * DT, -ARENA_BOUND, ARENA_BOUND)
        self.pos_e[2] = np.clip(self.pos_e[2], Z_MIN, Z_MAX)
        self.vel_e = v_e
        self.t += DT

        curr_dist = np.linalg.norm(self.pos_p - self.pos_e)
        captured = curr_dist < R_CAPTURE
        timeout  = self.t >= T_MAX
        reward_e = 0.01 * (curr_dist - self.prev_dist) + 0.1
        reward_e += 0.05 * abs(self.pos_e[2] - (Z_MIN + Z_MAX) / 2)
        if self.pos_e[2] <= Z_MIN + 0.1 or self.pos_e[2] >= Z_MAX - 0.1:
            reward_e -= 1.0
        if captured:
            reward_e -= 100.0
        self.prev_dist = curr_dist
        return self._obs(), float(reward_e), captured or timeout, False, {}

# --- Training ---
PPO_KWARGS = dict(
    policy="MlpPolicy",
    verbose=1,
    learning_rate=3e-4,
    clip_range=0.2,
    batch_size=256,
    policy_kwargs=dict(net_arch=[256, 256]),
)
MAX_STEPS = int(T_MAX / DT)

# Phase 1: train pursuer vs random evader
env_p1 = PursuitEvasion3DEnv(evader_policy="random")
model_pursuer = PPO(env=env_p1, **PPO_KWARGS)
model_pursuer.learn(total_timesteps=5_000 * MAX_STEPS)

# Phase 2: train evader vs frozen pursuer
env_p2 = EvaderEnv(pursuer_model=model_pursuer)
model_evader = PPO(env=env_p2, **PPO_KWARGS)
model_evader.learn(total_timesteps=5_000 * MAX_STEPS)

# Phase 3: self-play (alternating frozen updates)
env_p3 = PursuitEvasion3DEnv(evader_policy="ppo")
env_p3.evader_policy = model_evader
model_pursuer.set_env(env_p3)
model_pursuer.learn(total_timesteps=10_000 * MAX_STEPS)
```

---

## Expected Output

- Learning curves: episode reward vs training step for all 3 phases
- 3D policy quiver plots at z = 1.0, z = 2.5, z = 4.5 m slices showing horizontal action direction
- 3D streamline plot of pursuer policy showing dive-intercept behaviour (lines converging from above)
- Performance comparison table: RL pursuer vs pure pursuit, PNG, APF; RL evader vs straight, perpendicular, spiral, altitude-escape
- Altitude usage histogram: $z_P(t)$ and $z_E(t)$ distribution over 1 000 test episodes — does the RL policy avoid flat flight?
- Capture time CDF across 1 000 test episodes (3D RL vs 2D baselines at z = 2.5 m fixed)

---

## Extensions

1. MAPPO for 3v3 swarm — both teams trained end-to-end in 3D with altitude layer coordination
2. Add 3D obstacle field (from S004 3D) to observation and action space
3. Transfer learning from 3D simulation to real hardware: fine-tune with domain randomisation over $v_{z,max}$
4. Curriculum altitude difficulty: start with z-variation σ = 0 (flat), gradually increase to full ±5 m

---

## Related Scenarios

- Original 2D version: [S020](../S020_pursuit_evasion_game.md)
- Truly 3D references: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
