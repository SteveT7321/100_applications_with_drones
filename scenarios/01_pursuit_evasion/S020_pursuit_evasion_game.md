# S020 Pursuit-Evasion Game (RL Capstone)

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: Capstone of the pursuit-evasion domain. Both pursuer and evader are trained with Proximal Policy Optimisation (PPO). The trained RL policies are benchmarked against all hand-crafted strategies from S001–S019.

**Hypothesis**:
- RL pursuer will discover strategies beyond pure pursuit or APF
- RL evader will discover strategies beyond straight or perpendicular escape
- Joint training leads to emergent co-evolutionary dynamics

**Training curriculum**:
1. **Phase 1** (5 000 episodes): pursuer trains against random evader
2. **Phase 2** (5 000 episodes): evader trains against frozen Phase-1 pursuer
3. **Phase 3** (10 000 episodes): both train simultaneously (self-play)

---

## Mathematical Model

### State Space

Each agent observes:

$$\mathbf{s} = [\mathbf{p}_P,\; \mathbf{v}_P,\; \mathbf{p}_E,\; \mathbf{v}_E,\; t_{remaining}] \in \mathbb{R}^{13}$$

### Action Space

Continuous velocity direction command, clipped to $v_{max}$:

$$\mathbf{a} \in [-1, 1]^3, \quad \mathbf{v}_{cmd} = v_{max} \cdot \frac{\mathbf{a}}{\|\mathbf{a}\| + \varepsilon}$$

### Reward Functions

**Pursuer:**

$$r_P = \begin{cases} +100 & \text{on capture} \\ +0.01 \cdot (d_{prev} - d_{curr}) & \text{shaping (closing reward)} \\ -0.1 \cdot \|\mathbf{a}\|/v_{max} & \text{effort penalty per step} \end{cases}$$

**Evader:**

$$r_E = \begin{cases} -100 & \text{on capture} \\ +0.1 & \text{survival bonus per step} \\ +0.01 \cdot (d_{curr} - d_{prev}) & \text{shaping (distance reward)} \end{cases}$$

### Episode Termination

$$\|\mathbf{p}_P - \mathbf{p}_E\| < r_{capture} = 0.15 \text{ m} \quad \text{or} \quad t \geq T_{max} = 30 \text{ s}$$

---

## Implementation

```python
# Requires: pip install stable-baselines3 gymnasium

import gymnasium as gym
from stable_baselines3 import PPO

class PursuitEvasionEnv(gym.Env):
    def __init__(self):
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(13,))
        self.action_space      = gym.spaces.Box(-1, 1, shape=(3,))

    def step(self, action_pursuer):
        v_cmd = V_PURSUER * action_pursuer / (np.linalg.norm(action_pursuer) + 1e-8)
        self.pursuer.step(v_cmd)
        # Evader uses frozen policy or hand-crafted strategy
        v_evader = self.evader_policy(self.evader.pos, self.pursuer.pos)
        self.evader.step(v_evader)
        # Compute reward, done, obs ...

# Phase 1: train pursuer
model_pursuer = PPO("MlpPolicy", env, verbose=1,
                    learning_rate=3e-4, clip_range=0.2, batch_size=256,
                    policy_kwargs=dict(net_arch=[256, 256]))
model_pursuer.learn(total_timesteps=5_000 * MAX_STEPS)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer max speed | 5 m/s |
| Evader max speed | 3.5 m/s |
| Arena | [-5, 5] cubed m |
| Episode length | 30 s max |
| Observation noise sigma | 0.1 m |
| PPO clip range | 0.2 |
| Learning rate | 3e-4 |
| Batch size | 256 |
| Network architecture | 2-layer MLP, 256 units each |
| Phase 1 episodes | 5 000 |
| Phase 2 episodes | 5 000 |
| Phase 3 episodes | 10 000 |

---

## Expected Output

- Learning curve: episode reward vs training step (all 3 phases)
- Policy quiver plot: RL pursuer action direction across the XY plane
- Performance comparison table: RL pursuer vs pure pursuit, PNG, APF — and RL evader vs straight, perpendicular, spiral
- Capture time CDF across 1 000 test episodes (RL vs baselines)
- Emergent behaviour analysis: does the RL pursuer rediscover proportional navigation?

---

## Extensions

1. Multi-agent RL: 3v3 swarm trained end-to-end (MAPPO)
2. Add obstacle field (combine with S004) to observation space
3. Transfer learning: fine-tune simulation policy on real drone hardware
4. MARL with communication: pursuers share observations via message passing

---

## Related Scenarios

- Prerequisites: all S001–S019 (capstone of the pursuit-evasion domain)
- See [domains/01_pursuit_evasion.md](../../domains/01_pursuit_evasion.md)

## References

- Schulman, J. et al. (2017). "Proximal Policy Optimization Algorithms." arXiv:1707.06347.
- Isaacs, R. (1965). *Differential Games*. Wiley.
- Lowe, R. et al. (2017). "Multi-Agent Actor-Critic for Mixed Cooperative-Competitive Environments." *NeurIPS*.
