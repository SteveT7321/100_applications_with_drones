# S019 3D Upgrade — Dynamic Target Reassignment

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[x]` Complete
**Based on**: [S019 original](../S019_dynamic_reassignment.md)

---

## What Changes in 3D

The original has `new_pos[2] = 2.0` hardcoded every time a target appears or disappears — all evader positions are forced to the same altitude. In 3D, targets appear at **random altitudes** sampled uniformly from $[z_{min}, z_{max}]$. The Hungarian cost matrix uses full 3D Euclidean distance, and pursuers must track an **altitude memory** to efficiently handle altitude-intensive reassignments. The reassignment switching cost is extended to penalise large altitude jumps (which are slow to execute), preventing thrashing when two targets at very different altitudes are equidistant horizontally.

---

## Problem Definition

**Setup**: 3 pursuers vs 3 evaders (mixed strategies: straight, spiral, random). Every $T_{check}$ seconds, re-optimise the assignment using the 3D Hungarian algorithm. Evaders appear and disappear at random 3D positions throughout the simulation. Switching cost is extended to include altitude delta to prevent altitude-thrashing reassignments.

**Comparison**:
1. **Static 3D** — Hungarian at $t = 0$, never updated
2. **Dynamic 3D** — re-optimise every $T_{check}$ seconds with 3D cost and altitude-aware switching cost
3. **Greedy 3D** — each pursuer always chases its 3D-nearest evader

---

## Mathematical Model

### 3D Assignment Cost Matrix

$$C[i, j] = \frac{\|\mathbf{p}_{P_i} - \mathbf{p}_{E_j}\|_2}{v_P}$$

where $\|\cdot\|_2$ is the full 3D Euclidean norm.

### 3D Hungarian Assignment

$$\sigma^* = \arg\min_{\sigma \in S_M} \sum_{i=1}^{M} C[i, \sigma(i)]$$

### 3D Reassignment Condition with Altitude Penalty

Define the altitude-aware switching cost for reassignment of pursuer $i$ from current target $j_{old}$ to new target $j_{new}$:

$$\Delta C_{switch}^{3D}(i, j_{old} \to j_{new}) = \Delta C_{base} + w_z \cdot \frac{|z_{E_{j_{new}}} - z_{P_i}|}{v_{z,max}}$$

where $w_z = 0.5$ and $v_{z,max} = 2.5$ m/s is the vertical speed limit. This makes it expensive to reassign a pursuer to a target at very different altitude.

Reassign if:

$$\text{improvement} = \sum_i C[i, \sigma_{current}(i)] - \sum_i C[i, \sigma^*_{new}(i)] > \Delta C_{switch}^{3D}$$

### Pursuer Altitude Memory

Each pursuer maintains an exponential moving average of its assigned target's altitude:

$$\bar{z}_{target,i}(t + \Delta t) = (1 - \alpha_{mem}) \bar{z}_{target,i}(t) + \alpha_{mem} z_{E_{\sigma(i)}}(t)$$

where $\alpha_{mem} = 0.1$ is the memory decay. When a reassignment is triggered, the pursuer pre-adjusts its altitude toward $\bar{z}_{target, j_{new}}$ before the full pursuit begins — reducing the time wasted on altitude adjustment after the switch.

### Altitude Strategy

When the pursuer's current assignment is $j$ and a new target $j'$ appears at altitude $z_{j'}$, the pursuer begins adjusting altitude immediately if:

$$|z_{j'} - z_{P_i}| > \Delta z_{pre} = 1.0 \text{ m}$$

Pre-altitude adjustment velocity:

$$\dot{z}_{P_i}^{pre} = v_{z,max} \cdot \text{sign}(z_{j'} - z_{P_i})$$

This runs in parallel with horizontal pursuit of the current target.

### Target Appearance Model (3D)

New targets appear at random 3D positions:

$$\mathbf{p}_{E_{new}} \sim \mathcal{U}([-5,5] \times [-5,5] \times [z_{min}, z_{max}])$$

where $z_{min} = 0.8$ m, $z_{max} = 5.0$ m. Target altitude is drawn uniformly and independently of horizontal position.

### Evader Strategies (3D)

- **Straight escape** (3D): fly directly away from assigned pursuer in 3D
- **Spiral** (3D): helical spiral — horizontal circle plus sinusoidal altitude:
  $$\mathbf{v}_{spiral}(t) = v_E \begin{bmatrix}\cos(\omega t + \phi_0) \\ \sin(\omega t + \phi_0) \\ A_z \sin(2\omega t)\end{bmatrix}$$
- **Random walk** (3D): $\mathbf{v}_{rand}(t) = v_E \cdot \hat{\boldsymbol{\xi}}(t)$ where $\hat{\boldsymbol{\xi}}$ is a unit vector updated with slow 3D random rotation

---

## Key 3D Additions

- **Altitude strategy**: pursuer altitude memory EMA; pre-altitude adjustment when new target detected at different z; altitude-aware switching cost
- **3D guidance law**: full 3D Euclidean cost matrix; switching cost penalises large $|\Delta z|$; altitude pre-adjustment runs in parallel with horizontal pursuit
- **Vertical evasion / geometry**: targets appear at random altitudes; spiral evader uses helical trajectory; random evader uses 3D random walk

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuers | 3, at (−5, −2, 1.0), (−5, 0, 2.5), (−5, 2, 4.0) m |
| Evaders | 3, at (2, −2, 1.5), (2, 0, 3.0), (2, 2, 4.5) m |
| Pursuer horizontal speed | 5 m/s |
| Pursuer vertical speed | 2.5 m/s |
| Evader speed | 3.5 m/s |
| Evader strategies | straight 3D, helical spiral, 3D random walk |
| Re-check period T_check | 1.0 s |
| Base switching cost ΔC_base | 0.5 s |
| Altitude penalty weight w_z | 0.5 |
| Altitude memory decay α_mem | 0.1 |
| Pre-altitude threshold Δz_pre | 1.0 m |
| Target z range | 0.8 – 5.0 m |
| dt | 0.02 s |

---

## Implementation

```python
import numpy as np
from numpy.linalg import norm
from scipy.optimize import linear_sum_assignment

V_XY     = 5.0
V_Z_MAX  = 2.5
V_EVADER = 3.5
CAPTURE_R = 0.15
DT = 0.02
T_MAX = 40.0
T_CHECK = 1.0
DC_BASE = 0.5
W_Z = 0.5
ALPHA_MEM = 0.1
DZ_PRE = 1.0
Z_MIN, Z_MAX = 0.8, 5.0

def cost_3d(pos_p, pos_e, vxy=V_XY):
    return norm(pos_e - pos_p) / vxy

def cost_matrix_3d(pursuers, evaders):
    M, N = len(pursuers), len(evaders)
    return np.array([[cost_3d(pursuers[i], evaders[j])
                      for j in range(N)] for i in range(M)])

def switching_cost_3d(pos_pi, pos_e_new, w_z=W_Z, vz=V_Z_MAX, dc_base=DC_BASE):
    dz = abs(pos_pi[2] - pos_e_new[2])
    return dc_base + w_z * dz / vz

def pursuer_velocity_3d(pos_p, pos_e, vxy, vz_max, dt):
    """Decoupled XY + Z velocity toward target."""
    dxy_vec = pos_e[:2] - pos_p[:2]
    dxy = norm(dxy_vec)
    dz  = pos_e[2] - pos_p[2]
    vel = np.zeros(3)
    if dxy > 1e-8:
        vel[:2] = vxy * dxy_vec / dxy
    vel[2] = np.clip(dz / dt, -vz_max, vz_max)
    # Normalise total speed to not exceed combined limit
    total = norm(vel)
    if total > np.sqrt(vxy**2 + vz_max**2):
        vel = vel / total * np.sqrt(vxy**2 + vz_max**2)
    return vel

class Evader3D:
    def __init__(self, pos, strategy, phi0=0.0):
        self.pos = pos.copy()
        self.strategy = strategy
        self.phi0 = phi0
        self.t = 0.0
        self.vel_dir = np.random.randn(3)
        self.vel_dir /= norm(self.vel_dir) + 1e-8

    def step(self, pos_pursuer, dt):
        if self.strategy == "straight":
            away = self.pos - pos_pursuer
            if norm(away) > 1e-8:
                self.vel_dir = away / norm(away)
        elif self.strategy == "spiral":
            omega = 0.5
            Az = 0.3
            self.vel_dir = np.array([
                np.cos(omega * self.t + self.phi0),
                np.sin(omega * self.t + self.phi0),
                Az * np.sin(2 * omega * self.t)
            ])
            self.vel_dir /= norm(self.vel_dir) + 1e-8
        elif self.strategy == "random":
            # Slowly rotate the direction
            perturb = np.random.randn(3) * 0.3
            self.vel_dir = self.vel_dir + perturb
            self.vel_dir /= norm(self.vel_dir) + 1e-8
        self.pos += V_EVADER * self.vel_dir * dt
        self.pos[2] = np.clip(self.pos[2], Z_MIN, Z_MAX)
        self.t += dt
        return self.pos.copy()

def simulate_dynamic_3d(method="dynamic"):
    pursuers = np.array([
        [-5., -2., 1.0],
        [-5.,  0., 2.5],
        [-5.,  2., 4.0],
    ], dtype=float)
    evaders_obj = [
        Evader3D(np.array([2., -2., 1.5]), "straight"),
        Evader3D(np.array([2.,  0., 3.0]), "spiral",  phi0=0.5),
        Evader3D(np.array([2.,  2., 4.5]), "random"),
    ]
    M = len(pursuers)
    N = len(evaders_obj)
    evader_pos = np.array([e.pos for e in evaders_obj])
    altitude_memory = evader_pos[:, 2].copy()  # per-pursuer altitude memory

    C = cost_matrix_3d(pursuers, evader_pos)
    rows, cols = linear_sum_assignment(C)
    assignment = dict(zip(rows.tolist(), cols.tolist()))

    captured = [False] * N
    capture_times = [None] * N
    reassign_events = []
    last_check = 0.0

    for step in range(int(T_MAX / DT)):
        t = step * DT
        evader_pos = np.array([e.pos for e in evaders_obj])

        # Update altitude memory
        for i in range(M):
            j = assignment.get(i, 0)
            if not captured[j]:
                altitude_memory[i] = ((1 - ALPHA_MEM) * altitude_memory[i]
                                      + ALPHA_MEM * evader_pos[j, 2])

        # Dynamic reassignment check
        if method == "dynamic" and t - last_check >= T_CHECK:
            C = cost_matrix_3d(pursuers, evader_pos)
            rows, cols = linear_sum_assignment(C)
            new_assign = dict(zip(rows.tolist(), cols.tolist()))

            current_cost = sum(C[i, assignment.get(i, 0)]
                               for i in range(M) if not captured[assignment.get(i, 0)])
            new_cost     = sum(C[i, new_assign.get(i, 0)]
                               for i in range(M) if not captured[new_assign.get(i, 0)])
            improvement  = current_cost - new_cost

            # Altitude-aware switching cost
            min_sw_cost = min(
                switching_cost_3d(pursuers[i], evader_pos[new_assign[i]])
                for i in range(M)
                if new_assign.get(i) != assignment.get(i)
            ) if new_assign != assignment else np.inf

            if improvement > min_sw_cost:
                reassign_events.append(t)
                assignment = new_assign
            last_check = t

        # Move pursuers
        for i in range(M):
            if method == "greedy":
                remaining = [j for j in range(N) if not captured[j]]
                if not remaining:
                    break
                j_target = min(remaining,
                               key=lambda j: norm(pursuers[i] - evader_pos[j]))
                assignment[i] = j_target
            j = assignment.get(i, 0)
            if captured[j]:
                remaining = [jj for jj in range(N) if not captured[jj]]
                if not remaining:
                    continue
                assignment[i] = min(remaining,
                                    key=lambda jj: norm(pursuers[i] - evader_pos[jj]))
                j = assignment[i]

            vel = pursuer_velocity_3d(pursuers[i], evader_pos[j], V_XY, V_Z_MAX, DT)
            pursuers[i] += vel * DT
            pursuers[i, 2] = np.clip(pursuers[i, 2], Z_MIN, Z_MAX)

            if norm(pursuers[i] - evader_pos[j]) < CAPTURE_R:
                captured[j] = True
                capture_times[j] = t

        # Move evaders
        for j, ev in enumerate(evaders_obj):
            if not captured[j]:
                ev.step(pursuers[assignment.get(
                    next((i for i in range(M) if assignment.get(i) == j), 0), 0)], DT)

        if all(captured):
            break

    total = max(t for t in capture_times if t is not None)
    return capture_times, total, reassign_events
```

---

## Expected Output

- 3D trajectories colour-coded by assignment (dashed lines for reassignment moments, z clearly visible)
- Altitude time series: all pursuers and evaders — shows altitude memory pre-adjustment behaviour
- Assignment matrix heatmap over time (rows = pursuers, columns = evaders)
- Total mission time: static 3D vs dynamic 3D vs greedy 3D comparison
- Number of reassignment events vs switching cost threshold sweep

---

## Extensions

1. Auction algorithm (3D): distributed, scalable alternative to centralised 3D Hungarian
2. Altitude-tiered reassignment: pursuers never cross altitude tiers unless no other option
3. RL policy for 3D reassignment decisions — replace rule-based switching cost with learned value

---

## Related Scenarios

- Original 2D version: [S019](../S019_dynamic_reassignment.md)
- Truly 3D references: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
