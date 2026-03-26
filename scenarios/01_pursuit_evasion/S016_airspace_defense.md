# S016 Airspace Defense

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: A protected zone (sphere of radius 2 m centered at the origin) must be defended against intruders. An intruder approaches from outside and flies straight toward the zone center. A defender starts near the zone center and must intercept the intruder before it crosses the zone boundary.

**Roles**:
- **Defender** (pursuer): intercept the intruder before zone penetration; uses Pure Pursuit or Proportional Navigation
- **Intruder** (evader): fly straight toward the zone center at constant speed from one of four initial positions

**Objective**: determine the "capture guarantee region" — the set of intruder start positions from which the defender can always prevent penetration; compare Pure Pursuit vs Proportional Navigation effectiveness.

---

## Mathematical Model

### Protected Zone

$$\mathcal{Z} = \{ \mathbf{p} \in \mathbb{R}^3 \mid \|\mathbf{p}\| \leq R_{zone} \}$$

### Apollonius Intercept Condition

The defender can intercept the intruder before it reaches the zone surface if and only if:

$$\frac{\|\mathbf{p}_D - \mathbf{p}_I\|}{v_D} \leq \frac{\|\mathbf{p}_I\| - R_{zone}}{v_I}$$

The boundary of the guarantee region (Apollonius surface) satisfies:

$$\|\mathbf{p}_D - \mathbf{p}_I\| = \frac{v_D}{v_I} \bigl(\|\mathbf{p}_I\| - R_{zone}\bigr)$$

### Pure Pursuit Strategy

$$\mathbf{v}_D = v_D \cdot \frac{\mathbf{p}_I - \mathbf{p}_D}{\|\mathbf{p}_I - \mathbf{p}_D\|}$$

### Proportional Navigation (PN)

Define the line-of-sight (LOS) unit vector and its time derivative:

$$\hat{\boldsymbol{\lambda}} = \frac{\mathbf{p}_I - \mathbf{p}_D}{\|\mathbf{p}_I - \mathbf{p}_D\|}$$

$$\dot{\hat{\boldsymbol{\lambda}}} = \frac{d\hat{\boldsymbol{\lambda}}}{dt}$$

The closing speed is:

$$v_c = -\frac{d}{dt}\|\mathbf{p}_I - \mathbf{p}_D\|$$

The PN acceleration command is:

$$\mathbf{a}_{cmd} = N \cdot v_c \cdot \dot{\hat{\boldsymbol{\lambda}}}$$

where N = 3 is the navigation gain. The commanded acceleration is integrated to produce the velocity command, then clipped to v_D.

### Capture and Penetration Conditions

Capture (defender wins):

$$\|\mathbf{p}_D - \mathbf{p}_I\| < r_{capture} = 0.15 \text{ m}$$

Zone penetration (intruder wins):

$$\|\mathbf{p}_I\| \leq R_{zone}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Parameters ---
R_ZONE = 2.0
DEFENDER_START = np.array([0.0, 0.0, 2.0])
V_DEFENDER = 6.0   # m/s
V_INTRUDER = 4.0   # m/s
PN_GAIN = 3.0
CAPTURE_R = 0.15   # m
DT = 0.02          # s
T_MAX = 10.0

INTRUDER_STARTS = [
    np.array([ 8.0,  0.0, 2.0]),
    np.array([-8.0,  0.0, 2.0]),
    np.array([ 0.0,  8.0, 2.0]),
    np.array([ 6.0,  6.0, 2.0]),
]

def pure_pursuit_step(p_def, p_int, speed, dt):
    d = p_int - p_def
    d /= np.linalg.norm(d) + 1e-8
    return p_def + d * speed * dt

def pn_step(p_def, p_int, v_def, v_int, N, speed, dt):
    los = p_int - p_def
    los_dist = np.linalg.norm(los) + 1e-8
    los_hat = los / los_dist
    v_rel = v_int - v_def
    los_dot = (v_rel - np.dot(v_rel, los_hat) * los_hat) / los_dist
    v_c = -np.dot(v_rel, los_hat)
    a_cmd = N * v_c * los_dot
    v_new = v_def + a_cmd * dt
    v_new = v_new / (np.linalg.norm(v_new) + 1e-8) * speed
    return p_def + v_new * dt, v_new

def simulate(p_int_0, strategy="pure_pursuit"):
    p_def = DEFENDER_START.copy()
    p_int = p_int_0.copy()
    v_def = np.zeros(3)
    v_int = -p_int_0 / np.linalg.norm(p_int_0) * V_INTRUDER
    traj_d, traj_i = [p_def.copy()], [p_int.copy()]

    for _ in range(int(T_MAX / DT)):
        p_int = p_int + v_int * DT
        if np.linalg.norm(p_int) <= R_ZONE:
            return traj_d, traj_i, "intruder_wins"
        if strategy == "pure_pursuit":
            p_def = pure_pursuit_step(p_def, p_int, V_DEFENDER, DT)
        else:
            p_def, v_def = pn_step(p_def, p_int, v_def, v_int, PN_GAIN, V_DEFENDER, DT)
        traj_d.append(p_def.copy())
        traj_i.append(p_int.copy())
        if np.linalg.norm(p_def - p_int) < CAPTURE_R:
            return traj_d, traj_i, "defender_wins"

    return traj_d, traj_i, "timeout"

# Run all cases
results = {}
for p0 in INTRUDER_STARTS:
    for strat in ["pure_pursuit", "proportional_nav"]:
        traj_d, traj_i, outcome = simulate(p0, strat)
        results[(tuple(p0), strat)] = outcome
        print(f"Start {p0} | {strat}: {outcome}")
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Protected zone radius | 2 m |
| Defender start position | (0, 0, 2) m |
| Defender speed | 6 m/s |
| Intruder speed | 4 m/s |
| Speed ratio | 1.5 |
| Intruder start positions | (8,0,2), (-8,0,2), (0,8,2), (6,6,2) m |
| Proportional nav gain N | 3 |
| Capture radius | 0.15 m |
| Simulation timestep | 0.02 s |

---

## Expected Output

- **3D plot**: protected zone sphere, 4 intruder approach trajectories, 4 defender intercept paths (color-coded by strategy)
- **Success/fail table**: outcome for each initial position under Pure Pursuit and PN
- **Apollonius sphere visualization**: analytical guarantee region boundary overlaid on the 3D scene
- **Miss distance vs intercept geometry**: scatter plot of miss distance vs intruder approach angle

---

## Extensions

1. Multiple simultaneous intruders (salvo attack) — defender must prioritize targets
2. Intruder executes an evasive maneuver during the final approach phase
3. Two cooperating defenders — analyze how cooperation expands the guarantee region

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md), [S002](S002_evasive_maneuver.md), [S005](S005_stealth_approach.md)
- Next: [S017](S017_swarm_vs_swarm.md), [S018](S018_multi_target_intercept.md)
- See [domains/01_pursuit_evasion.md](../../domains/01_pursuit_evasion.md)
