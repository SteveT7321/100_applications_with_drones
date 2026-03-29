# S012 3D Upgrade — Relay Pursuit

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S012 original](../S012_relay_pursuit.md)

---

## What Changes in 3D

The original places all three relay pursuers at $z = 2.0$ m fixed. Standby drones fly toward the predicted evader position in a flat plane. In 3D, relay drones are pre-positioned at **different altitude tiers** (z = 1 m, 2 m, 4 m), and the handoff condition is evaluated in full 3D space. Standby drones intercept the evader along altitude-optimal trajectories, and the evader can exploit handoff moments by climbing or diving to an altitude tier that is far from the next active pursuer.

---

## Problem Definition

**Setup**: Three relay pursuers each operate from a different altitude tier. The active pursuer chases in 3D. When its battery drops below the handoff threshold, the standby drone with the best combination of remaining energy and 3D proximity to the predicted evader position takes over. The standby drones fly intercept paths that account for vertical distance to the expected handoff point.

**Objective**: compare flat-plane relay (all at z = 2 m) vs altitude-tiered relay; measure how altitude-stratified standby improves handoff continuity.

---

## Mathematical Model

### Battery Model (unchanged from 2D)

$$E_i(t + \Delta t) = E_i(t) - k \cdot \|\mathbf{v}_i\|^2 \cdot \Delta t$$

Note: climbing costs extra energy because motor thrust must overcome gravity:

$$k_{climb} = k \cdot \left(1 + \frac{g}{\|\mathbf{v}_i\|^2 / \Delta z + \varepsilon}\right) \quad \text{when } \dot{z} > 0$$

### Handoff Trigger

Active pursuer $i$ hands off when:

$$E_i < E_{threshold} = 0.2 \cdot E_0$$

### 3D Intercept Position for Standby Drones

Standby pursuer $k$ estimates the evader's position at handoff time $T_h$:

$$\hat{\mathbf{p}}_{E}(T_h) = \mathbf{p}_E(t) + \mathbf{v}_E(t) \cdot T_h$$

The standby drone's required velocity to arrive at that intercept point in time $T_h$:

$$\mathbf{v}_{intercept,k} = \frac{\hat{\mathbf{p}}_{E}(T_h) - \mathbf{p}_k}{\|\hat{\mathbf{p}}_{E}(T_h) - \mathbf{p}_k\|} \cdot v_{standby}$$

with 3D distance:

$$d_{3D,k} = \|\hat{\mathbf{p}}_{E}(T_h) - \mathbf{p}_k\|_2$$

### Handoff Selection Criterion

The next active drone maximises a composite score balancing energy and 3D proximity:

$$j^* = \arg\max_{k \neq \text{active}} \left[ w_E \cdot \frac{E_k}{E_0} - w_d \cdot \frac{d_{3D,k}}{d_{max}} \right]$$

where $w_E = 0.6$, $w_d = 0.4$ are tunable weights and $d_{max}$ normalises distance.

### Altitude Strategy

Each relay tier maintains a home altitude $z_k^{home} \in \{1.0, 2.5, 4.5\}$ m. When a standby drone is not in intercept mode, it holds its tier altitude:

$$\dot{z}_k^{standby} = k_{hold} \left(z_k^{home} - z_k\right)$$

where $k_{hold} = 2.0$ s$^{-1}$ is the altitude hold gain. This ensures vertical spread is maintained for maximum coverage.

### Estimated Time to Handoff

$$T_h = \frac{E_{active} - E_{threshold}}{k \cdot v_{active}^2}$$

---

## Key 3D Additions

- **Altitude strategy**: three relay drones at tiers z = 1.0, 2.5, 4.5 m; altitude-hold law keeps standby drones at their tier
- **3D guidance law**: intercept path computed in full 3D Euclidean space; climbing energy cost added to battery model
- **Vertical evasion / geometry**: evader can dive below tier 1 or climb above tier 3 during handoff gap; handoff score accounts for 3D distance

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of pursuers | 3 |
| Altitude tiers (home z) | 1.0, 2.5, 4.5 m |
| Battery capacity each | 60 J |
| Energy coefficient k | 0.4 W·s²/m² |
| Climb energy penalty | ×1.5 multiplier on k |
| Handoff threshold | 20% battery |
| Active pursuer speed | 5 m/s |
| Standby pursuer speed | 4 m/s |
| Evader speed | 3.5 m/s |
| Handoff weights (wE, wd) | 0.6, 0.4 |
| z range | 0.5 – 6.0 m |
| dt | 0.02 s |

---

## Implementation

```python
import numpy as np
from numpy.linalg import norm

N_PURSUERS = 3
E0 = 60.0
K_ENERGY   = 0.4
K_CLIMB    = 0.6     # energy coefficient during climb (extra gravity cost)
HANDOFF_THRESH = 0.2
V_ACTIVE   = 5.0
V_STANDBY  = 4.0
V_EVADER   = 3.5
Z_TIERS    = [1.0, 2.5, 4.5]
Z_MIN, Z_MAX = 0.5, 6.0
W_ENERGY   = 0.6
W_DIST     = 0.4
D_MAX      = 15.0    # normalisation distance
K_HOLD     = 2.0
DT = 0.02

def battery_drain(vel, dz, dt):
    """Drain accounting for climb cost."""
    k = K_CLIMB if dz > 0 else K_ENERGY
    return k * norm(vel)**2 * dt

def standby_score(energy, pos_k, pos_intercept, e0=E0):
    d = norm(pos_intercept - pos_k)
    return W_ENERGY * (energy / e0) - W_DIST * (d / D_MAX)

def simulate_relay_3d():
    # Initial positions: each pursuer at its tier altitude, spread in x
    pos_p = np.array([
        [-4.0, 0.0, Z_TIERS[0]],
        [ 0.0, 0.0, Z_TIERS[1]],
        [ 4.0, 0.0, Z_TIERS[2]],
    ], dtype=float)
    pos_e = np.array([0.0, 3.0, 2.5])
    vel_e = np.array([0.5, 1.0, 0.3])
    vel_e = V_EVADER * vel_e / norm(vel_e)

    energy = [E0] * N_PURSUERS
    active = 0
    traj_e = [pos_e.copy()]
    traj_p = [pos_p.copy()]
    handoff_events = []
    vel_p = [np.zeros(3)] * N_PURSUERS

    for step in range(int(40 / DT)):
        t = step * DT

        # Estimate handoff time
        e_active = energy[active]
        T_h = (e_active - HANDOFF_THRESH * E0) / (K_ENERGY * V_ACTIVE**2 + 1e-8)
        T_h = max(T_h, 0.0)
        p_intercept = pos_e + vel_e * T_h

        # Handoff check
        if e_active / E0 < HANDOFF_THRESH:
            scores = [
                standby_score(energy[k], pos_p[k], p_intercept)
                if k != active else -np.inf
                for k in range(N_PURSUERS)
            ]
            new_active = int(np.argmax(scores))
            handoff_events.append((t, active, new_active))
            active = new_active

        # Move active pursuer toward evader (pure pursuit)
        d_active = pos_e - pos_p[active]
        vel_p[active] = V_ACTIVE * d_active / (norm(d_active) + 1e-8)
        dz_a = vel_p[active][2] * DT
        energy[active] -= battery_drain(vel_p[active], dz_a, DT)
        pos_p[active] += vel_p[active] * DT
        pos_p[active, 2] = np.clip(pos_p[active, 2], Z_MIN, Z_MAX)

        # Move standby pursuers toward intercept, hold tier when far
        for k in range(N_PURSUERS):
            if k == active:
                continue
            d_int = p_intercept - pos_p[k]
            dist_int = norm(d_int)
            if dist_int > 0.5:
                vel_p[k] = V_STANDBY * d_int / (dist_int + 1e-8)
            else:
                vel_p[k] = np.array([0.0, 0.0, K_HOLD * (Z_TIERS[k] - pos_p[k, 2])])
            dz_k = vel_p[k][2] * DT
            energy[k] -= battery_drain(vel_p[k], dz_k, DT) * 0.5  # standby uses half power
            pos_p[k] += vel_p[k] * DT
            pos_p[k, 2] = np.clip(pos_p[k, 2], Z_MIN, Z_MAX)

        # Move evader (straight escape with slight vertical wander)
        vel_e[2] += 0.05 * np.random.randn()
        vel_e = V_EVADER * vel_e / (norm(vel_e) + 1e-8)
        pos_e += vel_e * DT
        pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)

        traj_e.append(pos_e.copy())
        traj_p.append(pos_p.copy())

        if norm(pos_p[active] - pos_e) < 0.15:
            print(f"Captured at t={t:.2f}s")
            break

    return np.array(traj_e), np.array(traj_p), handoff_events, energy
```

---

## Expected Output

- 3D trajectory colour-coded by active pursuer (red/blue/green segments) with real z variation
- Altitude time series: all three pursuers and evader showing tier-holding and intercept climbs
- Battery level vs time with vertical handoff event lines
- Comparison: flat-relay (all z = 2 m) vs altitude-tiered relay — handoff gap size and capture time
- Handoff score heat map: score surface as function of altitude and horizontal distance

---

## Extensions

1. Optimal tier spacing: grid-search over altitude tier values to minimise average handoff gap
2. 4-drone relay with two drones per tier — allows simultaneous active + backup at each altitude
3. Evader detects handoff (battery-low signal visible in radar cross-section) and dives to new altitude tier

---

## Related Scenarios

- Original 2D version: [S012](../S012_relay_pursuit.md)
- Truly 3D references: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
