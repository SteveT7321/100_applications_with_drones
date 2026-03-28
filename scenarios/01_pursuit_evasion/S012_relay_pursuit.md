# S012 Relay Pursuit

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: Three pursuers take turns chasing the evader. Each has a limited battery. When the active pursuer's battery falls below a handoff threshold, the next pursuer takes over. Standby pursuers fly toward the predicted evader position to be ready for seamless handoff.

**Objective**: capture the evader with continuous pursuit pressure despite individual battery limits.

**Comparison**: relay (3 pursuers, handoff) vs single pursuer (runs out of battery before capture).

---

## Mathematical Model

### Battery Model

$$E_i(t+\Delta t) = E_i(t) - k \cdot v_i^2 \cdot \Delta t$$

### Handoff Trigger

Active pursuer $i$ hands off when:

$$E_i < E_{threshold} = 0.2 \cdot E_0$$

### Standby Intercept Position

Each standby pursuer flies toward the predicted evader position at handoff time $T_h$:

$$\mathbf{p}_{intercept} = \mathbf{p}_E(t) + \mathbf{v}_E \cdot T_h$$

where $T_h$ is the estimated time until handoff (based on remaining energy of active pursuer).

### Handoff Selection

Next active pursuer: highest remaining energy among standby drones.

---

## Implementation

```python
N_PURSUERS = 3
E0 = 60.0            # J each
K_ENERGY = 0.4       # W·s²/m²
HANDOFF_THRESH = 0.2 # fraction of E0

active = 0
energy = [E0] * N_PURSUERS

for step in range(max_steps):
    # Drain active pursuer energy
    energy[active] -= K_ENERGY * PURSUER_SPEED**2 * DT

    # Handoff check
    if energy[active] / E0 < HANDOFF_THRESH:
        candidates = [(e, i) for i, e in enumerate(energy) if i != active]
        active = max(candidates)[1]   # highest energy standby

    # Active pursuer: pure pursuit
    pursuers[active].step(pure_pursuit_velocity(...))

    # Standby pursuers: fly to intercept position
    for i, p in enumerate(pursuers):
        if i != active:
            t_handoff = energy[active] / (K_ENERGY * PURSUER_SPEED**2)
            p_intercept = evader.pos + evader.vel * t_handoff
            pursuers[i].step(standby_velocity(p_intercept, ...))
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of pursuers | 3 |
| Battery capacity each | 60 J |
| Energy coefficient k | 0.4 W·s²/m² |
| Handoff threshold | 20% battery |
| Active pursuer speed | 5 m/s |
| Standby pursuer speed | 4 m/s |
| Evader speed | 3.5 m/s (straight escape) |
| Initial distance (P1 to evader) | 5 m |

---

## Expected Output

- 3D trajectory: colour-coded by active pursuer (red/blue/green segments)
- Battery level vs time for all 3 pursuers (vertical lines at handoff events)
- Distance to evader vs time: continuous pressure vs single-pursuer dropout
- Comparison: relay capture time vs single-pursuer timeout

---

## Extensions

1. Time-optimal handoff: optimise $E_{threshold}$ to minimise total capture time
2. 5 pursuers with 2 simultaneously active
3. Evader exploits handoff moment — sprints when it detects battery-low signal

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md), [S006](S006_energy_race.md)
- Next: [S013](S013_pincer_movement.md), [S015](S015_relay_tracking.md), [S019](S019_dynamic_reassignment.md)
