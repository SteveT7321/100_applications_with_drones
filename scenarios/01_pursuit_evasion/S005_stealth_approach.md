# S005 Stealth Approach

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Done

---

## Problem Definition

**Setup**: Pursuer approaches an evader while staying outside the evader's forward detection cone.

**Roles**:
- **Evader**: flies at constant heading; has a forward-facing detection cone (half-angle 60°, range 5 m) pointing in its velocity direction; if the pursuer enters the cone it becomes alerted and escapes at full speed
- **Pursuer**: must maneuver laterally around the evader to approach from the blind-spot rear; receives a speed bonus while remaining undetected

**Objective**: demonstrate that a stealth flanking approach is faster and more reliable than a direct frontal pursuit that triggers the alert.

---

## Mathematical Model

### Detection Cone Condition

The pursuer is detected when two conditions hold simultaneously:

$$\cos\alpha = \frac{(\mathbf{p}_P - \mathbf{p}_E) \cdot \hat{\mathbf{v}}_E}{\|\mathbf{p}_P - \mathbf{p}_E\|}$$

Detected if:

$$\alpha < \theta_{cone} \quad \text{AND} \quad \|\mathbf{p}_P - \mathbf{p}_E\| < R_{detect}$$

where $\theta_{cone} = 60°$ and $R_{detect} = 5$ m.

### Pursuer Stealth Strategy

Encircle laterally to stay outside the cone, then close from the rear:

$$\mathbf{v}_{cmd} = v_P \cdot \frac{\mathbf{p}_{target} - \mathbf{p}_P}{\|\mathbf{p}_{target} - \mathbf{p}_P\|}$$

where the intermediate target is offset 180° from the evader's heading:

$$\mathbf{p}_{target} = \mathbf{p}_E - R_{offset} \cdot \hat{\mathbf{v}}_E$$

### Alert Mode (Evader)

Once detected, the evader switches to maximum straight escape:

$$\mathbf{v}_E = v_{E,max} \cdot \hat{\mathbf{v}}_{escape}$$

where $\hat{\mathbf{v}}_{escape}$ is the direction away from the pursuer.

---

## Implementation

```python
# src/pursuit/s005_stealth_approach.py

PURSUER_SPEED_STEALTH  = 4.5 + 1.0   # bonus speed when undetected
PURSUER_SPEED_DETECTED = 4.5          # normal speed once alert triggered
EVADER_SPEED_NORMAL    = 3.0          # m/s constant heading
EVADER_SPEED_ALERT     = 3.0          # same speed, just direction changes
CONE_HALF_ANGLE_DEG    = 60.0
DETECT_RANGE           = 5.0          # m

def is_detected(pos_p, pos_e, vel_e, cone_deg, r_detect):
    """Return True if pursuer is inside evader's detection cone."""
    diff = pos_p - pos_e
    dist = np.linalg.norm(diff)
    if dist > r_detect:
        return False
    v_hat = vel_e / (np.linalg.norm(vel_e) + 1e-8)
    cos_alpha = np.dot(diff, v_hat) / (dist + 1e-8)
    alpha_deg = np.degrees(np.arccos(np.clip(cos_alpha, -1, 1)))
    return alpha_deg < cone_deg

def stealth_target(pos_e, vel_e, offset=2.0):
    """Intermediate waypoint behind the evader."""
    v_hat = vel_e / (np.linalg.norm(vel_e) + 1e-8)
    return pos_e - offset * v_hat

# Main loop
for step in range(MAX_STEPS):
    detected = is_detected(pos_p, pos_e, vel_e, CONE_HALF_ANGLE_DEG, DETECT_RANGE)

    # Pursuer
    if detected:
        speed = PURSUER_SPEED_DETECTED
        target = pos_e
    else:
        speed = PURSUER_SPEED_STEALTH
        target = stealth_target(pos_e, vel_e)

    dir_p = (target - pos_p) / (np.linalg.norm(target - pos_p) + 1e-8)
    pos_p += dir_p * speed * DT

    # Evader
    if detected:
        flee_dir = (pos_e - pos_p) / (np.linalg.norm(pos_e - pos_p) + 1e-8)
        pos_e += flee_dir * EVADER_SPEED_ALERT * DT
    else:
        pos_e += vel_e / (np.linalg.norm(vel_e) + 1e-8) * EVADER_SPEED_NORMAL * DT
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Detection cone half-angle | 60 deg |
| Detection range | 5.0 m |
| Pursuer speed (undetected) | 5.5 m/s (4.5 + 1.0 bonus) |
| Pursuer speed (detected) | 4.5 m/s |
| Evader speed (normal) | 3.0 m/s |
| Evader speed (alerted) | 3.0 m/s (direction changes) |
| Initial pursuer position | (-4, 2, 2) m |
| Initial evader position | (0, 0, 2) m heading +x |
| Capture radius | 0.15 m |
| Control frequency | 48 Hz |
| Max simulation time | 20 s |

---

## Expected Output

- 3D trajectory with detection cone visualization (translucent cone attached to evader)
- Detection status vs time (binary signal, 0 = undetected, 1 = detected)
- Comparison table and bar chart:
  - **Direct frontal approach**: pursuer enters cone immediately, evader alerts and flees — longer capture time
  - **Stealth flanking approach**: pursuer encircles, enters from rear blind-spot — shorter capture time with bonus speed

---

## Extensions

1. Multiple detection cones (front cone + two lateral cones at ±90°)
2. Time-varying evader heading (scanning/turning pattern) — pursuer must predict heading
3. Pursuer noise signature: speed above a threshold increases its acoustic visibility and triggers detection even outside the cone

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md), [S002](S002_evasive_maneuver.md)
- Follow-ups: [S006](S006_energy_race.md), [S016](S016_airspace_defense.md)
- See [domains/01_pursuit_evasion.md](../../domains/01_pursuit_evasion.md)
