# S003 Low-Altitude Tracking

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

The pursuer tracks the target at low altitude (0.5~1.5 m above ground), facing:
1. **Ground Effect**: thrust increases abnormally when flying close to the ground
2. **Terrain obstacles**: hills or buildings blocking the line of sight
3. **Boundary constraints**: cannot fly into the ground

---

## Mathematical Model

### Ground Effect Model

Thrust correction factor for CF2X at altitude $h$:

$$k_{GE}(h) = 1 + \frac{0.16 \cdot (R/h)^2}{1 + (R/h)^2}$$

where $R$ is the propeller radius (approximately 0.05 m).

When $h < 0.5$ m, thrust increases by approximately 10~20%.

### Altitude Safety Constraint

The pursuer's altitude command must satisfy:

$$z_{target} \geq h_{terrain}(x, y) + h_{safe}$$

where $h_{safe} = 0.3$ m (safety margin).

---

## Implementation

```python
# Use Physics.PYB_GND to enable ground effect
env = CtrlAviary(
    physics=Physics.PYB_GND,
    num_drones=2,
    initial_xyzs=np.array([[-3., 0., 0.8], [3., 0., 0.8]]),
)

# Terrain model (simulated hills)
def terrain_height(x, y):
    """Simple Gaussian hill terrain"""
    return 0.5 * np.exp(-((x-1)**2 + (y-0.5)**2) / 2.0)

# Altitude compensation
def get_safe_target(target_pos):
    h_terrain = terrain_height(target_pos[0], target_pos[1])
    z_safe = max(target_pos[2], h_terrain + 0.3)
    return np.array([target_pos[0], target_pos[1], z_safe])
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Flight altitude range | 0.5~1.5 m |
| Terrain type | Gaussian hills (replaceable) |
| Safety altitude margin | 0.3 m |
| Physics mode | `Physics.PYB_GND` |

---

## Expected Output

- 3D trajectory plot (with terrain surface)
- Altitude time series: showing altitude fluctuations caused by ground effect
- Pursuit success rate vs terrain complexity

---

## Extensions

1. Add altitude sensor noise (ultrasonic altimeter model)
2. Combine with S004 (obstacle chase)
3. Urban street canyon pursuit scenario
