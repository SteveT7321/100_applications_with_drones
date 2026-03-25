# Domain 5: Special Ops & Entertainment

## Theoretical Background

Entertainment and special ops scenarios emphasize **visual effects** and **coordination precision**:

- **Formation control**: distributed control for maintaining geometric shapes
- **Geometric shape interpolation**: trajectory for smoothly transitioning from shape A to shape B
- **Collision avoidance protocols (ORCA / Olfati-Saber)**: decentralized collision avoidance for large-scale swarms
- **Visual tracking**: identifying and tracking moving targets (people, vehicles)

## Common Physical Settings

```python
# Entertainment/special ops scenario settings
LED_SHOW_HEIGHT   = 50.0  # m (light show flight altitude)
FORMATION_DIST    = 2.0   # m (formation spacing)
COLLISION_RADIUS  = 0.5   # m (collision avoidance activation distance)
RACE_GATE_RADIUS  = 0.3   # m (racing gate radius)
```

## Scenario List (20 total)

| Scenario | Name | Core Problem | Difficulty |
|------|------|----------|------|
| [S081](../scenarios/05_special_entertainment/S081_selfie_follow.md) | Selfie Follow Mode | Target tracking + framing | ⭐⭐ |
| [S082](../scenarios/05_special_entertainment/S082_fpv_racing.md) | FPV Racing Circuit | Time-optimal trajectory | ⭐⭐⭐⭐ |
| [S083](../scenarios/05_special_entertainment/S083_light_show_single.md) | Light Show Single-Drone Test | Precision position control | ⭐ |
| [S084](../scenarios/05_special_entertainment/S084_wind_endurance.md) | Extreme Weather Wind Resistance Test | Strong wind disturbance rejection | ⭐⭐⭐ |
| [S085](../scenarios/05_special_entertainment/S085_light_matrix.md) | Drone Light Matrix Positioning | Precision grid-point positioning | ⭐⭐ |
| [S086](../scenarios/05_special_entertainment/S086_multi_angle_cinema.md) | Cooperative Multi-Angle Filming | Arc orbit cooperative flight | ⭐⭐ |
| [S087](../scenarios/05_special_entertainment/S087_led_text.md) | Aerial Light Show Text Display | Character encoding + positioning | ⭐⭐ |
| [S088](../scenarios/05_special_entertainment/S088_formation_morphing.md) | Formation Geometric Shape Morphing | Shape interpolation + collision avoidance | ⭐⭐⭐ |
| [S089](../scenarios/05_special_entertainment/S089_collision_avoidance.md) | Large-Scale Swarm Collision Avoidance Test | ORCA protocol 50+ drones | ⭐⭐⭐⭐ |
| [S090](../scenarios/05_special_entertainment/S090_racing_optimal.md) | Racing Drone Optimal Path | Minimum-Snap trajectory | ⭐⭐⭐⭐ |
| [S091](../scenarios/05_special_entertainment/S091_acrobatics.md) | Aerial Acrobatic Maneuvers | Backflips, spirals, and other extreme moves | ⭐⭐⭐⭐ |
| [S092](../scenarios/05_special_entertainment/S092_movie_chase.md) | Movie Car Chase Scene | High-speed tracking + obstacle traversal | ⭐⭐⭐ |
| [S093](../scenarios/05_special_entertainment/S093_rubble_sar.md) | Earthquake Rubble SAR | Unstructured environment exploration | ⭐⭐⭐⭐ |
| [S094](../scenarios/05_special_entertainment/S094_counter_drone.md) | Counter-Drone Intercept | Rapid interception of small target | ⭐⭐⭐⭐ |
| [S095](../scenarios/05_special_entertainment/S095_water_landing.md) | Water Surface Landing Simulation | Landing on wave-rocking surface | ⭐⭐⭐ |
| [S096](../scenarios/05_special_entertainment/S096_relay_race.md) | Drone Relay Race | Mid-air baton handoff + speed relay | ⭐⭐⭐ |
| [S097](../scenarios/05_special_entertainment/S097_aerial_puzzle.md) | Aerial Puzzle Assembly | Precision geometric assembly | ⭐⭐⭐ |
| [S098](../scenarios/05_special_entertainment/S098_synchronized_dance.md) | Swarm Synchronized Dance | Multi-drone temporal synchronization | ⭐⭐⭐ |
| [S099](../scenarios/05_special_entertainment/S099_obstacle_relay.md) | Cross-Obstacle Relay Pass | Mid-air object transfer | ⭐⭐⭐⭐ |
| [S100](../scenarios/05_special_entertainment/S100_grand_challenge.md) | Ultimate Grand Challenge: Multi-Domain Integration | Pursuit + logistics + SAR combination | ⭐⭐⭐⭐⭐ |

## Key Metrics

```python
metrics = {
    "formation_error":      float,   # Formation maintenance error (m)
    "collision_count":      int,     # Number of collisions (light show requires zero)
    "synchronization_error": float,  # Temporal synchronization error (ms)
    "race_time":            float,   # Racing completion time
    "visual_fidelity":      float,   # Shape reproduction accuracy (pixel error)
}
```

## Light Show Special Notes

For large-scale light shows (50+ drones), it is recommended to use a **pure mathematical model** rather than PyBullet to maintain real-time performance:

```python
# Mathematical model (without gym-pybullet-drones)
# Each drone is a simplified point mass, integrated with ODE
from scipy.integrate import odeint
```

See the performance optimization section in [docs/physics_guide.md](../docs/physics_guide.md).

## Related Documents

- [MATH_FOUNDATIONS.md §5](../MATH_FOUNDATIONS.md) — Multi-agent coordination
- [docs/algorithm_index.md](../docs/algorithm_index.md) — ORCA, Formation reference index
