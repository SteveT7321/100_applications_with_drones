# Domain 1: Pursuit & Evasion

## Theoretical Background

The pursuit problem lies at the intersection of control theory and game theory, with **Differential Games** at its core:
two (or more) agents each optimize their own objective, and their decisions mutually influence each other.

- **Zero-sum game**: the pursuer minimizes capture time, the evader maximizes it
- **Value function V(x)**: the outcome starting from state x under both parties' optimal strategies
- **Hamilton-Jacobi-Isaacs (HJI) equation**: the PDE for V; its numerical solution yields the optimal strategy

Real-world engineering applications: missile interception, counter-drone defense, police drone tracking.

## Common Physical Settings

```python
# Standard pursuit scenario settings
PURSUER_MAX_SPEED = 5.0   # m/s (CF2X thrust approx. 4~6 m/s)
EVADER_MAX_SPEED  = 4.0   # m/s
CAPTURE_RADIUS    = 0.15  # m (capture detection threshold)
ARENA_BOUNDS      = 5.0   # m (half-side length of cubic arena)
CTRL_FREQ         = 48    # Hz
SIM_DURATION      = 30    # seconds
```

## Scenario List (20 total)

### 1v1 Basics (S001~S010)

| Scenario | Name | Core Problem | Difficulty |
|------|------|----------|------|
| [S001](../scenarios/01_pursuit_evasion/S001_basic_intercept.md) | Basic Intercept | Proportional navigation vs. stationary target | ⭐ |
| [S002](../scenarios/01_pursuit_evasion/S002_evasive_maneuver.md) | Evasive Maneuver | Optimal evasion vs. pure pursuit | ⭐⭐ |
| [S003](../scenarios/01_pursuit_evasion/S003_low_altitude_tracking.md) | Low-Altitude Tracking | Terrain occlusion + ground effect | ⭐⭐ |
| [S004](../scenarios/01_pursuit_evasion/S004_obstacle_chase.md) | Obstacle Chase | RRT* dynamic re-planning | ⭐⭐⭐ |
| [S005](../scenarios/01_pursuit_evasion/S005_stealth_approach.md) | Stealth Approach | Sensor blind-spot strategy | ⭐⭐⭐ |
| [S006](../scenarios/01_pursuit_evasion/S006_energy_race.md) | Energy Depletion Race | MPC + battery constraints | ⭐⭐⭐ |
| [S007](../scenarios/01_pursuit_evasion/S007_jamming_blind_pursuit.md) | Jamming-Blind Pursuit | EKF target estimation | ⭐⭐⭐ |
| [S008](../scenarios/01_pursuit_evasion/S008_stochastic_pursuit.md) | Stochastic Disturbance Pursuit | Stochastic optimal control | ⭐⭐⭐ |
| [S009](../scenarios/01_pursuit_evasion/S009_differential_game.md) | Differential Game 1v1 | HJI numerical solution | ⭐⭐⭐⭐ |
| [S010](../scenarios/01_pursuit_evasion/S010_asymmetric_speed.md) | Asymmetric Speed Pursuit | Time-optimal Pontryagin | ⭐⭐⭐⭐ |

### Nv1 (S011~S015)

| Scenario | Name | Architecture | Difficulty |
|------|------|------|------|
| [S011](../scenarios/01_pursuit_evasion/S011_swarm_encirclement.md) | Swarm Encirclement | 3v1, potential field contraction | ⭐⭐ |
| [S012](../scenarios/01_pursuit_evasion/S012_relay_pursuit.md) | Relay Pursuit | Nv1, battery relay handoff | ⭐⭐⭐ |
| [S013](../scenarios/01_pursuit_evasion/S013_pincer_movement.md) | Multi-Dimensional Encirclement | 3v1, pincer cutoff | ⭐⭐⭐ |
| [S014](../scenarios/01_pursuit_evasion/S014_decoy_lure.md) | Decoy Lure | 2v1, decoy strategy | ⭐⭐⭐ |
| [S015](../scenarios/01_pursuit_evasion/S015_relay_tracking.md) | Communication Relay Tracking | Nv1, sensor range relay | ⭐⭐⭐ |

### NvM (S016~S020)

| Scenario | Name | Architecture | Difficulty |
|------|------|------|------|
| [S016](../scenarios/01_pursuit_evasion/S016_airspace_defense.md) | Airspace Defense Battle | 3v3, offense vs. defense | ⭐⭐⭐⭐ |
| [S017](../scenarios/01_pursuit_evasion/S017_swarm_vs_swarm.md) | Swarm vs. Swarm | NvM, self-organization | ⭐⭐⭐⭐ |
| [S018](../scenarios/01_pursuit_evasion/S018_multi_target_intercept.md) | Resource Allocation Intercept | Nv N, Hungarian assignment | ⭐⭐⭐⭐ |
| [S019](../scenarios/01_pursuit_evasion/S019_dynamic_reassignment.md) | Dynamic Target Reassignment | Nv M, real-time reallocation | ⭐⭐⭐⭐ |
| [S020](../scenarios/01_pursuit_evasion/S020_pursuit_evasion_game.md) | Pursuit-Evasion Game | NvM, full differential game | ⭐⭐⭐⭐⭐ |

## Key Metrics

```python
# Evaluation metrics for pursuit scenarios
metrics = {
    "capture_time":     float,   # Capture time (seconds), lower is better (pursuer)
    "miss_distance":    float,   # Closest approach distance (when not captured)
    "path_length":      float,   # Total path length
    "energy_consumed":  float,   # Energy consumed (thrust integral)
    "capture_success":  bool,    # Whether capture succeeded (within 30 seconds)
}
```

## Recommended Learning Path

1. **Start with S001** (PID + PNG), understand the full simulation framework
2. **S002** adds evader control, understand the two-party game
3. **S009** differential game, understand HJI equation numerical solution
4. **S011** multi-drone encirclement, understand distributed coordination
5. **S020** integrates all of the above concepts

## Related Documents

- [MATH_FOUNDATIONS.md §4](../MATH_FOUNDATIONS.md) — Pursuit and differential game mathematics
- [docs/drone_models.md](../docs/drone_models.md) — Drone physical parameters
- [docs/algorithm_index.md](../docs/algorithm_index.md) — Algorithm reference index
