# Domain 2: Logistics & Delivery

## Theoretical Background

The core problem of drone logistics is the combination of **combinatorial optimization** and **dynamic programming**:

- **Traveling Salesman Problem (TSP)**: shortest path for a single drone visiting multiple targets
- **Vehicle Routing Problem (VRP)**: joint optimization of assignment and routing for multiple drones and multiple targets
- **Trajectory planning**: time-optimal / energy-optimal path from A to B
- **Dynamic re-planning**: real-time adjustment when new orders are inserted mid-flight or the environment changes

Real-world applications: Amazon Prime Air, Meituan drone delivery, emergency medical supply delivery.

## Common Physical Settings

```python
# Standard logistics scenario settings
DELIVERY_HEIGHT   = 20.0  # m (urban delivery flight altitude)
MAX_PAYLOAD       = 0.1   # kg (CF2X maximum payload, conservative estimate)
BATTERY_CAPACITY  = 1.0   # normalized (1.0 = full charge)
HOVER_DRAIN       = 0.01  # consumption per second (hovering)
FLIGHT_DRAIN      = 0.015 # consumption per second (flying)
LANDING_THRESHOLD = 0.05  # m (successful landing detection threshold)
```

## Scenario List (20 total)

### Single-Drone Basics (S021~S025)

| Scenario | Name | Core Problem | Difficulty |
|------|------|----------|------|
| [S021](../scenarios/02_logistics_delivery/S021_point_delivery.md) | Single-Point Delivery | A→B shortest path + precision landing | ⭐ |
| [S022](../scenarios/02_logistics_delivery/S022_obstacle_avoidance.md) | Obstacle Avoidance | RRT* planning | ⭐⭐ |
| [S023](../scenarios/02_logistics_delivery/S023_moving_landing_pad.md) | Dynamic Landing Pad | Moving target tracking and landing | ⭐⭐⭐ |
| [S024](../scenarios/02_logistics_delivery/S024_wind_compensation.md) | Wind Disturbance Compensation | Disturbance rejection + trajectory correction | ⭐⭐ |
| [S025](../scenarios/02_logistics_delivery/S025_payload_cog_offset.md) | Payload Center-of-Gravity Offset | Asymmetric load attitude compensation | ⭐⭐⭐ |

### Nv1 (S026~S028)

| Scenario | Name | Architecture | Difficulty |
|------|------|------|------|
| [S026](../scenarios/02_logistics_delivery/S026_cooperative_heavy_lift.md) | Multi-Drone Cooperative Heavy Lift | 2~4 drones suspending the same object | ⭐⭐⭐⭐ |
| [S027](../scenarios/02_logistics_delivery/S027_aerial_refueling.md) | Aerial Refueling Relay | Tanker drone + mission drone | ⭐⭐⭐ |
| [S028](../scenarios/02_logistics_delivery/S028_cargo_escort.md) | Group Cargo Protection | Escort formation | ⭐⭐ |

### NvM (S029~S040)

| Scenario | Name | Core Problem | Difficulty |
|------|------|----------|------|
| [S029](../scenarios/02_logistics_delivery/S029_urban_scheduling.md) | Urban Logistics Network Scheduling | VRP + dynamic insertion | ⭐⭐⭐⭐ |
| [S030](../scenarios/02_logistics_delivery/S030_multi_depot.md) | Multi-Depot Delivery Allocation | Multi-depot VRP | ⭐⭐⭐⭐ |
| [S031](../scenarios/02_logistics_delivery/S031_deconfliction.md) | Path Conflict Deconfliction | Time-separation / layer-separation strategy | ⭐⭐⭐ |
| [S032](../scenarios/02_logistics_delivery/S032_charging_queue.md) | Fleet Charging Queue | Battery management scheduling | ⭐⭐⭐ |
| [S033](../scenarios/02_logistics_delivery/S033_online_insertion.md) | Real-Time Order Insertion | Online VRP | ⭐⭐⭐⭐ |
| [S034](../scenarios/02_logistics_delivery/S034_weather_rerouting.md) | Degraded Weather Re-routing | Dynamic obstacle detour | ⭐⭐⭐ |
| [S035](../scenarios/02_logistics_delivery/S035_utm_simulation.md) | Air Traffic Management | Corridor request conflict warning | ⭐⭐⭐⭐ |
| [S036](../scenarios/02_logistics_delivery/S036_last_mile_relay.md) | Last-Mile Relay | Beyond-range segmented delivery | ⭐⭐⭐ |
| [S037](../scenarios/02_logistics_delivery/S037_reverse_logistics.md) | Reverse Logistics Collection | Return path optimization | ⭐⭐ |
| [S038](../scenarios/02_logistics_delivery/S038_disaster_relief.md) | Post-Earthquake Emergency Drop | Precision airdrop without landing zone | ⭐⭐⭐ |
| [S039](../scenarios/02_logistics_delivery/S039_offshore_exchange.md) | Offshore Platform Cargo Exchange | Landing on a rocking platform | ⭐⭐⭐⭐ |
| [S040](../scenarios/02_logistics_delivery/S040_fleet_load_balancing.md) | Fleet Load Balancing | Dynamic reallocation | ⭐⭐⭐ |

## Key Metrics

```python
metrics = {
    "delivery_success_rate": float,   # Successful delivery rate
    "total_distance":        float,   # Total flight distance across all drones
    "total_time":            float,   # Time to complete all deliveries
    "battery_remaining":     list,    # Remaining battery per drone
    "landing_accuracy":      float,   # Landing accuracy (root mean square error)
    "conflict_count":        int,     # Number of path conflicts
}
```

## Related Documents

- [MATH_FOUNDATIONS.md §3](../MATH_FOUNDATIONS.md) — Path planning mathematics
- [docs/algorithm_index.md](../docs/algorithm_index.md) — VRP, A*, RRT* reference index
