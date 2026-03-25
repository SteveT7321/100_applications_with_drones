# Domain 4: Industrial & Agriculture

## Theoretical Background

Industrial inspection and agricultural applications emphasize **precision control** and **regular path patterns**:

- **Surface-following control**: maintaining a fixed distance and angle relative to the target surface
- **Agricultural spray coverage optimization**: trade-off between overlap rate and missed coverage rate
- **Cooperative operations**: rigid/flexible constraints for multi-drone heavy lifting and cooperative modeling
- **Task completion guarantee**: industrial applications demand extremely high precision and completeness

## Common Physical Settings

```python
# Industrial/agricultural scenario settings
INSPECTION_SPEED  = 1.0   # m/s (inspection flight speed)
INSPECTION_DIST   = 0.5   # m (distance from target surface)
SPRAY_WIDTH       = 2.0   # m (spray swath width)
SPRAY_OVERLAP     = 0.1   # 10% (strip overlap rate)
GPS_ERROR         = 0.1   # m (simulated GPS error)
```

## Scenario List (20 total)

| Scenario | Name | Core Problem | Difficulty |
|------|------|----------|------|
| [S061](../scenarios/04_industrial_agriculture/S061_power_line.md) | Power Line Inspection | Line-following flight + obstacle avoidance | ⭐⭐ |
| [S062](../scenarios/04_industrial_agriculture/S062_wind_turbine.md) | Wind Turbine Blade Inspection | Flight around rotating blades | ⭐⭐⭐ |
| [S063](../scenarios/04_industrial_agriculture/S063_precision_spraying.md) | Single-Plant Precision Spraying | Fixed-point hover accuracy | ⭐ |
| [S064](../scenarios/04_industrial_agriculture/S064_greenhouse.md) | Greenhouse Interior Precision Flight | Narrow GPS-denied environment | ⭐⭐⭐ |
| [S065](../scenarios/04_industrial_agriculture/S065_3d_scan_path.md) | Building 3D Modeling Sampling | Multi-angle scan path | ⭐⭐ |
| [S066](../scenarios/04_industrial_agriculture/S066_cooperative_crane.md) | Cooperative Crane Simulation | Rigid load multi-drone control | ⭐⭐⭐⭐ |
| [S067](../scenarios/04_industrial_agriculture/S067_spray_overlap.md) | Spray Coverage Overlap Optimization | Strip planning optimization | ⭐⭐ |
| [S068](../scenarios/04_industrial_agriculture/S068_large_field_spray.md) | Large-Scale Farmland Cooperative Spraying | Multi-drone TSP + charging strategy | ⭐⭐⭐ |
| [S069](../scenarios/04_industrial_agriculture/S069_warehouse_inventory.md) | Automated Warehouse Inventory | Indoor navigation + barcode scanning | ⭐⭐ |
| [S070](../scenarios/04_industrial_agriculture/S070_swarm_weeding.md) | Swarm Weeding Task | Multi-drone coverage + precision positioning | ⭐⭐⭐ |
| [S071](../scenarios/04_industrial_agriculture/S071_bridge_inspection.md) | Bridge Underside Structural Inspection | Upward-facing surface-following flight | ⭐⭐⭐ |
| [S072](../scenarios/04_industrial_agriculture/S072_pipeline_leak.md) | Oil Pipeline Leak Detection | Along-pipeline flight + chemical detection | ⭐⭐ |
| [S073](../scenarios/04_industrial_agriculture/S073_solar_thermal.md) | Solar Panel Thermal Imaging Inspection | Uniform low-speed scanning | ⭐ |
| [S074](../scenarios/04_industrial_agriculture/S074_mine_mapping.md) | Mine 3D Mapping | Mapping in GNSS-denied environment | ⭐⭐⭐⭐ |
| [S075](../scenarios/04_industrial_agriculture/S075_container_yard.md) | Port Container Yard Inventory | High-density obstacle environment | ⭐⭐ |
| [S076](../scenarios/04_industrial_agriculture/S076_thermal_management.md) | Extreme Heat Cooling Strategy | Mission under motor temperature limits | ⭐⭐ |
| [S077](../scenarios/04_industrial_agriculture/S077_pollination.md) | Precision Pollination Flight Pattern | Sequential precision docking at flowers | ⭐⭐⭐ |
| [S078](../scenarios/04_industrial_agriculture/S078_harvester_guidance.md) | Harvester Cooperative Guidance | Air-ground coordination | ⭐⭐ |
| [S079](../scenarios/04_industrial_agriculture/S079_offshore_wind.md) | Offshore Wind Farm Installation Assistance | Ocean swell environment | ⭐⭐⭐ |
| [S080](../scenarios/04_industrial_agriculture/S080_underground_pipe.md) | Underground Pipe CCTV Replacement | Navigation in extremely narrow pipes | ⭐⭐⭐⭐ |

## Key Metrics

```python
metrics = {
    "coverage_completeness": float,  # Task completion rate (%)
    "path_deviation":        float,  # Deviation from planned path (m)
    "inspection_quality":    float,  # Simulated image clarity (speed stability)
    "collision_count":       int,    # Number of collisions (industrial requires zero)
    "task_time":             float,  # Task completion time
}
```

## Related Documents

- [docs/physics_guide.md](../docs/physics_guide.md) — Physics simulation settings
- [docs/algorithm_index.md](../docs/algorithm_index.md) — Lawnmower, RRT* reference index
