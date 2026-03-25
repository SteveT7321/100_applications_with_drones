# Domain 3: Environmental Monitoring & SAR

## Theoretical Background

The core problem of search-and-rescue and environmental monitoring is **maximizing information acquisition**:

- **Coverage rate maximization**: scan the largest area in the shortest time
- **Information entropy minimization**: Bayesian search, continuously updating belief about target location based on observations
- **Multi-drone cooperative mapping**: distributed multi-drone version of SLAM (Simultaneous Localization and Mapping)
- **Gradient tracing**: following the gradient direction to locate the source in a chemical/temperature field

## Common Physical Settings

```python
# Standard SAR scenario settings
SENSOR_RADIUS    = 2.0    # m (sensor detection range)
SENSOR_FOV_DEG   = 60     # degrees (downward-facing camera field of view)
SCAN_HEIGHT      = 10.0   # m (standard scan altitude)
GRID_RESOLUTION  = 0.5    # m (occupancy grid resolution)
SEARCH_AREA      = (20, 20)  # m x m (search area)
```

## Scenario List (20 total)

| Scenario | Name | Core Problem | Difficulty |
|------|------|----------|------|
| [S041](../scenarios/03_environmental_sar/S041_wildfire_boundary.md) | Wildfire Boundary Scan | Dynamic boundary tracking | ⭐⭐ |
| [S042](../scenarios/03_environmental_sar/S042_missing_person.md) | Missing Person Localization | Bayesian search | ⭐⭐⭐ |
| [S043](../scenarios/03_environmental_sar/S043_confined_space.md) | Confined Space Exploration | Narrow-gap navigation | ⭐⭐⭐ |
| [S044](../scenarios/03_environmental_sar/S044_wall_crack.md) | Wall Crack Detection | Wall-following flight trajectory | ⭐⭐ |
| [S045](../scenarios/03_environmental_sar/S045_plume_tracing.md) | Chemical Plume Tracing | Gradient ascent method | ⭐⭐⭐ |
| [S046](../scenarios/03_environmental_sar/S046_trilateration.md) | Multi-Drone 3D Trilateration | Least squares | ⭐⭐ |
| [S047](../scenarios/03_environmental_sar/S047_signal_relay.md) | Base Station Signal Relay | Optimal relay link positioning | ⭐⭐ |
| [S048](../scenarios/03_environmental_sar/S048_lawnmower.md) | Full-Area Coverage Scan | Lawnmower planning | ⭐ |
| [S049](../scenarios/03_environmental_sar/S049_dynamic_zone.md) | Dynamic Zone Search | Multi-drone zoning + reallocation | ⭐⭐⭐ |
| [S050](../scenarios/03_environmental_sar/S050_slam.md) | Swarm Cooperative Mapping | Distributed EKF-SLAM | ⭐⭐⭐⭐ |
| [S051](../scenarios/03_environmental_sar/S051_post_disaster_comm.md) | Post-Disaster Communication Network Restoration | Node coverage planning | ⭐⭐⭐ |
| [S052](../scenarios/03_environmental_sar/S052_glacier.md) | Glacier Melt Area Measurement | Orthophoto mosaic | ⭐⭐ |
| [S053](../scenarios/03_environmental_sar/S053_coral_reef.md) | Coral Reef 3D Reconstruction | Point cloud reconstruction path | ⭐⭐⭐ |
| [S054](../scenarios/03_environmental_sar/S054_minefield.md) | Minefield Detection | Safe path planning | ⭐⭐⭐ |
| [S055](../scenarios/03_environmental_sar/S055_oil_spill.md) | Coastline Oil Spill Tracking | Dynamic contamination boundary | ⭐⭐⭐ |
| [S056](../scenarios/03_environmental_sar/S056_radiation.md) | Nuclear Plant Leak Detection | Radiation hotspot localization | ⭐⭐⭐ |
| [S057](../scenarios/03_environmental_sar/S057_wildlife.md) | Wildlife Population Census | Object detection simulation | ⭐⭐ |
| [S058](../scenarios/03_environmental_sar/S058_typhoon.md) | Typhoon Eye Probing | Flight under extreme disturbances | ⭐⭐⭐⭐ |
| [S059](../scenarios/03_environmental_sar/S059_sonar_relay.md) | Underwater Target Sonar Marking | Surface cooperative localization | ⭐⭐⭐ |
| [S060](../scenarios/03_environmental_sar/S060_meteorological.md) | Multi-Drone Cooperative Meteorological Measurement | 3D meteorological profile sampling | ⭐⭐⭐ |

## Key Metrics

```python
metrics = {
    "coverage_rate":     float,   # Area coverage rate (%)
    "detection_time":    float,   # Time to first target detection (seconds)
    "localization_error": float,  # Target localization error (m)
    "redundant_coverage": float,  # Redundant scan ratio (lower is better)
    "energy_per_area":   float,   # Energy consumption per unit area
}
```

## Related Documents

- [MATH_FOUNDATIONS.md §6, §7](../MATH_FOUNDATIONS.md) — Sensor models, coverage rate
- [docs/algorithm_index.md](../docs/algorithm_index.md) — KF, SLAM, Lawnmower reference index
