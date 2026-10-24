# 100 Drone Simulation Applications

> A systematic study of 100 drone simulation scenarios across 5 domains, built with Python, NumPy, and Matplotlib 3D.

---

## Overview

This project implements 100 carefully selected drone simulation scenarios — from basic 1v1 pursuit to large-scale swarm coordination — building a comprehensive knowledge base of drone control algorithms and multi-agent systems.

Each scenario includes a mathematical model, Python implementation, and 3D visualization.

---

## Tech Stack

| Component | Version | Role |
|-----------|---------|------|
| Python | 3.10 | Primary language |
| NumPy | 1.26 | Numerical computation |
| SciPy | 1.14.1 | Optimization, interpolation |
| Matplotlib | 3.10 | 3D trajectory visualization |
| gym-pybullet-drones | 2.0 | Installed in the env as a reference; not used by any scenario yet |
| conda env | `drones` | Environment name |

> **Physics model**: most scenarios use a NumPy kinematic point-mass model (`src/base/drone_base.py`). Hardware-oriented scenarios (S082, S084, S090, S091) use the 6-DOF rigid-body + motor model in `src/base/rigid_body_drone.py`. No scenario has been validated against real flight data yet.

---

## Five Domains

| # | Domain | Core Problems | Scenarios |
|---|--------|--------------|-----------|
| 1 | **Pursuit & Evasion** | Differential games, optimal control | [S001–S020](#domain-1) |
| 2 | **Logistics & Delivery** | Path planning, task assignment | [S021–S040](#domain-2) |
| 3 | **Environmental & SAR** | Coverage, information gathering | [S041–S060](#domain-3) |
| 4 | **Industrial & Agriculture** | Precision control, systematic paths | [S061–S080](#domain-4) |
| 5 | **Special Ops & Entertainment** | Formation control, cooperative vision | [S081–S100](#domain-5) |

Click a scenario range to jump to its list (each list can be collapsed); each title links to its simulation results (figures, GIF, numbers).

<!-- SCENARIO-INDEX:START -->

<a id="domain-1"></a>
<details open><summary><b>1 · Pursuit & Evasion</b> — 20/20 done</summary>

| ID | Scenario (results) | Card | 3D |
|----|--------------------|------|----|
| S001 | [Basic Intercept](outputs/01_pursuit_evasion/s001_basic_intercept/README.md) | [card](scenarios/01_pursuit_evasion/S001_basic_intercept.md) |  |
| S002 | [Evasive Maneuver](outputs/01_pursuit_evasion/s002_evasive_maneuver/README.md) | [card](scenarios/01_pursuit_evasion/S002_evasive_maneuver.md) | [3D](outputs/01_pursuit_evasion/3d/s002_3d_evasive_maneuver/README.md) |
| S003 | [Low-Altitude Tracking](outputs/01_pursuit_evasion/s003_low_altitude_tracking/README.md) | [card](scenarios/01_pursuit_evasion/S003_low_altitude_tracking.md) |  |
| S004 | [Obstacle-Course Chase](outputs/01_pursuit_evasion/s004_obstacle_chase/README.md) | [card](scenarios/01_pursuit_evasion/S004_obstacle_chase.md) | [3D](outputs/01_pursuit_evasion/3d/s004_3d_obstacle_chase/README.md) |
| S005 | [Stealth Approach](outputs/01_pursuit_evasion/s005_stealth_approach/README.md) | [card](scenarios/01_pursuit_evasion/S005_stealth_approach.md) | [3D](outputs/01_pursuit_evasion/3d/s005_3d_stealth_approach/README.md) |
| S006 | [Energy Race](outputs/01_pursuit_evasion/s006_energy_race/README.md) | [card](scenarios/01_pursuit_evasion/S006_energy_race.md) | [3D](outputs/01_pursuit_evasion/3d/s006_3d_energy_race/README.md) |
| S007 | [Jamming & Blind Pursuit](outputs/01_pursuit_evasion/s007_jamming_blind_pursuit/README.md) | [card](scenarios/01_pursuit_evasion/S007_jamming_blind_pursuit.md) | [3D](outputs/01_pursuit_evasion/3d/s007_3d_jamming_blind_pursuit/README.md) |
| S008 | [Stochastic Pursuit — Kalman Filter Tracking](outputs/01_pursuit_evasion/s008_stochastic_pursuit/README.md) | [card](scenarios/01_pursuit_evasion/S008_stochastic_pursuit.md) | [3D](outputs/01_pursuit_evasion/3d/s008_3d_stochastic_pursuit/README.md) |
| S009 | [Differential Game 1v1 (Differential Game — Lion & Man)](outputs/01_pursuit_evasion/s009_differential_game/README.md) | [card](scenarios/01_pursuit_evasion/S009_differential_game.md) | [3D](outputs/01_pursuit_evasion/3d/s009_3d_differential_game/README.md) |
| S010 | [Asymmetric Speed — Bounded Arena Trapping](outputs/01_pursuit_evasion/s010_asymmetric_speed/README.md) | [card](scenarios/01_pursuit_evasion/S010_asymmetric_speed.md) | [3D](outputs/01_pursuit_evasion/3d/s010_3d_asymmetric_speed/README.md) |
| S011 | [Swarm Encirclement](outputs/01_pursuit_evasion/s011_swarm_encirclement/README.md) | [card](scenarios/01_pursuit_evasion/S011_swarm_encirclement.md) | [3D](outputs/01_pursuit_evasion/3d/s011_3d_swarm_encirclement/README.md) |
| S012 | [Relay Pursuit](outputs/01_pursuit_evasion/s012_relay_pursuit/README.md) | [card](scenarios/01_pursuit_evasion/S012_relay_pursuit.md) | [3D](outputs/01_pursuit_evasion/3d/s012_3d_relay_pursuit/README.md) |
| S013 | [Pincer Movement](outputs/01_pursuit_evasion/s013_pincer_movement/README.md) | [card](scenarios/01_pursuit_evasion/S013_pincer_movement.md) | [3D](outputs/01_pursuit_evasion/3d/s013_3d_pincer_movement/README.md) |
| S014 | [Decoy & Lure](outputs/01_pursuit_evasion/s014_decoy_lure/README.md) | [card](scenarios/01_pursuit_evasion/S014_decoy_lure.md) | [3D](outputs/01_pursuit_evasion/3d/s014_3d_decoy_lure/README.md) |
| S015 | [Relay Tracking](outputs/01_pursuit_evasion/s015_relay_tracking/README.md) | [card](scenarios/01_pursuit_evasion/S015_relay_tracking.md) | [3D](outputs/01_pursuit_evasion/3d/s015_3d_relay_tracking/README.md) |
| S016 | [Airspace Defense](outputs/01_pursuit_evasion/s016_airspace_defense/README.md) | [card](scenarios/01_pursuit_evasion/S016_airspace_defense.md) | [3D](outputs/01_pursuit_evasion/3d/s016_3d_airspace_defense/README.md) |
| S017 | [Swarm vs Swarm](outputs/01_pursuit_evasion/s017_swarm_vs_swarm/README.md) | [card](scenarios/01_pursuit_evasion/S017_swarm_vs_swarm.md) | [3D](outputs/01_pursuit_evasion/3d/s017_3d_swarm_vs_swarm/README.md) |
| S018 | [Multi-Target Intercept](outputs/01_pursuit_evasion/s018_multi_target_intercept/README.md) | [card](scenarios/01_pursuit_evasion/S018_multi_target_intercept.md) | [3D](outputs/01_pursuit_evasion/3d/s018_3d_multi_target_intercept/README.md) |
| S019 | [Dynamic Target Reassignment](outputs/01_pursuit_evasion/s019_dynamic_reassignment/README.md) | [card](scenarios/01_pursuit_evasion/S019_dynamic_reassignment.md) | [3D](outputs/01_pursuit_evasion/3d/s019_3d_dynamic_reassignment/README.md) |
| S020 | [Pursuit-Evasion Game (RL Capstone)](outputs/01_pursuit_evasion/s020_pursuit_evasion_game/README.md) | [card](scenarios/01_pursuit_evasion/S020_pursuit_evasion_game.md) | [3D](outputs/01_pursuit_evasion/3d/s020_3d_pursuit_evasion_game/README.md) |

</details>

<a id="domain-2"></a>
<details open><summary><b>2 · Logistics & Delivery</b> — 20/20 done</summary>

| ID | Scenario (results) | Card | 3D |
|----|--------------------|------|----|
| S021 | [Point Delivery](outputs/02_logistics_delivery/s021_point_delivery/README.md) | [card](scenarios/02_logistics_delivery/S021_point_delivery.md) | [3D](outputs/02_logistics_delivery/3d/s021_3d_point_delivery/README.md) |
| S022 | [Obstacle Avoidance Delivery](outputs/02_logistics_delivery/s022_obstacle_avoidance_delivery/README.md) | [card](scenarios/02_logistics_delivery/S022_obstacle_avoidance_delivery.md) | [3D](outputs/02_logistics_delivery/3d/s022_3d_obstacle_avoidance/README.md) |
| S023 | [Moving Landing Pad](outputs/02_logistics_delivery/s023_moving_landing_pad/README.md) | [card](scenarios/02_logistics_delivery/S023_moving_landing_pad.md) | [3D](outputs/02_logistics_delivery/3d/s023_3d_moving_landing_pad/README.md) |
| S024 | [Wind Compensation](outputs/02_logistics_delivery/s024_wind_compensation/README.md) | [card](scenarios/02_logistics_delivery/S024_wind_compensation.md) | [3D](outputs/02_logistics_delivery/3d/s024_3d_wind_compensation/README.md) |
| S025 | [Payload CoG Offset](outputs/02_logistics_delivery/s025_payload_cog_offset/README.md) | [card](scenarios/02_logistics_delivery/S025_payload_cog_offset.md) | [3D](outputs/02_logistics_delivery/3d/s025_3d_payload_cog/README.md) |
| S026 | [Cooperative Heavy Lift](outputs/02_logistics_delivery/s026_cooperative_heavy_lift/README.md) | [card](scenarios/02_logistics_delivery/S026_cooperative_heavy_lift.md) | [3D](outputs/02_logistics_delivery/3d/s026_3d_cooperative_heavy_lift/README.md) |
| S027 | [Aerial Refueling Relay](outputs/02_logistics_delivery/s027_aerial_refueling_relay/README.md) | [card](scenarios/02_logistics_delivery/S027_aerial_refueling_relay.md) | [3D](outputs/02_logistics_delivery/3d/s027_3d_aerial_refueling/README.md) |
| S028 | [Cargo Escort Formation](outputs/02_logistics_delivery/s028_cargo_escort_formation/README.md) | [card](scenarios/02_logistics_delivery/S028_cargo_escort_formation.md) | [3D](outputs/02_logistics_delivery/3d/s028_3d_cargo_escort/README.md) |
| S029 | [Urban Logistics Scheduling](outputs/02_logistics_delivery/s029_urban_logistics_scheduling/README.md) | [card](scenarios/02_logistics_delivery/S029_urban_logistics_scheduling.md) | [3D](outputs/02_logistics_delivery/3d/s029_3d_urban_logistics/README.md) |
| S030 | [Multi-Depot Delivery](outputs/02_logistics_delivery/s030_multi_depot_delivery/README.md) | [card](scenarios/02_logistics_delivery/S030_multi_depot_delivery.md) | [3D](outputs/02_logistics_delivery/3d/s030_3d_multi_depot/README.md) |
| S031 | [Path De-confliction](outputs/02_logistics_delivery/s031_path_deconfliction/README.md) | [card](scenarios/02_logistics_delivery/S031_path_deconfliction.md) | [3D](outputs/02_logistics_delivery/3d/s031_3d_path_deconfliction/README.md) |
| S032 | [Charging Queue](outputs/02_logistics_delivery/s032_charging_queue/README.md) | [card](scenarios/02_logistics_delivery/S032_charging_queue.md) | [3D](outputs/02_logistics_delivery/3d/s032_3d_charging_queue/README.md) |
| S033 | [Online Order Insertion](outputs/02_logistics_delivery/s033_online_order_insertion/README.md) | [card](scenarios/02_logistics_delivery/S033_online_order_insertion.md) | [3D](outputs/02_logistics_delivery/3d/s033_3d_online_order/README.md) |
| S034 | [Weather Rerouting](outputs/02_logistics_delivery/s034_weather_rerouting/README.md) | [card](scenarios/02_logistics_delivery/S034_weather_rerouting.md) | [3D](outputs/02_logistics_delivery/3d/s034_3d_weather_rerouting/README.md) |
| S035 | [UTM Simulation](outputs/02_logistics_delivery/s035_utm_simulation/README.md) | [card](scenarios/02_logistics_delivery/S035_utm_simulation.md) | [3D](outputs/02_logistics_delivery/3d/s035_3d_utm_simulation/README.md) |
| S036 | [Last-Mile Relay](outputs/02_logistics_delivery/s036_last_mile_relay/README.md) | [card](scenarios/02_logistics_delivery/S036_last_mile_relay.md) | [3D](outputs/02_logistics_delivery/3d/s036_3d_last_mile_relay/README.md) |
| S037 | [Reverse Logistics](outputs/02_logistics_delivery/s037_reverse_logistics/README.md) | [card](scenarios/02_logistics_delivery/S037_reverse_logistics.md) | [3D](outputs/02_logistics_delivery/3d/s037_3d_reverse_logistics/README.md) |
| S038 | [Disaster Relief Drop](outputs/02_logistics_delivery/s038_disaster_relief_drop/README.md) | [card](scenarios/02_logistics_delivery/S038_disaster_relief_drop.md) | [3D](outputs/02_logistics_delivery/3d/s038_3d_disaster_relief/README.md) |
| S039 | [Offshore Platform Exchange](outputs/02_logistics_delivery/s039_offshore_platform_exchange/README.md) | [card](scenarios/02_logistics_delivery/S039_offshore_platform_exchange.md) | [3D](outputs/02_logistics_delivery/3d/s039_3d_offshore_platform/README.md) |
| S040 | [Fleet Load Balancing](outputs/02_logistics_delivery/s040_fleet_load_balancing/README.md) | [card](scenarios/02_logistics_delivery/S040_fleet_load_balancing.md) | [3D](outputs/02_logistics_delivery/3d/s040_3d_fleet_balancing/README.md) |

</details>

<a id="domain-3"></a>
<details open><summary><b>3 · Environmental & SAR</b> — 20/20 done</summary>

| ID | Scenario (results) | Card | 3D |
|----|--------------------|------|----|
| S041 | [Wildfire Boundary Scan](outputs/03_environmental_sar/s041_wildfire_boundary/README.md) | [card](scenarios/03_environmental_sar/S041_wildfire_boundary.md) | [3D](outputs/03_environmental_sar/3d/s041_3d_wildfire/README.md) |
| S042 | [Missing Person Localization](outputs/03_environmental_sar/s042_missing_person/README.md) | [card](scenarios/03_environmental_sar/S042_missing_person.md) | [3D](outputs/03_environmental_sar/3d/s042_3d_missing_person/README.md) |
| S043 | [Confined Space Exploration](outputs/03_environmental_sar/s043_confined_space/README.md) | [card](scenarios/03_environmental_sar/S043_confined_space.md) | [3D](outputs/03_environmental_sar/3d/s043_3d_confined_space/README.md) |
| S044 | [Wall Crack Inspection](outputs/03_environmental_sar/s044_wall_crack/README.md) | [card](scenarios/03_environmental_sar/S044_wall_crack.md) | [3D](outputs/03_environmental_sar/3d/s044_3d_wall_crack/README.md) |
| S045 | [Chemical Plume Tracing](outputs/03_environmental_sar/s045_plume_tracing/README.md) | [card](scenarios/03_environmental_sar/S045_plume_tracing.md) | [3D](outputs/03_environmental_sar/3d/s045_3d_plume_tracing/README.md) |
| S046 | [Multi-Drone 3D Trilateration](outputs/03_environmental_sar/s046_trilateration/README.md) | [card](scenarios/03_environmental_sar/S046_trilateration.md) | [3D](outputs/03_environmental_sar/3d/s046_3d_trilateration/README.md) |
| S047 | [Base Station Signal Relay](outputs/03_environmental_sar/s047_signal_relay/README.md) | [card](scenarios/03_environmental_sar/S047_signal_relay.md) | [3D](outputs/03_environmental_sar/3d/s047_3d_signal_relay/README.md) |
| S048 | [Full-Area Coverage Scan (Lawnmower)](outputs/03_environmental_sar/s048_lawnmower/README.md) | [card](scenarios/03_environmental_sar/S048_lawnmower.md) | [3D](outputs/03_environmental_sar/3d/s048_3d_lawnmower/README.md) |
| S049 | [Dynamic Zone Search](outputs/03_environmental_sar/s049_dynamic_zone/README.md) | [card](scenarios/03_environmental_sar/S049_dynamic_zone.md) | [3D](outputs/03_environmental_sar/3d/s049_3d_dynamic_zone/README.md) |
| S050 | [Swarm Cooperative Mapping (Distributed EKF-SLAM)](outputs/03_environmental_sar/s050_slam/README.md) | [card](scenarios/03_environmental_sar/S050_slam.md) | [3D](outputs/03_environmental_sar/3d/s050_3d_slam/README.md) |
| S051 | [Post-Disaster Communication Network Restoration](outputs/03_environmental_sar/s051_post_disaster_comm/README.md) | [card](scenarios/03_environmental_sar/S051_post_disaster_comm.md) |  |
| S052 | [Glacier Melt Area Measurement](outputs/03_environmental_sar/s052_glacier/README.md) | [card](scenarios/03_environmental_sar/S052_glacier.md) |  |
| S053 | [Coral Reef 3D Reconstruction](outputs/03_environmental_sar/s053_coral_reef/README.md) | [card](scenarios/03_environmental_sar/S053_coral_reef.md) |  |
| S054 | [Minefield Detection](outputs/03_environmental_sar/s054_minefield/README.md) | [card](scenarios/03_environmental_sar/S054_minefield.md) |  |
| S055 | [Coastline Oil Spill Tracking](outputs/03_environmental_sar/s055_oil_spill/README.md) | [card](scenarios/03_environmental_sar/S055_oil_spill.md) |  |
| S056 | [Nuclear Plant Leak Detection (Radiation Hotspot Localization)](outputs/03_environmental_sar/s056_radiation/README.md) | [card](scenarios/03_environmental_sar/S056_radiation.md) |  |
| S057 | [Wildlife Population Census](outputs/03_environmental_sar/s057_wildlife/README.md) | [card](scenarios/03_environmental_sar/S057_wildlife.md) |  |
| S058 | [Typhoon Eye Probing](outputs/03_environmental_sar/s058_typhoon/README.md) | [card](scenarios/03_environmental_sar/S058_typhoon.md) |  |
| S059 | [Underwater Target Sonar Marking (Surface Cooperative Localization)](outputs/03_environmental_sar/s059_sonar_relay/README.md) | [card](scenarios/03_environmental_sar/S059_sonar_relay.md) |  |
| S060 | [Multi-Drone Cooperative Meteorological Measurement](outputs/03_environmental_sar/s060_meteorological/README.md) | [card](scenarios/03_environmental_sar/S060_meteorological.md) |  |

</details>

<a id="domain-4"></a>
<details open><summary><b>4 · Industrial & Agriculture</b> — 20/20 done</summary>

| ID | Scenario (results) | Card | 3D |
|----|--------------------|------|----|
| S061 | [Power Line Inspection](outputs/04_industrial_agriculture/s061_power_line/README.md) | [card](scenarios/04_industrial_agriculture/S061_power_line.md) |  |
| S062 | [Wind Turbine Blade Inspection](outputs/04_industrial_agriculture/s062_wind_turbine/README.md) | [card](scenarios/04_industrial_agriculture/S062_wind_turbine.md) |  |
| S063 | [Single-Plant Precision Spraying](outputs/04_industrial_agriculture/s063_precision_spraying/README.md) | [card](scenarios/04_industrial_agriculture/S063_precision_spraying.md) |  |
| S064 | [Greenhouse Interior Precision Flight](outputs/04_industrial_agriculture/s064_greenhouse/README.md) | [card](scenarios/04_industrial_agriculture/S064_greenhouse.md) |  |
| S065 | [Building 3D Modeling — Multi-Angle Scan Path](outputs/04_industrial_agriculture/s065_3d_scan_path/README.md) | [card](scenarios/04_industrial_agriculture/S065_3d_scan_path.md) |  |
| S066 | [Cooperative Crane Simulation](outputs/04_industrial_agriculture/s066_cooperative_crane/README.md) | [card](scenarios/04_industrial_agriculture/S066_cooperative_crane.md) |  |
| S067 | [Spray Coverage Overlap Optimization](outputs/04_industrial_agriculture/s067_spray_overlap/README.md) | [card](scenarios/04_industrial_agriculture/S067_spray_overlap.md) |  |
| S068 | [Large-Scale Farmland Cooperative Spraying](outputs/04_industrial_agriculture/s068_large_field_spray/README.md) | [card](scenarios/04_industrial_agriculture/S068_large_field_spray.md) |  |
| S069 | [Automated Warehouse Inventory](outputs/04_industrial_agriculture/s069_warehouse_inventory/README.md) | [card](scenarios/04_industrial_agriculture/S069_warehouse_inventory.md) |  |
| S070 | [Swarm Weeding Task](outputs/04_industrial_agriculture/s070_swarm_weeding/README.md) | [card](scenarios/04_industrial_agriculture/S070_swarm_weeding.md) |  |
| S071 | [Bridge Underside Structural Inspection](outputs/04_industrial_agriculture/s071_bridge_inspection/README.md) | [card](scenarios/04_industrial_agriculture/S071_bridge_inspection.md) |  |
| S072 | [Oil Pipeline Leak Detection](outputs/04_industrial_agriculture/s072_pipeline_leak/README.md) | [card](scenarios/04_industrial_agriculture/S072_pipeline_leak.md) |  |
| S073 | [Solar Panel Thermal Imaging Inspection](outputs/04_industrial_agriculture/s073_solar_thermal/README.md) | [card](scenarios/04_industrial_agriculture/S073_solar_thermal.md) |  |
| S074 | [Mine 3D Mapping](outputs/04_industrial_agriculture/s074_mine_mapping/README.md) | [card](scenarios/04_industrial_agriculture/S074_mine_mapping.md) |  |
| S075 | [Port Container Yard Inventory](outputs/04_industrial_agriculture/s075_container_yard/README.md) | [card](scenarios/04_industrial_agriculture/S075_container_yard.md) |  |
| S076 | [Extreme Heat Cooling Strategy](outputs/04_industrial_agriculture/s076_thermal_management/README.md) | [card](scenarios/04_industrial_agriculture/S076_thermal_management.md) |  |
| S077 | [Precision Pollination Flight Pattern](outputs/04_industrial_agriculture/s077_pollination/README.md) | [card](scenarios/04_industrial_agriculture/S077_pollination.md) |  |
| S078 | [Harvester Cooperative Guidance](outputs/04_industrial_agriculture/s078_harvester_guidance/README.md) | [card](scenarios/04_industrial_agriculture/S078_harvester_guidance.md) |  |
| S079 | [Offshore Wind Farm Installation Assistance](outputs/04_industrial_agriculture/s079_offshore_wind/README.md) | [card](scenarios/04_industrial_agriculture/S079_offshore_wind.md) |  |
| S080 | [Underground Pipe CCTV Replacement](outputs/04_industrial_agriculture/s080_underground_pipe/README.md) | [card](scenarios/04_industrial_agriculture/S080_underground_pipe.md) |  |

</details>

<a id="domain-5"></a>
<details open><summary><b>5 · Special Ops & Entertainment</b> — 20/20 done</summary>

| ID | Scenario (results) | Card | 3D |
|----|--------------------|------|----|
| S081 | [Selfie Follow Mode](outputs/05_special_entertainment/s081_selfie_follow/README.md) | [card](scenarios/05_special_entertainment/S081_selfie_follow.md) | [3D](outputs/05_special_entertainment/3d/s081_3d_selfie_follow/README.md) |
| S082 | [FPV Racing Circuit](outputs/05_special_entertainment/s082_fpv_racing/README.md) | [card](scenarios/05_special_entertainment/S082_fpv_racing.md) |  |
| S083 | [Light Show Single-Drone Test](outputs/05_special_entertainment/s083_light_show_single/README.md) | [card](scenarios/05_special_entertainment/S083_light_show_single.md) | [3D](outputs/05_special_entertainment/3d/s083_3d_light_show/README.md) |
| S084 | [Extreme Weather Wind Resistance Test](outputs/05_special_entertainment/s084_wind_endurance/README.md) | [card](scenarios/05_special_entertainment/S084_wind_endurance.md) |  |
| S085 | [Drone Light Matrix Positioning](outputs/05_special_entertainment/s085_light_matrix/README.md) | [card](scenarios/05_special_entertainment/S085_light_matrix.md) |  |
| S086 | [Cooperative Multi-Angle Filming](outputs/05_special_entertainment/s086_multi_angle_cinema/README.md) | [card](scenarios/05_special_entertainment/S086_multi_angle_cinema.md) | [3D](outputs/05_special_entertainment/3d/s086_3d_multi_angle_cinema/README.md) |
| S087 | [Aerial Light Show Text Display](outputs/05_special_entertainment/s087_led_text/README.md) | [card](scenarios/05_special_entertainment/S087_led_text.md) |  |
| S088 | [Formation Geometric Shape Morphing](outputs/05_special_entertainment/s088_formation_morphing/README.md) | [card](scenarios/05_special_entertainment/S088_formation_morphing.md) |  |
| S089 | [Large-Scale Swarm Collision Avoidance Test](outputs/05_special_entertainment/s089_collision_avoidance/README.md) | [card](scenarios/05_special_entertainment/S089_collision_avoidance.md) |  |
| S090 | [Racing Drone Optimal Path](outputs/05_special_entertainment/s090_racing_optimal/README.md) | [card](scenarios/05_special_entertainment/S090_racing_optimal.md) |  |
| S091 | [Aerial Acrobatic Maneuvers](outputs/05_special_entertainment/s091_acrobatics/README.md) | [card](scenarios/05_special_entertainment/S091_acrobatics.md) |  |
| S092 | [Movie Car Chase Scene](outputs/05_special_entertainment/s092_movie_chase/README.md) | [card](scenarios/05_special_entertainment/S092_movie_chase.md) | [3D](outputs/05_special_entertainment/3d/s092_3d_movie_chase/README.md) |
| S093 | [Earthquake Rubble Search and Rescue](outputs/05_special_entertainment/s093_rubble_sar/README.md) | [card](scenarios/05_special_entertainment/S093_rubble_sar.md) |  |
| S094 | [Aerial Ball Catching](outputs/05_special_entertainment/s094_ball_catching/README.md) | [card](scenarios/05_special_entertainment/S094_ball_catching.md) |  |
| S095 | [Water Surface Landing Simulation](outputs/05_special_entertainment/s095_water_landing/README.md) | [card](scenarios/05_special_entertainment/S095_water_landing.md) |  |
| S096 | [Drone Relay Race](outputs/05_special_entertainment/s096_relay_race/README.md) | [card](scenarios/05_special_entertainment/S096_relay_race.md) |  |
| S097 | [Aerial Puzzle Assembly](outputs/05_special_entertainment/s097_aerial_puzzle/README.md) | [card](scenarios/05_special_entertainment/S097_aerial_puzzle.md) |  |
| S098 | [Swarm Synchronized Dance](outputs/05_special_entertainment/s098_synchronized_dance/README.md) | [card](scenarios/05_special_entertainment/S098_synchronized_dance.md) |  |
| S099 | [Cross-Obstacle Relay Pass](outputs/05_special_entertainment/s099_obstacle_relay/README.md) | [card](scenarios/05_special_entertainment/S099_obstacle_relay.md) |  |
| S100 | [Ultimate Grand Challenge — Multi-Domain Integration](outputs/05_special_entertainment/s100_grand_challenge/README.md) | [card](scenarios/05_special_entertainment/S100_grand_challenge.md) |  |

</details>

<!-- SCENARIO-INDEX:END -->

---

## Quick Start

```bash
# 1. Clone the repository
git clone https://github.com/SteveT7321/100_applications_with_drones.git
cd drones

# 2. Create and activate conda environment
conda create -n drones python=3.10
conda activate drones

# 2. Install dependencies
pip install numpy==1.26.4 scipy==1.14.1 matplotlib==3.10

# 3. Run the first scenario
python src/01_pursuit_evasion/s001_basic_intercept.py
```

See [SETUP.md](SETUP.md) for full installation instructions.

---

## Repository Structure

```
drones/
├── src/
│   ├── base/
│   │   └── drone_base.py              # Shared point-mass drone base class
│   └── pursuit/
│       └── s001_basic_intercept.py
├── scenarios/                         # Scenario cards (math model + parameters)
│   └── pursuit/
│       ├── S001_basic_intercept.md
│       └── ...
├── outputs/                           # Simulation outputs (PNG, GIF)
│   └── s001_basic_intercept/
├── domains/                           # Domain overviews with theory background
│   ├── 01_pursuit_evasion/README.md
│   └── ...
├── docs/                              # Technical reference docs
├── MATH_FOUNDATIONS.md                # Shared mathematical foundations
├── PROGRESS.md                        # Progress tracker (100 scenarios)
├── CITATION.cff                       # Citation metadata ("Cite this repository")
└── SETUP.md                           # Environment setup guide
```

---

## Progress

| Domain | Done | Total |
|--------|------|-------|
| Pursuit & Evasion | 20 | 20 |
| Logistics & Delivery | 20 | 20 |
| Environmental & SAR | 20 | 20 |
| Industrial & Agriculture | 20 | 20 |
| Special Ops & Entertainment | 20 | 20 |
| **Total** | **100** | **100** |

> **3D Upgrades**: 52 true-3D variants are available under `src/<domain>/3d/` and `outputs/<domain>/3d/` — 18 in Pursuit & Evasion (S002, S004–S020; S001 and S003 are already 3D), 20 in Logistics & Delivery (S021–S040), 10 in Environmental & SAR (S041–S050) and 4 in Special Ops & Entertainment (S081, S083, S086, S092). All were reviewed against [docs/REVIEW_STANDARD.md](docs/REVIEW_STANDARD.md); the S001–S050 reviews are summarised in [docs/review_3d_variants.md](docs/review_3d_variants.md). The S051–S100 variants are built only where the base scenario holds altitude fixed, and each one prints a fixed-altitude 2D baseline.

→ Full tracker: [PROGRESS.md](PROGRESS.md)

---

## Citation

If you use this project in your work, please cite it. The citation metadata is in [CITATION.cff](CITATION.cff), and GitHub's **"Cite this repository"** button in the sidebar generates APA and BibTeX from it.

**BibTeX**

```bibtex
@misc{tsai2026drones,
  author       = {Tsai, Yun-Hsiu},
  title        = {100 Drone Simulation Applications},
  year         = {2026},
  publisher    = {GitHub},
  howpublished = {\url{https://github.com/SteveT7321/100_applications_with_drones}},
  note         = {Software}
}
```

**APA (7th ed.)**

> Tsai, Y.-H. (2026). *100 Drone Simulation Applications* [Computer software]. GitHub. https://github.com/SteveT7321/100_applications_with_drones

**IEEE**

> Y.-H. Tsai, "100 Drone Simulation Applications," GitHub, 2026. [Online]. Available: https://github.com/SteveT7321/100_applications_with_drones

**MLA (9th ed.)**

> Tsai, Yun-Hsiu. *100 Drone Simulation Applications*. GitHub, 2026, github.com/SteveT7321/100_applications_with_drones.

**Chicago (author-date)**

> Tsai, Yun-Hsiu. 2026. "100 Drone Simulation Applications." Software. GitHub. https://github.com/SteveT7321/100_applications_with_drones.

---

## References

- Shneydor, N.A. (1998). *Missile Guidance and Pursuit*. Horwood.
- Isaacs, R. (1965). *Differential Games*. Wiley.
- Panerati, J., et al. (2021). [Learning to fly — a gym environment with PyBullet physics](https://arxiv.org/abs/2103.02142). IROS 2021.
- Mahony, R., et al. (2012). Multirotor aerial vehicles: Modeling, estimation, and control. *IEEE Robotics & Automation Magazine*.
