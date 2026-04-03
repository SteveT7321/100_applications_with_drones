# S030 Multi-Depot Delivery — Simulation Outputs

## Overview

4 depots, 10 drones, 20 customers spread across a 1200 × 1200 m area.
Three partition strategies are compared, followed by open-route return optimisation
(each drone lands at the nearest depot after its final delivery).

## Files

| File | Description |
|------|-------------|
| `route_map_nearest_depot.png` | Top-down route map — Nearest-Depot strategy |
| `route_map_kmeans.png` | Top-down route map — K-Means (depot-seeded) strategy |
| `route_map_ao.png` | Top-down route map — Alternating Optimisation strategy |
| `strategy_comparison.png` | Bar charts: total distance and deliveries-per-depot for all 3 strategies |
| `ao_convergence.png` | AO objective (distance + balance penalty) vs iteration |
| `per_drone_metrics_ao.png` | Per-drone range used and payload per sortie (AO strategy) |

## Key Results

| Strategy | Total Distance (m) | Balance Std | Open-Route Saving (m) |
|----------|--------------------|-------------|----------------------|
| Nearest-Depot | 9839.0 | 0.71 | 0.0 |
| K-Means | 9839.0 | 0.71 | 0.0 |
| Alternating Optimisation | **8661.8** | 0.71 | **225.1** |

- AO improved total fleet distance by **12.0%** over the nearest-depot baseline.
- Open-route return (AO) saved **225.1 m** across the fleet vs forced home-base return.
- AO converged in **3 iterations**.

## Per-Drone Summary (AO Strategy)

| Drone | Origin | Return | Stops | Route (m) | Load (kg) |
|-------|--------|--------|-------|-----------|-----------|
| D1-#1 | 0 | 0 | 2 | 552.2 | 2.37 |
| D1-#2 | 0 | 2 | 2 | 906.7 | 2.97 |
| D1-#3 | 0 | 0 | 1 | 681.8 | 1.41 |
| D2-#1 | 1 | 1 | 2 | 697.4 | 2.64 |
| D2-#2 | 1 | 1 | 4 | 1827.4 | 2.41 |
| D3-#1 | 2 | 2 | 2 | 495.4 | 2.81 |
| D3-#2 | 2 | 2 | 3 | 1316.9 | 2.65 |
| D4-#1 | 3 | 3 | 2 | 915.7 | 2.67 |
| D4-#2 | 3 | 3 | 2 | 1268.3 | 1.90 |

## Parameters

| Parameter | Value |
|-----------|-------|
| Depots | 4 (corners: SW, NW, SE, NE) |
| Fleet distribution | [3, 2, 3, 2] drones per depot |
| Customers | 20, random seed 7 |
| Payload per drone | 3.0 kg |
| Max range per sortie | 3500 m |
| Cruising speed | 14 m/s |
| Balance penalty λ | 50 m²/delivery² |
| AO convergence ε | 1.0 m |
