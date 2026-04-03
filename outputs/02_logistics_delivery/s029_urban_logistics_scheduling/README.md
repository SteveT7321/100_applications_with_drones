# S029 Urban Logistics Scheduling — Output Files

## Scenario
Multi-depot (3 depots) multi-drone (6 drones, 2 per depot) VRP with time windows
over a 1000 × 1000 m urban grid with 12 customer delivery points.

## Strategies Compared
| Strategy | Description |
|---|---|
| Greedy Nearest-Neighbour | Each customer assigned to nearest depot; NN tour built per drone |
| Clarke-Wright Savings | Routes merged greedily by savings criterion; capacity + range checked |

## Key Results
| Metric | Greedy NN | Clarke-Wright |
|---|---|---|
| Total flight distance | 4366.5 m | 4311.7 m |
| Total weighted tardiness | 0.00 s | 0.00 s |
| Composite objective (α=0.7) | 1309.95 | 1293.50 |
| Customers served | 12 / 12 | 12 / 12 |
| Active routes | 4 | 4 |

Clarke-Wright reduced total distance by **1.3 %** over the greedy baseline.
All deliveries met their time windows (zero tardiness) under both strategies.

## Output Files
| File | Description |
|---|---|
| `route_map_greedy_nn.png` | 2D top-down route map — Greedy NN |
| `route_map_clarke_wright.png` | 2D top-down route map — Clarke-Wright |
| `gantt_greedy_nn.png` | Drone Gantt chart with time-window markers — Greedy NN |
| `gantt_clarke_wright.png` | Drone Gantt chart with time-window markers — Clarke-Wright |
| `comparison_metrics.png` | Side-by-side bar charts: tardiness, distance, objective |
| `delivery_table_clarke_wright.png` | Per-customer delivery time, window, tardiness table |

## Parameters
- Depots: 3 at (100,100), (500,900), (900,200) m
- Drones: 6 total (2 per depot), speed 15 m/s, capacity 3 kg, range 4000 m
- Service time per stop: 10 s
- Time windows: earliest 0–120 s, width 60–180 s
- Demand per customer: 0.3–1.5 kg
- Tardiness weight α = 0.7
