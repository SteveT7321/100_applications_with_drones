# S043 3D Upgrade — Confined Space Exploration

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: A rescue drone explores a four-floor collapsed building with no prior map. The building is 30 × 30 × 10 m, held as 60 × 60 × 20 voxels of 0.5 m.
- **Floor plans**: every floor reuses the S043 plan (outer walls, internal walls with doorways, debris columns and the fallen beam), extruded over the floor's free layers. Floors F1–F3 also get six random 2 × 2-cell rubble columns each (seed 43).
- **Slabs**: layer 0 is the ground, layer 19 the roof, and layers 5, 10 and 15 are floor slabs. Each slab is solid except for one 2 × 2 m stairwell opening. F0–F2 have 2.0 m of free height and F3 has 1.5 m.
- **Hazards**:
  - **Low-ceiling room** in F0 (x 14–24 m, y 2–9 m): only 1.0 m of headroom remains, so the drone must fly at the lowest layer.
  - **Crawl space** in F0 (x 1–5 m, y 26–29 m): 0.5 m of headroom, which is below $h_{min}$, so it is impassable.

The drone starts at (2, 2, 0.5) m.

**Strategies** (same building, start and lidar-noise stream):

| Strategy | Description |
|----------|-------------|
| **2D-slice baseline** | Nodes and frontiers restricted to the start layer (z = 0.75 m held fixed). It never changes floor |
| **3D frontier-nearest** | Nearest frontier node by travel time, on any floor |
| **3D frontier + vertical-passage planner** | Finish the current floor first. When the floor has no frontier left, go to the nearest frontier on another floor, with a −2.0 s cost bonus for frontiers in detected stairwells |

**Objective (card)**: maximise the fraction of 3D reachable free voxels mapped, never enter a voxel with p > 0.65, and never pass a gap narrower than $g_{min}$ or lower than $h_{min}$.

---

## Mathematical Model

**Voxel map**: log-odds, +0.85 for a hit and −0.40 for a pass-through, clamped to [−2.0, 3.5]. A voxel is free if p < 0.35, occupied if p > 0.65, and unknown otherwise.

**Lidar**: 128 beams, with 92 Fibonacci directions over the full sphere and a 36-beam horizontal ring. Range is 2.0 m and range noise is $\mathcal N(0, 0.05^2)$. Each beam is ray-marched in 0.05 m steps against the ground truth. The voxels before the noisy range get one free update per beam; the hit voxel gets an occupied update when the range is < 2 m.

**Body and passability**: a node (i, j, k) is traversable when voxel k (the body) and voxel k + 1 (headroom) are both free. That gives 1.0 m ≥ $h_{min}$ = 0.6 m of vertical clearance, and a 0.5 m voxel is wider than $g_{min}$ = 0.4 m. The ground-truth reachable set $\mathcal C_{reach}$ is every voxel of the nodes reachable from the start (6-connected).

**Coverage**:
$$\text{Coverage}_{3D} = |\{v \in \mathcal C_{reach}: p(v) < 0.35\}| / |\mathcal C_{reach}|,$$
also reported per floor, where floor k holds layers 5k … 5k + 4.

**Frontier node**: a known-free traversable node whose body or headroom voxel borders an unknown voxel (6-neighbourhood).

**Planning**: Dijkstra (scipy csgraph) over the known-free traversable nodes. Edge costs are travel times: 1.0 s per horizontal voxel at 0.5 m/s and 1.67 s per vertical voxel at $v_{z,max}$ = 0.3 m/s. The drone replans when the goal stops being a frontier or the next node is no longer free in the map.

**Motion**: voxel-centre to voxel-centre, one axis at a time, at 0.5 m/s horizontally and 0.3 m/s vertically. Every entered voxel is asserted free in the ground truth and below p = 0.65 in the map.

**Stairwell detection**: a known-free connected region in a slab layer with ≥ 4 voxels (1 m² ≥ $g_{min}^2$) and a known-free vertical run of ≥ 2.5 m through its centroid (the card's span criterion).

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Environment | 30 × 30 × 10 m, 60 × 60 × 20 voxels of 0.5 m |
| Floors | F0–F3, slabs at z = 2.5, 5.0, 7.5 m; stairwells S1 (x 15–17, y 12.5–14.5 m), S2 (x 25–27, y 21–23 m), S3 (x 2.5–4.5, y 2.5–4.5 m), each 4.0 m² |
| Lidar | 128 beams (92 Fibonacci full sphere + 36 ring), $r_s$ = 2.0 m, σ_r = 0.05 m, 0.05 m march step |
| Log-odds | +0.85 / −0.40, clamp [−2.0, 3.5] |
| Thresholds | free < 0.35, occupied > 0.65 |
| Body / gap / headroom | $r_d$ = 0.15 m, $h_d$ = 0.20 m; $g_{min}$ = 0.40 m; $h_{min}$ = 0.60 m → 2 free voxels |
| Speeds | 0.5 m/s horizontal, $v_{z,max}$ = 0.3 m/s |
| Δt | 0.1 s |
| Horizon | 600 s (card); extended 3600 s for every 3D run |
| Stairwell bonus $\Delta p_{stair}$ | 2.0 s off the travel-time cost |
| Stairwell span criterion | ≥ 2.5 m |
| Start | (2.0, 2.0, 0.5) m → node centre (2.25, 2.25, 0.75) m |
| Lidar seeds | 0–9 (`default_rng([seed, 0])`). Every 3D run is flown to 3600 s and its first 600 s are its card-horizon result; the slice baseline is flown to 3600 s for seed 0 and to 600 s for seeds 1–9 |
| Parallel processes | 8 (runs are independent; the results do not depend on the number of processes) |
| Rubble seed | 43 |
| Animation | vertical-passage planner, seed 0, one frame per 30 s, 121 frames |

---

## Implementation

```
src/03_environmental_sar/3d/s043_3d_confined_space.py
```

```bash
conda activate drones
python src/03_environmental_sar/3d/s043_3d_confined_space.py
```

The script imports the fixed 2D S043 module for the floor plan and the log-odds constants. Runtime ≈ 3–4 min (173 s and 231 s in the two checks): 21 runs of 3600 s and 9 runs of 600 s in 8 parallel processes, plus the GIF. Two runs printed byte-identical output and wrote byte-identical PNG and GIF files.

---

## Results

### Building checks

| Quantity | Value |
|----------|-------|
| Occupied voxels | 37.0 % |
| Reachable nodes / $\mathcal C_{reach}$ voxels | 32735 / 44694 (F0 11604, F1 12240, F2 12260, F3 8590) |
| Reachable from the start layer alone | 3032 nodes = 13.6 % of $\mathcal C_{reach}$ |
| Low-ceiling room | traversable only at layer 1 (262 nodes; 0 at layer 2) |
| Crawl space | 48 free voxels with 0.5 m headroom, 0 reachable nodes |
| Card horizon | 600 s × 0.5 m/s = at most 300 m of flight |

### Card horizon, 600 s (10 lidar seeds)

| Strategy | Coverage at 600 s, mean (min–max) | Seed 0: total / F0 | Floor changes by 600 s |
|----------|-----------------------------------|--------------------|------------------------|
| 2D-slice baseline | 15.2 % (15.2–15.2) | 15.2 / 58.5 % | 0 in every seed |
| 3D frontier-nearest | 19.4 % (17.2–20.5) | 20.4 / 78.3 % | 1 in seed 3 (at 599 s), 0 otherwise |
| 3D + vertical-passage planner | 19.3 % (17.0–20.5) | 20.4 / 78.3 % | 0 in every seed |

Head-to-head:
- Frontier-nearest beats the slice baseline in 10/10 seeds (+4.2 points on average), and so does the vertical-passage planner (10/10, +4.1 points).
- The two 3D strategies are identical in 9/10 seeds (mean difference −0.0 points). Until the first floor change both pick the same nearest frontier on F0, so at 600 s they can only differ in seed 3, where frontier-nearest climbs at 599 s.

### Extended horizon, 3600 s (3D strategies, 10 seeds)

| Seed | Frontier-nearest at 1200 / 1800 / 2400 / 3600 s | Vertical-passage planner at 1200 / 1800 / 2400 / 3600 s | Time to 50 %: nearest / VP | First floor change: nearest / VP |
|------|----------------------------------------|----------------------------------------|----------------------------|---------------------|
| 0 | 38.0 / 59.2 / 73.8 / 90.0 % | 26.0 / 38.2 / 52.0 / 76.2 % | 1567 / 2309 s | 666 / 1319 s |
| 1 | 32.5 / 52.2 / 69.8 / 89.3 % | 26.0 / 40.7 / 52.0 / 80.1 % | 1750 / 2181 s | 867 / 1315 s |
| 2 | 38.4 / 58.3 / 72.2 / 89.9 % | 26.1 / 39.8 / 52.6 / 77.1 % | 1522 / 2203 s | 622 / 1334 s |
| 3 | 32.2 / 48.3 / 63.7 / 76.8 % | 24.3 / 30.0 / 48.2 / 68.5 % | 1863 / 2725 s | 599 / 1690 s |
| 4 | 31.7 / 50.7 / 68.8 / 87.3 % | 25.8 / 40.0 / 51.9 / 77.3 % | 1780 / 2294 s | 899 / 1337 s |
| 5 | 38.7 / 59.3 / 74.7 / 91.7 % | 26.1 / 37.1 / 51.2 / 74.3 % | 1528 / 2346 s | 620 / 1304 s |
| 6 | 36.6 / 53.3 / 69.6 / 77.2 % | 26.0 / 35.8 / 48.4 / 76.5 % | 1726 / 2451 s | 664 / 1472 s |
| 7 | 30.7 / 52.5 / 70.5 / 90.9 % | 25.8 / 38.1 / 52.2 / 80.0 % | 1744 / 2152 s | 929 / 1317 s |
| 8 | 38.6 / 59.6 / 74.3 / 89.7 % | 26.1 / 40.8 / 52.7 / 77.7 % | 1523 / 2171 s | 620 / 1316 s |
| 9 | 41.0 / 61.4 / 75.3 / 90.9 % | 26.1 / 41.0 / 51.3 / 77.4 % | 1463 / 2271 s | 624 / 1328 s |

| Time | Frontier-nearest, mean ± std [min, max] | Vertical-passage planner, mean ± std [min, max] | Nearest higher | Difference, mean [min, max] |
|------|-----------------------------------------|--------------------------------------------------|----------------|------------------------------|
| 1200 s | 35.8 ± 3.5 % [30.7, 41.0] | 25.8 ± 0.5 % [24.3, 26.1] | 10/10 | +10.0 pts [+4.9, +14.9] |
| 1800 s | 55.5 ± 4.3 % [48.3, 61.4] | 38.2 ± 3.2 % [30.0, 41.0] | 10/10 | +17.3 pts [+10.7, +22.2] |
| 2400 s | 71.3 ± 3.4 % [63.7, 75.3] | 51.3 ± 1.5 % [48.2, 52.7] | 10/10 | +20.0 pts [+15.5, +23.9] |
| 3600 s | 87.4 ± 5.3 % [76.8, 91.7] | 76.5 ± 3.1 % [68.5, 80.1] | 10/10 | +10.9 pts [+0.6, +17.4] |

Time to 50 % coverage is 1646.6 s on average for frontier-nearest (range 1463.0–1863.0 s) and 2310.3 s for the vertical-passage planner (range 2152.0–2725.0 s). Frontier-nearest is earlier in 10/10 seeds.

### Seed-0 details, 3600 s

| Strategy | Coverage at 600 / 1200 / 1800 / 2400 / 3600 s | Final F0 / F1 / F2 / F3 | Time to 25 / 50 / 75 % | Path | Plans | Frontiers exhausted |
|----------|-------------------------------------------|-------------------------|-------------------------|------|-------|---------------------|
| 2D-slice baseline | 15.2 / 25.9 / 25.9 / 25.9 / 25.9 % | 99.9 / 0.0 / 0.0 / 0.0 % | 1095.0 s / — / — | 590.0 m | 880 | 1132.0 s |
| 3D frontier-nearest | 20.4 / 38.0 / 59.2 / 73.8 / 90.0 % | 80.0 / 85.2 / 97.2 / 100.0 % | 785.0 / 1567.0 / 2510.0 s | 1788.1 m | 2646 | never |
| 3D + vertical-passage planner | 20.4 / 26.0 / 38.2 / 52.0 / 76.2 % | 100.0 / 100.0 / 83.1 / 0.6 % | 959.0 / 2309.0 / 3579.0 s | 1791.6 m | 2447 | never |

**Floor changes**:
- Frontier-nearest: 666 s F0→F1, 1433 s F1→F2, 1994 s F2→F3, 3105 s F3→F2, 3600 s F2→F1.
- Vertical-passage planner: 1319 s F0→F1, 2650 s F1→F2.

**Vertical-passage detection log** (first detection; cross-section and span as known at that moment):

| Run | Slab | Centroid | Free cross-section | Known vertical span | Detected at |
|-----|------|----------|--------------------|---------------------|-------------|
| Frontier-nearest | z = 2.5 m (S1) | (16.50, 13.50) m | 2.00 m² | 2.5 m | 144.0 s |
| Frontier-nearest | z = 5.0 m (S2) | (25.97, 22.42) m | 2.25 m² | 2.5 m | 1280.0 s |
| Frontier-nearest | z = 7.5 m (S3) | (2.85, 3.35) m | 1.25 m² | 3.0 m | 1562.0 s |
| Vertical-passage planner | z = 2.5 m (S1) | (16.50, 13.50) m | 2.00 m² | 2.5 m | 144.0 s |
| Vertical-passage planner | z = 5.0 m (S2) | (26.75, 22.00) m | 1.00 m² | 2.5 m | 2308.0 s |
| Vertical-passage planner | z = 7.5 m (S3) | (3.50, 3.25) m | 1.00 m² | 2.5 m | 2777.0 s |

All three detected centroids lie inside the true openings.

### Key findings

- **The card's 600 s horizon is too short to test the third dimension.** At 0.5 m/s the drone can fly at most 300 m. In 10 seeds only one 3D run leaves the ground floor before 600 s (seed 3, at 599 s). Within the card horizon the 3D strategies still beat the 2D slice in 10/10 seeds: 19.4 and 19.3 % against 15.2 % on average. They do this because they also move between the three usable layers of F0; in seed 0, F0 is 78.3 % mapped at 600 s against 58.5 % for the slice. The slice can only reach 13.6 % of $\mathcal C_{reach}$ as nodes, but its spherical lidar maps the headroom above it. Its ceiling is therefore 25.9 % (F0 at 99.9 %), reached when its frontiers run out at 1132.0 s (seed 0). At 600 s the two 3D strategies cannot be told apart: they are identical in 9/10 seeds, because both choose the nearest F0 frontier until a floor change.
- **Over the 3600 s extended horizon the card's vertical-passage planner is worse than plain frontier-nearest, in every seed.** Frontier-nearest has higher coverage in 10/10 seeds at 1200, 1800, 2400 and 3600 s. The mean gap is +10.0, +17.3, +20.0 and +10.9 points; at 3600 s it is 87.4 ± 5.3 % against 76.5 ± 3.1 %. Frontier-nearest also reaches 50 % first in 10/10 seeds (1646.6 vs 2310.3 s on average). The size of the final gap varies a lot between seeds, from +0.6 points (seed 6) to +17.4 points (seed 3), but its sign never changes. The reason is visible in the first floor change: frontier-nearest climbs to the empty F1 after 599–929 s, while finish-floor-first keeps the drone on F0 until 1304–1690 s to map its last frontiers. In seed 0, total coverage rises by only 5.6 points between 600 and 1200 s for the vertical-passage planner (20.4 → 26.0 %), against 17.6 points for frontier-nearest (20.4 → 38.0 %). Frontier-nearest never goes back for the last 20 % of F0 in seed 0, but it gains far more on F1–F3 (85.2 / 97.2 / 100.0 %). The −2 s stairwell bonus has little room to matter. It is only applied once the current floor has no frontier left, and at that point every remaining frontier lies beyond a stairwell anyway. This ranking holds for this building, this sensor range and these speeds; it is not a general result.
- **The vertical-passage detector works.** It finds all three stairwells, with centroids inside the true 2 × 2 m openings, as soon as the lidar sees through them (S1 at 144.0 s, long before either strategy uses it). The cross-section at first detection is 1.0–2.25 m² of the true 4.0 m², because only part of the opening is mapped by then.
- **Safety constraints hold.** No strategy entered an occupied or mapped-occupied voxel (asserted on every move). The 1.0 m low-ceiling room is traversed only at the lowest layer (262 nodes, 0 at layer 2), and the 0.5 m crawl space has 0 reachable nodes, so it is excluded from $\mathcal C_{reach}$ and never entered.
- **Seed sensitivity.** Coverage at 600 s varies by about 3.5 points across the ten lidar-noise seeds for the 3D strategies (17.0–20.5 %) and not at all for the slice (15.2 %). At 3600 s the spread is larger (frontier-nearest 76.8–91.7 %, vertical-passage planner 68.5–80.1 %), because small mapping differences change which frontier is chosen and when the drone changes floor.

### Figures

**Voxel map snapshots** (vertical-passage planner, seed 0) at t = 0, 600, 1800 and 3600 s. Rows are floors F0–F3, shown as a horizontal slice through each floor (white free, grey unknown, black occupied). The blue line is the path flown on that floor and the red dot is the drone:

![Map snapshots](map_snapshots.png)

**Coverage curves**: total 3D coverage and per-floor coverage. Thick lines are seed 0; thin lines are seeds 1–9 (3D strategies to 3600 s, slice to 600 s). The dashed line marks the card horizon, and dotted lines mark the vertical-passage planner's floor changes:

![Coverage curves](coverage_curves.png)

**3D trajectories and altitude** (seed 0, 3600 s). The orange squares are the stairwell openings. Frontier-nearest (orange) climbs to F3 by 1994 s, while the vertical-passage planner (blue) is still on F2 at 3600 s. The slice baseline stays at 0.75 m:

![3D trajectory](trajectory_3d.png)

**Animation**: vertical-passage planner, seed 0, one frame per 30 s. Each panel is one floor slice with the frontier voxels in cyan; the drone is shown on its current floor:

![Animation](animation.gif)

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| Building layout not specified | S043 plan extruded per floor, plus F1–F3 rubble (seed 43), three 2 × 2 m stairwells, one 1.0 m low-ceiling room and one 0.5 m crawl space | Needed concrete geometry. The two ceiling features test $h_{min}$ and altitude-constrained passage. |
| Beams over the "upper hemisphere" with θ ∈ [0°, 180°] | Full sphere | θ up to 180° is the full sphere, and downward beams are needed to see openings below the drone. The card has been corrected. |
| Exact 3D DDA (Amanatides & Woo) | Sampled ray-march, 0.05 m steps (one tenth of a voxel) | Vectorised over all beams; at a tenth of a voxel the missed-corner error is negligible. |
| Cylindrical swept-volume check ($r_d$ = 0.15 m, K = 5) | Body voxel + headroom voxel must be free (map p < 0.35 to plan, p < 0.65 and ground truth to enter) | With 0.5 m voxels a single voxel already exceeds $g_{min}$ and the 0.30 m body; $h_{min}$ = 0.6 m rounds up to 2 voxels. |
| Frontier clusters → centroids | Nearest frontier node by Dijkstra travel time | A centroid can lie in an obstacle; the nearest-node rule is the S043 2D frontier rule. |
| Stairwell bonus applied when floor coverage ≥ η_floor = 80 % | Applied when the current floor has no frontier left | The drone cannot know its true coverage; "no frontier left" is the observable equivalent of "floor fully mapped". |
| Altitude optimisation z* (max headroom) with $v_{z,max}$ smoothing | Not implemented as a separate step; vertical moves are 1.67× more expensive in the planner and limited to 0.3 m/s | The 2-voxel headroom rule already guarantees clearance. In the low-ceiling room only one layer is passable. |
| $T_{max}$ = 600 s | 600 s reported, plus a 3600 s extended horizon | 300 m of flight cannot leave F0. The card now notes this. |
| Snapshots at 0, 200, 400, 600 s as isometric voxel renders | 0, 600, 1800, 3600 s as per-floor slices | Nothing happens above F0 before 600 s, and per-floor slices show the map more clearly than isometric voxels. |
| Path coloured by floor, stairwell segments orange | Paths coloured by strategy, stairwell openings outlined in orange, altitude plot with slab bands | Same information. |
| — | Ten lidar-noise seeds, every 3D run flown to 3600 s | Gives per-seed wins and spread for the strategy comparison; the 600 s results are read from the same runs. |

---

## Extensions

1. Replace finish-floor-first with an information-gain-per-travel-time rule that weighs leftover frontiers against an unexplored floor.
2. Multi-drone floor partitioning (card Extension 1): one drone per floor would remove the floor-switching trade-off.
3. Exact DDA and a swept-cylinder check at a finer 0.1 m voxel size, where $g_{min}$ and $h_{min}$ really constrain the path.
4. Dynamic floor collapse (card Extension 3): flip stairwell voxels to occupied mid-mission and measure replanning.

---

## Related Scenarios

- Base 2D version: [S043 Confined Space Exploration](../../../../scenarios/03_environmental_sar/S043_confined_space.md)
- 3D card: [S043 3D Upgrade — Confined Space Exploration](../../../../scenarios/03_environmental_sar/3d/S043_3d_confined_space.md)
- Volumetric mapping: [S050 Swarm Cooperative Mapping (Distributed EKF-SLAM)](../../../../scenarios/03_environmental_sar/S050_slam.md)
- Post-mapping inspection: [S044 Wall Crack Inspection](../../../../scenarios/03_environmental_sar/S044_wall_crack.md)
- Multi-drone coordination: [S042 Missing Person Localization](../../../../scenarios/03_environmental_sar/S042_missing_person.md)
