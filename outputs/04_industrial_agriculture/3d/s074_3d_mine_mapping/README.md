# S074 3D Upgrade — Mine 3D Mapping

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: ✅ Completed
**Based on**: [S074 Mine 3D Mapping](../../s074_mine_mapping/README.md) (2D version)

---

## Problem Definition

**Setup**: The 2D S074 flies at a fixed 1.5 m with a horizontal 180-beam LiDAR and maps one plane of a single-level mine. Here the mine has **three galleries** (centre heights 1.5, 7.0 and 11.0 m, 3 × 3 m section) joined by **two vertical 2 × 2 m shafts** (x = 30 m from Level 1 to Level 2, x = 60 m from Level 2 to Level 3):

| Element | Extent |
|---|---|
| Level 1 | main drift x 0–80 m at y = 0, branch x = 50 m to y = −20 m |
| Level 2 | drift x 28.5–61.5 m at y = 0, branch x = 45 m to y = +20 m |
| Level 3 | drift x = 60 m from y = −1.5 to 21.5 m, cross drift y = 20 m to x = 80 m |
| Shafts | 2 × 2 m, z 1.5 → 7.0 m (x = 30) and 7.0 → 11.0 m (x = 60) |

The drone (entrance (1.5, 0, 1.5)) carries a **16-layer LiDAR** (360 × 16 beams, ±15°, 10 m, σ = 5 cm) and **up/down single-beam rangefinders** (added, 12 m). It builds a 0.2 m log-odds voxel map and explores by flying to the nearest **3D frontier**: lateral frontiers at a known gallery level, and vertical frontiers that open a candidate shaft. A candidate is tested with the rangefinders; a confirmed shaft is traversed as the card describes, by aligning under the opening centre and then climbing (or descending) to the gallery beyond. Altitude is therefore a planned state. Dust attenuates the beams (Beer–Lambert).

**Missions** (seed 42), mapped and navigated with the **true pose**:

| Key | Sensor | Altitude | Role |
|---|---|---|---|
| `3d` | 16-layer LiDAR + rangefinders | planned (shaft traversal) | card |
| `3d_fixed` | 16-layer LiDAR | held at 1.5 m | fair ablation: same sensor without altitude control |
| `2d` | 180-beam planar LiDAR (2D base) | held at 1.5 m | 2D baseline (compared on its own plane) |

**Robustness** of the exploration planner is measured by the companion script `s074_3d_mine_mapping_robustness.py` (other noise seeds, two other mine layouts, one-at-a-time changes of eight planner constants); its outputs are saved next to this README.

**Pose estimation** is evaluated separately on the flown 3D trajectory: dead reckoning against point-to-plane ICP (x, y, z, yaw, odometry prior, degeneracy-aware) with a rangefinder altitude reference, on 3 noise seeds; the map is rebuilt with the ICP poses; and a 200 s closed-loop probe navigates with the estimate.

---

## Mathematical Model Summary

**Free space**: union of axis-aligned boxes; LiDAR ranges by exact ray casting against the union.

**Occupancy**: log-odds per 0.2 m voxel, $+\log 9$ at a hit, $+\log(2/3)$ on every voxel stepped along the beam, clamped to ±10; free if $P < 0.3$, occupied if $P > 0.7$.

**Dust**: $c(z) = 0.8\,e^{-z/4}$; a beam is lost where $\int_0^r \mu_{ext} c\,ds = \ln(1/0.05)$, which has the closed form
$$r_{eff} = \frac{-H\ln\bigl(e^{-z_0/H} - \tau^* \sin\theta/(\mu_{ext} c_0 H)\bigr) - z_0}{\sin\theta}, \qquad \tau^* = \ln 20$$
(infinite when the argument is non-positive). Lost beams mark free space up to $r_{eff}$ only.

**ICP** (scan to previous scan, every 0.5 s): Gauss–Newton on $(t_x, t_y, t_z, \psi)$ minimising $\sum (\mathbf n_i\cdot(R\mathbf s_i + \mathbf t - \mathbf q_i))^2/2\sigma_r^2$ plus the odometry prior (σ = 0.02 m per 0.1 s step, as the 2D base's motion noise). Normals by PCA over 10 neighbours within 0.6 m, kept only for planar neighbourhoods. **Degeneracy-aware update**: eigen-directions of the scan Hessian with eigenvalue below $\lambda_{th}$ = 3·10⁴ keep the prior.

**Rangefinder altitude**: the up and down ranges are compared with the floor/ceiling heights seen before; the same surface (within 0.3 m) gives an absolute altitude, a new surface (opening, shaft, other level) is re-anchored at the predicted altitude.

**Planner** (0.4 m planning grid pooled from the map, Dijkstra on 26-connected traversable cells):
- *Clearance*: a planning cell is traversable only if none of its 0.2 m voxels touches the one-voxel dilation of an occupied voxel, i.e. its centre is at least 0.4 m from every occupied voxel.
- *Frontiers*: a cluster is **lateral** if it has cells within 0.6 m of a known gallery level; otherwise it is **vertical** if it reaches more than 1.8 m beyond every level (beyond the gallery's half-height). Floor and ceiling patches of a gallery are not pursued. Goal = nearest traversable cell within 2 cells of a kept cluster (path length).
- *Shafts*: a vertical cluster opens a candidate box (cluster extent + 0.6 m, ±3 m in z). Under the box the rangefinders decide: free space beyond a full gallery height (more than twice the gallery half-height measured at the start) confirms it, both ranges short within 0.3 m of the box centre reject it (vertical clusters within 1.0 m of a rejected test point stay excluded). A confirmed shaft is **traversed** (card's shaft-traverse mode): plan to the cell under (over) the estimated opening centre (centroid of the voxels mapped free in the first 0.4 m beyond the gallery's ceiling or floor, within 1.5 m of the drone) and align within 0.15 m; then a direct controller holds the shaft axis (horizontal speed ≤ 0.3 m/s) and moves vertically only while within 0.1 m of it, toward the gallery beyond (target = measured range minus the gallery half-height), which becomes a new level. The axis is re-centred from the steepest LiDAR layer: when the ±x (±y) beams both hit walls beyond the rim, the axis lies midway between the hits. Until it has been re-centred in both directions the drone waits 0.2 m inside the gallery, below the rim. A traverse not finished in 120 s is given up.

**Volumetric coverage** $\xi$ = (free voxels mapped free + wall voxels mapped occupied) / (free + wall voxels). A level counts as *reached* in the robustness runs when more than half of its free voxels are mapped free.

---

## Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Gallery centre heights / section | 1.5, 7.0, 11.0 m / 3 × 3 m | card |
| Shafts | 2 × 2 m, z 1.5–7.0 and 7.0–11.0 m | card |
| LiDAR | 360 × 16 beams, ±15°, 10 m, σ_r 0.05 m, scan every 0.5 s | card |
| Dust $c_0$ / $H_{dust}$ / $\mu_{ext}$ / $I_{threshold}$ | 0.8 / 4.0 m / 0.15 m⁻¹ppm⁻¹ / 0.05 | card |
| Voxel / log-odds / thresholds | 0.2 m / log 9, log(2/3), ±10 / 0.3, 0.7 | card |
| ICP iterations / tolerance | 20 / 10⁻⁴ m | card |
| Speeds | 1.0 m/s navigation (1.5 m/s cap), 0.8 m/s vertical | card |
| Altitude bounds | 0.3–12.5 m | card |
| Odometry / gyro noise | 0.02 m and 10⁻⁴ rad per 0.1 s step | 2D base |
| Rejection distance, normal radius, stop distance, safety radius | 0.5, 0.6, 0.5, 0.7 m; drone radius 0.25 m | 2D base |
| Up/down rangefinders | 12 m, σ 0.05 m, gate 0.3 m | added |
| ICP points / normals | every 5th azimuth / 10 neighbours, planarity ratios 0.1 and 0.3 | added |
| Degeneracy threshold $\lambda_{th}$ | 3·10⁴ (swept 0, 3·10⁴) | added |
| Planning grid / clearance / soft cost | 0.4 m / ≥ 0.4 m from occupied voxels / ×5 within 2 cells of occupied cells | added |
| Frontier rules | cluster ≥ 8 cells, lateral band 0.6 m, vertical beyond 1.8 m | added (robustness-swept) |
| Shaft rules | box margin 0.6 m, ±3 m in z; reject radius 0.3 m, exclusion radius 1.0 m; centre window 1.5 m; align 0.15 m (≤ 0.6 m from the centre estimate); axis tolerance 0.1 m, axis speed 0.3 m/s, rim guard 0.2 m; arrival 0.5 m; give up after 120 s | added (margin, z band, reject and exclusion radii swept) |
| Revisit rule | goal reached 3 times (counted ≥ 1.5 s apart) → frontier cells within 1 m ignored | added (robustness-swept; 2D base: once) |
| Replan interval / timeout | 3 s / 1500 s | added |
| Replay seeds / dust multipliers | 101–103 / ×1, ×6, ×10 | added |
| Closed-loop probe cap | 200 s | added |
| Robustness | seeds 43, 44, 7, 1000; layouts B and C; 8 constants × 2 values | added |

---

## Implementation

```
src/04_industrial_agriculture/3d/s074_3d_mine_mapping.py              # main simulation
src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py   # robustness set (parts: seeds, planner_a, planner_b, summary)
```

```bash
conda activate drones
python src/04_industrial_agriculture/3d/s074_3d_mine_mapping.py
python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py seeds
python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py planner_a
python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py planner_b
python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py summary
```

Runtime over three runs each: main script 163–171 s; robustness part `seeds` 338–444 s, `planner_a` 335–395 s, `planner_b` 398–415 s, `summary` 1 s. Each script was run twice single-threaded and once with default threading; stdout and every output file (PNG, GIF, robustness text files) were byte-identical.

---

## Results

### Geometry and dust facts

- From the gallery centre the lowest beam (−15°) reaches the floor 5.60 m away; on a shaft wall 1 m away the highest beam is 0.268 m above the drone. A ±15° LiDAR therefore cannot look up a shaft, which is why the rangefinders were added.
- A beam is lost at an optical depth of 2.996, but the largest possible depth over 10 m (at the floor) is 1.200. **With the card's dust values no beam is ever lost** (0 of 5 005 440 beams in the 3D mission; maximum depth 0.993). The card's approximation would have given $r_{eff}$ = 4.38 m at z = 1.5 m.

### Missions (true pose, seed 42)

| Mission | End | $T_{exp}$ | Path | $\xi$ | Free mapped free | Walls mapped | L1 / L2 / L3 / shafts free | Max altitude |
|---|---|---:|---:|---:|---:|---:|---|---:|
| `3d` | frontiers unreachable | 434.0 s | 345.5 m | **71.70 %** | 80.35 % | 40.46 % | 84.58 / 76.58 / 76.14 / 64.00 % | 11.89 m |
| `3d_fixed` | no frontier left | 96.5 s | 98.6 m | 38.44 % | 41.46 % | 27.53 % | 83.79 / 0.00 / 0.00 / 9.00 % | 1.50 m |

The 3D mission found galleries at z = 1.50, 6.88 and 10.94 m with 2 shaft traverses (none given up); 3 candidate boxes were traversed or led to a known level, 20 ceiling or floor patches were rejected (46 untested). It spent 22.3 % of its steps in shaft-traverse mode and 61.2 % above z = 4.25 m; minimum wall clearance 0.55 m (asserted ≥ 0.25 m), maximum climb rate 0.80 m/s. $\xi$ reached 25 % at 110 s and 50 % at 350 s. "Frontiers unreachable" means that the frontier clusters left could not be reached (floor and ceiling patches under the ±15° beams).

**Fair comparison** (same 16-layer sensor): holding altitude gives $\xi$ = 38.44 %, planning altitude through the shafts 71.70 % (+33.26 points, all on Levels 2 and 3). The cost is time: 434.0 s against 96.5 s (+337.5 s).

**2D baseline**: on its own plane (the Level 1 layer at z = 1.5 m) the planar 2D sensor maps 87.14 %, the same as the 3D mission. Its volumetric $\xi$ (2.43 %) is not a meaningful comparison: a planar sensor can only fill one 0.2 m layer, by construction.

### Robustness of the exploration (true pose; `robustness_*.txt`)

22 further 3D missions (`robustness_summary.txt`): four other noise seeds (43, 44, 7, 1000), two other mine layouts (B: shafts at x = 40 and 70 m; C: shaft 1 at x = 20 m and the Level 2 branch toward −y), and each of eight planner constants set low and high one at a time (`LAT_DZ` 0.4/0.8, `SHAFT_DZ` 1.6/2.2, `SHAFT_PAD` 0.4/0.8, `SHAFT_ZPAD` 2/4, `REJECT_R` 0.2/0.5, `REJECT_EXCL` 0.6/1.5, `MAX_VISITS` 2/5, `MIN_FRONTIER` 4/12).

| Outcome | Runs |
|---|---:|
| All three levels mapped | 21/22 |
| Two levels (Level 3 missed) | 1/22 (`SHAFT_DZ` = 2.2 m) |
| Aborted by an assertion | 0/22 |

$\xi$ over the 22 runs: median 71.71 %, range 56.32–72.35 %; the 21 runs that reached all three levels: 71.12–72.35 %. The altitude-held `3d_fixed` mission on the same seeds and layouts: 36.51–38.45 %. The one failure has Level 3 at 0.00 %: the drone never traversed shaft 2. *Hypothesis, not probed*: with `SHAFT_DZ` = 2.2 m a vertical frontier must reach 0.7 m above the ceiling of a 3 m gallery, and the shaft-2 opening frontier seen from Level 2 does not, so the shaft is never tested. Minimum wall clearances over these runs are listed in the text files (all ≥ the asserted 0.25 m).

Not swept: the soft-cost factor, the alignment and climb constants and the 120 s give-up time (no traverse was given up in the main run).

### Pose estimation on the flown 3D trajectory (3 noise seeds, 868 scan intervals, 20 inside shafts)

| Estimator | 3D RMSE | Horizontal | z | Final error | z error in shafts / galleries |
|---|---|---|---|---|---|
| Dead reckoning | 1.897 ± 0.281 m | 1.698 ± 0.314 m | 0.824 ± 0.131 m | 2.207 ± 0.869 m | 0.840 / 0.823 m |
| ICP + prior + rangefinder altitude | **1.326 ± 0.626 m** | 1.313 ± 0.625 m | **0.167 ± 0.081 m** | 1.738 ± 0.956 m | 0.156 / 0.167 m |

ICP beats dead reckoning on 2 of 3 seeds (RMSE per seed 1.758, 1.778, 0.442 m vs 2.294, 1.712, 1.684 m); the gain is mostly in altitude (z RMSE 0.167 vs 0.824 m), from the rangefinder reference. Per-scan horizontal increment errors: 45.9 mm in shafts and 39.3 mm in galleries for ICP, 64.0/63.2 mm for dead reckoning. Before the rangefinder altitude is applied, the ICP vertical increment error is 51.0 mm in shafts and 46.0 mm in galleries (larger in shafts in 3/3 seeds), against 44.7 mm for the prior alone: the scan does not constrain altitude better than odometry anywhere, and slightly worse in shafts. The altitude reference re-anchors 12–17 times per run.

**Degeneracy threshold** (seed 101, first 400 scan intervals): without it ($\lambda_{th}$ = 0) ICP diverges to 8.759 m RMSE (dead reckoning 1.461 m); with 3·10⁴ it gives 1.454 m.

**Map with estimated poses**: rebuilding the map from the same scans with the ICP poses of seed 101 gives $\xi$ = 48.70 % (walls 23.19 %) instead of 71.70 % (walls 40.46 %): drift costs 23.00 points.

**Closed-loop probe** (ICP estimate in the loop, 200 s cap): no wall contact, both shafts climbed (maximum altitude 11.44 m), $\xi$ 40.01 % (L1/L2/L3 33.26/62.67/51.08 %), pose RMSE 0.509 m (z 0.162 m). The true-pose mission had $\xi$ = 38.15 % at the same time. One capped run; whether the estimate-driven loop completes the mine was not tested.

### Dust (Level 1 scans rebuilt with stronger extinction)

| $\mu_{ext}$ | Horizontal reach at 1.5 m | Beams lost | Dust-zone voxels unknown | Level 1 free mapped |
|---|---:|---:|---:|---:|
| 0.15 (card) | 36.32 m | 0.00 % | 11.50 % | 84.50 % |
| 0.90 (×6) | 6.05 m | 6.56 % | 36.98 % | 73.28 % |
| 1.50 (×10) | 3.63 m | 18.86 % | 58.23 % | 57.49 % |

**Key findings**

- **With the true pose, shaft traversal nearly doubles the mapped volume, across seeds, layouts and planner settings.** Seed 42: $\xi$ 38.44 % holding altitude against 71.70 % with shaft traversal. Over 22 further runs (4 seeds, 2 layouts, 8 planner constants changed one at a time) all three levels were mapped in 21 (ξ 71.12–72.35 %) and no run aborted; the exception is `SHAFT_DZ` = 2.2 m, which never traverses shaft 2 (ξ 56.32 %). Holding altitude gives 36.51–38.45 % on the same seeds and layouts. All of this is for the true pose and this sensor: the frontier and shaft rules are tuned to a ±15° LiDAR in 3 m galleries.
- **The 2D baseline cannot be compared volumetrically.** On its own plane it maps 87.14 %, the same as the 3D mission; its 2.43 % volumetric coverage is fixed by its planar sensor.
- **Card expectations not met** (true pose): $\xi$ 71.70 % instead of ≥ 85 % (floors, ceilings and walls seen only at grazing angles stay unknown: walls mapped 40.46 %); 3D RMSE 1.326 ± 0.626 m instead of 0.45–0.65 m; dust deficit 0 instead of 8–15 % (no beam is lost at the card's dust values; the 11.50 % of dust-zone voxels left unknown are a geometric effect, present with or without dust). $T_{exp}$ = 434.0 s is below the card's 700–950 s.
- **Altitude drift in shafts.** Before the rangefinder reference, the LiDAR vertical increment error is slightly larger in shafts (51.0 mm) than in galleries (46.0 mm) in all 3 seeds, in the direction the card expects, but both are no better than the prior (44.7 mm) and only 20 of 868 scan intervals are in shafts. With the rangefinder reference the z error is about the same in shafts (0.156 m) and galleries (0.167 m). *Hypothesis, not probed*: inside a shaft the down beam keeps seeing the same floor far below, so the absolute reference continues through the shaft.
- **Degenerate geometry needs handling.** Plain point-to-plane ICP diverges in these tunnels (8.759 m RMSE); keeping the prior in the poorly constrained directions brings it to 1.454 m on the same scans.
- **True pose matters.** Rebuilding the same scans with the ICP poses loses 23.00 points of $\xi$ (48.70 % instead of 71.70 %); every coverage number above is for the true pose.
- **Dust matters only far above the card's values.** It needs about six times the card's extinction before beams are lost (6.56 %) and dust-zone voxels stay unknown (36.98 %).

### Figures

**3D occupancy map** (occupied voxels grey), true trajectory coloured by altitude, ICP estimate of replay seed 101 (red, dashed) and the 2D baseline path:

![Occupancy 3D](./occupancy_3d.png)

**x–z projection** of the final maps of the three missions (green = free in some y, red = only occupied, yellow = unknown), with the mine outline dotted and the flown path:

![x-z section](./xz_section.png)

**Altitude**: true and estimated altitude with the shaft-traverse mode shaded (top); altitude error of dead reckoning and of ICP + rangefinder for replay seed 101, shafts shaded (bottom):

![Altitude time series](./altitude_timeseries.png)

**Coverage against time**: per level for the 3D mission (left), $\xi$ for the three missions (right):

![Coverage vs time](./coverage_vs_time.png)

**Dust**: dust-zone voxels left unknown, Level 1 coverage and beams lost for three extinction coefficients:

![Dust deficit](./dust_deficit.png)

**Animation** (3D mission, 56 frames, one per 8 s): the trajectory grows while occupied voxels appear; the drone marker is yellow in shaft-traverse mode:

![Animation](./animation.gif)

**Robustness outputs** (text): [seeds and layouts](./robustness_seeds.txt), [planner constants, part A](./robustness_planner_a.txt), [planner constants, part B](./robustness_planner_b.txt), [summary](./robustness_summary.txt).

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| $r_{eff} \approx r_{lidar}\exp(-\mu\bar c r_{lidar})$ | closed-form solution of the card's own threshold law; card corrected | the approximation contradicts the stated law; at the card's values the law never cuts a beam |
| $\sigma_{ICP}^{3D}$ = 0.015 m as a parameter | odometry noise 0.02 m per step (2D base); drift measured | the drift is a result of the estimator, not an input; card annotated |
| 100 Hz IMU dead reckoning with bias random walk | odometry prior with random-walk error per 0.1 s step and gyro noise (2D base model) | same role (prior between scans); the bias model was not implemented |
| 3D ICP with full roll-pitch-yaw (SVD) | point-to-plane Gauss–Newton on x, y, z, yaw with the prior and a degeneracy threshold | roll and pitch are stabilised on a multirotor; plain ICP diverges in these tunnels (printed: 8.759 m) |
| Closed-loop exploration with the ICP pose | missions mapped and navigated with the true pose; the estimator is evaluated by replay, by a map rebuild and by a 200 s closed-loop probe | separates the exploration question from the estimation question; the rebuilt map shows what drift costs (−23.00 points) |
| No altitude sensor | up/down single-beam rangefinders, used for mapping, shaft confirmation, shaft traversal targets and an altitude reference | a ±15° LiDAR sees only 0.268 m up a shaft wall and never the floor near the drone |
| Vertical frontier "inside a shaft bounding box"; shaft-traverse mode positions over the shaft centre, then climbs | candidate boxes from vertical frontier clusters, confirmed or rejected by the rangefinders; traverse = align under the estimated opening centre, then climb or descend to the next gallery | the card's mode, with the detection made explicit; frontier-following up a shaft is unreliable with a ±15° sensor |
| — | traversable cells ≥ 0.4 m from every occupied voxel; axis-holding climb that waits below the rim until re-centred | keeps the 0.25 m drone clear of walls and shaft rims (the reviewed version of this script tripped the clearance assertion in some seeds and settings) |
| Nearest frontier by Euclidean distance | nearest by planned path length (2D base) | Euclidean distance ignores walls between levels |
| Expected $\xi \ge 85$ %, RMSE 0.45–0.65 m, dust deficit 8–15 %, $T_{exp}$ 700–950 s | 71.70 %, 1.326 m, 0, 434.0 s | reported as simulated |
| Dust-free vs dust scenario | card values (no beam lost) and ×6, ×10 extinction on the Level 1 scans | the card's values cannot produce a dust deficit |
| `s074_3d_occupancy.png`, `s074_3d_animation.gif` | `occupancy_3d.png`, `animation.gif` in `outputs/04_industrial_agriculture/3d/s074_3d_mine_mapping/` | project naming |
| Mine layout beyond the levels and shafts | branches and drifts as in the table above; two other layouts in the robustness set | the card gives only levels and shaft positions |

---

## Extensions

1. **Scan-to-map ICP and loop closure** across levels, to remove the drift that costs 23.00 points of $\xi$.
2. **Wider vertical field of view** (e.g. ±45°) instead of rangefinders, and its effect on shaft detection.
3. **Estimate-driven exploration to completion**, with the clearance margin sized for the drift.
4. **Dust convection** with ventilation, with the dust level as a function of position and time.

---

## Related Scenarios

- 2D card: [S074 Mine 3D Mapping](../../../../scenarios/04_industrial_agriculture/S074_mine_mapping.md)
- 3D card: [S074 3D Upgrade — Mine 3D Mapping](../../../../scenarios/04_industrial_agriculture/3d/S074_3d_mine_mapping.md)
- Collaborative mapping: [S050 Swarm Cooperative Mapping (Distributed EKF-SLAM)](../../../../scenarios/03_environmental_sar/S050_slam.md)
- Confined-space exploration: [S043 Confined Space Exploration](../../../../scenarios/03_environmental_sar/S043_confined_space.md)
