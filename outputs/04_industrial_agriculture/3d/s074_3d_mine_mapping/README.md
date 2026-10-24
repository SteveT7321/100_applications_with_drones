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
| `3d_fixed` | 16-layer LiDAR | held at 1.5 m | ablation: same LiDAR without altitude control |
| `2d` | 180-beam planar LiDAR (2D base) | held at 1.5 m | 2D baseline (compared on its own plane) |
| `3d`, rangefinder rays not mapped | as `3d` | planned | ablation: what the rangefinder rays add to the map |

**Robustness** of the exploration planner is measured on a **held-out set** by the companion script `s074_3d_mine_mapping_robustness.py` (the review's eleven settings, ten fresh noise seeds, two alternative layouts and every planner, traverse and safety-filter constant at two values); its outputs are saved next to this README.

**Pose estimation** is evaluated separately on the flown 3D trajectory: dead reckoning against point-to-plane ICP (x, y, z, yaw, odometry prior, degeneracy-aware) with a rangefinder altitude reference, on 3 noise seeds; the map is rebuilt with the ICP poses; and a 200 s closed-loop probe navigates with the estimate.

---

## Mathematical Model Summary

**Free space**: union of axis-aligned boxes; LiDAR ranges by exact ray casting against the union. **Wall clearance** (asserted ≥ 0.25 m, the drone radius) is computed exactly as the distance to the nearest rock cell of the grid spanned by the box bounds.

**Occupancy**: log-odds per 0.2 m voxel, $+\log 9$ at a hit, $+\log(2/3)$ on every voxel stepped along the beam, clamped to ±10; free if $P < 0.3$, occupied if $P > 0.7$.

**Ground truth for the coverage score**: a voxel is *free* if its whole cube lies inside the mine, *wall* if its cube contains the wall surface (walls that pass through voxel centres) or lies outside with a face on the surface (walls on voxel faces); the remaining rock is not scored. This is where a LiDAR return lands.

**Dust**: $c(z) = 0.8\,e^{-z/4}$; a beam is lost where $\int_0^r \mu_{ext} c\,ds = \ln(1/0.05)$, which has the closed form
$$r_{eff} = \frac{-H\ln\bigl(e^{-z_0/H} - \tau^* \sin\theta/(\mu_{ext} c_0 H)\bigr) - z_0}{\sin\theta}, \qquad \tau^* = \ln 20$$
(infinite when the argument is non-positive). Lost beams mark free space up to $r_{eff}$ only.

**ICP** (scan to previous scan, every 0.5 s): Gauss–Newton on $(t_x, t_y, t_z, \psi)$ minimising $\sum (\mathbf n_i\cdot(R\mathbf s_i + \mathbf t - \mathbf q_i))^2/2\sigma_r^2$ plus the odometry prior (σ = 0.02 m per 0.1 s step, as the 2D base's motion noise). Normals by PCA over 10 neighbours within 0.6 m, kept only for planar neighbourhoods. **Degeneracy-aware update**: eigen-directions of the scan Hessian with eigenvalue below $\lambda_{th}$ = 3·10⁴ keep the prior.

**Rangefinder altitude**: the up and down ranges are compared with the floor/ceiling heights seen before; the same surface (within 0.3 m) gives an absolute altitude, a new surface (opening, shaft, other level) is re-anchored at the predicted altitude.

**Motion**: the commanded velocity is limited to 1.0 m/s horizontally and 0.8 m/s vertically (asserted). The true motion adds the 2D base's odometry noise, a random walk of σ = 0.02 m per 0.1 s step and axis, so the true per-step speed exceeds the command (printed below).

**Planner** (0.4 m planning grid pooled from the map, Dijkstra on 26-connected traversable cells):
- *Clearance*: a planning cell is traversable only if none of its 0.2 m voxels touches the one-voxel dilation of an occupied voxel, i.e. its centre is at least 0.4 m from every occupied voxel.
- *Frontiers*: a cluster is **lateral** if it has cells within 0.6 m of a known gallery level; otherwise it is **vertical** if it reaches more than 1.8 m beyond every level (beyond the gallery's half-height). Floor and ceiling patches of a gallery are not pursued. Goal = nearest traversable cell within 2 cells of a kept cluster (path length).
- *Shafts*: a vertical cluster opens a candidate box (cluster extent + 0.6 m, ±3 m in z). Under the box the rangefinders decide: free space beyond a full gallery height (more than twice the gallery half-height measured at the start) confirms it, both ranges short within 0.3 m of the box centre reject it (vertical clusters within 1.0 m of a rejected test point stay excluded). A confirmed shaft is **traversed** (card's shaft-traverse mode): plan to the cell under (over) the estimated opening centre (centroid of the voxels mapped free in the first 0.4 m beyond the gallery's ceiling or floor, within 1.5 m of the drone) and align within 0.15 m (*alignment phase*); then a direct controller holds the shaft axis (horizontal speed ≤ 0.3 m/s) and moves vertically only while within 0.1 m of it, toward the gallery beyond (target = measured range minus the gallery half-height), which becomes a new level (*vertical phase*). The axis is re-centred from the steepest LiDAR layer: when the ±x (±y) beams both hit **shaft** walls (between this gallery's ceiling and the target gallery's floor), the axis lies midway between the hits. Returns above the target gallery's floor are walls of that gallery and are not used: in the round-2 review the drone, just above the rim of shaft 2, took the Level 3 drift walls for shaft walls, re-centred the axis metres away and slid sideways over the rim (the review's clearance aborts). Until the axis has been re-centred in both directions the drone waits 0.2 m inside the gallery, below the rim. A traverse not finished in 120 s is given up.
- *Safety filter* (2D base): velocity components toward any LiDAR return closer than 0.7 m are removed and a repulsion is added; in the 3D mission the filter also reads the true (noise-free) up and down ranges every 0.1 s.

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
| Speeds | 1.0 m/s navigation (1.5 m/s cap), 0.8 m/s vertical (commanded) | card |
| Altitude bounds | 0.3–12.5 m | card |
| Odometry / gyro noise | 0.02 m and 10⁻⁴ rad per 0.1 s step | 2D base |
| Rejection distance, normal radius, stop distance, safety radius | 0.5, 0.6, 0.5, 0.7 m; drone radius 0.25 m | 2D base |
| Up/down rangefinders | 12 m, σ 0.05 m, gate 0.3 m | added |
| ICP points / normals | every 5th azimuth / 10 neighbours, planarity ratios 0.1 and 0.3 | added |
| Degeneracy threshold $\lambda_{th}$ | 3·10⁴ (swept 0, 3·10⁴) | added |
| Planning grid / clearance / soft cost | 0.4 m / ≥ 0.4 m from occupied voxels / ×5 within 2 cells of occupied cells | added |
| Frontier rules | cluster ≥ 8 cells, lateral band 0.6 m, vertical beyond 1.8 m, goal within 2 cells | added |
| Shaft rules | box margin 0.6 m, ±3 m in z; reject radius 0.3 m, exclusion radius 1.0 m; centre window 1.5 m; align 0.15 m (≤ 0.6 m from the centre estimate); axis tolerance 0.1 m, axis speed 0.3 m/s, rim guard 0.2 m; arrival 0.5 m, stop 0.1 m; give up after 120 s | added |
| Revisit rule | goal reached 3 times (counted ≥ 1.5 s apart) → frontier cells within 1 m ignored | added (2D base: once) |
| Replan interval / carrot / timeout | 3 s / 0.6 m / 1500 s | added |
| Replay seeds / dust multipliers | 101–103 / ×1, ×6, ×10 | added |
| Closed-loop probe cap | 200 s | added |
| Held-out robustness set | 11 review settings, seeds 100–109, layouts B and C, 27 constants (48 values) | added |

The robustness set changes every planner, traverse and safety-filter constant listed above (27 constants) except the planning-grid size and the 1500 s timeout; the sensor, noise and mine parameters are not varied.

---

## Implementation

```
src/04_industrial_agriculture/3d/s074_3d_mine_mapping.py              # main simulation
src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py   # held-out robustness set (parts h01-h15, summary)
```

```bash
conda activate drones
python src/04_industrial_agriculture/3d/s074_3d_mine_mapping.py
python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py list      # the runs in each part
python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py h01       # ... through h15
python src/04_industrial_agriculture/3d/s074_3d_mine_mapping_robustness.py summary
```

Runtime over three runs each: main script 203–213 s; robustness parts h01–h15 159–266 s each (about 52 min in total), `summary` 1 s. Each script and part was run twice single-threaded and once with default threading; stdout and every output file (PNG, GIF, robustness text files) were byte-identical.

---

## Results

### Geometry and dust facts

- From the gallery centre the lowest beam (−15°) reaches the floor 5.60 m away; on a shaft wall 1 m away the highest beam is 0.268 m above the drone. A ±15° LiDAR therefore cannot look up a shaft, which is why the rangefinders were added.
- A beam is lost at an optical depth of 2.996, but the largest possible depth over 10 m (at the floor) is 1.200. **With the card's dust values no beam is ever lost** (0 of 4 682 880 beams in the 3D mission; maximum depth 0.993). The card's approximation would have given $r_{eff}$ = 4.38 m at z = 1.5 m.
- Ground truth: 196 374 free voxels (Level 1 / 2 / 3 / shafts: 103 530 / 50 572 / 40 672 / 1 600) and 60 078 wall voxels (42 602 containing the wall surface, 17 476 outside with a face on it).

### Missions (true pose, seed 42)

| Mission | End | $T_{exp}$ | Path | $\xi$ | Free mapped free | Walls mapped | L1 / L2 / L3 / shafts free | Max altitude |
|---|---|---:|---:|---:|---:|---:|---|---:|
| `3d` | frontiers unreachable | 406.4 s | 347.4 m | **97.11 %** | 97.51 % | 95.81 % | 96.31 / 99.48 / 99.43 / 64.00 % | 11.57 m |
| `3d_fixed` | no frontier left | 96.5 s | 98.6 m | 49.90 % | 50.45 % | 48.09 % | 95.55 / 0.00 / 0.00 / 9.00 % | 1.50 m |
| `3d`, rangefinder rays not mapped | frontiers unreachable | 453.5 s | — | 96.97 % | — | — | 96.32 / 99.47 / 99.42 / — % | — |

The 3D mission found galleries at z = 1.50, 6.88 and 10.95 m with 2 shaft traverses (none given up); 4 candidate boxes were traversed or led to a known level, 26 ceiling or floor patches were rejected (45 untested). It spent 4.0 % of its steps in the vertical phase of a shaft traverse and 1.2 % in the alignment phase (the two return descents through the already-mapped shafts are ordinary planned paths, not traverses), and 58.5 % above z = 4.25 m. Minimum wall clearance (exact) 0.547 m (asserted ≥ 0.25 m). $\xi$ reached 25 % at 85 s, 50 % at 200 s and the card's 85 % at 360 s. "Frontiers unreachable" means that the frontier clusters left could not be reached (floor and ceiling patches under the ±15° beams). The shaft columns between galleries stay the least mapped (64.00 % free): the ±15° beams cross them only at grazing angles.

**Speeds**: the commanded speed is asserted ≤ 1.0 m/s horizontally and ≤ 0.8 m/s vertically, and the 0.80 m/s maximum climb rate is that command limit. The true per-step speed includes the odometry noise inherited from the 2D base (0.02 m per 0.1 s step and axis) and reaches 1.70 m/s horizontally and 1.42 m/s vertically in the 3D mission (1.66 and 1.69 m/s horizontally in `3d_fixed` and `2d`).

**Ablation, same 16-layer LiDAR**: holding altitude gives $\xi$ = 49.90 %, planning altitude through the shafts 97.11 % (+47.21 points, almost all on Levels 2 and 3). The cost is time: 406.4 s against 96.5 s (+309.9 s). The comparison is not between identical sensor suites: the 3D mission also has the up/down rangefinders, which test the shafts, write their two rays into the map and feed the safety filter (with true ranges). Leaving their rays out of the map changes $\xi$ by −0.14 points (96.97 %, a different path that ends at 453.5 s), so the gain comes from the altitude control, not from the two extra rays.

**2D baseline**: on its own plane (the Level 1 layer at z = 1.5 m) all three missions finish near the same value (`2d` 99.39 %, `3d` 99.39 %, `3d_fixed` 98.04 %): this final number saturates for any mission that completes Level 1 and does not discriminate. What does is **when** the plane is mapped: the 2D mission maps 95 % of it by 110 s and 99 % by 115 s; the 3D mission leaves Level 1 early for the shafts, sits at 40.16 % of the plane at 100 s and at 200 s, and reaches 95 % only at 400 s, on its way back. Its volumetric $\xi$ (3.07 %) is not a meaningful comparison: a planar sensor can only fill one 0.2 m layer, by construction.

### Robustness of the exploration: held-out set (true pose; `robustness_summary.txt`)

The round-2 review ran eleven settings the planner had not been tuned against and found four failures (two clearance aborts just above the rim of shaft 2, one shaft-2 traverse given up, one run that stayed on Level 1). The two aborts and the given-up traverse were diagnosed by tracing the review's runs: after the drone had climbed out of shaft 2 into Level 3, the axis re-centring took the walls of the Level 3 drift for shaft walls and moved the axis goal metres away, so the drone slid sideways at rim height. The fix is local: only returns between the current gallery's ceiling and the target gallery's floor count as shaft walls. A second change tried for the traverse stall (re-centring from one wall when the other beam meets the ceiling) misread ceiling returns on seed 42 and was dropped. The constants are unchanged.

The set below was then run once with the final code: the review's eleven settings (now not fully held out, since three of them were traced for the diagnosis), ten fresh default seeds (100–109, each also with the altitude-held ablation), the two layouts used during development (B, C), and 27 planner, traverse and safety-filter constants one at a time at two values (one new value where a review setting already covers the other side). The per-run table (levels reached, $\xi$, abort, minimum exact wall clearance, traverses given up) is in `robustness_summary.txt`.

| Group | 3d runs | All three levels | One level | Aborted | $\xi$ (runs with all levels) | Min wall clearance |
|---|---:|---:|---:|---:|---|---:|
| Review settings | 11 | 10 | 1 | 0 | 96.83–97.81 % | 0.352 m |
| Fresh default seeds 100–109 | 10 | 10 | 0 | 0 | 96.84–97.74 % (median 97.09 %) | 0.416 m |
| Development layouts B, C | 2 | 2 | 0 | 0 | 96.72–97.08 % | 0.506 m |
| Planner constants, 48 values | 48 | 48 | 0 | 0 | 96.33–97.65 % | 0.410 m |
| **All** | **71** | **70** | **1** | **0** | 96.33–97.81 % | 0.352 m |

The altitude-held `3d_fixed` mission on the ten fresh seeds reaches only Level 1 in every run ($\xi$ 49.46–49.88 %).

**The one failure**, `GUARD_DZ` = 0.4 m (rim guard 0.4 m below the ceiling instead of 0.2 m), maps Level 1 only ($\xi$ 50.51 %, one traverse given up). It fails by construction: from 0.4 m below the ceiling the steepest beam (15°) reaches ceiling height only 0.4 / tan 15° = 1.49 m away, beyond the 1 m shaft walls, so it never returns from a shaft wall, the axis is never re-centred and the drone waits below the rim until the 120 s give-up. The guard must stay below $R_{shaft}\tan 15°$ = 1 · tan 15° = 0.268 m (the default 0.2 m does). The same geometry is a residual risk that did not occur in these 71 runs: if the aligned cell is more than 1 − 0.2 / tan 15° = 0.25 m off the shaft axis, the near-side beam at the guard height also passes under the ceiling and the traverse can stall until it is given up.

### Pose estimation on the flown 3D trajectory (3 noise seeds, 812 scan intervals, 20 inside shafts)

| Estimator | 3D RMSE | Horizontal | z | Final error | z error in shafts / galleries |
|---|---|---|---|---|---|
| Dead reckoning | 1.869 ± 0.227 m | 1.677 ± 0.273 m | 0.798 ± 0.144 m | 1.984 ± 0.964 m | 0.725 / 0.800 m |
| ICP + prior + rangefinder altitude | **1.043 ± 0.328 m** | 1.001 ± 0.299 m | **0.247 ± 0.209 m** | 1.013 ± 0.200 m | 0.275 / 0.247 m |

ICP beats dead reckoning on 3 of 3 seeds (RMSE per seed 1.233, 1.316, 0.582 m vs 2.188, 1.732, 1.687 m); the largest relative gain is in altitude (z RMSE 0.247 vs 0.798 m), from the rangefinder reference. Per-scan horizontal increment errors: 44.0 mm in shafts and 38.5 mm in galleries for ICP, 62.2/63.2 mm for dead reckoning. Before the rangefinder altitude is applied, the ICP vertical increment error is 53.4 mm in shafts and 44.9 mm in galleries (larger in shafts in 3/3 seeds), against 44.7 mm for the prior alone: the scan does not constrain altitude better than odometry. The altitude reference re-anchors 12–16 times per run.

**Degeneracy threshold** (seed 101, first 400 scan intervals): without it ($\lambda_{th}$ = 0) ICP diverges to 7.442 m RMSE (dead reckoning 1.461 m); with 3·10⁴ it gives 1.146 m.

**Map with estimated poses**: rebuilding the map from the same scans with the ICP poses of seed 101 gives $\xi$ = 60.49 % (walls 28.68 %) instead of 97.11 % (walls 95.81 %): drift costs 36.62 points, mostly on the walls, which are one voxel thick.

**Closed-loop probe** (ICP estimate in the loop, 200 s cap): no wall contact, both shafts climbed (maximum altitude 11.18 m), $\xi$ 52.26 % (L1/L2/L3 37.81/87.12/69.94 %), pose RMSE 0.483 m (z 0.066 m), final error 0.630 m. The true-pose mission had $\xi$ = 51.08 % at the same time. One capped run; whether the estimate-driven loop completes the mine was not tested.

### Dust (Level 1 scans rebuilt with stronger extinction)

| $\mu_{ext}$ | Horizontal reach at 1.5 m | Beams lost | Dust-zone voxels unknown | Level 1 free mapped |
|---|---:|---:|---:|---:|
| 0.15 (card) | 36.32 m | 0.00 % | 3.14 % | 96.30 % |
| 0.90 (×6) | 6.05 m | 6.38 % | 29.80 % | 84.94 % |
| 1.50 (×10) | 3.63 m | 18.77 % | 53.01 % | 69.04 % |

**Key findings**

- **With the true pose, shaft traversal doubles the mapped volume on the held-out set.** Seed 42: $\xi$ 49.90 % holding altitude against 97.11 % with shaft traversal. Over 71 further runs (the review's eleven settings, ten fresh seeds, two layouts, 27 constants at two values) all three levels were mapped in 70 ($\xi$ 96.33–97.81 %) and no run aborted (minimum exact wall clearance 0.352 m); the one failure, a rim guard of 0.4 m, cannot re-centre on a shaft by construction (0.4 m > 1 · tan 15° = 0.268 m). Holding altitude gives 49.46–49.88 % on the fresh seeds. All of this is for the true pose and this sensor: the frontier and shaft rules are built around a ±15° LiDAR in 3 m galleries.
- **The card's coverage target is met with the true pose.** $\xi$ = 97.11 % (≥ 85 % from 360 s on), with 97.51 % of the free voxels mapped free and 95.81 % of the wall voxels mapped occupied. An earlier version of this README reported $\xi$ below the card's 85 % and blamed grazing angles; that was a ground-truth error (voxels whose centre lay on the wall plane were labelled free and the wall label sat one voxel behind the surface), not a sensing limit.
- **The 2D baseline cannot be compared volumetrically, and its plane saturates.** All three missions end with 98.04–99.39 % of the Level 1 plane mapped; the difference is time: the 2D mission has 99 % of its plane by 115 s, the 3D mission only at 405 s, because it leaves Level 1 for the shafts first.
- **Card expectations not met** (true pose): 3D RMSE 1.043 ± 0.328 m instead of 0.45–0.65 m; dust deficit 0 instead of 8–15 % (no beam is lost at the card's dust values; the 3.14 % of dust-zone voxels left unknown are a geometric effect, present with or without dust). $T_{exp}$ = 406.4 s is below the card's 700–950 s.
- **Altitude drift in shafts.** Before the rangefinder reference, the LiDAR vertical increment error is larger in shafts (53.4 mm) than in galleries (44.9 mm) in all 3 seeds, in the direction the card expects, but both are no better than the prior (44.7 mm) and only 20 of 812 scan intervals are in shafts. With the rangefinder reference the z error is 0.275 m in shafts and 0.247 m in galleries.
- **Degenerate geometry needs handling.** Plain point-to-plane ICP diverges in these tunnels (7.442 m RMSE); keeping the prior in the poorly constrained directions brings it to 1.146 m on the same scans.
- **True pose matters.** Rebuilding the same scans with the ICP poses loses 36.62 points of $\xi$ (60.49 % instead of 97.11 %); every coverage number above is for the true pose.
- **Dust matters only far above the card's values.** It needs about six times the card's extinction before beams are lost (6.38 %) and dust-zone voxels stay unknown (29.80 %).

### Figures

**3D occupancy map** (occupied voxels grey), true trajectory coloured by altitude, ICP estimate of replay seed 101 (red, dashed) and the 2D baseline path:

![Occupancy 3D](./occupancy_3d.png)

**x–z projection** of the final maps of the three missions (green = free in some y, red = only occupied, yellow = unknown), with the mine outline dotted and the flown path:

![x-z section](./xz_section.png)

**Altitude**: true and estimated altitude with the vertical phase of the shaft traverses shaded (top); altitude error of dead reckoning and of ICP + rangefinder for replay seed 101, shafts shaded (bottom):

![Altitude time series](./altitude_timeseries.png)

**Coverage against time**: per level for the 3D mission (left), $\xi$ for the three missions (right):

![Coverage vs time](./coverage_vs_time.png)

**Dust**: dust-zone voxels left unknown, Level 1 coverage and beams lost for three extinction coefficients:

![Dust deficit](./dust_deficit.png)

**Animation** (3D mission, 52 frames, one per 8 s): the trajectory grows while occupied voxels appear; the drone marker is yellow in the vertical phase of a shaft traverse, orange while aligning under a shaft and blue otherwise:

![Animation](./animation.gif)

**Robustness outputs** (text): [summary with the per-run table](./robustness_summary.txt); per part: [h01](./robustness_h01.txt), [h02](./robustness_h02.txt), [h03](./robustness_h03.txt), [h04](./robustness_h04.txt), [h05](./robustness_h05.txt), [h06](./robustness_h06.txt), [h07](./robustness_h07.txt), [h08](./robustness_h08.txt), [h09](./robustness_h09.txt), [h10](./robustness_h10.txt), [h11](./robustness_h11.txt), [h12](./robustness_h12.txt), [h13](./robustness_h13.txt), [h14](./robustness_h14.txt), [h15](./robustness_h15.txt).

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| $r_{eff} \approx r_{lidar}\exp(-\mu\bar c r_{lidar})$ | closed-form solution of the card's own threshold law; card corrected | the approximation contradicts the stated law; at the card's values the law never cuts a beam |
| $\sigma_{ICP}^{3D}$ = 0.015 m as a parameter | odometry noise 0.02 m per step (2D base); drift measured | the drift is a result of the estimator, not an input; card annotated |
| 100 Hz IMU dead reckoning with bias random walk | odometry prior with random-walk error per 0.1 s step and gyro noise (2D base model); the same noise moves the true position, so the true per-step speed exceeds the commanded limits (1.70 m/s horizontal, 1.42 m/s vertical) | same role (prior between scans); the bias model was not implemented |
| 3D ICP with full roll-pitch-yaw (SVD) | point-to-plane Gauss–Newton on x, y, z, yaw with the prior and a degeneracy threshold | roll and pitch are stabilised on a multirotor; plain ICP diverges in these tunnels (printed: 7.442 m) |
| Closed-loop exploration with the ICP pose | missions mapped and navigated with the true pose; the estimator is evaluated by replay, by a map rebuild and by a 200 s closed-loop probe | separates the exploration question from the estimation question; the rebuilt map shows what drift costs (−36.62 points) |
| No altitude sensor | up/down single-beam rangefinders, used for mapping, shaft confirmation, shaft traversal targets and an altitude reference; the safety filter reads their **true** (noise-free) ranges every 0.1 s | a ±15° LiDAR sees only 0.268 m up a shaft wall and never the floor near the drone; mapping their rays changes $\xi$ by +0.14 points (ablation: −0.14 without them) |
| Vertical frontier "inside a shaft bounding box"; shaft-traverse mode positions over the shaft centre, then climbs | candidate boxes from vertical frontier clusters, confirmed or rejected by the rangefinders; traverse = align under the estimated opening centre, then climb or descend to the next gallery | the card's mode, with the detection made explicit; frontier-following up a shaft is unreliable with a ±15° sensor |
| — | traversable cells ≥ 0.4 m from every occupied voxel; axis-holding climb that waits below the rim until re-centred, re-centring only from shaft-wall returns | keeps the 0.25 m drone clear of walls and shaft rims |
| Nearest frontier by Euclidean distance | nearest by planned path length (2D base) | Euclidean distance ignores walls between levels |
| Expected $\xi \ge 85$ %, RMSE 0.45–0.65 m, dust deficit 8–15 %, $T_{exp}$ 700–950 s | 97.11 % (85 % at 360 s), 1.043 m, 0, 406.4 s | reported as simulated; $\xi$ is met, the others are not |
| Dust-free vs dust scenario | card values (no beam lost) and ×6, ×10 extinction on the Level 1 scans | the card's values cannot produce a dust deficit |
| `s074_3d_occupancy.png`, `s074_3d_animation.gif` | `occupancy_3d.png`, `animation.gif` in `outputs/04_industrial_agriculture/3d/s074_3d_mine_mapping/` | project naming |
| Mine layout beyond the levels and shafts | branches and drifts as in the table above; three other layouts in the robustness set | the card gives only levels and shaft positions |

---

## Extensions

1. **Scan-to-map ICP and loop closure** across levels, to remove the drift that costs 36.62 points of $\xi$.
2. **Wider vertical field of view** (e.g. ±45°) instead of rangefinders, and its effect on shaft detection and on the rim guard.
3. **Estimate-driven exploration to completion**, with the clearance margin sized for the drift.
4. **Dust convection** with ventilation, with the dust level as a function of position and time.

---

## Related Scenarios

- 2D card: [S074 Mine 3D Mapping](../../../../scenarios/04_industrial_agriculture/S074_mine_mapping.md)
- 3D card: [S074 3D Upgrade — Mine 3D Mapping](../../../../scenarios/04_industrial_agriculture/3d/S074_3d_mine_mapping.md)
- Collaborative mapping: [S050 Swarm Cooperative Mapping (Distributed EKF-SLAM)](../../../../scenarios/03_environmental_sar/S050_slam.md)
- Confined-space exploration: [S043 Confined Space Exploration](../../../../scenarios/03_environmental_sar/S043_confined_space.md)
