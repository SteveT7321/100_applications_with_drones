# S051 3D Upgrade — Post-Disaster Comm Network

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: ✅ Completed
**Based on**: [S051 Post-Disaster Communication Network Restoration](../../s051_post_disaster_comm/README.md) (2D version)

---

## Problem Definition

**Setup**: The S051 city: 2000 × 2000 m, a 40 × 40 grid of 50 m cells holding 18,672 persons, and a charge station at (1000, 100, 0) m. Five drones act as airborne cell towers. In the 2D version every drone hovers at 50 m, serves every cell within a 300 m ground disc, and links to other nodes within 600 m. In this version:

- each drone has its own altitude $z_k \in [20, 120]$ m;
- a cell is served when the slant range is at most $d_{cov} = 450$ m **and** the line of sight (LOS) to the cell clears the debris;
- collapsed buildings form a debris height map $h_u \in [0, 30]$ m. Cells with $h_u \ge 10$ m are dense debris and block rays as vertical columns;
- relay links need a 3D distance of at most $d_{relay} = 900$ m and a clear LOS;
- hover power rises with altitude, and each drone must fit the transit and a 1500 s hover into the 144 kJ battery.

**Objective**: maximise the covered population with all five drones connected to the charge station and every drone energy-feasible.

**Strategies** (all on the same debris maps and the same SA random numbers per seed):
1. **Flat 3D (card baseline)**: the 2D S051 SA solution (300 m disc, 600 m range, no debris) lifted to 50 m and evaluated in the 3D model.
2. **Fixed-z SA at 50 m (2D baseline, added)**: the S051 SA re-optimised in the 3D model, with every drone held at 50 m. This is the 2D strategy at fixed altitude, run with the same envelope as strategy 3. The same fixed-z SA is also run at 120 m (the card ceiling) and, in the lifted-ceiling probe, at 200 m, as the strongest uniform-altitude references.
3. **Altitude SA (card)**: the same SA with $z_k$ added to the state.
4. **Vertical daisy-chain (card)**: $M$ relay-only drones are stacked above the highest debris cell, and the other $K - M$ drones are placed by greedy (x, y, z) coverage. $M^*$ is the connected layout with the most coverage.

Six debris maps (seeds 0–5) form the Monte Carlo, and on each map four extra independent SA seeds measure the optimiser's own spread. Seed 0 is the one shown in the figures.

---

## Mathematical Model

**Coverage (slant range and LOS)**: drone $k$ at $(x_k, y_k, z_k)$ serves cell $u$ with ground centroid $(x_u, y_u, 0)$ if
$$d_{3D}(k,u) = \sqrt{\|\mathbf{c}_u^{xy} - \mathbf{p}_k^{xy}\|^2 + z_k^2} \le d_{cov},\qquad \text{LOS}(k,u) = \text{clear}.$$
The ground footprint radius $\sqrt{d_{cov}^2 - z^2}$ **shrinks** with altitude: 449.6 m at 20 m, 447.2 m at 50 m, 433.7 m at 120 m.

**Debris LOS**: the ray $\mathbf{r}(\tau) = (1-\tau)\mathbf{p}_k + \tau(x_u, y_u, 0)$ is sampled at $\tau = n/50$, $n = 0 \ldots 49$. It is blocked if a sample lies in a dense-debris column, $r_z(\tau) \le h(\mathbf{r}^{xy}(\tau))$. The survivor's own cell is excluded. Because the sample height is $(1-\tau) z_k$ and its horizontal track does not depend on $z_k$, the ray is clear if and only if $z_k > \max_n h_n / (1 - \tau_n)$. This clearance altitude is precomputed for every (lattice point, cell) pair, and the result is checked against direct ray marching for every reported layout.

**Relay links**: an edge $(i, j)$ exists if $\|\mathbf{p}_i - \mathbf{p}_j\| \le d_{relay}$ and the 49 interior samples of the link clear the debris. The base is node 0 at ground level. Connectivity is checked with a breadth-first search from the base (equivalent to $\lambda_2 > 0$ for the base's component).

**Energy**: $P_{hover}(z) = P_0(1 + \kappa z)$, $L_k = \sqrt{\|\mathbf{p}_k^{xy} - \mathbf{p}_{base}^{xy}\|^2 + z_k^2}$, feasible if $(E_{max} - 2 L_k P_{transit}/v)/P_{hover}(z_k) \ge T_{hover}$.

**SA**: $E = -C + \alpha\,(n_{disconnected} + n_{energy})$, with $C$ in persons and $\alpha = 10^6$. Each step moves one random drone by $\Delta x, \Delta y \sim \mathcal{U}(-\delta_{xy}, \delta_{xy})$ (rounded to the 25 m lattice) and $\Delta z \sim \mathcal{U}(-\delta_z, \delta_z)$, with $\delta_{xy} = \max(200\,T/T_0, 25)$ m and $\delta_z = \max(20\,T/T_0, 2)$ m. Proposals that put a drone inside a debris column ($z \le h$) are rejected. The cooling is $\beta = (T_{end}/T_0)^{1/N}$ with $T_0 = 500$, $T_{end} = 0.25$ and $N = 80\,000$. The fixed-z run draws the same random numbers every step and ignores $\Delta z$.

**Asserted invariants**: altitude inside its bounds, every drone above the debris under it, positions inside the area, coverage counters within $[0, K]$, and the incrementally tracked SA coverage equal to the direct ray-marched coverage (to 1e-9).

---

## Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Fleet $K$ / area / grid | 5 / 2000 × 2000 m / 40 × 40 | card (2D) |
| Population | 18,672 persons, 2D S051 map (seed 42) | 2D |
| Altitude bounds | 20–120 m | card |
| $f_c$ / $P_{tx}$ / $G_{tx} = G_{rx}$ / $P_{min}$ | 900 MHz / 1 W / 3 dBi / −100 dBm | card; see the $d_{cov}$ row |
| $d_{cov}$ / $d_{relay}$ | 450 m / 900 m | card's stated values, used directly; with the card's $P_{min}$ the formula gives 167.3 km (see Deviations) |
| Debris heights / dense threshold / LOS samples | 0–30 m / 10 m / 50 | card |
| Debris generator | $h = 30\sqrt{\rho/\rho_{max}}\,q$, $q$ = rank-uniform Gaussian-smoothed noise (σ = 1.5 cells) | added; dense cells 32.6–40.9 % per seed |
| $P_{hover,0}$ / $\kappa$ / $E_{max}$ / $v$ | 80 W / 4e-5 1/m / 144 kJ / 15 m/s | card |
| $P_{transit}$ | 80 W | added (card gives none) |
| $T_{hover}$ | 1500 s | 2D |
| $\alpha$ / $T_0$ / $T_{end}$ / iterations | 1e6 / 500 / 0.25 / 80 000 | card, 2D cooling fix |
| $\delta_{xy}$ / $\delta_z$ at $T_0$ | 200 m / 20 m | card, scaled with $T$ as in 2D |
| SA lattice / $\delta_z$ floor | 25 m / 2 m | added |
| Charge station | (1000, 100, 0) m | card |
| Daisy-chain altitude levels for greedy | 20, 40, …, 120 m | added |
| Monte Carlo | 6 debris maps (debris seed 1000 + map); main SA seed 500 + map; 4 extra SA seeds per map (3000 + 100 × map + replicate) | added |
| Sweeps | uniform altitude 20–300 m; debris scale 0, 0.5, 1, 1.5; correlation 0.5, 3 cells (20 000 SA iterations each) | added |

---

## Implementation

```
src/03_environmental_sar/3d/s051_3d_post_disaster_comm.py
```

```bash
conda activate drones
python src/03_environmental_sar/3d/s051_3d_post_disaster_comm.py
```

The maps run in a pool of up to 6 worker processes, and the results are collected in map order. Runtime ≈ 355 s run on its own with 6 workers (including the 80 000-iteration fixed-120 m runs); up to about 490 s when other multi-process simulations share the CPU. Runs with default threading and with `OMP_NUM_THREADS=1` printed byte-identical output and wrote byte-identical files.

---

## Results

### Coverage per seed (% of population; * = not connected)

Main runs: 80 000 iterations, SA seed 500 + map (decoupled from the debris seed), the same SA seed for every strategy on a map.

| Seed | Flat 3D | Fixed-z SA 50 m | Fixed-z SA 120 m | Altitude SA | Daisy-chain ($M^*$) | Altitude SA − fixed 50 m |
|---|---:|---:|---:|---:|---:|---:|
| 0 | 41.39 | 53.95 | 70.91 | 69.39 | 70.21* (0) | +15.45 pp |
| 1 | 37.92 | 47.66 | 67.49 | 66.84 | 63.75 (0) | +19.18 pp |
| 2 | 31.05 | 42.68 | 65.67 | 65.67 | 63.25 (0) | +22.99 pp |
| 3 | 39.01 | 50.89 | 71.62 | 69.99 | 68.68 (0) | +19.10 pp |
| 4 | 39.95 | 45.92 | 67.08 | 66.20 | 62.28 (0) | +20.28 pp |
| 5 | 29.42 | 39.51 | 63.27 | 64.44 | 63.82 (0) | +24.93 pp |
| **Mean** | **36.45** | **46.77** | **67.67** | **67.09** | 65.33 (64.36 over the 5 connected seeds) | **+20.32 pp** |

### Optimiser spread (4 independent SA seeds per map, 20 000 iterations)

Each replicate uses its own SA seed (3000 + 100 × map + replicate), shared by every strategy in that replicate.

| Map | Altitude SA ($z \le 120$ m) | Fixed 50 m | Fixed 120 m | Altitude SA ($z \le 300$ m, probe) | Fixed 200 m (probe) |
|---|---:|---:|---:|---:|---:|
| 0 | 67.98 ± 2.41 | 52.83 ± 1.05 | 69.99 ± 0.45 | 82.95 ± 1.99 | 85.36 ± 0.08 |
| 1 | 65.21 ± 1.38 | 46.80 ± 0.85 | 67.00 ± 0.38 | 83.46 ± 1.72 | 82.77 ± 0.51 |
| 2 | 63.82 ± 2.36 | 41.73 ± 0.29 | 64.25 ± 0.13 | 82.88 ± 2.17 | 83.74 ± 0.40 |
| 3 | 69.81 ± 1.24 | 50.10 ± 0.68 | 70.65 ± 1.07 | 83.32 ± 1.95 | 82.13 ± 1.80 |
| 4 | 65.45 ± 0.15 | 45.48 ± 0.52 | 66.03 ± 0.71 | 84.02 ± 0.11 | 82.46 ± 0.76 |
| 5 | 63.33 ± 2.40 | 38.95 ± 0.72 | 64.62 ± 1.11 | 83.10 ± 1.10 | 83.76 ± 0.14 |
| Mean within-map SD | 1.66 pp | 0.68 pp | 0.64 pp | 1.51 pp | 0.62 pp |

Paired over the 24 runs: altitude SA − fixed 50 m = +19.95 pp (+11.54 to +25.90, better 24/24); **altitude SA − fixed 120 m = −1.16 pp (−5.81 to +1.46, better in 6/24, worse in 17/24)**; fixed 120 m − fixed 50 m = +21.11 pp (24/24); lifted-ceiling altitude SA − fixed 200 m = −0.08 pp (−3.66 to +5.15, better 12/24, worse 12/24).

**Key findings**

- **Flying higher is what pays, not per-drone altitude optimisation.** Against the 2D baseline at 50 m, altitude SA gains +20.32 pp (6/6 maps, +15.45 to +24.93) and +19.95 pp over the 24 replicate runs (24/24). But the SA puts every drone at or near the 120 m ceiling (29/30 drones within 5 m of it, mean 119.3 m, minimum 114.1 m), and the plain 2D strategy flown at a fixed 120 m gets the same gain or more: fixed 120 m − fixed 50 m = +21.11 pp (24/24), and altitude SA − fixed 120 m = −1.16 pp (worse in 17/24). Adding $z$ to the SA state makes the optimiser noisier (within-map SD 1.66 pp vs 0.64 pp) and slightly worse. The same holds at the main 80 000-iteration budget on the main SA seeds: fixed 120 m reaches 67.67 % against 67.09 % for altitude SA, so altitude SA − fixed 120 m = −0.59 pp (−1.64 to +1.17; better in 1/6, worse in 4/6, 1 tie). The 20 000-iteration sweep runs on the main SA seeds agree (altitude SA − uniform 120 m: −0.29 pp, better in 1/6).
- **The 2D solution itself does not transfer**: it covers 62.97 % in its own 2D model but only 36.45 % once debris shadow is applied; re-optimising at 50 m with the debris in view gives +10.31 pp on average (+5.98 to +12.56).
- **The mechanism is debris shadow, and it is measured directly.** Averaged over seeds, 82.36 % of the population lies within slant range of the fixed-50 m layout, but 35.59 % of it is debris-shadowed. For the altitude SA the in-range share is 86.76 %, of which only 19.67 % is shadowed (flat 3D: 48.61 % shadowed). In the LOS map (`los_shadow_map.png`, seed 0) on average 0.38 of the 5 drones see a cell at 50 m, against 0.60 for the altitude-SA layout.
- **Ablation (debris height scale)**: with no debris (scale 0) altitude SA and fixed 50 m tie (93.84 % vs 93.99 %, mean −0.15 pp), and the SA leaves the drones at a mean 51.0 m (40.5–60.1 m), since the footprint radius barely changes between 20 m and 50 m (449.6 m vs 447.2 m). With half-height debris the gain over 50 m is +5.73 pp at a mean altitude of 92.2 m (70.2–111.0 m, i.e. below the ceiling), with full debris +20.30 pp at 117.0 m, and with 1.5 × debris +14.94 pp at 117.1 m (6/6 seeds at every scale above 0). The altitude rises because of the debris, not because of the bound.
- **Uniform-altitude sweep**: coverage rises steadily up to the ceiling: 36.25 % at 20 m, 45.95 % at 50 m, 66.54 % at 120 m. The card's expectation that the optimiser "lifts high-load relay drones higher" does not appear.
- **Probe with the ceiling lifted to 300 m** (beyond the card bounds, for insight only): uniform-altitude coverage peaks at 200 m (83.27 %) and falls to 74.85 % at 300 m, where the footprint has shrunk to 335.4 m. Altitude SA with $z \le 300$ m reaches 84.46 % in the 20 000-iteration sweep runs on the main SA seeds, but over the 24 replicate runs it is level with a fixed 200 m (−0.08 pp, 12/24 better, 12/24 worse). The spread of the five altitudes within a layout (27.6–151.1 m) therefore buys nothing measurable; there is no evidence of a per-drone altitude advantage above the ceiling either.
- **Sensitivity to the added debris generator**: with a correlation length of 0.5 cells the gain over 50 m is +32.40 pp, and with 3 cells +15.61 pp (6/6 seeds each).
- **The vertical daisy-chain never helps.** $M^* = 0$ in every seed: coverage falls with each added relay-only drone (seed 0: 70.21, 62.69, 53.07, 40.27 and 27.08 % for $M$ = 0–4), and no relay link needs a vertical bridge. In the final flat, fixed-z and altitude-SA layouts, no link within 900 m is debris-blocked (0 in all seeds; the uniform-20 m SA layouts have 5). For random drone pairs, links are blocked only at 20 m (1137/7352 pairs, 15.47 %), never at 50 m (0/8659). The card's premise that debris breaks drone-to-drone LOS holds only for drones flying below about 50 m. With $M = 0$ the heuristic is a 3D greedy placement, which always picks the 120 m level; altitude SA beats it by +2.27 pp over the 5 connected seeds (+0.62 to +3.92, 5/5), and in seed 0 the greedy layout is disconnected (70.21 %*).
- **Constraints**: energy never binds inside the bounds. The maximum transit distance is 2241 m at 20 m and 2196 m at 120 m, against at most 2150 m to the farthest corner. Hover power barely changes over the altitude range (80.064 W at 20 m, 80.384 W at 120 m). Both SA variants are connected and energy-feasible by iteration 158 at the latest.

### Figures

**3D relay network**, seed 0: altitude-SA drones coloured by altitude, the fixed-50 m layout in orange, dense-debris pillars in grey:

![Network 3D](./network_3d.png)

**Coverage maps**, seed 0: served population (coloured), dense debris (hatched), relay links (dashed) and each drone's altitude. The seed-0 daisy-chain (pure greedy, $M^* = 0$) is disconnected:

![Coverage maps](./coverage_maps.png)

**Debris and LOS shadow map**, seed 0: debris height (left) and, for each cell, the fraction of the 5 drones with a clear LOS, ignoring range (fixed-50 m centre, altitude SA right):

![LOS shadow](./los_shadow_map.png)

**Altitudes**: per-drone altitude in seed 0 (fixed 50 m, altitude SA at the 120 m ceiling, and the lifted-ceiling probe hatched), and the debris-scale ablation of the mean altitude chosen by altitude SA:

![Altitudes](./altitude_bars.png)

**Coverage by strategy** (main runs, 80 000 iterations): bars = mean over all 6 maps, dots = maps, red × = not connected. The fixed-z SA at 120 m (the 2D strategy at the ceiling) has a slightly higher mean than altitude SA (67.67 vs 67.09 %). Dotted line = the 2D S051 SA in its own 2D model:

![Coverage bar](./coverage_bar.png)

**Coverage vs uniform altitude** (fixed-z SA, 20 000 iterations, thin lines = maps), with the altitude-SA result for $z \le 120$ m. The lifted-ceiling altitude SA is not drawn: over the 24 replicate runs it is level with a fixed 200 m. The grey band is above the card ceiling:

![Altitude sweep](./altitude_sweep.png)

**Debris ablation**: fixed-50 m SA and altitude SA against the debris height scale (band = seed range):

![Debris sweep](./debris_sweep.png)

**SA convergence**: cost and coverage for seed 0 (top two panels; the green curve is the altitude SA's mean altitude), and the number of disconnected or energy-infeasible drones for every seed (bottom, offset per seed):

![SA convergence](./sa_convergence.png)

**Animation**: altitude SA in seed 0; drones climb as the debris shadow is cleared (62 frames: 61 samples every 1333 iterations, plus the final best layout):

![Animation](./animation.gif)

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| $d_{cov} = \frac{c}{4\pi f_c}\sqrt{P_{tx}G_{tx}G_{rx}/P_{min}}$ "≈ 450 m" with $P_{min}$ = −100 dBm; $d_{relay}$ "≈ 900 m" | $d_{cov}$ = 450 m and $d_{relay}$ = 900 m used directly | With the card's values the formula gives 167.3 km (printed), which would cover the whole city from anywhere. 450 m and 900 m correspond to free-space received powers of −48.6 and −54.6 dBm, the effective thresholds (urban margin included). Card corrected. |
| "Hovering higher extends the ground footprint (slant-range coverage grows with altitude)" | Under the card's own slant-range model the footprint **shrinks** with altitude (449.6 → 433.7 m from 20 to 120 m) | Altitude helps only through debris LOS. Card corrected. |
| Discrete LOS over $n = 0 \ldots N_{ray}$ | $n = 0 \ldots 49$ (survivor end excluded), survivor's own cell excluded; drones may not hover inside a debris column | At $n = N_{ray}$ the ray is at ground level, so every ray would be blocked. Card corrected. |
| Debris map "synthetic" (no generator given) | Building stock ∝ $\sqrt{\text{density}}$ × smoothed rank-uniform collapse field | Debris where the buildings are, and none at the charge station (asserted). Scale and correlation length are swept. |
| Flat baseline only | Added the fixed-z SA at 50 m and at 120 m (and 200 m in the probe) on the same seeds | The lifted 2D solution was optimised without debris. The fixed-z runs separate the value of flying higher from the value of per-drone altitude (X5). |
| Cooling $\beta = 0.995$; constant $\delta_{xy}$, $\delta_z$ | 2D schedule ($\beta$ from $T_0 \to T_{end}$, step sizes scaled with $T$, with floors) | Reuses the reviewed 2D fix; with $\beta = 0.995$ the temperature drops below 1 person after 1240 iterations, leaving 78760 iterations of near hill-climbing (printed). |
| Continuous $(x, y)$ | 25 m lattice (half a cell); $z$ continuous | Allows an exact precomputed clearance table, which makes 80 000-iteration runs with ray-traced coverage affordable. |
| $P_{transit}$ not given | 80 W ($= P_{hover,0}$) | Energy never binds either way (printed margins). |
| Daisy-chain: remaining drones "by greedy coverage above the unobstructed area" | Greedy over cell centroids × {20, 40, …, 120} m with the full 3D coverage model | Defines the card's greedy. Relay-only stack drones cover nothing (card). |
| Expected: optimiser lifts high-load relay drones; altitude chart differs per drone | Every drone ends at or near the 120 m ceiling; a fixed 120 m layout matches or beats altitude SA | Reported as found (X6). |
| Output path `outputs/03_environmental_sar/s051_3d_post_disaster_comm/` | `outputs/03_environmental_sar/3d/s051_3d_post_disaster_comm/` | 3D variants live under `3d/`. |

---

## Extensions

1. **Terrain-adaptive altitude floor** (card extension 1). It matters only below about 50 m, where the drones are never placed here.
2. **Regulated ceiling as a variable**: price the ceiling (e.g. a permit cost or a battery/wind penalty above 120 m) and optimise it jointly with the layout. The probe shows the interior optimum near 200 m.
3. **Antenna pattern**: a downtilted directional antenna would penalise high altitudes directly and create an interior optimum under the ceiling.
4. **Building-height-aware survivors**: survivors on upper floors or rooftops (receiver height > 0) change the shadow geometry.
5. **Dynamic debris / aftershocks** (card extension 4) with online re-annealing.

---

## Related Scenarios

- 2D version: [S051 Post-Disaster Communication Network Restoration](../../../../scenarios/03_environmental_sar/S051_post_disaster_comm.md)
- 3D card: [S051 3D Upgrade — Post-Disaster Comm Network](../../../../scenarios/03_environmental_sar/3d/S051_3d_post_disaster_comm.md)
- Relay geometry: [S047 Base Station Signal Relay](../../../../scenarios/03_environmental_sar/S047_signal_relay.md)
- Debris-aware flight: [S043 Confined Space Exploration](../../../../scenarios/03_environmental_sar/S043_confined_space.md)
- Follow-up: [S052 Glacier Melt Area Measurement](../../../../scenarios/03_environmental_sar/S052_glacier.md)
