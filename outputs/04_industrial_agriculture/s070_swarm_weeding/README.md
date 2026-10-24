# S070 Swarm Weeding Task

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: 30 weeds lie at uniformly random positions in a 40 × 40 m field. Three drones work in two phases.
- **Phase 1 (detection)**: each drone sweeps its own 40 × 13.33 m sub-strip in a lawnmower pattern at 1 m/s. A sensor with radius 1.5 m detects each weed it passes over with probability 0.85 per encounter; GPS noise is 0.1 m.
- **Phase 2 (treatment)**: the detected weeds are shared among the drones. Each drone flies to its weeds and hovers 2 s over each under PID control. A treatment succeeds if the hover RMS error is at most 0.15 m.

**Objective (card)**: treat as many weeds as possible (primary) and minimise the Phase-2 travel distance (secondary).

**Assignment strategies** (the same weeds, detections and hover noise in every trial):
1. **Greedy nearest-drone** (card baseline): repeatedly take the closest (drone, weed) pair, give the weed to that drone, and move the drone's notional position to the weed.
2. **Hungarian + NN order**: the card's capacitated Hungarian assignment on end-of-sweep distances (each drone row repeated $K_{max} = \lceil M/N \rceil$ times), after which each drone visits its weeds in nearest-neighbour order.
3. **Hungarian, solver order (card code)**: the same assignment, with each drone visiting its weeds in the order `linear_sum_assignment` returns them.

**Hover controllers**: the card's PID $(K_p, K_i, K_d) = (2, 0.1, 0.5)$, and the same loop without the derivative term (PI). Both are tested with the card's 0.3 m arrival offset and without it (ablation).

---

## Mathematical Model Summary

**Sweep**: $n = \lceil L_{strip} / 2r_s \rceil = 5$ passes per strip, evenly spaced at 13.33/5 = 2.67 m (≤ 2 $r_s$, so the strip is fully covered). Each drone flies 210.67 m, so Phase 1 takes 210.67 s.

**Detection** (card text): a pass *encounters* weed $j$ if $|y_j - (y_{pass} + \eta)| \le r_s$, with $\eta \sim \mathcal N(0, 0.1^2)$. Each encounter detects the weed with probability $P_{detect}$ = 0.85, independently. A weed in the overlap of two passes gets two chances. The draws $\eta$ and $u \sim U(0,1)$ are indexed by (drone, pass number, weed), so the card-code sweep layout (ablation) uses the same random numbers as the main layout.

**Assignment**: $C_{ij} = \|\mathbf p_i^{end} - \mathbf w_j\|$, solved as an LSAP with each drone row repeated $K_{max}$ times, so no drone gets more than $K_{max}$ weeds. Greedy is described above.

**Phase-2 cost**: $D_s = \sum_i \sum_k \|\mathbf p_{i,k-1} - \mathbf w_{i,k}\|$ (sequential route from the end of the sweep). The makespan is $\max_i (D_i / v + n_i T_{treat})$, and the total mission time is Phase 1 + makespan.

**Hover** (card code, plus a speed limit): starting offset $\mathcal N(0, 0.3^2)$ per axis, noisy measurement $\tilde{\mathbf p} = \mathbf p + \mathcal N(0, 0.1^2)$, $\mathbf e = \mathbf w - \tilde{\mathbf p}$, $\mathbf u = K_p\mathbf e + K_i\sum\mathbf e\,\Delta t + K_d\,\Delta\mathbf e/\Delta t$. The command $\mathbf u$ is saturated at $\|\mathbf u\| \le v$ = 1 m/s, then $\mathbf p \mathrel{+}= \mathbf u\,\Delta t$ with $\Delta t$ = 0.05 s for 2 s. Success means an RMS error over the dwell ≤ 0.15 m.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Field | 40 × 40 m |
| Drones / weeds | 3 / 30 |
| Sub-strip width | 13.33 m |
| Sensor radius $r_s$ / max pass spacing | 1.5 m / 3.0 m |
| Passes per strip / spacing | 5 / 2.67 m |
| $P_{detect}$ per encounter | 0.85 |
| Speed (cruise, and hover velocity limit) | 1.0 m/s |
| GPS noise | 0.10 m |
| Treatment hover | 2.0 s, success if RMS ≤ 0.15 m |
| PID gains | (2.0, 0.1, 0.5); PI variant (2.0, 0.1, 0) |
| Hover arrival offset (card code) | 0.3 m std per axis (ablation: 0) |
| Time step | 0.05 s |
| Monte Carlo (added) | 200 trials, seeds 42–241 |

---

## Implementation

```
src/04_industrial_agriculture/s070_swarm_weeding.py
```

```bash
conda activate drones
python src/04_industrial_agriculture/s070_swarm_weeding.py
```

Runtime: 18 s and 17 s in the two reproducibility runs (single-threaded BLAS). The code asserts that:
- every detected weed is assigned exactly once, and the Hungarian load cap holds;
- weeds and end-of-sweep positions lie inside the field;
- the hover velocity command never exceeds 1 m/s, and hover positions stay finite and within 2 m of the target.

---

## Results

### Phase 1: detection

| Sweep layout | Passes per strip | Unswept band per strip | Weeds detected (mean ± std of 30) |
|---|---:|---:|---:|
| Card formula $\lceil L/2r_s \rceil$ (used) | 5 | 0.000 m (0.0 %) | **25.71 ± 1.94 (85.72 %)** |
| Card code `arange(y0 + 1.5, y1, 3)` (same draws, CRN) | 4 | 1.333 m (10.0 %) | 22.48 ± 2.37 (74.92 %) |

With common random numbers the card-code layout detects fewer weeds in 188/200 trials and more in 4. 14.28 % of the weeds are missed in Phase 1. 11.05 % lie in the overlap of two passes and get two chances; 0.02 % are missed by every pass because of GPS noise at the band edge. The card's code draws a Bernoulli trial every 0.05 s while a weed is in range, which gives a mean detection probability of 1.0000 instead of 0.85. The implementation therefore follows the card text: one draw per encounter.

### Phase 2: assignment (200 trials, the same detections)

| Strategy | Phase-2 distance | Makespan | Max weeds per drone | Total mission (Phase 1 + makespan) |
|---|---:|---:|---:|---:|
| Greedy nearest-drone | **158.8 ± 16.0 m** | 142.0 ± 34.8 s | 17.62 ± 4.15 | 352.7 ± 34.8 s |
| Hungarian + NN order | 181.6 ± 20.9 m | **90.2 ± 12.7 s** | 8.92 ± 0.72 | **300.8 ± 12.7 s** |
| Hungarian, solver order (card code) | 389.2 ± 58.6 m | 178.0 ± 27.8 s | 8.92 ± 0.72 | 388.7 ± 27.8 s |

- $\rho = D_{greedy}/D_{Hungarian+NN}$ = 0.878 ± 0.067; $\rho \ge 1$ in only **7/200** trials. Hungarian + NN has the shorter makespan in **187/200** trials.
- $\rho = D_{greedy}/D_{Hungarian,card}$ = 0.415 ± 0.058; $\rho \ge 1$ in 0/200 trials, with a shorter makespan in 38/200.

Trial 0 (22/30 detected): greedy 142.4 m and 151.5 s; Hungarian + NN 162.8 m and 78.2 s; Hungarian in solver order 323.4 m and 142.8 s.

![Field overview](./field_overview.png)

![Assignment Monte Carlo](./mc_comparison.png)

### Phase 2: hover and treatment

| Controller | Arrival offset | Hover RMS over 2 s (mean) | Hovers ≤ 0.15 m | Weeds treated per trial |
|---|---|---:|---:|---:|
| Card PID (2, 0.1, 0.5) | 0.3 m (card) | 0.2363 m | 25.94 % | 6.67 ± 2.21 |
| PI (Kd = 0) | 0.3 m (card) | 0.1413 m | 61.17 % | 15.73 ± 2.73 |
| Card PID | none (ablation) | 0.0565 m | 100.00 % | — |
| PI | none (ablation) | 0.0298 m | 100.00 % | — |
| Card expectation | — | 0.06 m | — | 25.4 |

Each row is over 5143 hovers. The card PID's RMS over only the last 1 s (with the offset) is 0.1621 m. Its noisy derivative term saturates the 1 m/s velocity command in 85.61 % of the steps, against 2.93 % for PI. The card's own formula $\sigma_{GPS}/\sqrt{1 + 2K_p/K_d}$ gives 0.0333 m, not the 0.06 m it states.

![Metrics](./metrics.png)

### Animation

The animation shows trial 0 with Hungarian + NN order: the sweep (0–210.67 s), then the treatment flights. It has 146 frames, one every 2 s up to t = 288.9 s, at 12 fps.
- **Gold**: the weed has been detected. The time shown is approximate: the moment the owning drone's first pass over the weed's band reaches it. The true detection may come on a later pass.
- **Green**: the 2 s treatment hover is finished. The hover-accuracy outcome is not shown.

![Animation](./animation.gif)

### Key findings

- **Detection works as the card text intends (85.72 %) only after fixing the sweep.** The card code's 3 m `arange` leaves 10 % of every strip unswept: 74.92 % detected, lower in 188/200 trials on the same random numbers. Its per-time-step Bernoulli makes detection essentially certain (1.0000), which contradicts the stated 0.85.
- **The card's Hungarian does not minimise travel distance.** Its cost only measures the distance from the end of the sweep to each weed, while the drones then fly a sequential route. Greedy, which updates the drone position after every pick, is shorter in all but 7 of 200 trials ($\rho$ = 0.878 ± 0.067). The card's claim $\rho \ge 1$ does not hold. Visiting weeds in the solver's order, as the card code does, more than doubles the distance (389.2 m).
- **Hungarian does win on time.** Its load cap gives each drone at most ⌈M/3⌉ weeds (8.92 for the busiest drone on average), while greedy loads one drone with 17.62 on average. Hungarian + NN therefore finishes earlier (makespan 90.2 s against 142.0 s; total mission 300.8 s against 352.7 s) and has the shorter makespan in 187/200 trials. Shared work needs makespan as the metric, and there the capacity constraint matters more than the cost model.
- **Treatment failures come from the arrival transient, not from steady-state jitter.** Without the 0.3 m arrival offset, both controllers hold every hover within 0.15 m (PID 0.0565 m, PI 0.0298 m). With the offset, convergence to the weed falls inside the 2 s RMS window, and only 25.94 % (PID) or 61.17 % (PI) of hovers pass. The derivative term on the noisy GPS error makes this worse but is not the root cause: it roughly doubles the steady-state error (0.0565 against 0.0298 m), and it saturates the velocity command most of the time, which slows the convergence from the offset. As a result only 6.67 weeds per trial are treated with the card PID (15.73 with PI) instead of the expected 25.4. The card's arithmetic is also off: at $\sigma$ = 0.06 m the Rayleigh probability is 0.9561, not 0.997, and the card's own $\sigma_{hover}$ formula gives 0.0333 m, not 0.06 m.

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| Passes: formula $\lceil L/\delta_s \rceil$ (= 5) but code `arange(..., 3.0)` (= 4) | 5 evenly spaced passes (2.67 m); the card-code layout is reported as a CRN ablation | The card code leaves a 1.333 m unswept band per strip; the card text says each weed is covered. Card corrected. |
| Bernoulli draw "per (drone, weed, pass) encounter" (text) vs every 0.05 s step (code) | One draw per pass encounter | The per-step draw makes detection about 100 %, which contradicts $P_{detect}$ = 0.85. Card corrected. |
| Hungarian visits in solver order | Main variant: Hungarian assignment + NN visiting order; the card-code order is kept as a third strategy | Solver order is geometrically arbitrary (389.2 m); NN order gives Hungarian a fair route. |
| $\rho = D_{greedy}/D_{hung} \ge 1$ | Measured 0.878 ± 0.067 (≥ 1 in 7/200 trials) | Card claim does not hold; card note added. |
| Single seed (42) | 200 trials (seeds 42–241) with the same weeds, detections and hover noise for all strategies, layouts and controllers | Mean ± std and per-trial win counts (X2). |
| Travel distance only | Also makespan and total mission time | The card's primary objective includes mission time, and makespan separates the methods. |
| Unbounded hover velocity command | Saturated at the 1 m/s cruise speed; speed and divergence asserted | Physical limit (X4). Without it the PID command would exceed 1 m/s in 85.61 % of the steps. |
| $\sigma_{hover}^2 = \sigma^2/(1 + 2K_p/K_d)$ → "0.06 m"; $P_{success}$ ≈ 0.847; 25.4 treated | Simulated: 0.2363 m with the 0.3 m arrival offset (25.94 % pass, 6.67 treated), 0.0565 m without it; PI variant and no-offset ablation added | The formula actually gives 0.0333 m. The loop as written fails mainly through the arrival transient. Card arithmetic error (0.9561, not 0.997). Card corrected. |
| Hover target = weed position | Same (the camera re-acquires the weed, so no localisation bias is added) | Matches the card's error definition $\mathbf e = \mathbf w - \tilde{\mathbf p}$. |
| Animation `s070_weeding.gif`, 25 fps, 10-frame "Running Hungarian" pause | `animation.gif`, 12 fps, one frame per 2 s, no pause; detection times in the animation are approximate | Project naming convention and file size. |

---

## Extensions

1. **Route-aware assignment**: a capacitated vehicle routing formulation (or Hungarian followed by 2-opt with weed swaps between drones) to beat greedy on both distance and makespan.
2. **Settling before scoring**: start the RMS window once the error is below a threshold, or extend the dwell, so the arrival transient does not count against the treatment.
3. **Filtered derivative**: a low-pass on the derivative term to keep damping without saturating on GPS noise.
4. **Second detection sweep**: a perpendicular pass to recover the 14.28 % of weeds missed in Phase 1.
5. **Clustered weeds**: patchy weed distributions, where the card expects Hungarian to gain more.

---

## Related Scenarios

- Prerequisites: [S063 Single-Plant Precision Spraying](../../../scenarios/04_industrial_agriculture/S063_precision_spraying.md), [S067 Spray Coverage Overlap Optimization](../../../scenarios/04_industrial_agriculture/S067_spray_overlap.md), [S068 Large-Scale Farmland Cooperative Spraying](../../../scenarios/04_industrial_agriculture/S068_large_field_spray.md)
- Follow-ups: [S064 Greenhouse Interior Precision Flight](../../../scenarios/04_industrial_agriculture/S064_greenhouse.md)
- Cross-reference: [S018 Multi-Target Intercept](../../../scenarios/01_pursuit_evasion/S018_multi_target_intercept.md), [S040 Fleet Load Balancing](../../../scenarios/02_logistics_delivery/S040_fleet_load_balancing.md), [S042 Missing Person Localization](../../../scenarios/03_environmental_sar/S042_missing_person.md)
