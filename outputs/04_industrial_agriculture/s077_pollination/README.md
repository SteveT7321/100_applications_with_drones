# S077 Precision Pollination Flight Pattern

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: A drone with a vibrating pollen applicator must visit $F = 25$ flowers placed uniformly at random in a 10 × 10 m orchard, at heights between 1 and 3 m. The visit order is a nearest-neighbour TSP tour from the start point (0, 0, 3.5) m. At each flower the drone:

1. climbs vertically to the 3.5 m transit altitude and flies over the approach point;
2. descends onto a 30° approach line that ends 0.8 m from the flower, below it and on the side facing the previous flower, arriving at 0.5 m/s along the line;
3. approaches the flower, rising at 30° like a bee;
4. docks for 1.5 s while the applicator vibrates.

GPS noise is $\sigma_{GPS} = 0.1$ m per axis. Pollen transfer succeeds with probability $P_t = e^{-e_{dock}/\sigma_{dock}}$, where $\sigma_{dock} = 0.1$ m.

**Approach references compared** (everything else identical):
1. **Minimum-snap approach**: a degree-7 polynomial from the approach point (velocity 0.5 m/s along the approach line) to the flower (at rest, zero acceleration and jerk).
2. **Straight line (no FF at the stop)**: constant 0.5 m/s along the line. The reference stops abruptly at the flower, and the stop is not fed forward.
3. **Straight line + stop-impulse FF**: the same line, but the stop is fed forward as a one-step deceleration $\mathbf{a}_{ref} = -v_{approach}\hat{\mathbf{d}}/\Delta t$ (10 m/s² for one 0.05 s step). This gives the straight line the same acceleration feedforward that min-snap gets.

**Objective**: maximise $E[N_{poll}] = \sum_i P_t^{(i)}$.

---

## Mathematical Model Summary

### Approach direction (30° elevation)

$$\hat{\mathbf{d}}_i = (\cos 30°\cos\varphi_i,\ \cos 30°\sin\varphi_i,\ \sin 30°), \qquad \mathbf{p}_{start}^{(i)} = \mathbf{p}_i^{flower} - d_{standoff}\,\hat{\mathbf{d}}_i$$

$\varphi_i$ is the horizontal bearing from the previous flower (or the start point) to flower $i$.

### Minimum-snap segment

Each axis is a degree-7 polynomial $p(t) = \sum_{k=0}^{7} c_k t^k$. The coefficients follow from $\mathbf{A}_{bc}\mathbf{c} = \mathbf{b}_{bc}$, with position, velocity, acceleration and jerk fixed at both ends:

| End | Position | Velocity | Acceleration | Jerk |
|---|---|---|---|---|
| $t = 0$ | $\mathbf{p}_{start}$ | $v_{approach}\hat{\mathbf{d}}$ | 0 | 0 |
| $t = T_{seg}$ | $\mathbf{p}^{flower}$ | 0 | 0 | 0 |

$T_{seg} = d_{standoff}/v_{approach} = 1.6$ s for **every** method. Starting at 0.5 m/s and ending at rest, the min-snap reference peaks at 0.830 m/s and 1.57 m/s². Its snap cost $J = \int (p^{(4)})^2 dt$, from the card's $\mathbf{Q}$ matrix, averages 617.98 m²/s⁷ for seed 42. The climb, transit and descent legs are rest-to-rest degree-7 polynomials (peak speed 1.5 m/s). The descent leg ends with velocity $v_{approach}\hat{\mathbf{d}}$, so every approach starts from a drone that is already moving consistently with the reference.

### Drone and tracking loop

The drone is a 3D point mass whose velocity follows the command with time constant $\tau_v$, with its acceleration magnitude limited to $a_{max}$:

$$\mathbf{v}_{cmd} = \mathbf{v}_{ref} + \tau_v\,\mathbf{a}_{ref} + K_p(\mathbf{p}_{ref} - \tilde{\mathbf{p}}), \qquad \Delta\mathbf{v} = \text{clip}_{\|\cdot\| \le a_{max}\Delta t}\bigl((1 - e^{-\Delta t/\tau_v})(\mathbf{v}_{cmd} - \mathbf{v})\bigr), \qquad |\mathbf{v}_{cmd}| \le 3\ \text{m/s}$$

Here $\tilde{\mathbf{p}} = \mathbf{p} + \boldsymbol{\eta}$ is the GPS measurement, with $\boldsymbol{\eta} \sim \mathcal{N}(0, \sigma_{GPS}^2 I)$ drawn every 0.05 s. The term $\tau_v\mathbf{a}_{ref}$ is the **acceleration feedforward (FF)**. Four configurations are run: FF on or off, crossed with $a_{max}$ = 2 m/s² (main) or no limit. The main configuration uses a tight 2 m/s² limit: above the min-snap reference's 1.57 m/s² peak, and far below the 10 m/s² impulse the straight-line stop asks for. Typical multirotors manage 5–10 m/s². The limit is therefore swept from 1.5 to 12 m/s² and to no limit. The 10 m/s² impulse is $v_{approach}/\Delta t$, a property of the 0.05 s time step, not of the airframe.

### Docking error and transfer probability

$$e_{dock}^{(i)} = \frac{1}{N_{dwell}}\sum_{k=1}^{N_{dwell}} \bigl\|\mathbf{p}(t_k) - \mathbf{p}_i^{flower}\bigr\|, \qquad P_t^{(i)} = e^{-e_{dock}^{(i)}/\sigma_{dock}}, \qquad E[N_{poll}] = \sum_i P_t^{(i)}$$

Here $\mathbf{p}$ is the **true** position. Overshoot is the largest distance past the flower along $\hat{\mathbf{d}}$ during the dwell. The script also prints the mean distance over the first and second halves of the dwell.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Flowers $F$ / orchard | 25 / 10 × 10 m, $z \in [1, 3]$ m |
| Start point / transit altitude | (0, 0, 3.5) m / 3.5 m |
| Approach elevation / standoff | 30° / 0.8 m |
| Approach entry speed $v_{approach}$ / duration $T_{seg}$ | 0.5 m/s / 1.6 s |
| Transit speed (peak) | 1.5 m/s |
| Dwell $T_{dwell}$ | 1.5 s |
| $d_{dock} = \sigma_{dock}$ | 0.1 m |
| GPS noise $\sigma_{GPS}$ | 0.1 m per axis, white, every step |
| Time step $\Delta t$ | 0.05 s |
| Velocity time constant $\tau_v$ (added) | 0.3 s |
| Position gain $K_p$ (added) | 2.0 s⁻¹ |
| Speed limit (added) | 3 m/s |
| Acceleration limit $a_{max}$ (added) | 2 m/s² (main, deliberately tight); sweep: none, 12, 10, 8, 6, 4, 3, 2, 1.5 m/s² |
| Hover above the approach point before the descent (added) | ≥ 1.0 s (docking window 8.35 s) |
| Seed of the example run | 42 |
| Monte Carlo | 100 seeds (layout + GPS noise), shared by all methods and configurations |
| Sweeps | GPS σ = 0, 0.05, 0.1, 0.2 m and $a_{max}$; 40 seeds each |

---

## Implementation

```
src/04_industrial_agriculture/s077_pollination.py   # Main simulation script
```

```bash
conda activate drones
python src/04_industrial_agriculture/s077_pollination.py
```

Runtime ≈ 1.5–2.5 min (100–139 s measured while other simulations shared the CPU). All docking windows of all seeds are flown in one vectorised loop. The climb and transit legs are not scored and are flown kinematically on their references.

---

## Results

### Monte Carlo, 100 seeds, GPS σ = 0.1 m — $E[N_{poll}]$ (of 25)

| Configuration | Minimum-snap | Straight (no FF at stop) | Straight + stop-impulse FF | Min-snap wins vs straight + FF |
|---|---:|---:|---:|---:|
| **FF, $a_{max}$ = 2 m/s² (main)** | **17.29 ± 0.33** (69.2 %) | 12.87 ± 0.37 (51.5 %) | 13.27 ± 0.37 (53.1 %) | **100/100** (+4.027 ± 0.320) |
| FF, no acceleration limit | 17.58 ± 0.31 (70.3 %) | 13.57 ± 0.37 (54.3 %) | 17.57 ± 0.31 (70.3 %) | 60/100 (+0.014 ± 0.054) |
| No FF, $a_{max}$ = 2 m/s² | 10.07 ± 0.41 (40.3 %) | 11.28 ± 0.38 (45.1 %) | 11.28 ± 0.38 (45.1 %) | 0/100 (−1.213 ± 0.198) |
| No FF, no acceleration limit | 12.79 ± 0.33 (51.2 %) | 13.15 ± 0.37 (52.6 %) | 13.15 ± 0.37 (52.6 %) | 0/100 (−0.351 ± 0.109) |

Percentages are the transfer success rate $E[N_{poll}]/25$. Without FF, the two straight-line variants are identical by construction. In the main configuration, the mean docking error is 0.0374 m for min-snap, 0.0675 m for the straight line and 0.0644 m for the straight line with FF. The mean overshoot is 0.0327, 0.0985 and 0.0922 m.

### Acceleration-limit sweep (FF, σ = 0.1 m, 40 seeds)

| $a_{max}$ (m/s²) | none | 12 | 10 | 8 | 6 | 4 | 3 | 2 | 1.5 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Minimum-snap | 17.55 | 17.55 | 17.55 | 17.55 | 17.55 | 17.55 | 17.54 | 17.26 | 14.32 |
| Straight + stop-impulse FF | 17.54 | 17.54 | 17.52 | 17.19 | 16.32 | 15.13 | 14.44 | 13.29 | 10.63 |
| Straight (no FF at stop) | 13.58 | 13.58 | 13.58 | 13.58 | 13.58 | 13.57 | 13.51 | 12.89 | 10.42 |
| Paired gap, min-snap − straight + FF | +0.011 | +0.011 | +0.025 | +0.362 | +1.231 | +2.418 | +3.102 | +3.971 | +3.686 |
| Min-snap wins vs straight + FF | 22/40 | 22/40 | 30/40 | 40/40 | 40/40 | 40/40 | 40/40 | 40/40 | 40/40 |

The tour is 52.43 m long. The mission takes 312.05 s with min-snap and 311.90 s with the straight lines; only the kinematic climb after each dwell differs slightly, so mission time is not a comparison metric.

**Key findings**:
- **With the stop fed forward and no acceleration limit, the trajectory shape does not matter.** Feeding the straight line's stop forward as a one-step deceleration lets it match min-snap: 17.57 vs 17.58 flowers, min-snap wins only 60/100 paired seeds. Noise-free for seed 42 it reaches 23.95 against min-snap's 24.99. The earlier 100/100 min-snap win came from denying the straight line this feedforward. Its 10 m/s² stop impulse is only possible with an unlimited acceleration.
- **Min-snap's advantage grows as the acceleration limit tightens, and vanishes around $v_{approach}/\Delta t$ = 10 m/s².**
  - At the deliberately tight $a_{max}$ = 2 m/s², the min-snap reference (peak 1.57 m/s²) stays flyable and the drone reaches the flower almost at rest (0.004 m/s at the start of the dwell, seed 42 noise-free). The straight line's stop is clipped to 2 m/s², so even with FF the drone is still moving at 0.402 m/s when the dwell starts and overshoots by 0.0837 m. Min-snap wins all 100 paired seeds (+4.027 flowers).
  - In the 40-seed sweep the paired gap against the FF straight line is +3.971 at 2 m/s², +2.418 at 4, +1.231 at 6, +0.362 at 8, +0.025 at 10, and +0.011 at 12 m/s² and with no limit. Min-snap wins 40/40 down to 8 m/s², then 30/40 at 10 and 22/40 at 12.
  - The gap closes at 10 m/s², the size of the one-step stop impulse. Above that, the FF straight line can execute its stop exactly.
  - That threshold is set by the 0.05 s simulation step, not by an airframe. For multirotors with the more typical 6–10 m/s² limit, the sweep gives a gap of only +1.231 to +0.025 flowers, not 4.
  - The physical point stands: a smooth reference wins only when the airframe cannot decelerate as abruptly as the straight-line stop demands.
- **Without feedforward, min-snap loses**, in both limit settings (0/100). Its reference must speed up to 0.830 m/s mid-segment and then brake, and the lagging drone overshoots more: 0.1362 m vs 0.0951 m with no limit, 0.1715 vs 0.1220 m at 2 m/s².
- **Why min-snap wins noise-free but loses with noise when there is no FF** (no limit).
  - **Single run, seed 42, noise-free:** min-snap has the larger, earlier overshoot (mean distance 0.0910 m in the first half of the dwell vs 0.0787 m) but settles closer (0.0105 vs 0.0244 m in the second half). That gives a slightly lower mean, 0.0507 vs 0.0515 m, and E[N] 15.06 vs 14.94.
  - **Same seed, σ = 0.1 m:** the noise sets a floor in the second half (0.0426 vs 0.0490 m), so the settling advantage mostly disappears. The larger first-half error then dominates: 0.1019 vs 0.0896 m, mean 0.0722 vs 0.0693 m.
  - **40-seed sweep:** min-snap wins 40/40 at σ = 0 and 0/40 at σ = 0.05, 0.1 and 0.2 m.
  - Overshoot alone does not explain the reversal: min-snap overshoots more in both cases.
- **GPS noise sets the ceiling.** In the main configuration, min-snap falls from 24.50 (σ = 0) to 20.82, 17.26 and 10.28 flowers at σ = 0.05, 0.1 and 0.2 m. The straight line with FF falls from 15.55 to 14.88, 13.29 and 8.43. Min-snap wins 40/40 at every σ.
- **Card expectations**: min-snap meets all three targets: mean $e_{dock}$ 0.0374 m (< 0.12), 69.2 % (> 35 %), 17.29 (> 9). The straight-line targets do not hold: 0.0675 m (card 0.15–0.25), 51.5 % (card < 25 %), 12.87 (card < 7). The P loop filters the white GPS noise, so the true docking error stays well below $\sigma_{GPS}$. The card's own error model (error = the GPS sample itself) gives $E\|\eta\| = 0.1596$ m and $P_t = 0.203$ for **every** method, so it cannot separate them.
- The descent curls under the approach point to join the 30° line. The lowest altitude flown in seed 42 is 0.46 m; the lowest approach point is 0.61 m.

### 3D Tour

![Tour 3D](./tour_3d.png)

### TSP Tour Map

Flowers coloured by the min-snap $P_t$ (main configuration); red ticks are the 0.8 m approach lines:

![TSP Map](./tsp_map.png)

### Docking Transient (visit 1)

Distance to the flower (log scale) from 0.5 s before the approach until the end of the dwell.
- Top left: no FF, no limit. Every method overshoots during the dwell, min-snap the most.
- Top right: FF, no limit. Min-snap and the FF straight line both settle at the flower; the straight line without FF still overshoots.
- Bottom left: FF with $a_{max}$ = 2 m/s². The FF straight line can no longer stop instantly and overshoots like the plain one; min-snap still settles.
- Bottom right: the same with GPS noise.

![Docking Transient](./docking_transient.png)

### Docking Error per Flower (seed 42, main configuration)

![Docking Error per Flower](./dock_error_per_flower.png)

### Transfer Probability per Visit (seed 42, main configuration)

![Transfer Probability](./transfer_prob.png)

### Docking Error Distribution (100 seeds, main configuration)

![Docking Error Histogram](./dock_error_hist.png)

### Monte Carlo

Top: paired seeds, min-snap vs straight line + stop-impulse FF, with and without the acceleration limit. Bottom: acceleration-limit and GPS-noise sweeps:

![Monte Carlo](./monte_carlo.png)

### Summary

$E[N_{poll}]$ for every method and configuration (100 seeds):

![Summary](./summary_bar.png)

### Animation

Seed 42, main configuration: all three drones fly the tour; flowers turn green once visited, and the running $E[N_{poll}]$ of each method is shown in the title (157 frames, 2 s per frame):

![Animation](./animation.gif)

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| $\hat{\mathbf{d}} = (0.5\cos\varphi, 0.5\sin\varphi, \sqrt{3}/2)$ described as a 30° elevation | $(\cos 30°\cos\varphi, \cos 30°\sin\varphi, \sin 30°)$ | The card's vector has a 60.0° elevation (printed by the script). Card formula and code corrected. |
| Docking error = mean $\|\eta\|$ of the GPS noise added to the commanded flower position | True distance of a point mass with a first-order velocity response ($\tau_v$ = 0.3 s) and an acceleration limit, tracking the reference with a P loop ($K_p$ = 2 s⁻¹) on GPS measurements | The card's error is the same noise for every method ($P_t$ = 0.203 each), so it cannot compare them. The card's role description already asks for "point-mass dynamics with first-order velocity response". |
| Tracking controller and actuator limits not specified | Velocity feedforward, with and without acceleration feedforward; acceleration limit 2 m/s² or none; all four combinations reported | The ranking depends on both (see Key findings), so all are shown. |
| Two methods (min-snap, straight line) | Plus a straight line whose stop is fed forward as a one-step deceleration | Gives the straight line the same feedforward as min-snap, so the comparison tests the trajectory shape, not who gets feedforward. |
| Straight line is "faster per visit" | Every method takes $T_{seg}$ = 1.6 s (card's own rule $T_{seg} = \max(d/v_{approach}, 0.5)$) | Mission time is practically identical; reported but not used as a comparison. |
| Expected straight-line $e_{dock}$ 0.15–0.25 m, rate < 25 %, $E[N]$ < 7 | 0.0675 m, 51.5 %, 12.87 (main configuration) | Reported as is (X6); card annotated. |
| Linear descent from the via point at $v_{approach}$; drone at rest at the approach point while the min-snap reference starts at 0.5 m/s | Degree-7 descent ending with velocity $v_{approach}\hat{\mathbf{d}}$ | Gives every reference a consistent initial state; the card's version starts every approach with a 0.5 m/s velocity jump. |
| Departure "vertically upward"; card code flew straight from the flower to the next via point | Vertical climb to 3.5 m, then transit | Follows the card's problem statement. |
| Transit legs part of the simulated flight | Climb and transit flown kinematically on smooth references; each docking window (hover ≥ 1 s at the via point, descent, approach, dwell) flown dynamically | Docking is the only scored part. The fixed-length windows let all seeds run in one vectorised loop. |
| Single run with seed 42 | Seed 42 for the figures, plus 100-seed Monte Carlo and 40-seed sweeps with common random numbers | Results depend on random noise (X2). |
| Added parameters | $\tau_v$ = 0.3 s, $K_p$ = 2 s⁻¹, $v_{max}$ = 3 m/s, $a_{max}$ = 2 m/s², via-point hover ≥ 1 s | Needed for the dynamic model; not in the card. $a_{max}$ = 2 m/s² is deliberately tight; the headline gap depends on it, so it is swept from 1.5 to 12 m/s² and no limit. |

---

## Extensions

1. **Coloured GPS noise**: model GPS error as a slowly varying bias plus white noise; a bias cannot be averaged out during the dwell.
2. **Dwell-time optimisation**: shorten $T_{dwell}$ once $e_{dock}$ settles, to trade transfer probability for tour time.
3. **Time-optimal min-snap**: choose $T_{seg}$ per flower to minimise time subject to the acceleration limit, which turns the method comparison into a speed/accuracy trade-off.
4. **Trapezoidal straight line**: a straight line with a deceleration ramp at $a_{max}$ is the natural middle ground between the two references.
5. **Wind gusts during the dwell** with a PI position loop.

---

## Related Scenarios

- Prerequisites: [S063 Single-Plant Precision Spraying](../../../scenarios/04_industrial_agriculture/S063_precision_spraying.md), [S064 Greenhouse Interior Precision Flight](../../../scenarios/04_industrial_agriculture/S064_greenhouse.md)
- Cross-reference: [S001 Basic Intercept](../../../scenarios/01_pursuit_evasion/S001_basic_intercept.md), [S021 Point Delivery](../../../scenarios/02_logistics_delivery/S021_point_delivery.md)
- Follow-ups: [S078 Harvester Cooperative Guidance](../../../scenarios/04_industrial_agriculture/S078_harvester_guidance.md)
