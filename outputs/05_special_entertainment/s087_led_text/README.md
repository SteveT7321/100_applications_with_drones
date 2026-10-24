# S087 Aerial Light Show Text Display

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: A drone swarm takes off from the ground and spells "DRONE" at $z = 50$ m in a 6 × 5 bitmap font (one drone per lit pixel, 2 m pixel pitch, 2 blank columns between characters), then morphs into "SHOW". Before each transition the drones are matched to the lit pixels with the Hungarian algorithm (minimum total distance). Each drone flies to its pixel with a PID acceleration command limited to 3 m/s² and 5 m/s. During a transition a priority rule gives the drone with the longer remaining path right of way: a drone within $r_{sep} = 1.5$ m of a higher-priority drone is limited to 50 % speed.

**Fleet size**: the card's font gives 74 lit pixels for "DRONE" and 58 for "SHOW", not 30 for each (verified by the script). The fleet therefore has 74 drones. For "SHOW" the 16 spare drones switch their LEDs off and park in a row 5 m below the display.

**Objective**: maximise the visual fidelity $V$ (share of lit pixels with their drone within 0.5 m) quickly, with low travel cost and without collisions.

**Configurations** (the same staging layout for every configuration in each seed):
1. **Hungarian + priority yield** (card).
2. **Hungarian, no avoidance** (added): shows what the priority rule contributes.
3. **NN greedy + priority yield**: the card's cost baseline (each drone in index order takes the nearest free pixel), here also flown.

---

## Mathematical Model Summary

Pixel $(i, j)$ of character $c$: $\mathbf p = \bigl((c(W + g) + j)\,d_{px},\ (H - 1 - i)\,d_{px},\ z_{show}\bigr)$ with $W = 5$, $H = 6$, $g = 2$, $d_{px} = 2$ m. "DRONE" is 64 m wide, "SHOW" 50 m.

Assignment: $\sigma^* = \arg\min_\sigma \sum_k \|\mathbf q_k - \mathbf p_{\sigma(k)}\|$ (`linear_sum_assignment`), for the ascent (from the staging positions) and again for the morph (from the hover positions after the ascent). Cost penalty of greedy: $\Delta_{cost} = (D_{greedy} - D_{Hungarian})/D_{Hungarian}$.

Dynamics per drone (double integrator):

$$\mathbf a = \operatorname{sat}_{a_{max}}\bigl(K_p\mathbf e + K_i\!\int\!\mathbf e\,dt - K_d\mathbf v\bigr), \qquad \mathbf v^* = \operatorname{sat}_{v_{cap}}(\mathbf v + \mathbf a\Delta t), \qquad \mathbf v \leftarrow \mathbf v + \operatorname{sat}_{a_{max}\Delta t}(\mathbf v^* - \mathbf v), \qquad \mathbf q \leftarrow \mathbf q + \mathbf v\Delta t$$

The integral is frozen while $\|\mathbf a\|$ would exceed $a_{max}$. $v_{cap} = 5$ m/s, or 2.5 m/s for a yielding drone. When a drone starts yielding it brakes down to 2.5 m/s at no more than $a_{max}$, so the realised acceleration $\|\Delta\mathbf v\|/\Delta t$ never exceeds 3 m/s² (asserted). Priority: rank drones by remaining distance (largest first); drone $k$ yields if any higher-ranked drone is within 1.5 m.

Metrics per transition (40 s simulated each): $V(t)$ and the first time $V \ge 0.95$; the first time all drones are within 0.5 m; minimum pairwise separation; number of drone pairs that ever come within the 0.5 m collision radius; drone-steps spent yielding, and how many of them the 2.5 m/s cap actually limited the speed. For the showcase morph every close pair is classified by what the two drones were doing at the moment of closest approach, and the overshoot of each drone past its own pixel (distance beyond the pixel along its travel direction) is recorded.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Fleet size | 74 (card: 30, see Deviations) |
| Words / lit pixels | "DRONE" 74, "SHOW" 58 (+ 16 dark spares at z = 45 m, y = 5 m, 2 m apart) |
| Glyph / pixel pitch / gap | 6 × 5 / 2.0 m / 2 columns |
| Display altitude | 50 m |
| PID $K_p$ / $K_i$ / $K_d$ | 2.0 / 0.05 / 0.8 |
| $v_{max}$ / $a_{max}$ (commanded and realised) | 5.0 m/s / 3.0 m/s² |
| $r_{sep}$ / yield speed | 1.5 m / 50 % |
| Fidelity tolerance / acceptance | 0.5 m / V = 0.95 |
| Collision radius (domain constant) | 0.5 m |
| Time step / phase length | 0.05 s / 40 s |
| Staging volume (added) | x ∈ [0, 64], y ∈ [−2, 12], z ∈ [0.5, 5] m, pads ≥ 1.5 m apart |
| Monte Carlo (added) | 20 staging layouts (seeds 0–19); showcase seed 0 |

---

## Implementation

```
src/05_special_entertainment/s087_led_text.py   # Main simulation script
```

```bash
conda activate drones
python src/05_special_entertainment/s087_led_text.py
```

Runtime ≈ 1 min on a heavily loaded machine.

---

## Results

**Showcase layout (seed 0)**:

| Configuration | Phase | Cost | $V \ge 0.95$ at | All within 0.5 m at | Min separation | Pairs < 0.5 m | Yielding |
|---|---|---:|---:|---:|---:|---:|---:|
| Hungarian + yield | ascent | 3505.8 m | 15.55 s | 15.60 s | 1.435 m | 0 | 0.12 % |
| Hungarian + yield | morph | 708.0 m | 9.65 s | 13.50 s | **0.007 m** | **13** | 1.66 % |
| Hungarian, no avoidance | ascent | 3505.8 m | 15.55 s | 15.60 s | 1.437 m | 0 | — |
| Hungarian, no avoidance | morph | 708.0 m | 9.65 s | 15.35 s | 0.007 m | 16 | — |
| NN greedy + yield | ascent | 3540.8 m | 17.10 s | 18.10 s | 0.540 m | 0 | 0.94 % |
| NN greedy + yield | morph | 771.8 m | 13.85 s | 16.10 s | 0.015 m | 39 | 3.26 % |

At the end of each 40 s phase every drone is within 1.91 cm of its pixel (worst case: greedy ascent), so the final fidelity is $V = 1$ for every configuration. The realised acceleration peaks at exactly 3.000 m/s² in every configuration.

$\Delta_{cost}$ of NN greedy: 1.00 % (ascent) and 8.71 % (morph) for seed 0; over 20 layouts 1.68 % (0.47–3.38 %) for the ascent and 7.36 % (3.91–10.00 %) for the morph.

**How often the yield rule actually acts** (showcase; drone-steps spent yielding / drone-steps in which the 2.5 m/s cap really limited the speed): Hungarian ascent 72 / 72, Hungarian morph 983 / 181, greedy ascent 554 / 534, greedy morph 1932 / 317. In the Hungarian morph the cap binds in only 181 of the 983 yielding drone-steps; in the rest the yielding drone is already slower than 2.5 m/s.

**Monte Carlo over 20 staging layouts** (mean ± std):

| Configuration | Ascent $V \ge 0.95$ | Ascent all within 0.5 m | Ascent min sep | Ascent pairs < 0.5 m | Morph $V \ge 0.95$ | Morph all within 0.5 m | Morph min sep | Morph pairs < 0.5 m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Hungarian + yield | 15.54 ± 0.04 s | 15.85 ± 0.49 s | 1.453 ± 0.165 m | 0.00 | 9.65 ± 0.00 s | 13.78 ± 0.56 s | 0.009 ± 0.001 m | 13.00 ± 0.00 |
| Hungarian, no avoidance | 15.54 ± 0.03 s | 15.63 ± 0.08 s | 1.495 ± 0.082 m | 0.00 | 9.65 ± 0.00 s | 15.35 ± 0.00 s | 0.009 ± 0.001 m | 16.00 ± 0.00 |
| NN greedy + yield | 17.02 ± 0.93 s | 18.52 ± 1.11 s | 0.224 ± 0.160 m | 3.10 ± 2.02 | 10.38 ± 1.40 s | 16.50 ± 0.75 s | 0.026 ± 0.014 m | 29.30 ± 4.50 |

All configurations converge in all 20 layouts. Layouts with at least one pair closer than 0.5 m: Hungarian + yield 0 (ascent) / 20 (morph), Hungarian free 0 / 20, greedy 18 / 20. Per layout, the Hungarian ascent reaches $V \ge 0.95$ before the greedy one in 20/20 layouts.

**The Hungarian morph is essentially one case repeated.** The ascent always ends in the same "DRONE" formation (to within centimetres), so the morph starts from almost the same state in every layout: without avoidance the morph timing and close-pair count are the same in all 20 layouts (std 0.00), and with yielding $T_{all}$ takes only 2 distinct values (std 0.560 s). The per-layout counts for the morph ("larger morph min sep in 15/20, ties 5", "faster morph in 20/20") are therefore not 20 independent confirmations, and the morph comparison between yield and no avoidance rests on essentially one case.

**Close pairs in the showcase morph** (Hungarian + yield; first matching class at the moment of closest approach). 58 lit drones: 26 stay on their pixel (move < 0.1 m) and 32 move; their 32 straight paths have **0 proper crossings**, as expected for a minimum-total-distance assignment.

| Class | Pairs | Closest example |
|---|---:|---|
| A drone has overshot its own pixel by > 0.5 m | 7 | drones 19 (50,2)→(48,2) and 42 (56,2)→(50,2): 0.007 m at t = 4.05 s |
| Spare drone involved (converging on the parking row, slots 2 m apart) | 3 | drones 54 (56,8)→(18,5,45) and 55 (62,10)→(16,5,45): 0.304 m at t = 10.75 s |
| Moving drone's path runs within 0.5 m of the other drone's pixel | 2 | drones 27 (28,6)→(22,6) and 71 (42,8)→(20,6): 0.200 m at t = 4.85 s |
| Moving drone passes a stationary lit drone | 1 | drones 4 (56,0)→(28,2) and 67 staying at (50,0): 0.429 m at t = 2.00 s |
| Follower on the same row / straight paths cross / other | 0 / 0 / 0 | — |

Without avoidance the classes are the same except 6 spare-drone pairs instead of 3.

**Overshoot**: every one of the 48 drones that move in the morph (32 lit + 16 spares) passes its own pixel by more than 0.5 m; the mean of the per-drone maximum overshoot is 2.11 m with yielding and 2.35 m without, and the largest is 3.00 m. Braking from 5 m/s at 3 m/s² takes 4.17 m, but this PD ($\zeta = K_d/(2\sqrt{K_p}) \approx 0.28$, hand calculation) starts braking too late, so a drone typically ends up on or beyond the neighbouring pixel 2 m further on and comes back.

**Key findings**:
- **The card's word sizes are wrong**: its own font gives 74 and 58 lit pixels, so the show needs 74 drones and 16 of them are dark during "SHOW".
- **The Hungarian assignment pays off mainly in the ascent's safety and speed.** Its ascent is collision-free in every layout (closest pair 1.453 m on average), while the naive greedy assignment brings a pair within 0.5 m in 18/20 layouts and reaches $V \ge 0.95$ about 1.5 s later (17.02 vs 15.54 s). The distance saving is small for the ascent (1.68 %) because every drone must climb 45–49.5 m (from the staging heights), and larger for the morph (7.36 %).
- **The morph is not collision-free with any configuration, and crossing paths are not the reason.** In "DRONE" → "SHOW", 13 drone pairs (Hungarian + yield) come within 0.5 m, the closest to 0.007 m, although no two lit drones' straight paths cross. The main cause is **overshoot**: the underdamped PID carries each moving drone 2.11 m (on average) past its pixel, into the neighbouring pixel's space, where it meets the drone that is settling there (7 of 13 pairs, including the closest one). The other close pairs are spares converging on the parking row (3), paths that run close to another drone's pixel (2) and a drone passing a stationary one (1). A real show needs a well-damped (or trajectory-planned) arrival and time- or altitude-separated transitions (see Extensions).
- **The yield rule helps only a little, and in this one morph case.** It cuts the close pairs from 16 to 13 and the morph settling time from 15.35 s to 13.50 s (showcase), mainly by slowing drones and so reducing overshoot (mean per-drone maximum 2.35 → 2.11 m); the minimum separation is the same 0.007 m with and without it. The cap actually limits a drone's speed in only 181 of 983 yielding drone-steps of the morph. The previous version of this README reported a larger benefit that came from an instantaneous 5 → 2.5 m/s speed cut (50 m/s²); with braking limited to 3 m/s² the benefit is as stated here.
- **Final visual fidelity is not a discriminating metric**: $V = 1$ for every configuration once the formation has settled, so the comparison uses the time to $V \ge 0.95$ instead.

**Physical checks** (asserted at every step): speed ≤ 5 m/s, commanded **and realised** acceleration ≤ 3 m/s², altitude > 0. Close approaches are counted, not prevented, because preventing them is the controller's job.

### Formations

Top-down view of the drones at the end of each phase (Hungarian + yield, seed 0): grey discs = lit pixels, red = drones with LED on, black crosses = the 16 dark spares 5 m below "SHOW":

![Formations](./formations.png)

### Assignment Cost

Left: total distance of the Hungarian and greedy assignments for both transitions (seed 0). Right: $\Delta_{cost}$ over the 20 layouts:

![Assignment Cost](./assignment_cost.png)

### Visual Fidelity

$V(t)$ during the ascent and the morph (seed 0) for the three configurations:

![Fidelity](./fidelity.png)

### Monte Carlo

Morph time to $V \ge 0.95$, morph time until all drones are within 0.5 m, and the minimum separation in the morph and the ascent (red line = 0.5 m collision radius):

![Monte Carlo](./monte_carlo.png)

### Animation

3D view of the "DRONE" → "SHOW" morph (Hungarian + yield, seed 0) until 2 s after all drones are within 0.5 m: red = LED on, pink = yielding, black = spare drone; grey = target pixels. 79 frames at 20 fps, one frame every 0.2 s:

![Animation](./animation.gif)

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| $N = 30$; "DRONE" and "SHOW" each have 30 lit pixels | $N = 74$; "SHOW" uses 58 drones and 16 dark spares parked 5 m below | The card's font gives 74 and 58 lit pixels (printed by the script). Card corrected. |
| `pid_step`: per-axis clipping, a derivative term mixing `err/dt` and velocity; integral always accumulates | $\mathbf a = K_p\mathbf e + K_i\int\mathbf e - K_d\mathbf v$, norm-clipped; integral frozen while saturated | The card's derivative term is ill-formed; without anti-windup the long saturated climb winds up the integral (as in S085). |
| Yield implemented as a half time step in the PID update | Speed cap halved (2.5 m/s) for yielding drones, reached by braking at ≤ 3 m/s² | A half time step is not half speed; an instant speed cut would need 50 m/s². Card note added. |
| Straight-line interpolation $\tau_k(t)$ in the math; PID in the code | PID-driven flight (as in the code) | The text and code disagree; PID with limits is the more physical choice. |
| Fidelity $V$ at display time | $V(t)$, time to $V \ge 0.95$, and time until all drones are within 0.5 m | $V$ at display time is 1 by construction. |
| Staging uniform in [0, 60] × [0, 12] × [0, 5] m | [0, 64] × [−2, 12] × [0.5, 5] m with pads ≥ 1.5 m apart | Room for 74 drones; no drone starts on the ground plane or inside another. |
| One run (seed 42) | 20-layout Monte Carlo, three configurations flown with the same layouts | The card asks for a comparison; results depend on the layout. |
| Output names not fixed; GIF of the morph | `formations.png`, `assignment_cost.png`, `fidelity.png`, `monte_carlo.png`, `animation.gif` | Project naming convention. The formation figure is a top-down 2D view (the display is planar); the animation is 3D. |
| Collision avoidance by the priority queue | Same rule, but close approaches are measured and classified: 13 pairs < 0.5 m in the showcase morph, mostly from overshoot | Reported as found: the rule does not prevent collisions in the morph. |

---

## Extensions

1. **Damped arrival**: raise $K_d$ (or plan a minimum-jerk trajectory to each pixel) so drones stop at their pixel instead of overshooting by about 2 m into the neighbouring one, the main cause of the close pairs here.
2. **Altitude-layered morph**: send each moving drone through its own altitude layer (e.g. ±1 m by priority) and descend into the display plane only at its pixel, so it cannot meet stationary or settling neighbours on the way.
3. **Separate the spares**: give the dark spares a parking row with wider spacing, or a different altitude per slot, so they do not converge on each other.
4. **Font with a fixed pixel count**: design glyphs or use a dot-matrix that pads every word to the same $N$.
5. **Vertical display plane** facing the audience instead of a horizontal one.

---

## Related Scenarios

- Prerequisites: [S083 Light Show Single-Drone Test](../../../scenarios/05_special_entertainment/S083_light_show_single.md), [S085 Drone Light Matrix Positioning](../../../scenarios/05_special_entertainment/S085_light_matrix.md)
- Next: [S088 Formation Geometric Shape Morphing](../../../scenarios/05_special_entertainment/S088_formation_morphing.md)
- Cross-reference: [S018 Multi-Target Intercept](../../../scenarios/01_pursuit_evasion/S018_multi_target_intercept.md), [S019 Dynamic Target Reassignment](../../../scenarios/01_pursuit_evasion/S019_dynamic_reassignment.md), [S040 Fleet Load Balancing](../../../scenarios/02_logistics_delivery/S040_fleet_load_balancing.md)
