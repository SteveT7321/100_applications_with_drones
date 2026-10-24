# S071 Bridge Underside Structural Inspection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: A bridge deck spans 50 m (x) by 10 m (y); its underside is the plane z = 8 m. A drone flies boustrophedon passes 0.5 m below the deck (z = 7.5 m) at 1 m/s. An upward rangefinder (noise σ = 0.02 m) measures the clearance to the deck, and a PD controller holds the 0.5 m standoff. An upward camera with a 45° half-angle sees a 1 × 1 m square footprint on the deck. Thirty cracks are seeded at random positions on the deck.

**Objective** (card): cover 100 % of the underside, stay below the safety ceiling z_max = 7.8 m (0.2 m below the deck), detect as many cracks as possible, and log each crack's (x, y) position. Card targets: coverage ≥ 99 %, crack detection ≥ 90 %, zero safety violations.

**Questions studied**:
1. **Nominal mission**: the card's plan and seeds, with a 0.1 m horizontal position error (the card's `GPS error` parameter).
2. **Strip overlap**: coverage and detection for 0 %, 10 %, 20 % and 30 % overlap, as a 50-seed Monte Carlo with common random numbers (same cracks, same position-error and noise sequences for every overlap).
3. **Safety margin**: how much rangefinder noise the 0.2 m ceiling margin tolerates, with and without the card's emergency-descent override (20 seeds, common random numbers).

---

## Mathematical Model Summary

### Height controller

The height error uses the measured upward range $d_{meas} = (z_{deck} - z) + \eta$, $\eta \sim \mathcal{N}(0, \sigma_{range}^2)$:

$$e_z = d_{meas} - d_{target}, \qquad u_z = K_p\,e_z + K_d\,\dot e_z = K_p\,e_z - K_d\,\dot z$$

with $K_p = \omega_n^2 = 16\ \text{s}^{-2}$ and $K_d = 2\omega_n = 8\ \text{s}^{-1}$ (critically damped, $\omega_n = 4$ rad/s). The vertical axis is a double integrator, $\dot v_z = u_z$, $\dot z = v_z$ (semi-implicit Euler, $\Delta t = 0.05$ s). When $z > z_{max}$ the command is replaced by the card's emergency descent $u_z^{emg} = -K_{emg}(z - z_{max})$, $K_{emg} = 30\ \text{s}^{-2}$. A safety violation is one timestep with $z > z_{max}$.

### Strip geometry

$$h_{strip} = d_{target}\tan 45° = 0.5\ \text{m}, \qquad w_{strip} = 1.0\ \text{m}, \qquad \Delta y = w_{strip}(1 - \rho_{ov}) = 0.9\ \text{m}$$

Edge-aligned plan (used): $y_k = h_{strip} + (k-1)\Delta y$, last pass clamped to $W - h_{strip}$, with

$$N_{pass} = \left\lceil \frac{W - w_{strip}}{\Delta y} \right\rceil + 1 = 11 \quad (\rho_{ov} = 0.1)$$

### Horizontal position error (added)

Without GPS under the deck, the drone's horizontal estimate drifts. The true position is the planned position plus a first-order Gauss–Markov error with stationary standard deviation $\sigma_{nav} = 0.1$ m per axis and correlation time $\tau = 20$ s:

$$\mathbf{e}_{k+1} = \phi\,\mathbf{e}_k + \sigma_{nav}\sqrt{1 - \phi^2}\;\boldsymbol{\xi}_k, \qquad \phi = e^{-\Delta t/\tau}$$

Coverage and detection use the true position. The crack log uses the estimated position (true minus $\mathbf{e}$), so each logged crack is off by $\|\mathbf{e}(t_{det})\|$.

### Detection and coverage

A crack is inside the footprint when $|x - x_i| \le h_{strip}$ and $|y - y_i| \le h_{strip}$; at each timestep inside, it is detected with probability 0.9. The deck is a 500 × 100 grid of 0.1 m cells; a cell counts as covered when its centre is inside the footprint at some timestep. Coverage $C$ is the covered fraction.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Deck size / height | 50 × 10 m, $z_{deck}$ = 8.0 m |
| Nominal height / standoff | $z_{nom}$ = 7.5 m, $d_{target}$ = 0.5 m |
| Safety ceiling | $z_{max}$ = 7.8 m (0.2 m margin) |
| Rangefinder noise | $\sigma_{range}$ = 0.02 m |
| PD gains | $K_p$ = 16 s⁻², $K_d$ = 8 s⁻¹ |
| Emergency descent gain | $K_{emg}$ = 30 s⁻² |
| Camera FOV half-angle | 45° → $h_{strip}$ = 0.5 m, $w_{strip}$ = 1.0 m |
| Strip overlap (nominal) | 10 % → $\Delta y$ = 0.9 m |
| Passes (edge-aligned) | 11 (card formula: 12) |
| Cruise speed | 1.0 m/s |
| Crack count / radius | 30, U(0.05, 0.20) m (radius is not used by the footprint test) |
| Crack area | x ∈ U(1, 49) m, y ∈ U(0.5, 9.5) m |
| Detection probability per timestep | 0.90 |
| Timestep | 0.05 s |
| Coverage grid | 0.1 m |
| Nominal seeds | cracks 7, noise 42 (card) |
| Horizontal error $\sigma_{nav}$ / $\tau$ (added) | 0.1 m / 20 s |
| Overlap study (added) | ρ ∈ {0, 10, 20, 30} %, 50 seeds (1000–1049) |
| Noise stress test (added) | $\sigma_{range}$ ∈ {0.02, 0.10, 0.20, 0.30, 0.35, 0.40} m, 20 seeds (1000–1019), override on/off |

---

## Implementation

```
src/04_industrial_agriculture/s071_bridge_inspection.py   # Main simulation script
```

```bash
conda activate drones
python src/04_industrial_agriculture/s071_bridge_inspection.py
```

Runtime ≈ 45 s (measured while other simulations shared the machine).

---

## Results

### Nominal mission (10 % overlap, $\sigma_{nav}$ = 0.1 m, card seeds)

| Metric | Value | Card target |
|--------|------:|------------:|
| Coverage | 97.59 % (1204 of 50 000 cells unseen) | ≥ 99 % |
| Crack detection | 29/30 = 96.7 % | ≥ 90 % |
| Safety violations | 0 | 0 |
| Highest point / minimum clearance | 7.5161 m / 0.4839 m | ≥ 0.2 m |
| Height std / max deviation | 4.54 mm / 16.1 mm | — |
| Flight time / path length | 559.0 s / 559.0 m | — |
| Logged crack position error | mean 122 mm, max 273 mm | — |
| Largest horizontal error during the flight | 347 mm | — |

Without the horizontal error, the same plan reaches 100.00 % coverage and 100.0 % detection. A crack inside a pass then spends at least 20 timesteps in the footprint, so the chance of missing it is at most $0.1^{20} = 10^{-20}$. The per-timestep detection probability therefore plays no role here: **a crack is detected if and only if it lies in covered area**. The one missed crack, at (15.55, 2.91) m, was never inside the footprint (0 frames); it lies in a gap between passes 3 and 4.

### Strip overlap study (50 seeds, common random numbers)

| Overlap | Passes | Flight time | Coverage mean ± std (min) | Detection mean ± std | Seeds with C ≥ 99 % | Seeds with 100 % detection | Logged crack error |
|---|---:|---:|---|---|---:|---:|---:|
| 0 % | 10 | 509.0 s | 96.47 ± 0.74 % (94.43 %) | 96.1 ± 3.7 % | 0/50 | 17/50 | 123 mm |
| **10 % (card)** | 11 | 559.0 s | 98.51 ± 0.65 % (96.79 %) | 99.3 ± 1.7 % | 11/50 | 41/50 | 125 mm |
| 20 % | 13 | 659.0 s | 99.32 ± 0.46 % (97.89 %) | 99.8 ± 0.8 % | 37/50 | 47/50 | 126 mm |
| 30 % | 14 | 709.0 s | 99.49 ± 0.45 % (98.20 %) | 99.9 ± 0.5 % | 44/50 | 49/50 | 123 mm |

Paired coverage comparison against 10 % overlap (same seed): 0 % loses on all 50 seeds (mean −2.05 pp); 20 % wins 48, loses 2 (+0.80 pp); 30 % wins 48, loses 2 (+0.98 pp).

Share of the unseen cells that lie in the 0.5 m border band along the deck edges (the band is 11.8 % of the deck): 0 % → 17 %, 10 % → 37 %, 20 % → 68 %, 30 % → 90 %. In the nominal run it is 52.4 %.

### Safety stress test (20 seeds per noise level, 559 s mission)

| $\sigma_{range}$ | Override | Violation steps, mean (runs with any) | Highest z, mean | Deck strikes |
|---:|:--:|---|---:|---:|
| 0.02 m | on / off | 0.0 (0/20) / 0.0 (0/20) | 7.5155 / 7.5155 m | 0 / 0 |
| 0.10 m | on / off | 0.0 (0/20) / 0.0 (0/20) | 7.5777 / 7.5777 m | 0 / 0 |
| 0.20 m | on / off | 0.0 (0/20) / 0.0 (0/20) | 7.6555 / 7.6555 m | 0 / 0 |
| 0.30 m | on / off | 0.0 (0/20) / 0.0 (0/20) | 7.7332 / 7.7332 m | 0 / 0 |
| 0.35 m | on / off | 1.8 (3/20) / 0.3 (3/20) | 7.7828 / 7.7721 m | 0 / 0 |
| 0.40 m | on / off | 13.0 (15/20) / 2.2 (15/20) | 7.8685 / 7.8109 m | 1/20 / 0 |

**Key findings**:
- **Coverage misses the card's 99 % target at the card's 10 % overlap once the position is uncertain.** With a perfect position the plan covers 100 % by construction. With a 0.1 m position error, the nominal mission covers 97.59 % and only 11 of 50 seeds reach 99 %. The 10 % overlap is 0.1 m, the same size as the position error, so two adjacent passes that drift apart leave a gap (the white lines in the coverage map).
- **Crack detection meets the card's 90 % target** (96.7 % nominal, 99.3 ± 1.7 % over 50 seeds at 10 % overlap). Detection follows coverage exactly, because a covered crack is seen for at least 20 timesteps.
- **20 % overlap is the knee of the trade-off.** Going from 10 % to 20 % overlap adds 2 passes (559.0 → 659.0 s) and raises mean coverage by 0.80 pp, winning on 48 of 50 seeds; 37/50 seeds then reach 99 %. Going on to 30 % (709.0 s) lifts mean coverage only from 99.32 % to 99.49 %. At 30 % overlap, 90 % of the remaining unseen cells are in the border band: the edge passes are centred exactly 0.5 m from the deck edge, so any outward drift of the drone pulls the footprint off the edge, and more overlap between passes cannot fix that. Pushing the two edge passes outward (or widening their footprint) would be the next step.
- **Height hold is far inside the margin.** With $\sigma_{range}$ = 0.02 m, z has a standard deviation of 4.54 mm and the highest point is 7.5161 m, so the 0.2 m margin is never approached and the violation count is zero for every overlap and seed. This metric only becomes informative under stress: violations first appear at $\sigma_{range}$ = 0.35 m (3 of 20 runs), against the card's 0.02 m sensor noise.
- **The card's emergency override makes violations worse, not better.** Under the same noise, the override raises the mean violation count from 0.3 to 1.8 steps at 0.35 m and from 2.2 to 13.0 steps at 0.40 m, and causes the only deck strike. Two reasons, both from the card's formula: (1) it is weaker than the PD law it replaces. Without noise and at $\dot z = 0$, the two restoring accelerations are equal only at z = 8.143 m, above the deck; at z = 7.9 m the override gives −3.0 m/s² against −6.4 m/s² for the PD law. (2) It has no damping term, so an upward velocity is not braked.
- **Crack positions are logged with about 0.12 m error** (mean 122 mm nominal; 123–126 mm in the overlap study), which matches the mean magnitude of a 2D error with σ = 0.1 m per axis (analytic value $0.1\sqrt{\pi/2} \approx 0.125$ m, not printed by the script). The overlap does not change it.

### 3D Flight Path

Red: the true flight path 0.5 m below the translucent grey deck (viewed from below). Faint blue lines on the deck are the strip boundaries. Green stars are detected cracks; the grey circle is the missed crack:

![3D flight path](./flight_path_3d.png)

### Coverage Map

Top-down map of the deck. Green cells were seen at least once; white cells were never seen. The thin white lines are gaps between adjacent passes, and the white slivers along y = 0 and y = 10 are the edge losses. Red crosses: detected cracks; the black circle: the missed crack:

![Coverage map](./coverage_map.png)

### Height Time Series

Top: z(t) with the nominal height (blue dashed), the safety ceiling (red dashed) and the deck (grey). Bottom: zoom around 7.5 m with the ±σ_range band:

![Height time series](./height_timeseries.png)

### Strip-by-Strip Coverage

Cumulative coverage after each of the 11 passes; labels are the increment of each pass in percentage points. The final bar is 97.59 %, below the 99 % target line:

![Strip coverage](./strip_coverage.png)

### Overlap Study

Coverage and detection rate over 50 seeds for each overlap (red dashed: card targets), and the flight time of each plan:

![Overlap study](./overlap_study.png)

### Safety Sweep

Left: mean violation steps per mission (symmetric log scale). Right: mean highest point reached. Red: with the card's emergency override; blue: PD only:

![Safety sweep](./safety_sweep.png)

### Animation

Top-down view of the nominal mission (113 frames, one every 5 s of flight). The coverage map fills in green behind the drone (red dot, red square = camera footprint); cracks turn into green stars when detected, with a gold ring on the frame of detection:

![Animation](./animation.gif)

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| $N_{pass} = \lceil W/\Delta y \rceil = 12$, $y_k = \Delta y/2 + (k-1)\Delta y$ | 11 edge-aligned passes, $y_k = 0.5 + 0.9(k-1)$ m | The card's 12th pass is at y = 10.35 m, 0.35 m outside the deck, where the upward rangefinder has nothing to range on. Edge-aligned passes give the same 100 % coverage (both printed as 100.00 % without position error) with one pass fewer: 559.0 m instead of 609.9 m. Card updated with a note. |
| Card code `state[5] = uz * DT` | $v_z \mathrel{+}= u_z\,\Delta t$ | $u_z$ is an acceleration; the card's line replaces the velocity by one step's velocity increment instead of integrating it. Card code corrected. |
| Horizontal motion is error-free; "GPS error (horizontal) 0.1 m" is listed but not used | Gauss–Markov horizontal error, σ = 0.1 m per axis, τ = 20 s (added parameter) | Without it, coverage is 100 % and detection is 100 % by construction (both printed), so the card's coverage and detection metrics could not discriminate anything. τ is our choice; the card gives none. |
| Waypoint follower with a 0.1 m arrival tolerance | Constant-speed sampling of the waypoint polyline | Same path, without the tolerance shortening each pass. |
| Emergency override triggered by $z(t) > z_{max}$ | Same, using the true z | The card does not say which sensor triggers it; we assume an ideal height reading for the trigger, which favours the override. It still does worse (see findings). |
| Strip chart "12 bars" | 11 bars | One bar per pass of the 11-pass plan. |
| Single nominal run | Nominal run plus a 50-seed overlap study and a 20-seed noise stress test, all with common random numbers | Needed to show whether the card targets hold beyond one seed and whether the zero-violation result has any margin. |
| Crack radius r ∈ U(0.05, 0.20) m | Drawn but not used | The card's footprint test uses the crack centre only; kept as in the card. |
| Animation "flashes" on detection | Gold ring on the frame in which a crack is detected | Same meaning. |

---

## Extensions

1. **GPS-denied lateral drift with visual odometry**: replace the stationary 0.1 m error by a random walk (σ_drift = 0.05 m/√s) and use detected cracks as landmarks to bound the drift.
2. **Edge-pass offset**: shift the two edge passes outward by about 2σ_nav (still under the deck) to remove the border-band loss that dominates at 20–30 % overlap.
3. **Cambered deck**: $z_{deck}(x) = 8.0 + 0.1\sin(\pi x/L)$; the controller must follow a slowly varying ceiling.
4. **Damped emergency override**: add a $-K_d\dot z$ term and a stronger gain, and repeat the stress test.
5. **Size-dependent detection**: detection probability as a function of crack radius (as in S044), with detection rate per size bin.
6. **Two drones**: split the width in two halves to halve the inspection time.

---

## Related Scenarios

- Prerequisites: [S044 Wall Crack Inspection](../../../scenarios/03_environmental_sar/S044_wall_crack.md), [S061 Power Line Inspection](../../../scenarios/04_industrial_agriculture/S061_power_line.md), [S065 Building 3D Modeling — Multi-Angle Scan Path](../../../scenarios/04_industrial_agriculture/S065_3d_scan_path.md)
- Next: [S072 Oil Pipeline Leak Detection](../../../scenarios/04_industrial_agriculture/S072_pipeline_leak.md), [S074 Mine 3D Mapping](../../../scenarios/04_industrial_agriculture/S074_mine_mapping.md)
- Cross-reference: [S067 Spray Coverage Overlap Optimization](../../../scenarios/04_industrial_agriculture/S067_spray_overlap.md), [S064 Greenhouse Interior Precision Flight](../../../scenarios/04_industrial_agriculture/S064_greenhouse.md), [S048 Full-Area Coverage Scan (Lawnmower)](../../../scenarios/03_environmental_sar/S048_lawnmower.md)
