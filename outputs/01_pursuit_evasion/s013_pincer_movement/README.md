# S013 Pincer Movement

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: Two pursuers start behind the evader and try to take positions on opposite sides of it (pincer slots at φ = 0° and 180°, i.e. ahead of and behind the evader along x) with a shrinking offset R(t). The evader reacts to the pursuers every step.

**Comparison**: coordinated pincer vs two independent pure-pursuit drones — same start positions, same speeds, same controller — against two evaders:
- **perpendicular** (card): flies at 90° to the direction from the pursuers' centroid to itself;
- **flee**: flies straight away from the pursuers' centroid.

---

## Mathematical Model

### Coordinated Pincer Targets (card)

$$\mathbf{p}_{tgt,i}(t) = \mathbf{p}_E + R(t)\,[\cos\phi_i,\ \sin\phi_i,\ 0], \qquad \phi_1 = 0°,\ \phi_2 = 180°$$

$$R(t) = \max(R_0 - V_{shrink} \cdot t,\; R_{min})$$

Once R reaches R_min (t = (3.0 − 0.5)/0.3 = 8.33 s) the pincer **closes**: both pursuers fly straight at the evader (terminal phase). Without this phase the pursuers would park 0.5 m from the evader and never reach the 0.15 m capture radius.

### Independent Pure Pursuit

Both pursuers target $\mathbf{p}_E$ directly.

Both strategies use the same saturated go-to-target command: fly at V_P toward the target, or stop exactly on it if it is reachable within one step.

### Evader

$$\mathbf{v}_E = V_E\,\mathbf{R}_{+90°}\,\frac{\mathbf{p}_E - \bar{\mathbf{p}}_P}{\|\mathbf{p}_E - \bar{\mathbf{p}}_P\|}\ \ \text{(perpendicular, card)}, \qquad \mathbf{v}_E = V_E\,\frac{\mathbf{p}_E - \bar{\mathbf{p}}_P}{\|\mathbf{p}_E - \bar{\mathbf{p}}_P\|}\ \ \text{(flee)}$$

with $\bar{\mathbf{p}}_P = \tfrac12(\mathbf{p}_{P_1} + \mathbf{p}_{P_2})$; if the centroid coincides with the evader the previous direction is kept.

### Pincer Angle

$$\theta = \arccos\!\left(\frac{(\mathbf{p}_1 - \mathbf{p}_E)\cdot(\mathbf{p}_2 - \mathbf{p}_E)}{\|\mathbf{p}_1-\mathbf{p}_E\|\|\mathbf{p}_2-\mathbf{p}_E\|}\right)$$

Ideal pincer: θ = 180°.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer speed | 5.0 m/s |
| Evader speed | 3.5 m/s |
| Initial offset R₀ | 3.0 m |
| Min offset R_min | 0.5 m (then terminal direct closing) |
| Shrink rate V_shrink | 0.3 m/s |
| Pincer slot angles φ₁, φ₂ | 0°, 180° |
| Capture radius | 0.15 m |
| Time step | 1/48 s |
| Max time | 20 s |
| P1 start | (−4, −2, 2) m |
| P2 start | (−4, +2, 2) m |
| Evader start | (2, 0, 2) m |
| Shrink-rate sensitivity sweep | 0.3, 0.6, 1.0, 2.0 m/s |

---

## Implementation

```
src/base/drone_base.py                  # Point-mass drone base
src/01_pursuit_evasion/s013_pincer_movement.py     # Main simulation
```

```bash
conda activate drones
python src/01_pursuit_evasion/s013_pincer_movement.py
```

Runtime ≈ 25 s (4 main runs + 8 sweep runs + 135-frame GIF, every 3rd step).

---

## Results

| Strategy vs evader | Capture | Initial / max pincer angle | First ≥ 170° | Evader path |
|--------------------|---------|----------------------------|--------------|-------------|
| **Pincer** vs perpendicular (card) | P2 @ **8.42 s** | 36.9° / 179.5° | 1.71 s | 29.46 m |
| **Independent** vs perpendicular (card) | P2 @ **1.08 s** | 36.9° / 73.2° | never | 3.79 m |
| **Pincer** vs flee | P1 @ **8.38 s** | 36.9° / 178.2° | 4.77 s | 29.31 m |
| **Independent** vs flee | P1 @ **4.04 s** | 36.9° / 36.9° | never | 14.15 m |

**Pincer shrink-rate sensitivity** (terminal phase starts at (R₀ − R_min)/v_shrink):

| v_shrink (m/s) | Terminal phase at | vs perpendicular | vs flee |
|----------------|-------------------|------------------|---------|
| 0.3 (card) | 8.33 s | 8.42 s (max angle 179.5°) | 8.38 s (max angle 178.2°) |
| 0.6 | 4.17 s | 4.25 s (179.5°) | 4.19 s (103.5°) |
| 1.0 | 2.50 s | 2.56 s (178.6°) | 4.06 s (36.9°) |
| 2.0 | 1.25 s | 1.27 s (136.4°) | 4.06 s (36.9°) |

**Key Findings**:
- **Independent pure pursuit wins against both evaders.** With V_P/V_E = 5/3.5 every pursuer is faster than the evader, so a direct chase closes at ≥ 1.5 m/s; the pincer spends its time taking up and holding slots instead of closing.
- **The pincer forms, but its capture time is set by the shrink schedule.** The 180° geometry is reached (179.5° / 178.2°) and then held at 171.9–179.5° / 170.7–178.2° until the terminal phase, yet capture happens only 0.05–0.09 s after the terminal phase starts at 8.33 s. The sweep confirms it: capture time tracks (R₀ − R_min)/v_shrink for the perpendicular evader (8.42 → 4.25 → 2.56 → 1.27 s). Even at 2.0 m/s the pincer (1.27 s) is still slower than independent pursuit (1.08 s).
- **The card's perpendicular evader is weak.** At the start the perpendicular direction is +y, which takes it across the pursuers' approach; independent P2 cuts it off at 1.08 s after only 3.79 m. Against the pincer, once the formation is set the centroid sits within centimetres of the evader, so its "perpendicular" heading is set by the formation's small lag; it ends up flying an almost straight line while the pincer escorts it until the terminal phase.
- **Flee evader**: independent pursuit becomes a tail chase from behind (pincer angle → 0°, capture 4.04 s). In the pincer run P1 passes within 0.35 m of the evader at t = 4.17 s on its way to the forward slot without capturing, then the formation holds until the terminal phase (8.38 s). With faster shrink (1.0–2.0 m/s) the slots collapse before the pincer can form and the run degenerates to the same tail chase (4.06 s, max angle 36.9°).
- The earlier claim that the pincer pays off "with an evader that can change direction" is **not supported**: both evaders here re-plan every step and the pincer is slower in every case tested.

**Top-Down Trajectories** (2 × 2: strategy × evader):

![Trajectories 2D](trajectories_2d.png)

**Pincer Angle vs Time** (the pincer reaches ≈ 180° at 1.71 s / 4.77 s; it starts at 36.9°, not 180°):

![Pincer Angle](pincer_angle.png)

**Pursuer Distances to Evader**:

![Distances](distances.png)

**Animation** (card perpendicular evader; pincer left, independent right):

![Animation](animation.gif)

### Deviations from the scenario card

| Card | Code | Reason |
|------|------|--------|
| R_min = 0.5 m with capture radius 0.15 m | Same, plus a terminal phase (pursuers fly at the evader once R = R_min) | Without it the pincer could never capture |
| Pursuer command `V·d/‖d‖` | Saturated: stop on the target if reachable in one step (both strategies) | With the un-saturated law each step is V_P·Δt = 0.104 m, so pursuers overshot and chattered around their slots, which made the centroid-based evader jitter |
| Evader: perpendicular escape only | Perpendicular (card) + "flee from centroid" | A second, more sensible evader checks that conclusions do not hinge on the card's evader |
| Single shrink rate | Card rate 0.3 m/s plus sensitivity sweep 0.6–2.0 m/s | Shows the capture time is set by the shrink schedule |

(Previous version: pursuers started ahead at (6, ±4), R₀ = 4, 0.5 m/s, R_min = 0.05, slots on ±y, and the evader ignored the pursuers and flew straight +x. All restored to the card.)

---

## Extensions

1. 3-pursuer pincer (triangle formation) for 2D escape
2. Evader with adversarial direction-change strategy
3. Pincer with asymmetric speeds (one fast blocker, one slow flanker)

---

## Related Scenarios

- Prerequisites: [S011](../../../scenarios/01_pursuit_evasion/S011_swarm_encirclement.md), [S012](../../../scenarios/01_pursuit_evasion/S012_relay_pursuit.md)
- Follow-ups: [S014](../../../scenarios/01_pursuit_evasion/S014_decoy_lure.md), [S017](../../../scenarios/01_pursuit_evasion/S017_swarm_vs_swarm.md)
