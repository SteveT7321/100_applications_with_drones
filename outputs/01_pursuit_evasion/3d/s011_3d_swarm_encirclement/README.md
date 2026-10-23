# S011 3D — Swarm Encirclement

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: ✅ Completed
**Scenario card**: [`scenarios/01_pursuit_evasion/3d/S011_3d_swarm_encirclement.md`](../../../../scenarios/01_pursuit_evasion/3d/S011_3d_swarm_encirclement.md)
**Source**: [`src/01_pursuit_evasion/3d/s011_3d_swarm_encirclement.py`](../../../../src/01_pursuit_evasion/3d/s011_3d_swarm_encirclement.py)

---

## Problem Definition

**Setup**: N pursuers fly to slots on a shrinking sphere around the evader. The slots are the card's Fibonacci-sphere directions, so slot 1 is the top pole and slot N is the bottom pole. The evader looks for the direction with the largest angular gap to the pursuers, commits to it and tries to break out.

**Key question** (card objective): what is the smallest N that prevents a 3D breakout, as a function of the speed ratio v_E/v_P?

**What changed in this version**: the card writes the sphere centre as the evader's *current* position. Re-centred every step, the sphere translates rigidly with the evader (V_P > V_E), so no breakout is possible and capture just follows the shrink schedule. A static evader is caught at 15.00 s, and a fleeing one at 14.60–14.93 s. In this version the swarm centres the sphere on its shared evader fix, which is τ = 0.1 s old (tracking + communication latency). The ideal τ = 0 law is kept as a reference run.

---

## Mathematical Model

### Fibonacci Sphere Slots (card)

$$\mathbf{q}_i = \begin{bmatrix}\sqrt{1-y_i^2}\cos\phi_i \\ \sqrt{1-y_i^2}\sin\phi_i \\ y_i\end{bmatrix}, \quad y_i = 1 - \frac{2i}{N-1}, \quad \phi_i = i\cdot\frac{2\pi}{\varphi^2}$$

### Encirclement Law

$$\mathbf{p}_i^{target}(t) = \mathbf{p}_E(t-\tau) + R(t)\,\mathbf{q}_i, \qquad R(t) = \max\!\left(R_0 - \frac{R_0 - R_{capture}}{T}\,t,\; R_{capture}\right)$$

Targets are clipped to the z band [0.5, 6.0] m. Each pursuer flies straight at its slot at up to V_P and stops exactly on it when it can reach it within one step.

### Evader (committed max-gap breakout)

Each step the evader samples K = 200 random unit directions (card). It drops any direction that points into a floor or ceiling closer than 0.6 m. It scores each direction by its minimum angle to the blocking bearings: every pursuer, plus straight down/up when the floor/ceiling is closer than R(t). It switches to the best direction only if that direction beats its current one by more than 20°. This is the same commitment rule used in the fixed 2D S011.

### Breakout and Outcome

- **Largest gap**: $g = \max_{\hat u}\min_i \angle(\hat u, \mathbf{b}_i)$, measured over 1000 fixed Fibonacci directions. $g > 90°$ means the evader is outside the convex hull of the pursuers and walls ("hull broken").
- **Capture**: any pursuer closer than R_capture = 0.3 m. **Escape**: no capture within 30 s.
- **Card's minimum angular gap**: the smallest angle between two pursuer bearings, plotted over time.

### Card Breakout Formula

$$N_{critical} = \left\lceil \frac{2}{1 - \cos(\arcsin(v_E/v_P))} \right\rceil$$

This formula is printed next to the simulated N_crit. It *decreases* with v_E/v_P (44 at 0.3, 4 at 0.9), because the card's condition $v_E > v_P\sin\delta_{gap}$ becomes easier to meet as the gap δ shrinks. So the formula is inverted, and the simulation does not follow it.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of pursuers N (main run) | 6 |
| Initial sphere radius R₀ | 4.0 m |
| Shrink time T | 15 s |
| Capture radius R_capture (= sphere floor) | 0.3 m |
| Pursuer speed V_P | 5.0 m/s |
| Evader speed V_E (main run) | 3.5 m/s (v_E/v_P = 0.7) |
| Evader start | (0, 0, 2) m |
| z range | 0.5 – 6.0 m |
| Breakout candidates K | 200 per step |
| Gap-switch hysteresis | 20° |
| Evader wall margin | 0.6 m |
| Sphere-centre fix latency τ | 0.1 s (reference run: 0) |
| dt | 0.05 s |
| Max simulation time | 30 s |
| Sweep | N ∈ {4, 6, 8, 10, 12} × v_E/v_P ∈ {0.3, 0.5, 0.7, 0.9}, 20 seeds per cell |
| Random numbers | seed s is used for the evader's candidate directions in every cell (common random numbers); main run seed 0 |

---

## Implementation

```bash
conda activate drones
python src/01_pursuit_evasion/3d/s011_3d_swarm_encirclement.py
```

Runtime ≈ 75 s: 2 main runs, 1 static-evader check, 400 sweep runs, 100 reference runs and a 151-frame GIF.

---

## Results

### Main runs (N = 6, v_E/v_P = 0.7, seed 0)

| Run | Outcome | Hull broken (g > 90°) | Max gap | Direction switches | Evader z range | Final nearest pursuer |
|-----|---------|-----------------------|---------|--------------------|----------------|-----------------------|
| τ = 0.1 s fix latency | **escaped** (102.73 m net displacement) | at 10.60 s | 145.2° | 2 | 1.05 – 5.41 m | 0.50 m |
| τ = 0 (ideal card law) | captured by P3 @ **14.95 s** | at 14.35 s | 100.7° | 2 | 1.05 – 5.41 m | 0.30 m |

- Static evader (τ = 0.1 s): captured at 15.00 s, when R(t) reaches 0.3 m.
- Largest gap seen from the centre of the undisturbed Fibonacci slots: N=4: 89.6°, N=6: 68.4°, N=8: 58.9°, N=10: 50.4°, N=12: 44.5°.

### Sweep: escape rate over N × v_E/v_P (τ = 0.1 s, 20 seeds per cell)

| v_E/v_P | N = 4 | N = 6 | N = 8 | N = 10 | N = 12 | Simulated N_crit | Card formula N_crit |
|---------|-------|-------|-------|--------|--------|------------------|---------------------|
| 0.3 | 20/20 | 0/20 | 0/20 | 0/20 | 0/20 | **6** | 44 |
| 0.5 | 20/20 | 20/20 | 0/20 | 0/20 | 0/20 | **8** | 15 |
| 0.7 | 20/20 | 20/20 | 19/20 | 15/20 | 9/20 | **> 12** | 7 |
| 0.9 | 20/20 | 20/20 | 20/20 | 20/20 | 19/20 | **> 12** | 4 |

Simulated N_crit is the smallest swept N with 0 escapes in 20 seeds.

Hull-broken counts and capture times (mean ± sd over the captured seeds):

| v_E/v_P | N | Hull broken | Capture time (s) | Median final nearest pursuer in escapes (m) |
|---------|---|-------------|------------------|---------------------------------------------|
| 0.3 | 4 | 20/20 | – | 0.37 |
| 0.3 | 6 | 20/20 | 16.78 ± 3.34 | – |
| 0.3 | 8 | 15/20 | 14.70 ± 0.11 | – |
| 0.3 | 10 | 0/20 | 14.52 ± 0.04 | – |
| 0.3 | 12 | 0/20 | 14.46 ± 0.04 | – |
| 0.5 | 4 | 20/20 | – | 0.48 |
| 0.5 | 6 | 20/20 | – | 0.37 |
| 0.5 | 8 | 20/20 | 14.73 ± 0.54 | – |
| 0.5 | 10 | 19/20 | 14.44 ± 0.14 | – |
| 0.5 | 12 | 10/20 | 14.17 ± 0.14 | – |
| 0.7 | 4 | 20/20 | – | 0.60 |
| 0.7 | 6 | 20/20 | – | 0.49 |
| 0.7 | 8 | 20/20 | 15.50 ± 0.00 | 0.37 |
| 0.7 | 10 | 20/20 | 14.09 ± 0.27 | 0.35 |
| 0.7 | 12 | 15/20 | 14.38 ± 2.08 | 0.34 |
| 0.9 | 4 | 20/20 | – | 0.73 |
| 0.9 | 6 | 20/20 | – | 0.61 |
| 0.9 | 8 | 20/20 | – | 0.49 |
| 0.9 | 10 | 20/20 | – | 0.47 |
| 0.9 | 12 | 20/20 | 14.15 ± 0.00 | 0.45 |

### Reference: τ = 0 (sphere re-centred on the current evader every step), v_E/v_P = 0.7

| N | Escape | Hull broken | Capture time (s) |
|---|--------|-------------|------------------|
| 4 | 20/20 | 20/20 | – |
| 6 | 0/20 | 20/20 | 14.93 ± 0.04 |
| 8 | 0/20 | 0/20 | 14.77 ± 0.04 |
| 10 | 0/20 | 0/20 | 14.65 ± 0.03 |
| 12 | 0/20 | 0/20 | 14.60 ± 0.03 |

**Key findings**:
- **With the ideal card law (τ = 0), the encirclement is never tested.** The sphere moves with the evader, so for N ≥ 6 capture comes at 14.60–14.93 s in every seed. That is the shrink schedule: a static evader is caught at 15.00 s. The evader's choices change the capture time by less than 0.4 s.
- **A 0.1 s old fix is enough for real breakouts.** The slots then trail the evader by about v_E·τ (0.35 m at 3.5 m/s). Once R(t) shrinks to a similar size, the evader is outside the hull. In the main run this happens at 10.60 s, and the evader escapes 102.73 m.
- **Simulated N_crit rises with the speed ratio:** 6 at v_E/v_P = 0.3, 8 at 0.5, and above 12 at 0.7 and 0.9. At 0.7, N = 12 still lets 9/20 seeds escape. Faster evaders open a larger latency offset, so the hull breaks while R is still large. Denser spheres (smaller gaps) delay the break.
- **The card's N_crit formula points the wrong way** (44 → 4 as v_E/v_P rises). It cannot be used to size the swarm.
- **N = 4 never holds, even with τ = 0.** The Fibonacci construction puts slots 1 and 4 at the poles, so the pole-to-pole segment passes through the centre. The evader starts on the hull boundary (89.6° gap), and any lag puts it outside.
- **What "escape" means here.** The encirclement law has no terminal pursuit term. After a breakout, the formation keeps trailing the evader, and the nearest pursuer ends 0.34–0.73 m behind (median per cell). This is just outside R_capture, but the slots never close the gap. A terminal switch to direct pursuit would change this (see Extensions).
- **Real 3D motion.** The evader climbs to 5.41 m and dives to 1.05 m before settling on a shallow climb. The top and bottom pole drones spend long stretches pinned at the ceiling and floor, because a 4 m sphere around z = 2 does not fit in the 0.5–6 m band. The floor and ceiling count as blockers in the evader's gap score, so it never runs into the pinned pole drones.

### 3D Trajectories (main run vs ideal card law)

![3D Trajectories](trajectories_3d.png)

### Altitude vs Time (τ = 0.1 s)

The top (P1) and bottom (P6) slot drones sit at the ceiling and floor while the evader climbs and then dives.

![Altitude vs Time](altitude_time.png)

### Shrinking Radius R(t) and Nearest Pursuer Distance (log scale)

![Radius vs Time](radius_time.png)

### Angular Gaps vs Time

The top panel is the evader's largest gap, with breakout above 90°. The bottom panel is the card's minimum angle between pursuer bearings.

![Angular gap](angular_gap.png)

### Breakout Rate vs N (card output; one line per speed ratio, plus τ = 0)

![Breakout vs N](breakout_vs_N.png)

### Escape Rate over N × v_E/v_P

![Speed ratio sweep](speed_ratio_sweep.png)

### Animation (main run, τ = 0.1 s, every 4th step, 151 frames)

![Animation](animation.gif)

### Deviations from the scenario card

| Card | This implementation | Reason |
|------|--------------------|--------|
| Sphere centred on $\mathbf{p}_E(t)$ every step | Centre = $\mathbf{p}_E(t-\tau)$, τ = 0.1 s; τ = 0 run as the reference | With τ = 0 the sphere translates with the evader and capture follows the shrink schedule (14.60–14.93 s vs 15.00 s for a static evader), so breakout cannot be studied |
| Evader re-picks the best of K = 200 random directions every step | Same K = 200, plus 20° switching hysteresis, a 0.6 m wall margin, and the floor/ceiling counted as blocking bearings within R(t) | Without commitment the evader dithers (2D S011 fix). Without wall awareness it climbs straight into the ceiling-pinned top drone, and the "capture" is a clipping artefact |
| Pursuers move V_P·dt toward the target every step | Saturated go-to: stop on the slot if it is reachable within one step | Avoids chattering around the slot (2D S011 fix) |
| Breakout condition $v_E > v_P\sin\delta_{gap}$, $N_{critical}$ formula | Breakout = largest gap > 90° (hull broken); outcome = capture or escape in 30 s; card formula printed for comparison | The card formula is inverted (N_crit falls as v_E/v_P rises) |
| "Remaining N − 2 drones fill the equatorial band" | Card's Fibonacci formula used as written, with slots spread over all latitudes | The card's own formula does not produce an equatorial band |
| Sweep N = 4…12 (single speed ratio) | N = 4…12 × v_E/v_P = 0.3–0.9, 20 seeds per cell (common random numbers), plus a τ = 0 reference row | Card objective is N_crit *as a function of* v_E/v_P |
| Sphere radius 4 m around z = 2 in a 0.5–6 m band | Slot targets clipped to the band (bottom slot starts at 0.5 m) | The sphere does not fit; reported, not hidden |

---

## Extensions

1. Terminal pursuit: switch the nearest drone to pure pursuit once the hull breaks, and measure whether the trailing formation then captures
2. Latency sweep: τ from 0 to 0.3 s to map the latency budget for a given N and v_E/v_P
3. Dynamic Fibonacci rebalancing: re-orient the slot set each step (card extension)
4. Heterogeneous altitude tiers: two pursuers at each of z = 1, 3, 5 m (card extension)
5. Weighted breakout: the evader weights gaps by pursuer distance and speed (card extension)

---

## Related Scenarios

- Original 2D version: [S011 Swarm Encirclement](../../../../scenarios/01_pursuit_evasion/S011_swarm_encirclement.md)
- Follow-ups: [S012 3D Upgrade — Relay Pursuit](../../../../scenarios/01_pursuit_evasion/3d/S012_3d_relay_pursuit.md), [S013 3D Upgrade — Pincer Movement](../../../../scenarios/01_pursuit_evasion/3d/S013_3d_pincer_movement.md)
- Altitude floor reference: [S003 Low-Altitude Tracking](../../../../scenarios/01_pursuit_evasion/S003_low_altitude_tracking.md)
