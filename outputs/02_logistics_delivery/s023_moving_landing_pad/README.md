# S023 Moving Landing Pad

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: A delivery drone must land on a moving ground vehicle (truck) travelling along a straight road (the $x$-axis, $y = z = 0$) at constant speed. The drone starts at rest, 15 m up and offset from the road. It predicts the truck's future intercept position, flies there with proportional navigation, then descends while matching the truck's velocity to achieve a soft touchdown within a pad of radius $r_{pad} = 0.5$ m.

**Objective**: Achieve touchdown with lateral miss ≤ 0.5 m, vertical speed ≤ 1.0 m/s, and horizontal velocity mismatch ≤ 0.5 m/s.

---

## Mathematical Model Summary

**Kinematics** (double integrator, gravity cancelled by thrust): $\dot{\mathbf{p}} = \mathbf{v}$, $\dot{\mathbf{v}} = \mathbf{a}_{cmd}$, $\|\mathbf{a}_{cmd}\| \le a_{max}$, $\|\mathbf{v}\| \le v_{max}$.

**Intercept point prediction** (fixed-point iteration, 5 iterations each step):

$$\mathbf{p}^*_{xy} = \mathbf{p}_{T,xy} + \mathbf{v}_{T,xy}\,\Delta t_{fly}, \qquad z^* = z_{switch}, \qquad \Delta t_{fly} = \frac{\|\mathbf{p}^* - \mathbf{p}_D\|}{v_{max}}$$

**Phase 1 — 3-D Proportional Navigation** toward $\mathbf{p}^*$ (card form):

$$\mathbf{a}_{PN} = N\,V_c\,\dot{\hat{\boldsymbol\lambda}}, \qquad \hat{\boldsymbol\lambda} = \frac{\mathbf{p}^* - \mathbf{p}_D}{\|\mathbf{p}^* - \mathbf{p}_D\|}, \qquad \dot{\hat{\boldsymbol\lambda}} \approx \frac{\hat{\boldsymbol\lambda}(t) - \hat{\boldsymbol\lambda}(t-\Delta t)}{\Delta t}, \qquad V_c = -\frac{d}{dt}\|\mathbf{p}^* - \mathbf{p}_D\|$$

plus an along-LOS speed-hold term (see Deviations): $\mathbf{a}_{cmd} = \mathbf{a}_{PN} + \text{clip}\big(K_{ax}(v_{max} - \mathbf{v}_D\cdot\hat{\boldsymbol\lambda}),\ \pm\sqrt{a_{max}^2 - \|\mathbf{a}_{PN}\|^2}\big)\,\hat{\boldsymbol\lambda}$, with $\|\mathbf{a}_{PN}\| \le a_{max}$ (steering has priority).

**Phase 2 — descent + velocity matching** (entered when $z_D \le z_{switch}$ and lateral distance to the truck $\le r_{switch}$):

$$\mathbf{a}_{xy} = K_p(\mathbf{p}_{T,xy} - \mathbf{p}_{D,xy}) + K_v(\mathbf{v}_{T,xy} - \mathbf{v}_{D,xy})$$

$$\dot z_{cmd}(z) = -v_{land,max}\tanh\!\left(\frac{z}{z_{flare}}\right), \qquad a_z = K_{\dot z}\,(\dot z_{cmd} - \dot z_D)$$

The tanh flare commands ≈ 1 m/s descent high up and slows smoothly towards 0 as $z \to 0$ (not a constant descent rate).

**Touchdown conditions** (checked when $z_D \le 0.05$ m):

$$\|\mathbf{p}_{xy,D} - \mathbf{p}_{xy,T}\| \leq r_{pad}, \quad |\dot{z}| \leq v_{land,max}, \quad \|\dot{\mathbf{p}}_{xy,D} - \dot{\mathbf{p}}_{xy,T}\| \leq \Delta v_{max}$$

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Drone start position | (-20, 8, 15) m, at rest |
| Truck start / speed | x = 0 m, 4.0 m/s along +x |
| Drone max speed $v_{max}$ | 8.0 m/s |
| Drone max acceleration $a_{max}$ | 5.0 m/s² |
| PN navigation constant $N$ | 3 |
| Along-LOS speed-hold gain $K_{ax}$ | 1.0 s⁻¹ |
| Phase-switch altitude $z_{switch}$ | 5.0 m |
| Phase-switch lateral radius $r_{switch}$ | 3.0 m |
| Phase-2 gains $K_p$ / $K_v$ / $K_{\dot z}$ | 1.2 / 1.8 / 2.0 |
| Flare altitude constant $z_{flare}$ | 2.0 m |
| Max touchdown vertical speed $v_{land,max}$ | 1.0 m/s (also the flare's max commanded descent rate) |
| Pad radius $r_{pad}$ | 0.5 m |
| Max horizontal velocity mismatch $\Delta v_{max}$ | 0.5 m/s |
| Touchdown detection altitude | 0.05 m |
| Time step / control frequency | 0.02 s / 50 Hz |
| Simulation budget | 30 s |

### Deviations from the scenario card

| Item | Card | Code | Reason |
|------|------|------|--------|
| Phase-1 acceleration | pure PN, $\mathbf{a} = N V_c \dot{\hat{\boldsymbol\lambda}}$ | PN **plus** an along-LOS speed-hold term $K_{ax}(v_{max} - \mathbf{v}\cdot\hat{\boldsymbol\lambda})$ using the acceleration budget PN leaves | PN only commands acceleration normal to the LOS; the drone starts at rest, so pure PN has no closing speed and never moves toward the target. The speed hold also makes the drone fly at $v_{max}$, which is what the intercept predictor assumes. |
| Aim point altitude | predictor returns the truck's future position on the ground | aim point is placed at $z_{switch} = 5$ m above the predicted truck position | Phase 1 is meant to hand over at $z_{switch}$; aiming at the ground would dive at 8 m/s and could hit the ground before the Phase-2 lateral condition is met. |
| LOS rate | card maths: finite difference; card code: $\mathbf{v}_{\perp,rel}/r$ with $\mathbf{v}_T$ as target velocity | finite difference of $\hat{\boldsymbol\lambda}$ and of the range (card maths) | the target is the re-predicted intercept point, not the truck, so its motion is captured by differencing rather than by assuming it moves with $\mathbf{v}_T$ |

The previous version of this script never called its PN function (Phase 1 was a pure-pursuit velocity tracker with an altitude hold) while the README and plot labels claimed PN; that has been replaced by the guidance above.

---

## Simulation Results

| Metric | Value | Limit | Status |
|--------|-------|-------|--------|
| Initial predicted aim point | (21.34, 0.00, 5.00) m | — | — |
| Phase 1 → 2 switch | t = 7.62 s at (30.67, -0.03, 4.96) m | — | — |
| Lateral miss / Δv_horiz / \|vz\| at switch | 0.188 m / 3.860 m/s / 1.298 m/s | — | — |
| Number of phase changes | 1 (no chattering) | — | — |
| Phase 2: max lateral miss / duration | 1.741 m / 8.56 s | — | — |
| PN steering \|a_PN\| in Phase 1 | max 1.066 m/s², mean 0.154 m/s² | 5.0 m/s² | — |
| Touchdown time | **16.18 s** | 30 s | ✅ |
| Lateral miss | **0.0778 m** | 0.5 m | ✅ |
| Vertical speed \|vz\| | **0.0405 m/s** | 1.0 m/s | ✅ |
| Horizontal velocity mismatch | **0.0031 m/s** | 0.5 m/s | ✅ |
| Overall result | **SUCCESS** | — | — |

**Phase 1.** The first prediction places the intercept point at x = 21.34 m (sanity check: distance from the start ≈ 43.3 m / 8 m/s ≈ 5.4 s, truck covers 4 × 5.4 ≈ 21.6 m). Because the drone starts from rest and only reaches $v_{max}$ gradually (the speed-hold gain is 1 s⁻¹), the actual hand-over happens later, at 7.62 s, and the predicted point drifts forward to x = 30.45 m — the truck is at 4 × 7.62 = 30.48 m then. PN absorbs this drift with small steering accelerations (max 1.07 m/s²), and the drone arrives 0.19 m from the truck at 4.96 m altitude. It arrives still flying at close to $v_{max}$, so the horizontal velocity mismatch at hand-over is 3.86 m/s.

**Phase 2.** The PD law brings the 3.86 m/s mismatch below the 0.5 m/s limit within 0.76 s; the drone overshoots ahead of the truck (lateral miss peaks at 1.741 m, Δv rebounds to 0.763 m/s) and then settles. The tanh flare brings the descent rate from 1.30 m/s at hand-over to 0.04 m/s at touchdown. Phase 2 takes 8.56 s (an ideal tanh profile from 5 m to 0.05 m would take $2\ln[\sinh(2.5)/\sinh(0.025)] \approx 11$ s; it is shorter here because the drone enters Phase 2 descending at 1.3 m/s). Touchdown at 16.18 s meets all three criteria with wide margins.

Compared with the previous (pure-pursuit) version, which touched down at 13.46 s and failed on Δv_horiz (0.53 m/s), the current run lands later but cleanly. The difference is not only due to PN vs pure pursuit: the old Phase 1 also held altitude and aimed differently, so the two runs are not a controlled comparison.

---

## Output Files

### 3D Trajectory

Truck road (orange dashed), drone Phase 1 (red, PN + speed hold) and Phase 2 (dark red, descent), phase-switch point (yellow), and the pad circle (green) at the truck's touchdown position:

![3D Trajectory](trajectory_3d.png)

### Metrics Time Series

Altitude, lateral miss distance, vertical speed and horizontal velocity mismatch versus time; Phase 1 is shaded blue, Phase 2 yellow:

![Metrics Time Series](metrics_time_series.png)

### Animation

Top view and side view (150 frames sub-sampled from the 50 Hz simulation, played at 20 fps):

![Animation](animation.gif)

---

## Extensions

1. Add a velocity pre-matching segment at the end of Phase 1 (reduce the closing speed to the truck speed) to remove the 3.9 m/s hand-over mismatch and the Phase-2 overshoot
2. Use a Kalman filter to handle truck velocity uncertainty (noisy telemetry)
3. Extend to sinusoidal truck path — the drone must continuously update the intercept prediction

---

## Related Scenarios

- Prerequisites: [S021](../../../scenarios/02_logistics_delivery/S021_point_delivery.md) — basic landing
- Follow-ups: [S039](../../../scenarios/02_logistics_delivery/S039_offshore_platform_exchange.md) — landing on a ship deck with 6-DOF motion
