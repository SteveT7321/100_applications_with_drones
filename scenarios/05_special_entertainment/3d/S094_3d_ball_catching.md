# S094 3D Upgrade — Aerial Ball Catching

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S094 original](../S094_ball_catching.md)

---

## What Changes in 3D

The original S094 already flies in $\mathbb{R}^3$, but it treats the net cup as an
orientation-free sphere of radius $r_c$ and the air as still. Two genuinely three-dimensional
effects are missing:

1. **Net orientation coupled to attitude.** A multirotor tilts its thrust vector to accelerate,
   and the cup is rigidly mounted on top, so the cup axis points along the thrust direction
   $\hat{\mathbf{n}} = (\mathbf{u} + g\hat{\mathbf{z}}) / \|\mathbf{u} + g\hat{\mathbf{z}}\|$.
   A ball can only be caught if it enters through the cup mouth, i.e. its relative velocity
   points into the cup within a cone around $-\hat{\mathbf{n}}$. A drone that is still
   accelerating hard at the moment of the catch tilts its cup away from the ball.
2. **Altitude-dependent wind.** An indoor arena with ventilation (or an outdoor show) has a
   horizontal wind that grows with height and gusts; the ball drifts by tens of centimetres and
   the drift depends on how high it is thrown.

The question becomes: **does planning for a level, decelerated arrival (tilt-aware) matter more
than raw interception speed, and how much does wind-aware prediction recover?**

---

## Problem Definition

Same arena, throw distribution, drone limits, sensing and strategies as
[S094](../S094_ball_catching.md), plus:

- **Cup acceptance cone**: catch requires distance $\le r_c$ (segment closest approach, as in
  S094) **and** $\angle(-\mathbf{v}_{rel}, \hat{\mathbf{n}}) \le \alpha_{cup} = 40°$, where
  $\mathbf{v}_{rel} = \mathbf{v}_b - \mathbf{v}_d$.
- **Wind field** acting on the ball only (the drone's position controller is assumed to reject
  it): power-law shear plus a first-order Gauss–Markov gust,
  $$\mathbf{w}(z, t) = \left( \bar w_{ref} \left(\frac{z}{z_{ref}}\right)^{\alpha_w} + g_w(t) \right) \hat{\mathbf{e}}_{w},
  \qquad \dot g_w = -\frac{g_w}{\tau_g} + \sigma_g \sqrt{\frac{2}{\tau_g}}\, \xi(t)$$
  with the ball drag using the air-relative velocity $\mathbf{v}_b - \mathbf{w}$.

Strategies compared on identical throws and identical wind realisations:

1. PP, PN, PCP exactly as in S094 (wind-unaware predictor).
2. **PCP-W** — PCP with the mean shear profile in the predictor (gust unknown).
3. **PCP-WT** — PCP-W plus a **tilt-aware arrival**: the trajectory is planned to arrive at the
   catch point with commanded acceleration $\|\mathbf{u}\| \le a_{level} = 2$ m/s² during the
   last 0.15 s, so the cup is within ≈ 12° of vertical.
4. **Oracle** with the true wind and the cone constraint, as the reachability bound.

---

## Mathematical Model

### Ball Dynamics with Wind

$$\dot{\mathbf{v}}_b = \mathbf{g} - k \,\|\mathbf{v}_b - \mathbf{w}\|\,(\mathbf{v}_b - \mathbf{w})$$

### Cup Axis

$$\hat{\mathbf{n}} = \frac{\mathbf{u} + g\hat{\mathbf{z}}}{\|\mathbf{u} + g\hat{\mathbf{z}}\|}, \qquad
\theta_{tilt} = \arccos(\hat{\mathbf{n}} \cdot \hat{\mathbf{z}}) = \arctan\frac{\|\mathbf{u}_{xy}\|}{g + u_z}$$

At $a_{max} = 10$ m/s² horizontal the tilt is ≈ 45°, beyond the 40° cone for a vertically
falling ball. Apply a first-order attitude lag $\tau_{att} = 0.05$ s to $\hat{\mathbf{n}}$
(the cup cannot snap level instantly).

### Catch Condition

$$d_{seg} \le r_c \quad \wedge \quad
\arccos\!\left(\frac{-\mathbf{v}_{rel} \cdot \hat{\mathbf{n}}}{\|\mathbf{v}_{rel}\|}\right) \le \alpha_{cup}
\quad \wedge \quad v_{b,z} < 0 \quad \wedge \quad z_b \ge z_{min}$$

Failures are classified as **distance miss** (never within $r_c$) or **angle reject** (within
$r_c$ but outside the cone) — the split is the key diagnostic.

### Metrics

- Catch rate (95 % Wilson interval), efficiency vs oracle, paired win/tie/loss counts.
- Failure split: distance miss vs angle reject per strategy.
- Cup tilt at closest approach (distribution) per strategy.
- Catch rate vs wind strength $\bar w_{ref} \in \{0, 1, 2, 3\}$ m/s and vs throw apex height.

---

## Key 3D Additions

| Addition | Why it is 3D |
|----------|--------------|
| Cup axis = thrust direction | couples horizontal manoeuvre to the vertical acceptance cone |
| Acceptance cone on $\mathbf{v}_{rel}$ | catch depends on the full 3D approach direction, not distance |
| Shear wind $\propto z^{\alpha_w}$ | drift depends on how high the ball goes |
| Tilt-aware arrival | trades interception speed for a level cup at the catch |

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| All S094 parameters | — | unchanged |
| Cup acceptance half-angle | $\alpha_{cup}$ | 40° |
| Attitude lag | $\tau_{att}$ | 0.05 s |
| Level-arrival acceleration cap | $a_{level}$ | 2 m/s² (last 0.15 s) |
| Reference wind speed | $\bar w_{ref}$ | 2 m/s at $z_{ref} = 3$ m (sweep 0–3) |
| Shear exponent | $\alpha_w$ | 0.2 |
| Gust std / time constant | $\sigma_g, \tau_g$ | 0.5 m/s / 1.0 s |
| Wind direction | $\hat{\mathbf{e}}_w$ | random per throw, same for all strategies |
| Monte Carlo throws | $N_{throws}$ | 300 |

---

## Expected Output

- 3D plot of one throw showing ball path, drone paths and the cup axis drawn as short arrows
  along each drone path, with the acceptance cone at the catch/closest point.
- Catch-rate bars with the failure split (distance miss / angle reject) stacked.
- Cup-tilt-at-closest-approach histogram per strategy.
- Catch rate vs $\bar w_{ref}$ and vs apex height.
- Animation (GIF) of the representative throw with the cup orientation visible.

Hypotheses to test (report honestly whether they hold):

1. Without tilt awareness, a large share of PCP's near-catches become angle rejects.
2. PCP-WT recovers most of them; PN and PP suffer most from the cone because they are still
   accelerating at contact.
3. Wind-aware prediction (PCP-W) matters more for high throws than low ones.

---

## Extensions

1. Full rigid-body attitude dynamics from `src/base/rigid_body_drone.py` instead of the
   first-order cup-axis lag.
2. Gimballed cup (one actuated axis) to decouple cup orientation from acceleration.
3. Online wind estimation from the ball's observed drift during the first 0.3 s of flight.
4. Soft catch: velocity matching along $\hat{\mathbf{n}}$ to limit the impact impulse.

---

## Related Scenarios

- Original 2D/3D version: [S094 Aerial Ball Catching](../S094_ball_catching.md)
- Wind modelling: [S024 Wind Compensation](../../02_logistics_delivery/S024_wind_compensation.md)
- Attitude–thrust coupling: [S091 Aerial Acrobatic Maneuvers](../S091_acrobatics.md)
