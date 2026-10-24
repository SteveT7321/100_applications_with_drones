# S092 Movie Car Chase Scene

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: A camera drone films a stunt car on a winding road ($x = 8t$, $y = 15\sin(2\pi x/80)$, x from 0 to 300 m, 37.5 s). The drone should stay 8 m behind and 5 m above the car while flying around five concrete pillars (radius 0.6 m, height 6 m) with a safety clearance of 1.5 m. It only receives noisy car positions (σ = 0.3 m at 20 Hz), filters them with a constant-velocity Kalman filter (KF), and plans with a 10-step model predictive controller (MPC, 0.2 s steps, 2 s horizon).

**Objective**: keep the drone near the desired offset point, keep the car at a fixed size in the frame, keep 1.5 m from every pillar.

**Variants** (same measurement noise per seed, 10 seeds):

| Variant | Reference over the horizon | MPC cost | Pillar constraints |
|---------|----------------------------|----------|--------------------|
| **Proposed** | KF constant-velocity prediction, 0–2 s ahead | position + velocity tracking | pillars on the camera path; every 0.05 s of the horizon; normals from the previous plan |
| Knots only | KF prediction | position + velocity | same, but only at the 0.2 s knots (the card's form) |
| Card cost | KF prediction | position only (card) | as proposed |
| No prediction | current KF estimate, held constant | position + velocity | as proposed |
| Oracle | true future desired positions | position + velocity | as proposed |
| Oracle, card cost | true future | position only | as proposed |
| No obstacle constraints | KF prediction | position + velocity | none |
| Card linearisation | KF prediction | position + velocity | one normal per pillar from the current position (card) |
| Card pillar layout | KF prediction | position + velocity | the card's pillar positions |

An **offline optimum** is also computed as a reference point: one QP over the whole 37.5 s with the true future, the card's weighted position/control cost sampled at the 0.2 s knots, the same limits and no pillars.

---

## Mathematical Model Summary

**Car**: $\mathbf{p}_{car} = (8t,\ 15\sin(2\pi \cdot 8t/80),\ 0)$, heading $\psi = \operatorname{atan2}(\dot y, \dot x)$. Its speed along the road varies from 8.00 to 12.36 m/s and the heading swings ±49.7°.

**Desired camera position**: $\mathbf{p}_d = \mathbf{p}_{car} + R(\psi)\,(-8, 0)^\top + (0, 0, 5)^\top$, i.e. 9.43 m from the car. This point needs up to 12.36 m/s and lateral accelerations up to $|a_y| = 9.79$ m/s², more than the drone's 6 m/s² per axis.

**Kalman filter**: state $(x, y, v_x, v_y)$, $F = \begin{pmatrix} I & \Delta t I \\ 0 & I\end{pmatrix}$, white-noise-acceleration $Q$ with $q = 1$ m²/s³, $R = 0.3^2 I$, $\Delta t = 0.05$ s. Prediction $\hat{\mathbf{p}}(t+\tau) = \hat{\mathbf{p}} + \tau\hat{\mathbf{v}}$; the reference heading is $\operatorname{atan2}(\hat v_y, \hat v_x)$.

**MPC** (condensed QP in the accelerations $\mathbf{u}_0..\mathbf{u}_9$, double integrator with $\Delta t_{mpc} = 0.2$ s, acceleration held constant within each step):

$$J = \sum_{i=1}^{N-1} Q\|\mathbf{p}_i - \mathbf{r}_i\|^2 + P_f\|\mathbf{p}_N - \mathbf{r}_N\|^2 + \sum_{i=0}^{N-1} R\|\mathbf{u}_i\|^2 + \underbrace{\sum_{i=1}^{N} Q_v\|\mathbf{v}_i - \dot{\mathbf{r}}_i\|^2}_{\text{added}} + \sum_k \left(w_q s_k^2 + w_l s_k\right)$$

subject to $|v_{i,a}| \le 12$ m/s and $|u_{i,a}| \le 6$ m/s² per axis, and for each pillar $k$ within 40 m, at every simulation instant $\tau_j = 0.05\,j$ s of the horizon ($j = 1..40$):

$$\hat{\mathbf{n}}_{k,j}^\top\big(\mathbf{p}^{xy}(\tau_j) - \mathbf{o}_k\big) + s_k \ge r_{safe}, \quad s_k \ge 0, \qquad \hat{\mathbf{n}}_{k,j} = \frac{\bar{\mathbf{p}}^{xy}(\tau_j) - \mathbf{o}_k}{\|\bar{\mathbf{p}}^{xy}(\tau_j) - \mathbf{o}_k\|}$$

$\mathbf{p}(\tau_j)$ is exactly linear in the accelerations (piecewise-constant acceleration), and $\bar{\mathbf{p}}$ is the previous plan shifted by one MPC step. Because $\hat{\mathbf{n}}^\top(\mathbf{p} - \mathbf{o}) \le \|\mathbf{p} - \mathbf{o}\|$, every constrained instant is at least $r_{safe} - s_k$ from the pillar centre. The executed 0.2 s segment is exactly the first segment of the plan, so these instants are also points of the flown path. $\dot{\mathbf{r}}_i$ is the finite difference of the reference. The QP is solved by a dense primal-dual interior-point method (Mehrotra predictor–corrector).

**Clearance check**: the true integrated path is evaluated at 10 points per 0.05 s step (constant acceleration within the step). It is asserted that no point is inside a pillar (horizontal distance ≤ 0.6 m below 6 m height) for every variant with 0.05 s constraints.

**Framing**: $f_{height} = 2\arctan\!\big(h_{car}/(2d)\big)/60°$ with $h_{car} = 1.5$ m. At the nominal $d = 9.43$ m this is 15.2 %, so the score uses the band 10–20 % (d = 7.14–14.31 m). The card's 20–40 % band (d = 3.53–7.14 m) is also reported.

**Metrics**: RMS of $\|\mathbf{p}_{drone} - \mathbf{p}_d\|$ (also for t ≥ 3 s, after the start from rest), share of time within 1 m, framing score, minimum horizontal distance from the integrated path to a pillar centre.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Road | $x = 8t$, amplitude 15 m, wavelength 80 m, x = 0–300 m (arc length 386.2 m), 37.5 s |
| Camera offset (body frame) | 8 m behind, 5 m above: (−8, 0, 5) m |
| Pillars | radius 0.6 m, height 6 m, $r_{safe}$ = 1.5 m; moved onto the camera path at the card's x stations, 0.5 m to the side: (40.4, −0.1), (90.4, 12.9), (149.7, −10.9), (209.6, −13.6), (260.1, 17.5) m; swept 0.5–3.0 m |
| Measurement noise / rate | σ = 0.3 m, 20 Hz |
| KF process noise $q$ / prediction horizon | 1.0 m²/s³ / 2.0 s |
| MPC horizon / step | 10 / 0.2 s |
| Pillar constraint points | every 0.05 s of the horizon (40 per pillar) |
| Weights $Q$, $P_f$, $R$ | 10, 20, 0.1 (card) |
| Velocity-tracking weight $Q_v$ (added) | 10 (sensitivity 1–100) |
| Slack weights $w_q$, $w_l$ (added) | $10^4$, $10^3$ |
| Pillars added to the QP | within 40 m |
| Speed / acceleration limit | 12 m/s / 6 m/s² per axis (as in the card; the speed magnitude can reach $12\sqrt{2}$) |
| Camera FoV / car height | 60° / 1.5 m |
| Drone start | at the desired point, at rest |
| Monte Carlo | 10 noise seeds (0–9); seed 0 for figures |

---

## Implementation

```
src/05_special_entertainment/s092_movie_chase.py   # Main simulation script
```

```bash
conda activate drones
python src/05_special_entertainment/s092_movie_chase.py
```

All 222 runs (printed: 72 variant runs, 150 sweep runs) go through a process pool of 6 workers. Results are collected in input order, so the output does not depend on scheduling. Runtime ≈ 1–2 min (76, 95 and 116 s measured in the latest three runs while other jobs shared the CPU).

---

## Results

### Monte Carlo, 10 noise seeds (mean ± std; oracle rows are noise-free single runs)

| Variant | RMS error | RMS, t ≥ 3 s | Within 1 m | Framing 10–20 % (card 20–40 %) | Min pillar clearance (worst) | Runs < $r_{safe}$ / inside a pillar |
|---------|---:|---:|---:|---:|---:|---:|
| **Proposed** | **3.834 ± 0.057 m** | **3.578 ± 0.064 m** | 13.8 ± 0.6 % | **89.6 ± 0.1 %** (0.0 %) | **1.482 ± 0.012 m (1.457)** | 10/10 / **0/10** |
| Knots only | 3.943 ± 0.141 m | 3.704 ± 0.162 m | 13.8 ± 0.7 % | 89.5 ± 0.4 % (0.0 %) | 1.122 ± 0.072 m (0.946) | 10/10 / 0/10 |
| Card cost | 9.739 ± 0.253 m | 10.010 ± 0.268 m | 1.1 ± 0.1 % | 42.0 ± 0.9 % (0.0 %) | 2.762 ± 0.578 m (1.522) | 0/10 / 0/10 |
| No prediction | 12.895 ± 0.014 m | 13.094 ± 0.015 m | 0.3 ± 0.0 % | 1.3 ± 0.0 % (0.0 %) | 1.553 ± 0.016 m (1.528) | 0/10 / 0/10 |
| Oracle | 2.518 m | 1.930 m | 32.6 % | 96.3 % (0.0 %) | 1.482 m | 1/1 / 0/1 |
| Oracle, card cost | 7.106 m | 7.209 m | 6.3 % | 53.4 % (0.0 %) | 1.482 m | 1/1 / 0/1 |
| No obstacle constraints | 2.623 ± 0.031 m | 2.075 ± 0.050 m | 20.1 ± 0.7 % | 96.4 ± 0.0 % (0.0 %) | 0.661 ± 0.231 m (0.210) | 10/10 / **4/10** |
| Card linearisation | 5.683 ± 0.831 m | 5.644 ± 0.909 m | 7.9 ± 0.9 % | 78.9 ± 3.2 % (0.0 %) | 1.576 ± 0.325 m (1.238) | 5/10 / 0/10 |
| Card pillar layout | 2.623 ± 0.031 m | 2.075 ± 0.051 m | 20.1 ± 0.7 % | 96.4 ± 0.0 % (0.0 %) | 3.317 ± 0.198 m (2.817) | 0/10 / 0/10 |

QP solves: none failed to converge for the proposed controller (0 of 1880; 3.8 ± 1.9 solves per run used obstacle slack). 9 of 1880 did not converge with the card linearisation.

Paired per seed (steady RMS): proposed beats no prediction in 10/10 seeds and the card cost in 10/10 seeds; the oracle (1.930 m) beats the proposed controller in 10/10 seeds. KF: filtered position RMS 0.312 m (seed 0, after the first 1 s); the 2-s-ahead prediction RMS is 10.85 ± 0.01 m.

Speed: the proposed controller reaches 15.41 m/s in magnitude while each axis stays at or below 12.00 m/s. The card's bound $\|\mathbf{v}\|_\infty \le 12$ m/s is per axis, and the implementation keeps it; a 12 m/s bound on the speed magnitude is **not** enforced.

### Pillar-offset sweep (10 seeds per cell)

Offset = distance of each pillar from the desired camera path. Clearance is measured on the integrated path, 10 points per 0.05 s:

| Offset | Constraints | Steady RMS | Min clearance on the fine path (worst) | Worst at the 0.05 s instants | Runs inside a pillar | Slack-active solves |
|---:|---|---:|---:|---:|---:|---:|
| 0.5 m | every 0.05 s | 3.578 ± 0.064 m | 1.482 ± 0.012 m (1.457) | 1.4668 m | **0/10** | 3.8 ± 1.9 |
| 0.5 m | knots only | 3.704 ± 0.162 m | 1.122 ± 0.072 m (0.946) | 0.9595 m | 0/10 | 6.4 ± 1.3 |
| 1.0 m | every 0.05 s | 3.772 ± 0.085 m | 1.474 ± 0.012 m (1.454) | 1.4644 m | **0/10** | 5.2 ± 1.5 |
| 1.0 m | knots only | 3.942 ± 0.239 m | 0.801 ± 0.202 m (0.638) | 0.6377 m | 0/10 | 6.7 ± 1.8 |
| 1.5 m | every 0.05 s | 3.943 ± 0.072 m | 1.469 ± 0.010 m (1.454) | 1.4574 m | **0/10** | 4.4 ± 1.8 |
| 1.5 m | knots only | 4.137 ± 0.189 m | 0.611 ± 0.151 m (0.432) | 0.4316 m | **5/10** | 9.0 ± 1.7 |
| 2.0 m | every 0.05 s | 4.164 ± 0.100 m | 1.473 ± 0.012 m (1.453) | 1.4581 m | **0/10** | 5.9 ± 1.6 |
| 2.0 m | knots only | 4.286 ± 0.223 m | 0.639 ± 0.181 m (0.364) | 0.3643 m | **5/10** | 7.9 ± 1.7 |
| 3.0 m | every 0.05 s | 4.648 ± 0.096 m | 1.468 ± 0.020 m (1.439) | 1.4476 m | **0/10** | 6.9 ± 1.8 |
| 3.0 m | knots only | 4.898 ± 0.157 m | 1.018 ± 0.269 m (0.338) | 0.3375 m | **1/10** | 5.5 ± 1.7 |

### Velocity-weight sensitivity (proposed, offset 0.5 m, 10 seeds each)

| $Q_v$ | Steady RMS | Within 1 m | Worst min clearance (fine path) | Worst at the 0.05 s instants | Runs inside a pillar |
|---:|---:|---:|---:|---:|---:|
| 1 | 8.824 ± 0.795 m | 1.9 ± 0.7 % | 1.489 m | 1.5000 m | 0/10 |
| 3 | 6.171 ± 1.054 m | 5.7 ± 1.8 % | 1.484 m | 1.4993 m | 0/10 |
| **10** | **3.578 ± 0.064 m** | **13.8 ± 0.6 %** | 1.457 m | 1.4668 m | 0/10 |
| 30 | 3.897 ± 0.117 m | 12.6 ± 1.6 % | 1.395 m | 1.4036 m | 0/10 |
| 100 | 5.016 ± 0.301 m | 4.3 ± 2.3 % | 1.061 m | 1.0623 m | 0/10 |

### Offline optimum (reference point)

With the true future over the whole mission, the card's $Q$/$R$ weights at the 0.2 s knots, the same speed and acceleration limits and no pillars: RMS 1.876 m (0.800 m for t ≥ 3 s), within 1 m 85.6 % of the time with $a_{max}$ = 6 m/s²; with 10 m/s² it is 0.851 m / 0.087 m / 95.2 %. It minimises a weighted cost at the knots, not the RMS of the continuous error, so it is a reference point and not a strict lower bound on the RMS. Its total RMS (1.876 m) is close to the oracle's steady RMS (1.930 m) only by coincidence; the two measure different things.

**Key findings**:
- **Prediction is essential.** Holding the current estimate leaves the drone 13.094 m RMS behind its target; the KF-predicted reference cuts this to 3.578 m, in every seed. The 2-s constant-velocity prediction itself is poor on this road (10.85 m RMS, up to 16.97 m, because a straight-line extrapolation over a fifth of a 10 s wave misses the curve), but the near end of the horizon, which drives the first control move, is accurate.
- **The card's MPC cost is the main weakness.** With position-only cost and a 2 s horizon, even perfect knowledge of the future (oracle, card cost) gives 7.209 m steady RMS: the controller builds up speed toward the reference and cannot brake in time. Adding velocity tracking lowers the oracle to 1.930 m and the KF version from 10.010 to 3.578 m. $Q_v = 10$ is the best of the five tested weights: 1 or 3 still lag (8.824 / 6.171 m), 30 or 100 over-weight the velocity reference (3.897 / 5.016 m).
- **Pillar constraints must hold between knots.** With constraints only at the 0.2 s knots, only the planned positions every 0.2 s are held at $r_{safe}$. At the drone's speeds (up to 15.41 m/s) consecutive knots can be up to about 3 m apart, and the path between them cuts the corner around the pillar. With pillars 1.5 or 2.0 m from the camera path the drone then passes **inside a pillar in 5/10 runs** (closest 0.432 / 0.364 m from the centre, pillar radius 0.6 m), and in 1/10 at 3.0 m. Imposing the same half-plane at every 0.05 s instant removes every pillar entry (0/10 at every offset and at every tested $Q_v$). With $Q_v = 10$ the worst fine-path clearance over the offset sweep is 1.439–1.457 m.
- **At $Q_v = 10$ the 1.5 m margin is met to within 6.1 cm, not exactly.** In the worst runs most of the shortfall is the soft constraint giving way at the constrained instants themselves: the worst clearance at the 0.05 s instants is 1.4476–1.4668 m across the offsets, so the soft constraint gives way by up to 5.2 cm, and the chord between two instants adds about 1 cm more (fine path 1.439–1.457 m). This split is not the same in every run: in runs where the slack stays inactive the instants hold 1.5 m and the whole (smaller) shortfall is the chord between instants, which can then be a few centimetres. The margin depends on $Q_v$: at $Q_v$ = 1 the instants hold 1.5000 m, while a large velocity weight lets the slack give way much more, to 1.4036 m at $Q_v$ = 30 and **1.0623 m at $Q_v$ = 100 (0.44 m inside $r_{safe}$)**, still without touching a pillar. Without any constraints the drone passes through a pillar in 4/10 runs (closest 0.210 m).
- **Detours cost tracking accuracy.** Pillars on the camera path raise the steady RMS from 2.075 m (no constraints) to 3.578 m (offset 0.5 m), and it rises further, to 4.648 m, when the pillars stand 3.0 m from the path: the drone stays farther out for longer.
- **The card's linearisation is too conservative.** One normal per pillar from the current position blocks the whole half-space beyond the pillar, so the drone hangs back: 5.644 m steady RMS, 41.7 slack-active solves per run and 9 non-converged QPs, yet still below $r_{safe}$ in 5/10 runs.
- **The card's pillar layout never tests avoidance.** Those pillars are 4.61–14.99 m from the desired camera path; the run is practically identical to the run without constraints (steady RMS 2.075 ± 0.051 vs 2.075 ± 0.050 m), and the clearance never drops below 2.817 m.

**Card targets**:

| Card target | Result (proposed) |
|-------------|-------------------|
| Framing score ≥ 0.80 (20–40 % band) | 0.0 % for every variant: unreachable at 9.43 m (needs 3.53–7.14 m). With the corrected 10–20 % band: 89.6 ± 0.1 % |
| RMS tracking error ≤ 1.5 m | Not met: 3.834 ± 0.057 m. The oracle reaches 2.518 m and the offline optimum reference point 1.876 m over the whole mission; the start from rest and the 9.79 m/s² lateral demand exceed the 6 m/s² limit |
| Min obstacle clearance ≥ 1.5 m | Not met exactly: at $Q_v = 10$, 1.482 ± 0.012 m (worst 1.457 m; 1.439 m over all pillar offsets), within 6.1 cm of the margin, and never inside a pillar. With $Q_v = 100$ the soft margin gives way to 1.061 m |
| Max drone speed ≤ 12 m/s | Met per axis (12.00 m/s, the card's ∞-norm bound); the speed magnitude reaches 15.41 m/s |

### 3D Trajectory

Seed 0, proposed controller: road (blue), desired camera path (green dashed), drone (red), pillars (grey):

![3D trajectory](./trajectory_3d.png)

### Tracking Error

Seed 0, log scale. Grey bands mark the car within 15 m of a pillar. The card-cost variants (pink, grey) swing to large errors every half wave (printed maxima 14.57 m and 14.52 m); the proposed controller stays at a few metres:

![Tracking error](./tracking_error.png)

### Camera Framing

Seed 0. Every curve stays below the card's 20–40 % band; the proposed controller stays in the corrected 10–20 % band most of the time:

![Framing](./framing.png)

### Obstacle Clearance

Seed 0, horizontal distance to the nearest pillar centre (sampled every 0.05 s). Without constraints (orange) the drone passes inside the pillar radius near t ≈ 6 s (time read from the figure; 12 fine-path points inside a pillar are printed). With knots-only constraints (olive) it dips to 0.946 m. The proposed controller (red) stays near the 1.5 m line (seed-0 minimum on the fine path 1.476 m):

![Clearance](./clearance.png)

### KF Prediction

Top: 2-s constant-velocity predictions (cyan diamonds) against the true car position 2 s later; bottom: prediction error over time:

![KF prediction](./kf_prediction.png)

### Monte Carlo

![Monte Carlo](./monte_carlo.png)

### Animation

Seed 0, proposed controller, top view in a window that follows the car: car (blue), drone (red) with a 3 s trail, desired position (green cross), KF 2-s prediction (cyan diamond), pillars (grey, dotted = 1.5 m margin). 189 frames at 10 fps (every 0.2 s):

![Animation](./animation.gif)

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| $\mathbf{d}_{offset} = (0, -8, 5)$ "behind and above" | (−8, 0, 5) | With the card's $R_{car}$ (x forward, y left), (0, −8) puts the drone 8 m to the right, at −90.0° to the heading (verified). Card corrected. |
| Car at "constant tangential speed 8 m/s" | $x = 8t$ as in the card's equations and code; along-road speed 8.00–12.36 m/s | The equations, the 37.5 s duration and the code all use constant x speed. Card text corrected. |
| Framing band 20–40 % | 10–20 % (card band also printed) | At the nominal 9.43 m the car fills 15.2 %; 20–40 % needs 3.53–7.14 m. Card corrected. |
| Pillars at (40, 8), (90, −10), (150, 12), (210, −6), (260, 9) | Same x stations, moved onto the camera path (0.5 m to the side, swept to 3.0 m); card layout kept as a variant | The card's pillars are 4.61–14.99 m from the camera path, so the constraint never binds (shown by the card-layout variant). Card note added. |
| Obstacle constraints at the MPC knots ($i = 1..N$) | At every 0.05 s instant of the horizon (40 points); knots-only kept as a variant | Knots-only lets the path cut the corner between knots 2–3 m apart; with pillars 1.5–2.0 m from the path the drone enters a pillar in 5/10 runs. |
| Obstacle normals from the current position $\mathbf{p}_0$ for all steps | One normal per constraint point from the previous plan; card form kept as a variant | The card form over-constrains and degrades tracking (5.644 vs 3.578 m). |
| Hard obstacle constraints | Soft, one slack per pillar ($w_q = 10^4$, $w_l = 10^3$) | Keeps the QP feasible when the acceleration limit cannot avoid the margin in time; the margin is then missed by up to 6.1 cm at $Q_v = 10$ (up to 0.44 m at $Q_v = 100$). |
| MPC cost: positions and accelerations only | Added velocity tracking $Q_v = 10$; card cost kept as two variants; $Q_v$ sweep 1–100 printed | The card cost overshoots and lags (10.010 m RMS with the KF, 7.209 m even with the oracle). $Q_v$ was first picked from {1, 3, 10} on seed 0; the 10-seed sweep confirms 10 as the best tested value. |
| Speed limit 12 m/s | Per axis, as in the card's constraint $\|\mathbf{v}_i\|_\infty \le 12$ | The magnitude reaches 15.41 m/s; a norm bound is not modelled. |
| cvxpy + OSQP | Own dense interior-point QP solver | cvxpy is not installed in the `drones` environment. |
| Card code computes the KF prediction but builds the reference from the true future | Reference from the KF prediction; the true future is the oracle variant | The card code silently used ground truth. Card note added. |
| Tolerance-ball objective without a value | 1.0 m (the card's plot line) | Card's expected-output figure. |
| MP4/GIF top view | GIF in a window following the car | Repository convention; the whole 300 m road would make the drone invisible. |
| Single run | 10 noise seeds with common random numbers; pillar-offset and $Q_v$ sweeps; offline optimum as a reference point | Results depend on the measurement noise and on the added design choices. |

---

## Extensions

1. **Coordinated-turn KF**: replace the constant-velocity model with a constant-turn-rate model and measure how much of the 3.578 → 1.930 m prediction gap closes.
2. **Longer horizon** (3–5 s) or a terminal velocity constraint, to approach the 0.800 m offline reference.
3. **Hard margin**: add a velocity-dependent braking constraint and a chord margin so the 1.5 m margin holds without slack.
4. **Occlusion-aware framing**: penalise lines of sight that pass within a pillar radius.
5. **Moving obstacles**: oncoming cars predicted by their own KFs as time-varying half-planes.

---

## Related Scenarios

- Prerequisites: [S081 Selfie Follow Mode](../../../scenarios/05_special_entertainment/S081_selfie_follow.md), [S086 Cooperative Multi-Angle Filming](../../../scenarios/05_special_entertainment/S086_multi_angle_cinema.md)
- Next: [S094 Aerial Ball Catching](../../../scenarios/05_special_entertainment/S094_ball_catching.md)
- Cross-reference: [S003 Low-Altitude Tracking](../../../scenarios/01_pursuit_evasion/S003_low_altitude_tracking.md), [S004 Obstacle-Course Chase](../../../scenarios/01_pursuit_evasion/S004_obstacle_chase.md), [S008 Stochastic Pursuit — Kalman Filter Tracking](../../../scenarios/01_pursuit_evasion/S008_stochastic_pursuit.md)
