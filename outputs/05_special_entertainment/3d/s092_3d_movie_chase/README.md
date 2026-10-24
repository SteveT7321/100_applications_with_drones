# S092 3D Upgrade — Movie Chase Scene

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: ✅ Completed
**Based on**: [S092 Movie Car Chase Scene](../../s092_movie_chase/README.md) (2D version)

---

## Problem Definition

**Setup**: The S092 camera drone films a stunt car on a winding road ($x = 8t$, $y = 15\sin(2\pi x/80)$, 300 m, 37.5 s). It filters 20 Hz car fixes with a Kalman filter and plans with the S092 model predictive controller (10 steps of 0.2 s, position + velocity tracking, pillar half-planes at every 0.05 s of the horizon, soft constraints with slack, pillars moved onto the camera path). In S092 the drone flew at a fixed altitude and the pillars were treated as infinitely tall. Here the world is three-dimensional and **altitude becomes a planned state**:

- the car drives over a 2 m ramp (logistic rise centred at $x$ = 60 m, descent at 100 m), measured by a barometer ($\sigma_z$ = 0.1 m) and tracked by a 6-state KF;
- the five pillars have a finite height (card: 4 m; 6 m and 8 m also run), so the drone may fly **over** a pillar ($z \ge H + 0.5$ m) instead of around it — the MPC enumerates the around/over choice for each nearby pillar and keeps the cheaper plan;
- a bridge deck (slab $x$ 140–160 m, $|y| \le$ 20 m, $z$ 3–6 m) crosses the road; the car passes under it, and a drone at the nominal height would fly into it. The drone must **underfly** ($z \le$ 2.5 m, the car stays in view) or **overfly** ($z \ge$ 6.5 m, the deck hides the car). The card's rule picks the mode with the smaller altitude change;
- the desired camera position is 8 m behind and **4.62 m** above the car (the card's 5 m gives a 32.0° gimbal pitch, outside its own ±30° envelope; its adjustment rule gives $8\tan 30°$ = 4.619 m);
- the MPC gets a linearised gimbal-pitch-rate penalty ($w_\phi$ = 2) and separate vertical limits ($|v_z| \le$ 4 m/s, $|a_z| \le$ 3 m/s²).

**Variants** (10 seeds; every variant sees the same measurement noise on a given seed):

| Variant | Altitude | Pillars | Deck | Source |
|---|---|---|---|---|
| 2D baseline | fixed 4.62 m (S092 plan, horizontal KF) | always around (infinitely tall, as S092) | unknown | X5 baseline |
| 3D, card deck rule | tracks car $z$ + 4.62 m | MPC chooses around / over | card rule (smaller $\lvert\Delta z\rvert$) | card |
| **3D, underfly deck** | as above | MPC chooses | always under | added |
| 3D, no pillar overfly | as above | always around | under | added ablation |
| 3D underfly, $w_\phi$ = 0 | as above | MPC chooses | under | added ablation |
| 3D underfly, no z prediction | car altitude held at its estimate over the horizon | MPC chooses | under | added ablation |
| 3D underfly, oracle | true future reference | MPC chooses | under | added reference |

---

## Mathematical Model

**Car**: $z_{car}(x) = h_{ramp}\bigl[\sigma((x - s_1)/w) - \sigma((x - s_2)/w)\bigr]$, $h_{ramp}$ = 2 m, $w$ = 3 m (5–95 % rise over 17.67 m of road, maximum 1.995 m).

**KF**: 6-state constant velocity, $Q = q\,[\tfrac{\Delta t^3}{3}I, \tfrac{\Delta t^2}{2}I; \tfrac{\Delta t^2}{2}I, \Delta t I]$, $q$ = 1 m²/s³, $R = \text{diag}(0.3^2, 0.3^2, 0.1^2)$, 20 Hz. The reference over the horizon is the constant-velocity prediction $\hat{\mathbf p}(\tau) = \hat{\mathbf p} + \tau\hat{\mathbf v}$, offset 8 m behind along $\hat\psi$ and 4.62 m up.

**MPC** (condensed QP in the accelerations, as S092): $J = \sum_i \|\mathbf p_i - \mathbf p_{d,i}\|^2_{Q} + Q_v\|\mathbf v_i - \mathbf v_{d,i}\|^2 + R\|\mathbf u_i\|^2 + P_f\|\mathbf p_N - \mathbf p_{d,N}\|^2 + w_\phi\sum_i(\phi_{i+1} - \phi_i)^2/\Delta t^2 + \text{slack}$, with $\phi_i$ linearised about the reference. Limits $|v_{x,y}| \le 12$, $|u_{x,y}| \le 6$, $|v_z| \le 4$, $|u_z| \le 3$, $1 \le z \le 12$ m. Soft obstacle constraints at every 0.05 s instant: pillar "around" $\hat{\mathbf n}^\top(\mathbf p_{xy} - \mathbf o) \ge r_{safe}$ (normal from the previous plan); pillar "over" $z \ge H + 0.5$ at instants within 2.5 m horizontally of the pillar; deck "under" $z \le 2.5$ / "over" $z \ge 6.5$ at instants within the deck span ± 1.5 m. Every around/over combination of the nearby pillars is solved and the lowest objective kept.

**Metrics**: tracking error to the nominal desired point; RMS after 3 s outside the deck span (where the nominal point lies inside the deck) split into horizontal and vertical parts; framing $f = 2\arctan(h/2d)\cos\phi/\text{FoV}$ to the car's centre, counted only when the line of sight is not blocked by the deck or a pillar (10–20 % band, as S092); gimbal pitch $\phi$ to the car's centre; 3D distance to the nearest obstacle surface along the integrated path (10 points per 0.05 s).

---

## Key Parameters

| Parameter | Value | Source |
|---|---|---|
| Road, car speed | $x = 8t$, $A$ = 15 m, $L$ = 80 m, 300 m | card / S092 |
| Ramp | 2 m, $w$ = 3 m, centres at $x$ = 60 m and 100 m | card (positions added) |
| Offset | 8 m behind, 4.62 m above (card 5 m) | card's gimbal rule |
| Pillars | 5, radius 0.6 m, height 4 m (swept 6, 8 m), 0.5 m from the camera path | card / S092 layout |
| $r_{safe}$ / vertical clearance | 1.5 m / 0.5 m | card |
| Deck | $x$ 140–160 m, $y$ ±20 m, $z$ 3–6 m | card (slab interpretation) |
| Altitude bounds | 1–12 m | card |
| KF | $\sigma_{xy}$ 0.3 m, $\sigma_z$ 0.1 m, $q$ = 1 m²/s³, 20 Hz | card |
| MPC | $N$ = 10, 0.2 s, $Q$ = 10, $R$ = 0.1, $P_f$ = 20, $Q_v$ = 10 | card / S092 |
| $w_\phi$ | 2 (swept 0, 2, 200, 2000) | card (+ sweep) |
| Limits | $v_{xy}$ 12, $a_{xy}$ 6, $v_z$ 4 m/s, $a_z$ 3 m/s² | card |
| Gimbal envelope / rate | ±30°, 30 °/s | card |
| Camera FoV / car height | 60° / 1.5 m | card |
| Framing band | 10–20 % (card 20–40 % also computed) | S092 correction |
| Overfly rows / deck margin | within 2.5 m of a pillar / deck span ± 1.5 m | added |
| Seeds | 10, noise of seed $s$ from `default_rng(9200 + s)` | added (as S092) |

---

## Implementation

```
src/05_special_entertainment/3d/s092_3d_movie_chase.py
```

```bash
conda activate drones
python src/05_special_entertainment/3d/s092_3d_movie_chase.py
```

The QP solver, the condensed-MPC matrices, the road and the pillar layout are imported from the S092 script. 170 runs go through a process pool of 4 workers; results are collected in job order. Runtime ≈ 2 min. Two single-threaded runs and one default-threading run printed byte-identical output and produced byte-identical PNGs and GIF.

---

## Results

### Main comparison (10 seeds, mean ± std)

| Variant | RMS error | RMS, $t \ge 3$ s, outside deck | horizontal / vertical part | Within 1 m | Framed and visible | Visible (in deck span) | Gimbal within ±30° | Ramp altitude RMS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 2D baseline | 3.896 ± 0.104 m | 3.687 ± 0.119 m | 3.613 / 0.730 m | 10.7 % | 80.2 % | 91.0 % (7.0 %) | 95.2 % | 1.394 m |
| 3D, card deck rule | 2.759 ± 0.064 m | 2.205 ± 0.047 m | 2.167 / 0.408 m | 17.2 % | 88.6 % | 92.3 % (20.8 %) | 77.9 % | 0.229 m |
| **3D, underfly deck** | 2.901 ± 0.029 m | 2.291 ± 0.042 m | 2.249 / 0.439 m | 17.1 % | **92.8 %** | 96.6 % (**70.6 %**) | 91.8 % | 0.229 m |
| 3D, no pillar overfly | 3.901 ± 0.104 m | 3.640 ± 0.120 m | 3.614 / 0.440 m | 13.4 % | 85.4 % | 96.2 % (70.6 %) | 94.5 % | 0.230 m |
| 3D underfly, $w_\phi$ = 0 | 2.901 ± 0.029 m | 2.291 ± 0.042 m | 2.249 / 0.439 m | 17.1 % | 92.8 % | 96.6 % (70.6 %) | 91.8 % | 0.229 m |
| 3D underfly, no z prediction | 2.914 ± 0.029 m | 2.310 ± 0.041 m | 2.248 / 0.530 m | 17.1 % | 92.8 % | 96.6 % (70.6 %) | 90.1 % | 0.612 m |
| 3D underfly, oracle | 2.450 m | 1.721 m | 1.676 / 0.394 m | 40.7 % | 94.1 % | 98.0 % (83.0 %) | 100.0 % | 0.028 m |

**Paired per seed against 3D underfly**: against the 2D baseline it has a lower RMS outside the deck in 10/10 seeds (−1.395 m), lower horizontal RMS in 10/10 (−1.365 m), higher framing in 10/10 (+12.6 pp) and lower ramp altitude error in 10/10 (−1.165 m). Against the card deck rule: higher RMS outside the deck in 10/10 (+0.086 m) but higher framing in 8/10 (+4.2 pp). Against no pillar overfly: lower RMS in 10/10 (−1.349 m), higher framing in 10/10 (+7.5 pp). Against no z prediction: lower ramp altitude error in 10/10 (−0.382 m). Against $w_\phi$ = 0: no measurable difference (mean differences +0.000 m and +0.0 pp).

**Safety** (over all seeds; 3D variants asserted never inside a pillar or the deck):

| Variant | Min 3D clearance to any surface | Min horizontal pillar clearance below the top | Min height above a pillar top (within $r_{safe}$) | Fine-path points inside pillar / deck | Max $\lvert v_{xy}\rvert_\infty$ / $\lvert v_z\rvert$ | Altitude range |
|---|---|---|---|---|---|---|
| 2D baseline | **0.000 m** | — (never below a top) | 0.619 m | 0 / **4594** | 12.00 / 0.00 m/s | 4.62 m |
| 3D, card deck rule | 0.500 m | 1.502 m | 0.537 m | 0 / 0 | 12.00 / 2.08 m/s | 2.40–6.97 m |
| 3D, underfly deck | 0.500 m | 1.488 m | 0.537 m | 0 / 0 | 12.00 / 2.08 m/s | 2.40–6.97 m |
| 3D, no pillar overfly | 0.500 m | 1.488 m | 0.522 m | 0 / 0 | 12.00 / 2.08 m/s | 2.40–6.97 m |
| 3D underfly, oracle | 0.500 m | 2.212 m | 0.613 m | 0 / 0 | 12.00 / 1.89 m/s | 2.41–6.63 m |

MPC: 1880 solves per variant, none failed to converge; slack was active in 57 solves for the 2D baseline and 10 for 3D underfly. 3D underfly chose overfly for at least one pillar in 394 planning cycles and underflew the deck in 235; the card rule overflew it in 188 and underflew in 47.

**Gimbal pitch rate** after 3 s (RMS / max): 2D 4.98 / 14.27 °/s, card rule 4.24 / 12.49, underfly 4.68 / 14.83, no overfly 5.37 / 15.55, $w_\phi$ = 0 4.69 / 14.84, oracle 3.72 / 11.81 °/s. Over the whole run the maximum is 32.2 °/s for every variant (start-up). The card's "rate ≤ 30 °/s" fraction is 99.9 % for every variant, so it does not discriminate.

**$w_\phi$ sweep** (3D underfly; pitch-rate RMS after 3 s / RMS outside deck / framed): 0: 4.685 °/s / 2.291 m / 92.8 %; 2 (card): 4.684 / 2.291 / 92.8 %; 200: 4.578 / 2.304 / 92.7 %; 2000: 4.122 / 2.357 / 91.1 %.

**Pillar-height sweep** (RMS outside deck, horizontal part in brackets):

| Pillar height | 2D baseline | 3D underfly (overfly allowed) | 3D, no pillar overfly | Overfly vs no overfly (horizontal) |
|---|---|---|---|---|
| 4 m (card) | 3.687 m (3.613) | 2.291 m (2.249) | 3.640 m (3.614) | −1.365 m, better on 10/10 |
| 6 m | 3.687 m (3.613) | 3.615 m (3.588) | 3.640 m (3.614) | −0.025 m, better on 9/10 |
| 8 m | 3.687 m (3.613) | 3.627 m (3.600) | 3.640 m (3.614) | −0.014 m, better on 8/10 |

**Key findings**

- **A fixed-altitude plan is not feasible in this world.** The 2D baseline flies straight into the bridge deck (4594 fine-path points, at 0.005 s spacing, inside the slab over the 10 seeds; 458 on seed 0) and sees the car only 7.0 % of the deck passage. Altitude planning is needed for safety, not only for the shot.
- **Underflying the deck keeps the shot; the card's mode rule loses it.** The card's "smaller $|\Delta z|$" rule picks overfly (1.88 m up vs 2.12 m down), and the deck then hides the car: visible 20.8 % of the deck passage vs 70.6 % when underflying, framing 88.6 % vs 92.8 % (underfly better in 8/10 seeds), gimbal within ±30° 77.9 % vs 91.8 %. The card rule tracks slightly better outside the deck (2.205 vs 2.291 m), possibly because its altitude change is smaller (1.88 vs 2.12 m); this was not probed.
- **Flying over the card's 4 m pillars removes the detour cost.** The nominal camera height (4.62 m) is already above the 4 m pillars, so overflying costs no climb: the horizontal RMS drops from 3.614 m (always around, the S092 behaviour) to 2.249 m on 10/10 seeds, and framing rises by 7.5 pp. With 6 m or 8 m pillars the drone would have to climb to 6.5 m or 8.5 m ($H$ + 0.5 m) and the benefit nearly disappears (−0.025 m and −0.014 m). The vertical-evasion benefit therefore depends on the pillar height the card chose.
- **The 6-state KF does anticipate the ramp.** Predicting the car's vertical velocity cuts the altitude error on the ramp from 0.612 m to 0.229 m (10/10 seeds); the horizontal error is unchanged.
- **The gimbal-rate penalty at the card's weight has no effect.** With $w_\phi$ = 2 the pitch-rate RMS is 4.684 °/s against 4.685 °/s without the penalty. Only $w_\phi$ = 2000 lowers it noticeably, to 4.122 °/s, and then the RMS error rises from 2.291 m to 2.357 m and framing falls from 92.8 % to 91.1 %.
- **The 2D baseline is not worse because of altitude alone.** Its horizontal error (3.613 m) equals the 3D no-overfly variant's (3.614 m); the 3D improvement on the main comparison comes from overflying the pillars, plus the ramp tracking and the deck.
- **Card targets**: framing ≥ 0.75 — met (92.8 % with underfly, 88.6 % with the card rule); RMS ≤ 2.0 m — not met (2.901 m; 2.291 m after 3 s outside the deck; the oracle reaches 2.450 m / 1.721 m); minimum clearance ≥ 1.5 m — met horizontally below pillar tops (1.488 m, within the S092 soft-constraint margin), but vertical clearances are 0.5 m by the card's own $z_{clearance}$ (the two requirements conflict); gimbal within ±30° ≥ 95 % — not met (91.8 %; on seed 0 the pitch leaves the envelope briefly on some bends, read from the figure, while under the deck it is shallow); speed ≤ 12 m/s per axis and $|v_z| \le$ 4 m/s — met (12.00 and 2.08 m/s).

### 3D trajectory

Seed 0: car with the ramp (blue), desired offset (green dashed), underfly (red), card rule (orange, over the deck) and the 2D baseline (grey dotted, through the deck); pillars and deck in grey:

![3D trajectory](./trajectory_3d.png)

### Altitude

Seed 0: car altitude (ramp), desired altitude, the drone altitude of four variants, and the deck slab during its span. The underfly drone drops to 2.5 m, the card rule climbs to 6.5 m, the 2D baseline stays at 4.62 m inside the slab; without z prediction the climb onto the ramp lags:

![Altitude](./altitude.png)

### Gimbal pitch and framing

Seed 0. Top: pitch to the car's centre with the ±30° envelope. Bottom: car height in the frame; gaps are times when the car is hidden:

![Gimbal and framing](./gimbal_framing.png)

### Obstacle clearance

Seed 0: 3D distance to the nearest obstacle surface along the integrated path; the 2D baseline reaches 0 (inside the deck):

![Clearance](./clearance.png)

### Monte Carlo

Horizontal RMS outside the deck span, framing, ramp altitude error (10 seeds), and the pillar-height sweep:

![Monte Carlo](./monte_carlo.png)

### Animation

Seed 0, 3D underfly. Top: 3D view following the car (sight line black when the car is visible, red when hidden). Bottom: side elevation with the deck and pillars. 189 frames at 10 fps (one frame every 0.2 s):

![Animation](./animation.gif)

### Deviations from the scenario card

| Card | Implementation | Reason |
|---|---|---|
| $\mathbf d_{offset} = (0, -8, 5)$ in the Frenet frame | 8 m behind in the yaw frame, height 4.62 m; card corrected | (0, −8) is to the side (S092 correction); the Frenet normal of the sine road flips at every inflection, so the "up" axis would flip; 5 m violates the card's own ±30° envelope (32.0°) |
| Pillar height 4 m | 4 m, plus 6 m and 8 m | at 4 m the pillars are below the 4.62 m camera height, so overflying needs no climb; the sweep shows the result depends on this |
| Deck box $[0, z_{deck}]$, "car passes beneath" | slab $z$ 3–6 m; card corrected | a box from the ground would block the car |
| Ramp position not given | centres at $x$ = 60 m and 100 m | added |
| Overfly "when the horizontal corridor is tighter than $r_{safe}$" | the MPC solves every around/over combination of the nearby pillars and keeps the cheapest | a corridor test is not defined for single pillars |
| Deck mode by smaller $\lvert\Delta z\rvert$ | run as the card rule and as forced underfly | the card rule loses the shot |
| $f_{height} = \theta/(\text{FoV}\cos\phi)$, band 20–40 % | $\theta\cos\phi/\text{FoV}$, 10–20 %, only when visible; card corrected | foreshortening makes a vertical object smaller; 20–40 % is unreachable (S092) |
| Clearance ≥ 1.5 m from every surface | horizontal clearance below pillar tops, vertical clearance above tops / below the deck reported separately | the card's own vertical clearance is 0.5 m |
| Pitch to the car's ground point | pitch to the car's centre (0.75 m) | at the ground point the nominal camera sits exactly on the ±30° limit, so the in-envelope fraction would flip with noise; nominal pitch to the centre is 25.8° |
| Gimbal-rate cost on the nonlinear pitch | linearised about the reference | keeps the QP convex |
| Velocity-reference weight not in the card cost | $Q_v$ = 10 (S092) | S092 showed the card cost lags badly |
| Single run | 10 seeds, same noise for every variant; sweeps of pillar height and $w_\phi$ | X2, added-parameter sweeps |

---

## Extensions

1. **Occlusion-aware underflight**: add a visibility cost; the car is visible only 70.6 % of the deck passage even when underflying (the causes — the pier at $x$ ≈ 150 m or the descent near the deck edges — were not separated).
2. **Mode choice by shot value**: choose under/over by predicted visibility and framing, not by $|\Delta z|$.
3. **Taller pillars**: with 6–8 m pillars, a lateral offset of the desired shot near each pillar may beat both climbing and detouring.
4. **Rotor–deck interaction**: underflight 0.5 m below a deck changes thrust (ceiling effect); model it before trusting the 0.5 m margin.
5. **Gimbal smoothing outside the MPC**: since $w_\phi$ needs to be ~1000× the card value to matter, smooth the camera with its own controller instead.

---

## Related Scenarios

- Original 2D version: [S092 Movie Car Chase Scene](../../../../scenarios/05_special_entertainment/S092_movie_chase.md)
- 3D card: [S092 3D Upgrade — Movie Chase Scene](../../../../scenarios/05_special_entertainment/3d/S092_3d_movie_chase.md)
- 3D pursuit reference: [S003 Low-Altitude Tracking](../../../../scenarios/01_pursuit_evasion/S003_low_altitude_tracking.md)
- Gimbal / framing: [S081 Selfie Follow Mode](../../../../scenarios/05_special_entertainment/S081_selfie_follow.md), [S086 Cooperative Multi-Angle Filming](../../../../scenarios/05_special_entertainment/S086_multi_angle_cinema.md)
- Obstacle-course pursuit: [S004 Obstacle-Course Chase](../../../../scenarios/01_pursuit_evasion/S004_obstacle_chase.md)
