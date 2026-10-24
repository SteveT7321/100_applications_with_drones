# S076 3D Upgrade — Thermal Management in Hot Environment

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: ✅ Completed
**Based on**: [S076 Extreme Heat Cooling Strategy](../../s076_thermal_management/README.md) (2D version)

---

## Problem Definition

**Setup**: The 2D S076 flies a 100 × 100 m lawnmower (20 strips, 5 m apart, 1.0 m/s) at one height in a uniform 45 °C ambient. Here the ambient depends on altitude: a superheated surface layer (55 °C at the ground, falling 5 °C/m up to $z_{bl}$ = 2 m, where it is 45 °C) and a weak lapse rate of 0.0065 °C/m above it. The drone may fly between 1 and 6 m; climbs and descents cost $k_z v_z^2$ of extra motor power. Motor and battery temperatures follow first-order models driven by $T_{amb}(z(t))$, and the battery capacity is derated above 25 °C (the card's 80 % capacity floor means $T_{bat} \le 65$ °C).

**Strategies** — all use the same speed rules, so they differ only in the altitude plan: the 2D base's fastest safe motor law (above $T_{warn}$ = 80 °C, throttle to the speed whose steady state is $T_{max}$ = 85 °C at the current ambient; cooling pause at $T_{max}$) and a battery rule of the same form that enforces the card's 80 % capacity floor (above $T_{bat,warn}$ = 63 °C, throttle to the speed whose battery steady state is 65 °C; pause at 65 °C):

| Key | Altitude plan | Source |
|---|---|---|
| `fixed_2m` | 2.0 m throughout (the 2D base: $T_{amb}(2\ \text{m})$ = 45 °C is the base's ambient) | 2D baseline |
| `fixed_1m` | 1.0 m throughout (low inspection pass inside the hot layer) | added |
| `alt_opt` | climb once to $z^* = \arg\min_z T_{m,\infty}(v_0, z)$ | card |
| `mpc` | before each strip, simulate the strip from the current state for each altitude 1.0, 1.5, …, 6.0 m and pick the one minimising strip time + $w\,(T_{m,peak} - T_{warn})^+$ | card ("3D thermal corridor MPC") |

All strategies start at 2.0 m with motor and battery at the ambient there. The model is deterministic, so no Monte Carlo is needed.

**Cases**: the card ($T_{surf}$ = 55 °C, $z_{bl}$ = 2 m); a hot case $T_{surf}$ = 63 °C (ambient 53 °C at 2 m); a sweep of $T_{surf}$ = 55–65 °C for hot-layer depths $z_{bl}$ = 2, 3 and 4 m.

---

## Mathematical Model

$$T_{amb}(z) = \begin{cases} T_{surf} - 5z & z < z_{bl} \\ T_{surf} - 5z_{bl} - 0.0065\,(z - z_{bl}) & z \ge z_{bl}\end{cases}$$

$$P_{motor} = P_{hover} + k_v v_h^2 + k_z v_z^2, \qquad \tau\,\dot T_m = R_{th}P_{motor} - (T_m - T_{amb}(z)), \qquad \tau_{bat}\,\dot T_{bat} = R_{bat}\frac{P_{motor}}{\eta_{esc}} - (T_{bat} - T_{amb}(z))$$

integrated exactly over each 0.1 s step (piecewise-constant power, as in the 2D base). A cooling pause relaxes the motor toward $T_{amb}(z)$ with $\tau_{cool}$ = 20 s (2D base model).

**Speed law** (2D base, throttle to $T_{max}$): above $T_{warn}$, $v = \min\bigl(v_0, \sqrt{((T_{max} - T_{amb}(z))/R_{th} - P_{hover})/k_v}\bigr)$.

**Climbs** (card $f_{trap}$, corrected duration): $z(t) = z_0 + \Delta z\,f_{trap}(t, T)$ with $T = 2|\Delta z|/v_{z,max}$, so the peak vertical speed is exactly 1.5 m/s; the drone hovers horizontally during a climb.

**Battery**: $Q_{avail}/Q_{nom} = 1 - 0.005\max(0, T_{bat} - 25)$; the constraint $Q_{avail}/Q_{nom} \ge 0.8$ is $T_{bat} \le 65$ °C. It is enforced as a speed cap for every strategy, 2D baseline included: above $T_{bat,warn}$, $v = \min\bigl(v_{motor}, \max(v_{min}, \sqrt{((T_{bat,max} - T_{amb}(z))\,\eta_{esc}/R_{bat} - P_{hover})/k_v})\bigr)$, and a pause when $T_{bat}$ reaches 65 °C. Where even $v_{min}$ would settle above 65 °C the drone has to alternate pauses and flight (the 1 m pass in the hot case). The script asserts peak $T_{bat} \le$ 65.5 °C in every run with the rule.

---

## Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Field / strips / speed | 100 × 100 m / 20 strips 5 m apart / 1.0 m/s | card |
| Altitude range | 1.0–6.0 m | card |
| $T_{surf}$ / layer gradient / $z_{bl}$ / lapse above | 55 °C / −5 °C/m / 2 m / 0.0065 °C/m | card |
| $T_{max}$ / $T_{warn}$ | 85 / 80 °C | card |
| $R_{th}$ / $\tau$ / $\tau_{cool}$ | 0.8 °C/W / 40 s / 20 s | card |
| $P_{hover}$ / $k_v$ / $k_z$ | 30 W / 15 W·s²/m² / 20 W·s²/m² | card |
| $v_{z,max}$ | 1.5 m/s | card |
| $R_{bat}$ / $\tau_{bat}$ / $\beta_{bat}$ / $\eta_{esc}$ | 0.3 °C/W / 120 s / 0.005 °C⁻¹ / 0.92 | card |
| $Q_{min}/Q_{nom}$ → $T_{bat,max}$ | 0.80 → 65 °C | card |
| Battery throttle threshold $T_{bat,warn}$ | 63 °C (hot case also run with 60 and 64.5 °C) | added |
| Pause / time step | 30 s / 0.1 s | card |
| Turn / speed floor | 3 s at hover power / 0.05 m/s | 2D base |
| Low pass altitude | 1.0 m | added |
| MPC candidates / weight $w$ | 1.0–6.0 m in 0.5 m steps / 10 s per °C (swept 0, 10, 100, 1000 in six cases) | added |
| Hot case / sweeps | $T_{surf}$ 63 °C; $T_{surf}$ 55–65 °C × $z_{bl}$ 2, 3, 4 m | added |
| Animation | hot case, one frame per 30 s of mission, 105 frames (ends at 51.97 min, when the last of the three other strategies finishes; the 1 m pass is still flying) | added |

---

## Implementation

```
src/04_industrial_agriculture/3d/s076_3d_thermal_management.py
```

```bash
conda activate drones
python src/04_industrial_agriculture/3d/s076_3d_thermal_management.py
```

Runtime 201–265 s over three runs. Two single-threaded runs and one run with default threading printed byte-identical output and produced byte-identical PNG and GIF files.

---

## Results

### Steady state against altitude (card values)

| Altitude | $T_{amb}$ | $T_{m,\infty}(v_0)$ | $T_{bat,\infty}(v_0)$ |
|---|---:|---:|---:|
| 1 m | 50.00 °C | 86.00 °C | 64.67 °C |
| 2 m | 45.00 °C | 81.00 °C | 59.67 °C |
| 6 m | 44.974 °C | 80.974 °C | 59.648 °C |

From 1 to 6 m the ambient falls 5.026 °C, but from 2 to 6 m only 0.026 °C. The safe corridor starts at $z_{safe,lo}$ = 1.20 m; the card's $z^*$ = 6.00 m. Throttle-to-$T_{max}$ speed: 0.957 m/s at 1 m, 1.155 m/s (above $v_0$) at 2 m. A 2 → 6 m climb takes 5.333 s at a peak motor power of 75 W. Battery-rule speed at 2 m: 1.445 m/s in the card case (never binding) and 0.673 m/s in the hot case; hovering at 2 m in the hot case settles at 62.78 °C, so the battery floor is reachable there only by flying slowly.

### Card case ($T_{surf}$ = 55 °C, $z_{bl}$ = 2 m) and hot case ($T_{surf}$ = 63 °C)

| Case | Strategy | Mission | Pauses | Peak $T_m$ | Time above $T_{max}$ | Peak $T_{bat}$ | Min $Q/Q_{nom}$ | Throttled (by battery rule) | Climbed | Energy | Altitude |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| card | Fixed 2 m (2D baseline) | 2152.0 s (35.87 min) | 0 | 80.93 °C | 0.0 s | 59.59 °C | 0.8271 | 0.0 s (0.0 s) | 0.00 m | 104.33 kJ | 2.0 m |
| card | Fixed 1 m | 2243.1 s (37.39 min) | 0 | 84.95 °C | 0.0 s | 64.19 °C | 0.8040 | 2109.1 s (0.0 s) | 1.00 m | 105.92 kJ | 1.0 m |
| card | Altitude-optimised | 2157.3 s (35.96 min) | 0 | 80.91 °C | 0.0 s | 59.56 °C | 0.8272 | 0.0 s (0.0 s) | 4.00 m | 104.59 kJ | 6.0 m |
| card | Strip-ahead MPC | 2152.0 s (35.87 min) | 0 | 80.93 °C | 0.0 s | 59.59 °C | 0.8271 | 0.0 s (0.0 s) | 0.00 m | 104.33 kJ | 2.0 m |
| hot | Fixed 2 m (2D baseline) | 3118.4 s (51.97 min) | 0 | 84.53 °C | 0.0 s | 64.98 °C | 0.8001 | 3005.9 s (2896.8 s) | 0.00 m | 125.19 kJ | 2.0 m |
| hot | Fixed 1 m | 63553.4 s (1059.22 min) | 765 | 83.95 °C | 0.0 s | 65.00 °C | 0.8000 | 40503.3 s (40441.0 s) | 1.00 m | 1326.63 kJ | 1.0 m |
| hot | Altitude-optimised | 3109.0 s (51.82 min) | 0 | 84.50 °C | 0.0 s | 64.98 °C | 0.8001 | 2996.4 s (2886.7 s) | 4.00 m | 125.07 kJ | 6.0 m |
| hot | Strip-ahead MPC | 3118.4 s (51.97 min) | 0 | 84.53 °C | 0.0 s | 64.98 °C | 0.8001 | 3005.9 s (2896.8 s) | 0.00 m | 125.19 kJ | 2.0 m |

Against the 2D baseline: in the card case `alt_opt` takes +5.3 s and is −0.03 °C cooler (motor and battery), `mpc` is identical (+0.0 s), `fixed_1m` takes +91.1 s and runs +4.01 °C hotter. In the hot case `alt_opt` is 9.4 s faster, `mpc` identical, and `fixed_1m` takes +60435.0 s: at 1 m even the slowest flight heats the battery past 65 °C, so it alternates 765 cooling pauses with flight. The card-case fixed 2 m run reproduces the 2D base's throttle-to-$T_{max}$ result (35.87 min at 45 °C); the battery rule never binds there.

**What the battery rule costs** (hot case, same runs without the rule, motor rule only):

| Strategy | Without the rule | Peak $T_{bat}$ / min $Q/Q_{nom}$ without | With the rule | Cost |
|---|---:|---:|---:|---:|
| Fixed 2 m (2D baseline) | 2610.4 s | 66.00 °C / 0.7950 | 3118.4 s | +508.0 s |
| Fixed 1 m | 4206.5 s | 69.00 °C / 0.7800 | 63553.4 s | +59346.9 s |
| Altitude-optimised | 2612.8 s | 65.99 °C / 0.7951 | 3109.0 s | +496.2 s |
| Strip-ahead MPC | 2610.4 s | 66.00 °C / 0.7950 | 3118.4 s | +508.0 s |

Without the rule every strategy breaks the 80 % floor in the hot case; with it every strategy keeps it (min $Q/Q_{nom}$ ≥ 0.8000) at a cost of 496.2–508.0 s at 2 m or above. The throttle threshold matters little: with $T_{bat,warn}$ = 60, 63 and 64.5 °C the baseline takes 3135.7, 3118.4 and 3100.4 s (peak $T_{bat}$ 64.98 °C each) and the MPC exactly the same.

**MPC weight** (strip altitudes chosen, peak motor temperature, mission time):

| Case | $w$ = 0 | $w$ = 10 (default) | $w$ = 100 | $w$ = 1000 s/°C |
|---|---|---|---|---|
| card (55 °C, $z_{bl}$ 2 m) | 2.0 m, 80.93 °C, 2152.0 s | 2.0 m, 80.93 °C, 2152.0 s | 2.0 m, 80.93 °C, 2152.0 s | 2.0 and 6.0 m, 80.91 °C, 2157.3 s |
| 60 °C, $z_{bl}$ 3 m | 2.5 m, 83.43 °C, 2152.7 s | 3.0 m, 80.93 °C, 2153.3 s | 3.0 m, 80.93 °C, 2153.3 s | 3.0 and 6.0 m, 80.91 °C, 2157.3 s |
| 58 °C, $z_{bl}$ 4 m | 2.0 m, 83.93 °C, 2152.0 s | 2.5 and 3.0 m, 79.11 °C, 2153.3 s | 2.5 and 3.0 m, 79.11 °C, 2153.3 s | 2.5 and 3.0 m, 79.11 °C, 2153.3 s |
| 63 °C, $z_{bl}$ 3 m | 3.0 m, 83.93 °C, 2153.3 s | 3.0 m, 83.93 °C, 2153.3 s | 3.0 m, 83.93 °C, 2153.3 s | 3.0 and 6.0 m, 83.91 °C, 2157.3 s |
| 65 °C, $z_{bl}$ 4 m | 3.5 m, 83.43 °C, 2154.0 s | 4.0 m, 80.93 °C, 2154.7 s | 4.0 m, 80.93 °C, 2154.7 s | 4.0 and 6.0 m, 80.92 °C, 2157.3 s |
| 63 °C, $z_{bl}$ 2 m (hot case) | 2.0 m, 84.53 °C, 3118.4 s | 2.0 m, 84.53 °C, 3118.4 s | 2.0 m, 84.53 °C, 3118.4 s | 2.0 m, 84.53 °C, 3118.4 s |

In the first five cases no weight leads to any throttling (0.0 s) and min $Q/Q_{nom}$ stays between 0.8121 and 0.8371. The weight changes the chosen altitude and the peak motor temperature (83.93 vs 79.11 °C at 58 °C, $z_{bl}$ 4 m), but every mission time stays between 2152.0 and 2157.3 s. In the hot case throttling cannot be avoided at any altitude (3005.9 s throttled, 2896.8 s of it by the battery rule, min $Q/Q_{nom}$ 0.8001) and the weight has no effect: every $w$ keeps 2.0 m. The per-strip MPC is myopic here — one 2 → 6 m climb pays back over the whole mission (`alt_opt` is 9.4 s faster) but not within one strip. The time conclusions below do not depend on $w$.

**Constant altitude vs MPC** (63 °C): with $z_{bl}$ = 3 m a constant 3.0 m takes 2153.3 s, the same as the MPC (which flies 3.0 m throughout); with $z_{bl}$ = 4 m a constant 4.0 m takes 2154.7 s, the same as the MPC (3.5 and 4.0 m); a constant 6.0 m takes 2157.3 s in both.

### Sweep: mission time (min) against surface temperature and hot-layer depth

| $z_{bl}$ | Strategy | 55 | 56 | 57 | 58 | 59 | 60 | 61 | 62 | 63 | 64 | 65 °C |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2 m | fixed 2 m | 35.87 | 35.87 | 35.87 | 35.87 | 35.87 | 37.37 | 39.09 | 43.61 | 51.97 | 69.31 | 160.95 |
| 2 m | fixed 1 m | 37.39 | 39.11 | 43.58 | 51.84 | 68.95 | 159.28 | 798.58 | 915.34 | 1059.22 | 1246.48 | 1506.72 |
| 2 m | alt-opt | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 37.42 | 39.14 | 43.56 | 51.82 | 68.79 | 152.65 |
| 2 m | MPC | 35.87 | 35.87 | 35.87 | 35.87 | 35.87 | 37.37 | 39.09 | 43.61 | 51.97 | 69.31 | 152.40 |
| 3 m | fixed 2 m | 35.87 | 35.87 | 35.87 | 35.87 | 35.87 | 37.37 | 39.09 | 43.61 | 51.97 | 69.31 | 160.95 |
| 3 m | alt-opt | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 37.43 |
| 3 m | MPC | 35.88 | 35.88 | 35.89 | 35.89 | 35.89 | 35.89 | 35.89 | 35.89 | 35.89 | 35.89 | 37.39 |
| 4 m | fixed 2 m | 35.87 | 35.87 | 35.87 | 35.87 | 35.87 | 37.37 | 39.09 | 43.61 | 51.97 | 69.31 | 160.95 |
| 4 m | alt-opt | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 | 35.96 |
| 4 m | MPC | 35.88 | 35.88 | 35.89 | 35.89 | 35.89 | 35.90 | 35.90 | 35.91 | 35.91 | 35.91 | 35.91 |

(Fixed 1 m and fixed 2 m are identical for every $z_{bl}$ because both altitudes are inside the layer in all three, where the ambient does not depend on $z_{bl}$; the script asserts this and computes them once.) Gain of the best 3D plan (alt-opt or MPC) over the 2D baseline, printed as baseline time minus 3D time: for $z_{bl}$ = 2 m 0.0 s up to 61 °C, then 3.2, 9.4, 31.6 and 513.1 s at 62–65 °C (the battery-rule speed is very sensitive near the floor, so the 0.026 °C cooler air at 6 m is worth 513.1 s at 65 °C); for $z_{bl}$ = 3 m −0.7 to −1.3 s (the 3D plans are slower) up to 59 °C and +88.6 → +7413.6 s (faster) from 60 to 65 °C; for $z_{bl}$ = 4 m +88.0 → +7502.6 s from 60 to 65 °C. No run in the sweep spent any time above $T_{max}$ (longest 0.0 s).

**Battery** (rule enforced in every run): min $Q/Q_{nom}$ of fixed 2 m, alt-opt and MPC never falls below 0.800. At 2 m the rule throttles from 62 °C on (2363.6 s at 62 °C, 9485.1 s at 65 °C), for every hot-layer depth. When $z_{bl}$ = 2 m the 3D plans need it just as much (MPC 2363.6 → 8966.7 s). With $z_{bl}$ = 3 or 4 m the alt-opt and MPC plans never trigger it (0.0 s at every $T_{surf}$) and keep min $Q/Q_{nom}$ ≥ 0.804 ($z_{bl}$ = 3 m) and ≥ 0.827 ($z_{bl}$ = 4 m) for both.

**Key findings**

- **With the card's atmosphere, altitude planning above the 2D height buys almost nothing.** The 2D base already flies at the top of the hot layer; climbing to $z^*$ = 6 m lowers the steady-state motor temperature by 0.03 °C and costs 5.3 s of climbing (card case). With the default weight the MPC stays at 2 m on every strip in the card and hot cases (it climbs to 6 m only with $w$ = 1000 s/°C in the card case) and ties the 2D baseline exactly at every surface temperature up to 64 °C when $z_{bl}$ = 2 m. Only at 65 °C, where the battery-rule speed is close to its floor and very sensitive to the ambient, does the 0.026 °C cooler air above 2 m pay off (MPC 152.40 vs 160.95 min). The card's expectation that the altitude-optimised and MPC strategies keep "larger margins" than the baseline does not hold.
- **The altitude that matters is the one inside the hot layer.** A 1 m inspection pass runs 4.01 °C hotter, has to throttle for 2109.1 s and takes 91.1 s longer at 55 °C. At 63 °C its battery cannot stay below 65 °C even at the slowest speed, so it needs 765 cooling pauses and 1059.22 min instead of the baseline's 51.97 min; from 61 °C on it takes 798.58–1506.72 min. The motor safe corridor starts at 1.20 m.
- **If the hot layer is deeper than the 2D height, altitude planning pays.** With $z_{bl}$ = 3 or 4 m the baseline at 2 m sits inside the layer and needs the battery rule from 62 °C on: at $T_{surf}$ = 65 °C the 3D plans are 7413.6 s ($z_{bl}$ = 3 m) and 7502.6 s ($z_{bl}$ = 4 m) faster, never trigger the battery rule and keep min $Q/Q_{nom}$ ≥ 0.804 (MPC and alt-opt, $z_{bl}$ = 3 m) and ≥ 0.827 ($z_{bl}$ = 4 m). Below $T_{surf}$ = 60 °C nothing needs throttling and the 3D plans are 0.7–1.3 s slower because they climb. The gain comes from **choosing a different constant altitude** above the layer, not from varying altitude during the mission: a constant 3.0 m (or 4.0 m) matches the MPC's mission time exactly, and the altitude-optimised plan is a single climb to 6 m.
- **The battery limit, not the motor limit, decides the hot cases.** The motor law keeps every run below $T_{max}$; without the battery rule every strategy's battery would reach 65.99–69.00 °C at 63 °C (capacity 0.7800–0.7951). Enforcing the 80 % floor as a speed cap for all strategies keeps it (min $Q/Q_{nom}$ 0.8000–0.8001) and costs 496.2–508.0 s at 2 m or above; no altitude in [1, 6] m avoids that cost when the layer is 2 m thick.
- **Only the low pass pauses.** In the card and hot cases the throttle laws keep every strategy at 2 m or above away from both limits, so they never pause; the 1 m pass pauses 765 times in the hot case because even its slowest flight heats the battery past 65 °C.

### Figures

**3D paths** (hot case), dots coloured by motor temperature; orange plane = top of the hot layer:

![Trajectories 3D](./trajectories_3d.png)

**Time series**: altitude, motor temperature (dashed $T_{max}$, dotted $T_{warn}$) and battery temperature (dashed 65 °C) for the card case (left) and the hot case (right; the time axis ends with the three strategies at 2 m or above, the 1 m pass continues off-scale to 1059 min):

![Time series](./time_series.png)

**Safe corridor**: steady-state motor (solid) and battery (dashed) temperatures and the ambient (dotted) against altitude for three hot-layer depths and two surface temperatures; grey band = the 2D height:

![Safe corridor](./safe_corridor.png)

**Sweep**: mission time (log scale) against surface temperature for three hot-layer depths:

![Sweep](./sweep.png)

**Metrics**: mission time, time throttled, peak motor temperature and minimum capacity ratio (dark = card case, light = hot case; the 1 m hot-case bars are clipped and labelled):

![Metrics](./metrics.png)

**Animation** (hot case, 105 frames, one per 30 s of mission, ending at 51.97 min while the 1 m pass is still flying): the four drones over the field with their current motor temperatures:

![Animation](./animation.gif)

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| $\Gamma_{atm}$ = 0.0065 °C/m called the dry adiabatic lapse rate | value kept; called the standard lapse rate; card corrected | the dry adiabatic rate is $g/c_p$ = 0.0098 °C/m |
| $t_{climb} = |\Delta z|/v_{z,max}$ with the $f_{trap}$ profile | $T = 2|\Delta z|/v_{z,max}$; card corrected | the profile's peak speed is $2|\Delta z|/T$; the card's duration would need 3.0 m/s |
| $T_{bat,ss} = T_{amb} + R_{bat}P_{total}/\eta_{esc}$ with $P_{total}$ already divided by $\eta_{esc}$ | $T_{amb} + R_{bat}P_{motor}/\eta_{esc}$; card corrected | $\eta_{esc}$ counted twice |
| Baseline "reactive throttle only"; strategies 2 and 3 with their own pause rules | every strategy uses the 2D base's throttle-to-$T_{max}$ law and pause at $T_{max}$, plus the battery rule below | same speed envelope for all, so the comparison isolates the altitude plan; the 2D base found this law the fastest safe one |
| MPC "minimising the weighted sum of mission time and peak motor temperature" at every step | per-strip receding horizon over 11 candidate altitudes, cost = strip time + $w$ (peak − $T_{warn}$)⁺, weight swept 0–1000 s/°C in six cases | a full per-step MPC is unnecessary here: the ambient depends on altitude only; the weight changes the chosen altitude and the peak temperature (83.93 vs 79.11 °C in one case) but each case's mission times stay within 5.3 s of each other (2152.0–2157.3 s; 3118.4 s for every $w$ in the hot case). A candidate is abandoned once its strip time exceeds the best cost so far (it cannot win), which does not change the choice |
| Expected: baseline approaches $T_{warn}$, battery near/above 65 °C only for the baseline, ≈ 5 °C gain from altitude | 80.93 °C, battery 59.59 °C for every plan at 2 m or above, 0.03 °C gain above 2 m | reported as simulated; the 5 °C exists only between 1 and 6 m |
| Single card case | plus a hot case, a surface-temperature sweep and hot-layer depths 3 and 4 m | the card case alone cannot show when altitude matters |
| — | `fixed_1m` added | shows the cost of flying inside the hot layer |
| Battery derating as a hard constraint | enforced as a speed cap for every strategy (2D baseline included): above $T_{bat,warn}$ = 63 °C throttle to the speed whose battery steady state is 65 °C, pause at 65 °C; the hot case is also reported without it | the card's floor is a constraint, and slowing down satisfies it at every altitude: at 63 °C the baseline keeps $Q/Q_{nom}$ ≥ 0.8001 at a cost of +508.0 s; the 1 m pass needs 765 pauses |
| Pause model | cooling toward $T_{amb}$ at the current altitude (2D base), motor and battery | kept from the 2D base; in the card and hot cases only the 1 m pass pauses |

---

## Extensions

1. **Wind and convective cooling with altitude** ($\tau_{cool}(z)$), which would give the upper altitudes a real benefit above the layer.
2. **Diurnal layer depth**: $z_{bl}$ and $T_{surf}$ through the day, with the altitude plan re-optimised per strip.
3. **Inspection-quality constraint** (e.g. GSD), which would penalise high altitudes and make the trade-off two-sided.
4. **Battery-aware MPC** that includes $T_{bat}$ in the cost.

---

## Related Scenarios

- 2D card: [S076 Extreme Heat Cooling Strategy](../../../../scenarios/04_industrial_agriculture/S076_thermal_management.md)
- 3D card: [S076 3D Upgrade — Thermal Management in Hot Environment](../../../../scenarios/04_industrial_agriculture/3d/S076_3d_thermal_management.md)
- Lawnmower geometry: [S048 Full-Area Coverage Scan (Lawnmower)](../../../../scenarios/03_environmental_sar/S048_lawnmower.md)
- 3D atmospheric environment: [S058 Typhoon Eye Probing](../../../../scenarios/03_environmental_sar/S058_typhoon.md)
