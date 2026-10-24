# S063 3D Upgrade — Precision Per-Plant Spraying

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: The S063 mission is kept:
- 20 plants in a 20 × 20 m field, visited in nearest-neighbour tour order (74.14 m) from the depot;
- reference legs at 1 m/s, a settle test, then a 3 s spray window;
- a PID with GPS noise and a 3 m/s² acceleration limit;
- sinusoidal gust plus turbulence, with the integrator reset per spray window.

It is flown in 3D over plants with canopy heights $h_i \sim \mathcal{TN}(1.0, 0.3^2; 0.3, 2.5)$ m and crowns of radius 0.5 m. A vertical PID loop (1.5 × the horizontal gains) holds altitude against a rotor-downwash disturbance of $-0.15 + \mathcal N(0, 0.05^2)$ m/s². A droplet drift model couples the nozzle standoff to where the spray lands.

**Altitude strategies** (same plants, heights, wind, GPS and altimeter noise per seed, CRN):

| Strategy | Description |
|----------|-------------|
| **Fixed 1.5 m** (card, 2D baseline) | The S063 behaviour: one altitude for every plant |
| **Fixed, clears all plants** (added 2D baseline) | One altitude $\max_i h_i + 0.5$ m that never touches a crown |
| **Per-plant adaptive** (card) | $z_{cmd,i} = h_i + 0.5$ m from a prior height map. On each leg the reference is the higher of the two plant altitudes (climb first), then it descends over the plant |
| **Lidar terrain following** (card) | Continuous: $z_{ref}$ = canopy surface under the drone + 0.5 m, measured by a lidar with 0.05 m noise |

Two horizontal gain sets: the card's **Tuned** (2.0, 0.2, 0.5) and S063's best, **Tight** (5.0, 0.5, 1.0).

**Card questions**:
- Nozzle standoff, 3D hover RMS, droplet offset and $C_{eff}^{3D}$ per strategy.
- The trade-off between canopy-height spread $\sigma_h$ and $C_{eff}^{3D}$.

**Scoring rule (added)**: a mission in which the drone enters any crown (canopy contact, see below) is a *collision*. Every comparison shows two scores:
- $C_{eff}^{3D}$, the dose quality from the card;
- the number of collision-free missions, and a *safe* $C_{eff}^{3D}$ in which collision missions score 0.

The main rule treats the drone as a **point with zero clearance**: a contact is the drone's centre inside a crown cylinder below its top. A clearance sweep widens the cylinder by a drone radius $r_d$ and raises its top by a vertical margin $m_z$, with $(r_d, m_z)$ = (0, 0), (0.1, 0.05), (0.2, 0.1) m.

**Statistics (added)**: every strategy comparison is paired over the same seeds. For each A − B the script prints:
- the mean difference;
- a 95 % bootstrap interval (10 000 resamples, fixed rng seed 6301), which is *descriptive only*;
- the per-seed higher / equal / lower counts;
- the p-value of an exact two-sided paired sign-flip permutation test. All $2^k$ sign flips of the $k$ non-zero differences are enumerated, and seeds with equal scores contribute nothing.

A difference is called *resolved* only when this exact test gives p < 0.05; everything else is *not resolved*, whatever its interval or counts. Collision splits (seeds where only A or only B collides) use an exact McNemar test, and win / loss counts an exact sign test. With 10 seeds and no ties the smallest possible p is 0.002. **Multiplicity**: the script prints roughly 100 comparisons, each at the 5 % level, so a few are expected to come out nominally resolved by chance. An isolated resolved comparison with p near 0.05 should be treated with caution.

---

## Mathematical Model

**Vertical loop**: $a_z = K_p^{(z)}e_z + K_i^{(z)}\!\int e_z + K_d^{(z)}(0 - \dot z)$, with $K^{(z)} = 1.5\,K^{(xy)}$ and Euler integration at 50 Hz. The altitude bounds $z \in [0.3, 6]$ m are enforced by the controller, not by clamping the state. A braking barrier raises the vertical command whenever the descent speed exceeds what full braking can stop before $z_{min}$ + 5 mm. The plan uses a braking rate of 2.60 m/s² = $A_{max}$ − worst-case downwash (0.15 + 4 × 0.05) − 0.05 margin, with the exact discrete-time stopping distance; the same applies towards the 6 m ceiling. The barrier uses the true altitude and vertical speed, an ideal state estimate, while the PID uses the noisy altimeter or lidar. A barrier fed by the noisy measurement would presumably need an extra altitude margin; that was *not probed*.

**Asserts**:
- At every step, $z \ge$ 0.3 m and $z$ > 0.
- From the logged trajectory, the thrust part of the vertical acceleration never exceeds $A_{max}$.
- **Broken-loop check**: the barrier may override the vertical command in at most 50 % of a mission's steps. The barrier only acts during transients (braking a descent, unwinding the integrator), so a loop that needs it for most of the mission is not flying the aircraft.

**Droplet drift** (card; fall time corrected):
$$t_{fall} = \frac{\sqrt{w_{dw}^2 + 2g\,\delta z} - w_{dw}}{g},\qquad \mathbf p_{impact} = \mathbf q_{xy} + \mathbf u_w\,t_{fall},\qquad \mathbf u_w = \frac{A_w\sin(2\pi t/T_w + \varphi)}{c_w}$$

**Dose per spray sample** (card), averaged over the 3 s window:
$$D = \exp\Bigl(-\frac{d^2}{2\sigma_{spray}^2}\Bigr)\exp\Bigl(-\frac{(\delta z - 0.5)^2}{2\sigma_z^2}\Bigr),\qquad D = 0\ \text{if}\ \delta z \le 0\ (\text{nozzle inside the crown})$$
$C_{eff}^{3D}$ is the mean over the 20 plants.

**Canopy contact**: the drone is inside a crown cylinder (horizontal distance < 0.5 m) below its top. Each contact is attributed either to the plant being approached or sprayed, or to another plant.

**Hover RMS**: $\varepsilon^{3D}_i$ is measured to $(x_i, y_i, h_i + 0.5)$; the horizontal part is also reported.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Field / plants / layout | 20 × 20 m / 20 / S063 `default_rng(0)`, NN tour 74.14 m |
| Canopy heights | TN(1.0, 0.3²; 0.3, 2.5) m, per seed from `default_rng(3000 + seed)` (inverse CDF) |
| Crown radius (added) | 0.5 m; sweep 0.25, 0.5, 1.0 m (10 seeds) |
| Vertical-damping probe (added) | adaptive with $K_d^{(z)}$ × 2.5 |
| Lidar / altimeter noise | 0.05 m |
| Nominal standoff / $\sigma_z$ / $\sigma_{spray}$ | 0.5 m / 0.2 m / 0.5 m |
| Altitude bounds / fixed altitude | [0.3, 6.0] m / 1.5 m |
| $\alpha_z$ | 1.5 (vertical gains 1.5 × horizontal) |
| Downwash disturbance | −0.15 m/s² mean, 0.05 m/s² std |
| Gust / turbulence | $A_w$ = 0.3 m/s², $T_w$ = 4 s / 0.1 m/s² (S063) |
| Horizontal PID | Tuned (2.0, 0.2, 0.5) (card); Tight (5.0, 0.5, 1.0) (S063 best, added) |
| $\Delta t$ / spray time / transit speed | 0.02 s / 3 s / 1 m/s |
| Altitude-bound barrier (added) | braking rate 2.60 m/s² (= 3.0 − 0.35 worst downwash − 0.05), stops 5 mm inside the bound |
| Collision rule (added) | any crown contact → collision mission, safe $C_{eff}^{3D}$ = 0; point drone, zero clearance |
| Clearance sweep (added) | $(r_d, m_z)$ = (0, 0), (0.1, 0.05), (0.2, 0.1) m |
| Broken-loop check (added) | barrier active in ≤ 50 % of a mission's steps (assert) |
| Settle test | S063 (0.3 m, 0.1 m/s, 0.2 s, 10 s time-out) plus altitude error < 0.15 m (added) |
| Wind drag rate $c_w$ (added) | 0.3 s⁻¹ (wind amplitude 1 m/s); sweep 0.15, 0.3, 0.6 |
| Downwash droplet speed $w_{dw}$ (added) | 2 m/s; sweep 0, 2, 5 m/s |
| Seeds | 20 per strategy and gain set (seeds 0–19) at $\sigma_h$ = 0.3 m; $\sigma_h$ sweep 0–0.6 m with 10 seeds (0–9, Tuned); low-spread check at $\sigma_h$ = 0.15 m with 20 seeds (0–19) for both gain sets; crown-radius sweep 10 seeds |

---

## Implementation

```
src/04_industrial_agriculture/3d/s063_3d_precision_spraying.py
```

```bash
conda activate drones
python src/04_industrial_agriculture/3d/s063_3d_precision_spraying.py
```

The script imports the S063 module for the layout, tour, gains and noise streams. Runtime ≈ 4 min, including the crown-radius sweep (all four strategies), the low-spread check and the damping probe (218 s and 206 s with `OMP/OPENBLAS/MKL_NUM_THREADS=1`, 220 s with default threading). The three runs printed byte-identical output and wrote byte-identical PNG and GIF files.

---

## Results

**Checks**:
- Heights over 20 seeds × 20 plants: mean 1.011 m, std 0.298 m, range 0.313–1.935 m.
- 23 plants (5.8 %) are taller than the fixed 1.5 m altitude, and 46 more are within 0.2 m below it.
- Fall time at a 0.5 m standoff: 0.319 / 0.175 / 0.092 s for $w_{dw}$ = 0 / 2 / 5 m/s; at 1.5 m: 0.553 / 0.386 / 0.242 s.
- The highest descent speed from which full braking ($A_{max}$ − mean downwash) can stop between 0.3 m and the ground is 1.31 m/s. The previous version (commit 1c61058; numbers from its output, not printed by this script) clamped the altitude and zeroed the vertical speed in one step at up to 2.78 m/s. That was a physically impossible stop, and in reality a ground strike. With the barrier, the lowest altitude in any job group is 0.3050 m and no step needs more than $A_{max}$.
- Vertical loop poles:
  - Tuned × 1.5: −0.324 ± 1.682j and −0.102 (damping of the complex pair 0.189);
  - Tight × 1.5: −0.699 ± 2.621j and −0.102 (damping 0.258).

### $\sigma_h$ = 0.3 m, 20 seeds

**Tuned PID (card)**

| Strategy | $C_{eff}^{3D}$ | Collision-free missions | Safe $C_{eff}^{3D}$ | Hover RMS 3D / horiz. | Standoff deviation | Drift offset $d$ | Crown contacts per mission (target / other) | Plants sprayed from inside the crown | Time | Saturated | Floor-barrier steps per mission | z range |
|---|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Fixed 1.5 m | 0.428 ± 0.069 | 2/20 | 0.051 | 49.5 / 40.3 cm | 24.1 cm | 34.8 cm | 1.60 (1.60 / 0.00) | 1.50 | 333.3 s | 0.00 % | 0.0 | 1.404–1.53 m |
| Fixed, clears all | 0.170 ± 0.082 | **20/20** | 0.170 | 70.6 / 40.3 cm | 54.2 cm | 35.6 cm | 0.00 | 0.00 | 333.3 s | 0.00 % | 0.0 | 1.804–2.47 m |
| Per-plant adaptive | **0.760 ± 0.011** | 5/20 | 0.190 | 40.6 / 40.3 cm | **4.5 cm** | 33.9 cm | 1.25 (0.65 / 0.60) | 0.00 | 333.5 s | 0.01 % | 1.5 | 0.305–2.91 m |
| Lidar terrain following | 0.445 ± 0.066 | **0/20** | 0.000 | 52.0 / 40.5 cm | 25.2 cm | 35.3 cm | 19.55 (19.40 / 0.15) | 0.90 | 334.3 s | 9.35 % | 379.6 | 0.305–3.45 m |

**Tight PID**

| Strategy | $C_{eff}^{3D}$ | Collision-free missions | Safe $C_{eff}^{3D}$ | Hover RMS 3D / horiz. | Standoff deviation | Drift offset $d$ | Crown contacts per mission (target / other) | Plants sprayed from inside the crown | Time | Saturated | Floor-barrier steps per mission | z range |
|---|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Fixed 1.5 m | 0.483 ± 0.072 | 3/20 | 0.081 | 27.3 / 10.3 cm | 23.8 cm | 23.8 cm | 1.50 (1.50 / 0.00) | 1.30 | 265.3 s | 0.27 % | 0.0 | 1.438–1.54 m |
| Fixed, clears all | 0.165 ± 0.080 | **20/20** | 0.165 | 58.5 / 10.3 cm | 56.7 cm | 36.7 cm | 0.00 | 0.00 | 265.3 s | 0.27 % | 0.0 | 1.839–2.46 m |
| Per-plant adaptive | **0.871 ± 0.004** | 7/20 | 0.305 | 10.6 / 10.3 cm | **1.8 cm** | 24.7 cm | 0.75 (0.30 / 0.45) | 0.00 | 275.0 s | 0.98 % | 0.1 | 0.350–2.79 m |
| Lidar terrain following | 0.838 ± 0.016 | **0/20** | 0.000 | 12.3 / 10.5 cm | 3.7 cm | 25.5 cm | 17.45 (16.65 / 0.80) | 0.00 | 289.3 s | 9.73 % | 423.1 | 0.305–3.61 m |

Mean $C_{eff}^{3D}$ of the collision-free missions only: fixed 1.5 m 0.5110 / 0.5405, clears-all 0.1705 / 0.1652, adaptive 0.7585 / 0.8703 (Tuned / Tight).

**Clearance sweep** (main runs, $\sigma_h$ = 0.3 m, 20 seeds; collision-free missions, safe $C_{eff}^{3D}$ for drone radius / vertical margin 0 / 0, 0.1 / 0.05, 0.2 / 0.1 m):

| Strategy | Tuned | Tight |
|---|---|---|
| Fixed 1.5 m | 2/20, 0.051 · 0/20, 0.000 · 0/20, 0.000 | 3/20, 0.081 · 2/20, 0.056 · 0/20, 0.000 |
| Fixed, clears all | 20/20, 0.170 · 20/20, 0.170 · 20/20, 0.170 | 20/20, 0.165 · 20/20, 0.165 · 20/20, 0.165 |
| Per-plant adaptive | 5/20, 0.190 · 0/20, 0.000 · 0/20, 0.000 | 7/20, 0.305 · 7/20, 0.305 · 2/20, 0.087 |
| Lidar terrain following | 0/20, 0.000 in all three | 0/20, 0.000 in all three |

Paired safe $C_{eff}^{3D}$ per clearance (0 / 0, 0.1 / 0.05, 0.2 / 0.1 m; mean, descriptive bootstrap interval, higher / equal / lower, exact p; **bold** = resolved):
- Adaptive − clears-all:
  - Tuned: +0.019 [−0.120, +0.180] (5/0/15, p 0.81), **−0.170 [−0.208, −0.135]** (0/0/20, p < 0.0001), **−0.170 [−0.208, −0.135]** (0/0/20, p < 0.0001).
  - Tight: +0.139 [−0.038, +0.330] (7/0/13, p 0.17), +0.139 [−0.038, +0.330] (7/0/13, p 0.17), −0.078 [−0.174, +0.057] (2/0/18, p 0.23). None resolved.
- Adaptive − fixed 1.5 m:
  - Tuned: +0.139 [−0.026, +0.313] (5/13/2, p 0.14), 0 (0/20/0, p 1), 0 (0/20/0, p 1). None resolved.
  - Tight: +0.224 [+0.023, +0.429] (7/11/2, p 0.051, not resolved), **+0.248 [+0.040, +0.453]** (7/11/2, p 0.043), +0.087 [+0.000, +0.218] (2/18/0, p 0.50). The one resolved cell has p just below 0.05, and its neighbour with the same counts is not resolved, so it is weak evidence (see the multiplicity note).

Plants receiving a dose > 0.9: at most 4.0 % (adaptive, Tight). Settle time-outs per mission: Tuned 19.9–20.0 of 20 for every strategy; Tight 4.3 / 4.3 / 5.2 / 7.2. Vertical travel per mission: Tuned 6.5 / 6.5 / 24.9 / 151.2 m; Tight 6.9 / 6.9 / 20.8 / 73.2 m.

**Floor barrier over every job group** (main runs, $\sigma_h$ sweep, the Tight runs at $\sigma_h$ 0.15, crown-radius sweep and damping probe). 15 of 46 groups ever use the barrier. All are terrain following, plus adaptive at Tuned $\sigma_h$ ≥ 0.3 and Tight $\sigma_h$ = 0.3. No step needed more than $A_{max}$, and the lowest altitude is 0.3050 m:

| Job group | Barrier steps per mission | Missions using it | Lowest altitude | Longest stretch | Largest share of steps |
|---|---:|:---:|---:|---:|---:|
| TF, Tuned, $\sigma_h$ 0.0 / 0.15 (20 seeds) / 0.3 / 0.45 / 0.6 | 339.4 / 353.4 / 379.6 / 438.9 / 480.9 | all | 0.3050 m | 0.80 / 0.88 / 0.96 / 1.30 / 2.14 s | 2.29 / 2.72 / 2.89 / 3.21 / 3.59 % |
| TF, Tuned, 0.3, $R_{crown}$ 0.25 / 1.0 m | 233.7 / 257.6 | all | 0.3050 m | 1.36 / 0.98 s | 2.46 / 1.96 % |
| TF, Tight, 0.3, $R_{crown}$ 0.25 / 0.5 / 1.0 m | 494.9 / 423.1 / 356.3 | all | 0.3050 m | 0.94 / 1.20 / 1.02 s | 3.64 / 3.65 / 2.91 % |
| TF, Tight, $\sigma_h$ 0.15 (20 seeds) | 426.8 | all | 0.3050 m | 0.90 s | 3.25 % |
| Adaptive, Tuned, $\sigma_h$ 0.3 / 0.45 / 0.6 | 1.5 / 17.0 / 28.5 | 1/20, 7/10, 9/10 | 0.3051 / 0.3051 / 0.3050 m | 0.32 / 0.78 / 0.92 s | 0.18 / 0.23 / 0.28 % |
| Adaptive, Tight, 0.3 | 0.1 | 1/20 | 0.3501 m | 0.04 s | 0.02 % |

The largest share of steps on the barrier in any mission is 3.65 %, against the 50 % assert. A sign-flipped vertical loop puts the barrier on 99.5 % of the steps and trips the assert; this was checked in a separate test, not printed. For scale, braking the fastest descent possible inside the 0.3–6 m envelope (6.18 m/s) at 2.60 m/s² takes 2.38 s. The longest continuous stretch, 2.14 s (TF, Tuned, $\sigma_h$ 0.6), is close to that, which suggests the barrier also holds the drone while a wound-up integrator unwinds (*not probed*). A stretch-length assert would therefore not separate a working loop from a broken one; the share of steps does.

**Paired comparisons** (A − B over the same 20 seeds: mean difference [descriptive bootstrap interval], per-seed higher / equal / lower, exact sign-flip p; **bold** = resolved, p < 0.05):

| Gains | A − B | $C_{eff}^{3D}$ | Safe $C_{eff}^{3D}$ |
|---|---|---|---|
| Tuned | Adaptive − fixed 1.5 m | **+0.332 [+0.301, +0.364]**, 20 / 0 / 0, p < 0.0001 | +0.139 [−0.026, +0.313], 5 / 13 / 2, p 0.14 |
| Tuned | Adaptive − clears-all | **+0.590 [+0.555, +0.622]**, 20 / 0 / 0, p < 0.0001 | +0.019 [−0.120, +0.180], 5 / 0 / 15, p 0.81 |
| Tuned | Adaptive − terrain following | **+0.315 [+0.285, +0.344]**, 20 / 0 / 0, p < 0.0001 | +0.190 [+0.075, +0.342], 5 / 15 / 0, p 0.063 |
| Tuned | Terrain following − fixed 1.5 m | +0.018 [−0.021, +0.057], 10 / 0 / 10, p 0.38 | −0.051 [−0.131, +0.000], 0 / 18 / 2, p 0.50 |
| Tuned | Clears-all − fixed 1.5 m | **−0.257 [−0.311, −0.201]**, 0 / 0 / 20, p < 0.0001 | **+0.119 [+0.049, +0.178]**, 18 / 0 / 2, p 0.0029 |
| Tuned | Clears-all − terrain following | **−0.275 [−0.330, −0.216]**, 0 / 0 / 20, p < 0.0001 | **+0.170 [+0.135, +0.208]**, 20 / 0 / 0, p < 0.0001 |
| Tight | Adaptive − fixed 1.5 m | **+0.388 [+0.355, +0.420]**, 20 / 0 / 0, p < 0.0001 | +0.224 [+0.023, +0.429], 7 / 11 / 2, p 0.051 |
| Tight | Adaptive − clears-all | **+0.706 [+0.670, +0.740]**, 20 / 0 / 0, p < 0.0001 | +0.139 [−0.038, +0.330], 7 / 0 / 13, p 0.17 |
| Tight | Adaptive − terrain following | **+0.033 [+0.025, +0.040]**, 19 / 0 / 1, p < 0.0001 | **+0.305 [+0.131, +0.479]**, 7 / 13 / 0, p 0.016 |
| Tight | Terrain following − fixed 1.5 m | **+0.356 [+0.322, +0.388]**, 20 / 0 / 0, p < 0.0001 | −0.081 [−0.172, +0.000], 0 / 17 / 3, p 0.25 |
| Tight | Clears-all − fixed 1.5 m | **−0.318 [−0.373, −0.261]**, 0 / 0 / 20, p < 0.0001 | **+0.084 [+0.006, +0.150]**, 17 / 0 / 3, p 0.039 |
| Tight | Clears-all − terrain following | **−0.673 [−0.709, −0.635]**, 0 / 0 / 20, p < 0.0001 | **+0.165 [+0.131, +0.202]**, 20 / 0 / 0, p < 0.0001 |

The script prints p 0.0000 for p < 0.0001. Two safe-score cells whose bootstrap interval excludes 0 are *not* resolved by the exact test: Tuned adaptive − terrain following (p 0.063; only 5 non-zero differences, so the smallest possible p is 0.0625) and Tight adaptive − fixed 1.5 m (p 0.051).

Adaptive with Tight − Tuned gains ($C_{eff}^{3D}$, same seeds): **+0.111 [+0.106, +0.116]**, 20 / 0 / 0, p < 0.0001.

**Crown-radius sweep** ($\sigma_h$ = 0.3 m, 10 seeds; $C_{eff}^{3D}$ / collision-free missions / safe $C_{eff}^{3D}$ / crown contacts per mission). Clears-all flies above every crown, so the radius does not affect it:

| $R_{crown}$ | Gains | Fixed 1.5 m | Fixed clears-all | Adaptive | Terrain following | Safe: adaptive − clears-all | Safe: adaptive − TF | Dose only: TF − fixed 1.5 m |
|---|---|---|---|---|---|---|---|---|
| 0.25 m | Tuned | 0.407 / 0/10 / 0.000 / 1.70 | 0.152 / 10/10 / 0.152 / 0.00 | 0.760 / 4/10 / 0.302 / 0.70 | 0.020 / 0/10 / 0.000 / 18.80 | +0.150 [−0.096, +0.404], 4/0/6, p 0.29 | +0.302 [+0.075, +0.529], 4/6/0, p 0.13 | **−0.386 [−0.414, −0.356]**, 0/0/10, p 0.002 |
| 0.25 m | Tight | 0.458 / 0/10 / 0.000 / 1.70 | 0.144 / 10/10 / 0.144 / 0.00 | 0.871 / 4/10 / 0.348 / 0.80 | 0.850 / 0/10 / 0.000 / 19.50 | +0.203 [−0.076, +0.490], 4/0/6, p 0.19 | +0.348 [+0.087, +0.609], 4/6/0, p 0.13 | **+0.391 [+0.365, +0.420]**, 10/0/0, p 0.002 |
| 0.50 m (subset) | Tuned | 0.407 / 0/10 / 0.000 / 1.80 | 0.152 / 10/10 / 0.152 / 0.00 | 0.760 / 4/10 / 0.302 / 1.20 | 0.459 / 0/10 / 0.000 / 19.40 | +0.150 [−0.096, +0.404], 4/0/6, p 0.29 | +0.302 [+0.075, +0.529], 4/6/0, p 0.13 | +0.053 [+0.002, +0.103], 7/0/3, p 0.092 |
| 0.50 m (subset) | Tight | 0.458 / 0/10 / 0.000 / 1.70 | 0.144 / 10/10 / 0.144 / 0.00 | 0.871 / 4/10 / 0.348 / 0.80 | 0.836 / 0/10 / 0.000 / 17.40 | +0.203 [−0.076, +0.490], 4/0/6, p 0.19 | +0.348 [+0.087, +0.609], 4/6/0, p 0.13 | **+0.378 [+0.353, +0.404]**, 10/0/0, p 0.002 |
| 1.00 m | Tuned | 0.407 / 0/10 / 0.000 / 1.80 | 0.152 / 10/10 / 0.152 / 0.00 | 0.760 / 1/10 / 0.075 / 2.20 | 0.708 / 0/10 / 0.000 / 16.70 | −0.077 [−0.182, +0.092], 1/0/9, p 0.40 | +0.075 [+0.000, +0.225], 1/9/0, p 1 | **+0.302 [+0.274, +0.330]**, 10/0/0, p 0.002 |
| 1.00 m | Tight | 0.458 / 0/10 / 0.000 / 1.70 | 0.144 / 10/10 / 0.144 / 0.00 | 0.871 / 3/10 / 0.260 / 1.20 | 0.823 / 0/10 / 0.000 / 13.20 | +0.116 [−0.131, +0.388], 3/0/7, p 0.38 | +0.260 [+0.000, +0.521], 3/7/0, p 0.25 | **+0.364 [+0.339, +0.390]**, 10/0/0, p 0.002 |

The cells give $C_{eff}^{3D}$ / collision-free missions / safe $C_{eff}^{3D}$ / crown contacts per mission; the paired columns give the difference [descriptive bootstrap interval], higher / equal / lower, exact p (**bold** = resolved). The 0.50 m rows are seeds 0–9 of the 20-seed main runs; for 0.5 m crowns cite the 20-seed paired table above. No safe-score difference in this sweep is resolved, neither adaptive − clears-all (p 0.19 to 0.40) nor adaptive − terrain following (p 0.13 to 1). Adaptive − terrain following has only 1 to 4 non-zero differences per row, so its smallest possible p is 0.125. Terrain following always scores 0 on the safe score, so TF − fixed 1.5 m is 0 there. By dose only, adaptive beats terrain following in 10/10 seeds at every radius (resolved, p 0.002 each, +0.022 to +0.739).

**Vertical-damping probe** (adaptive, $\sigma_h$ = 0.3 m, 20 seeds). Contacts with the target plant are split by whether the leg to it climbs to a taller plant or descends / stays level:

| Gains | $K_d^{(z)}$ scale (damping) | Target-plant contacts (climb / descent or level) | Other plants | Lowest altitude | $C_{eff}^{3D}$ | Collision-free | Safe $C_{eff}^{3D}$ |
|---|---|---|---:|---:|---:|:---:|---:|
| Tuned | × 1 (0.189) | 0.65 (0.10 / 0.55) | 0.60 | 0.31 m | 0.760 | 5/20 | 0.190 |
| Tuned | × 2.5 (0.527) | 0.10 (0.10 / 0.00) | 0.50 | 0.60 m | 0.765 | 10/20 | 0.382 |
| Tight | × 1 (0.258) | 0.30 (0.20 / 0.10) | 0.45 | 0.35 m | 0.871 | 7/20 | 0.305 |
| Tight | × 2.5 (0.683) | 0.20 (0.20 / 0.00) | 0.35 | 0.72 m | 0.874 | 11/20 | 0.480 |

Paired over the same seeds (× 2.5 − × 1; mean [descriptive bootstrap interval], higher / equal / lower, exact p; **bold** = resolved):
- Tuned:
  - safe $C_{eff}^{3D}$ +0.192 [+0.001, +0.382], 10/9/1, p 0.058, not resolved;
  - collision-free with both 4, only with × 2.5 in 6, only with × 1 in 1; McNemar p 0.13, not resolved;
  - target-plant contacts per mission **−0.550 [−0.850, −0.250]**, 0/11/9, p 0.0039;
  - other-plant contacts per mission −0.100 [−0.300, +0.100], 1/16/3, p 0.63, not resolved.
- Tight:
  - safe $C_{eff}^{3D}$ **+0.176 [+0.044, +0.350]**, 8/9/3, p 0.013;
  - collision-free with both 7, only with × 2.5 in 4, only with × 1 in 0; McNemar p 0.13, not resolved;
  - target-plant contacts −0.100 [−0.250, +0.000], 0/18/2, p 0.50, not resolved;
  - other-plant contacts −0.100 [−0.300, +0.100], 1/16/3, p 0.63, not resolved.

**Probe: dose without the standoff factor** (drift and hover error only, $C_{eff}^{3D}$; collisions as in the main tables):

| Gains | Fixed 1.5 m | Fixed clears-all | Adaptive | Terrain following | Adaptive − fixed 1.5 m | Adaptive − clears-all |
|---|---:|---:|---:|---:|:---:|:---:|
| Tuned | 0.718 ± 0.039 | 0.765 ± 0.016 | 0.783 ± 0.011 | 0.665 ± 0.039 | **+0.065 [+0.049, +0.084]**, 20/0/0, p < 0.0001 | **+0.018 [+0.014, +0.022]**, 20/0/0, p < 0.0001 |
| Tight | 0.814 ± 0.035 | 0.751 ± 0.028 | 0.876 ± 0.004 | 0.869 ± 0.004 | **+0.063 [+0.047, +0.078]**, 18/0/2, p < 0.0001 | **+0.125 [+0.113, +0.137]**, 20/0/0, p < 0.0001 |

**Sensitivity to the added drift parameters** (Tuned, mean $C_{eff}^{3D}$, dose only; fixed 1.5 m / clears-all / adaptive / terrain following). The drift parameters do not change the flight, so the collision counts above apply unchanged:

| $c_w$ (wind amplitude) | $w_{dw}$ = 0 m/s | $w_{dw}$ = 2 m/s | $w_{dw}$ = 5 m/s |
|---|---|---|---|
| 0.15 s⁻¹ (2.0 m/s) | 0.310 / 0.106 / 0.561 / 0.351 | 0.409 / 0.154 / 0.738 / 0.436 | 0.427 / 0.170 / 0.760 / 0.445 |
| 0.30 s⁻¹ (1.0 m/s) | 0.417 / 0.162 / 0.745 / 0.441 | 0.428 / 0.170 / 0.760 / 0.445 | 0.420 / 0.169 / 0.744 / 0.436 |
| 0.60 s⁻¹ (0.5 m/s) | 0.428 / 0.171 / 0.759 / 0.445 | 0.419 / 0.169 / 0.743 / 0.436 | 0.412 / 0.165 / 0.729 / 0.427 |

By mean dose the order is adaptive > terrain following > fixed 1.5 m > clears-all in all nine cases.
- Resolved in all nine (p < 0.0001 each): adaptive − terrain following (+0.210 to +0.315, 20/0/0 every time) and fixed 1.5 m − clears-all (+0.204 to +0.257, 20/0/0).
- Terrain following − fixed 1.5 m is resolved only for 2 m/s wind with no downwash (+0.040 [+0.016, +0.066], 14/0/6, p 0.0063). The other eight cases are not resolved (+0.016 to +0.027, 10/0/10 or 11/0/9, p 0.16–0.43). One resolved case out of nine is an isolated result; treat it with caution.

The mean drift offset ranges from 33.9 to 76.5 cm.

**$C_{eff}^{3D}$ vs canopy-height spread** (Tuned, 10 seeds; $C_{eff}^{3D}$, collision-free missions, safe $C_{eff}^{3D}$):

| $\sigma_h$ | Fixed 1.5 m | Fixed clears-all | Adaptive | Terrain following |
|---|---|---|---|---|
| 0.00 | 0.757 ± 0.010, 10/10, 0.757 | 0.757 ± 0.010, 10/10, 0.757 | 0.757 ± 0.010, 10/10, 0.757 | 0.463 ± 0.067, 0/10, 0.000 |
| 0.15 | 0.593 ± 0.036, 10/10, 0.593 | 0.367 ± 0.070, 10/10, 0.367 | 0.758 ± 0.009, 6/10, 0.454 | 0.457 ± 0.065, 0/10, 0.000 |
| 0.30 | 0.407 ± 0.050, 0/10, 0.000 | 0.152 ± 0.057, 10/10, 0.152 | 0.760 ± 0.012, 4/10, 0.302 | 0.459 ± 0.065, 0/10, 0.000 |
| 0.45 | 0.315 ± 0.055, 0/10, 0.000 | 0.097 ± 0.035, 10/10, 0.097 | 0.760 ± 0.012, 0/10, 0.000 | 0.459 ± 0.063, 0/10, 0.000 |
| 0.60 | 0.271 ± 0.051, 0/10, 0.000 | 0.081 ± 0.025, 10/10, 0.081 | 0.760 ± 0.012, 0/10, 0.000 | 0.453 ± 0.064, 0/10, 0.000 |

Paired over the same 10 seeds (difference [descriptive bootstrap interval], higher / equal / lower, exact p; **bold** = resolved). Rows marked *subset* are seeds 0–9 of a 20-seed group: $\sigma_h$ 0.15 of the low-spread check below, $\sigma_h$ 0.30 of the main runs. For those two spreads cite the 20-seed results:

| $\sigma_h$ | Dose: adaptive − fixed 1.5 m | Safe: adaptive − fixed 1.5 m | Safe: adaptive − clears-all | Safe: fixed 1.5 m − clears-all |
|---|---|---|---|---|
| 0.00 | 0 (identical), 0/10/0, p 1 | 0, 0/10/0, p 1 | 0, 0/10/0, p 1 | 0, 0/10/0, p 1 |
| 0.15 (subset) | **+0.164 [+0.143, +0.186]**, 10/0/0, p 0.002 | −0.140 [−0.371, +0.090], 6/0/4, p 0.29 | +0.087 [−0.156, +0.321], 6/0/4, p 0.49 | **+0.226 [+0.176, +0.277]**, 10/0/0, p 0.002 |
| 0.30 (subset) | **+0.353 [+0.322, +0.386]**, 10/0/0, p 0.002 | +0.302 [+0.075, +0.529], 4/6/0, p 0.13 | +0.150 [−0.096, +0.404], 4/0/6, p 0.29 | **−0.152 [−0.187, −0.117]**, 0/0/10, p 0.002 |
| 0.45 | **+0.445 [+0.411, +0.483]**, 10/0/0, p 0.002 | 0, 0/10/0, p 1 | **−0.097 [−0.119, −0.076]**, 0/0/10, p 0.002 | **−0.097 [−0.119, −0.076]**, 0/0/10, p 0.002 |
| 0.60 | **+0.489 [+0.458, +0.525]**, 10/0/0, p 0.002 | 0, 0/10/0, p 1 | **−0.081 [−0.096, −0.066]**, 0/0/10, p 0.002 | **−0.081 [−0.096, −0.066]**, 0/0/10, p 0.002 |

**Low height spread, $\sigma_h$ = 0.15 m, per seed and per clearance** (20 seeds, 0–19, both gain sets; the $\sigma_h$ table above uses seeds 0–9 of these runs). Each strategy cell gives collision-free missions and mean safe $C_{eff}^{3D}$; the paired columns give the safe-score difference [descriptive bootstrap interval], higher / equal / lower, exact p (**bold** = resolved). Terrain following is 0/20 (0.000) in every row:

| Gains | Clearance $r_d$ / $m_z$ | Fixed 1.5 m | Clears-all | Adaptive | Adaptive − fixed 1.5 m | Adaptive − clears-all | Fixed 1.5 m − clears-all |
|---|---|---|---|---|---|---|---|
| Tuned | 0 / 0 m | 19/20, 0.575 | 20/20, 0.389 | 15/20, 0.569 | −0.006 [−0.161, +0.146], 15/0/5, p 0.95 | **+0.180 [+0.027, +0.319]**, 15/0/5, p 0.035 | **+0.185 [+0.130, +0.238]**, 19/0/1, p < 0.0001 |
| Tuned | 0.1 / 0.05 m | 18/20, 0.546 | 20/20, 0.389 | 14/20, 0.530 | −0.016 [−0.197, +0.164], 14/0/6, p 0.86 | +0.141 [−0.023, +0.294], 14/0/6, p 0.11 | **+0.157 [+0.089, +0.220]**, 18/0/2, p 0.0005 |
| Tuned | 0.2 / 0.1 m | 15/20, 0.458 | 20/20, 0.389 | 12/20, 0.455 | −0.003 [−0.198, +0.198], 12/2/6, p 0.98 | +0.066 [−0.098, +0.228], 12/0/8, p 0.45 | +0.068 [−0.029, +0.160], 15/0/5, p 0.18 |
| Tight | 0 / 0 m | 19/20, 0.656 | 20/20, 0.372 | 18/20, 0.784 | +0.128 [−0.012, +0.253], 18/0/2, p 0.089 | **+0.411 [+0.275, +0.520]**, 18/0/2, p 0.0001 | **+0.283 [+0.220, +0.341]**, 19/0/1, p < 0.0001 |
| Tight | 0.1 / 0.05 m | 19/20, 0.656 | 20/20, 0.372 | 15/20, 0.653 | −0.003 [−0.184, +0.172], 15/0/5, p 0.97 | **+0.280 [+0.100, +0.443]**, 15/0/5, p 0.0078 | **+0.283 [+0.220, +0.341]**, 19/0/1, p < 0.0001 |
| Tight | 0.2 / 0.1 m | 18/20, 0.623 | 20/20, 0.372 | 14/20, 0.609 | −0.014 [−0.221, +0.192], 14/0/6, p 0.90 | **+0.237 [+0.050, +0.412]**, 14/0/6, p 0.026 | **+0.251 [+0.171, +0.322]**, 18/0/2, p 0.0001 |

Adaptive vs fixed 1.5 m in the same rows. The margin column gives the average per-seed safe-score margin in the seeds adaptive wins and in the seeds it loses. The sign test is an exact binomial test on the win / loss counts; the McNemar test compares the seeds where only adaptive collides with those where only fixed 1.5 m collides. By dose alone ($C_{eff}^{3D}$, collisions ignored) adaptive is higher in 20/0/0 seeds in every row:

| Gains | Clearance | Mean margin when adaptive wins / loses | Sign test p (wins / losses) | Only adaptive collides | Only fixed 1.5 m collides | McNemar p | Both / neither |
|---|---|---|:---:|:---:|:---:|:---:|:---:|
| Tuned | 0 / 0 m | +0.188 / −0.588 | 0.041 (15 / 5) | 5 | 1 | 0.22 | 0 / 14 |
| Tuned | 0.1 / 0.05 m | +0.229 / −0.588 | 0.12 (14 / 6) | 6 | 2 | 0.29 | 0 / 12 |
| Tuned | 0.2 / 0.1 m | +0.292 / −0.592 | 0.24 (12 / 6) | 6 | 3 | 0.51 | 2 / 9 |
| Tight | 0 / 0 m | +0.217 / −0.672 | 0.0004 (18 / 2) | 2 | 1 | 1.00 | 0 / 17 |
| Tight | 0.1 / 0.05 m | +0.220 / −0.672 | 0.041 (15 / 5) | 5 | 1 | 0.22 | 0 / 14 |
| Tight | 0.2 / 0.1 m | +0.268 / −0.671 | 0.12 (14 / 6) | 6 | 2 | 0.29 | 0 / 12 |

**What changed with the physics fix** (main runs, $\sigma_h$ = 0.3 m, 20 seeds). The *Before* column is taken from the output of the previous version, commit 1c61058; the current script does not print it:

| Quantity | Before (commit 1c61058, state clamp) | After (braking barrier) |
|---|---|---|
| TF Tuned / Tight $C_{eff}^{3D}$ | 0.448 / 0.839 | 0.445 / 0.838 |
| TF Tuned / Tight crown contacts per mission | 19.50 / 17.50 | 19.55 / 17.45 |
| TF Tuned plants sprayed from inside the crown | 0.70 | 0.90 |
| TF floor events | 21.4 / 18.5 clamps per mission; 170 / 218 touchdowns > 1 m/s, up to 2.78 m/s in the sweeps | 0 contacts; barrier active 379.6 / 423.1 steps per mission; lowest altitude 0.3050 m |
| Adaptive, fixed 1.5 m, clears-all | – | unchanged to the printed precision (adaptive Tuned now uses the barrier 1.5 steps per mission) |
| Crown-radius sweep, TF Tuned at 0.25 / 0.5 / 1.0 m | 0.021 / 0.461 / 0.709 | 0.020 / 0.459 / 0.708 |
| Newly reported | – | collision-free missions and safe $C_{eff}^{3D}$ in every table |

**Key findings** (resolved = exact sign-flip p < 0.05; everything else is stated as not resolved; see the multiplicity note under Statistics):
- **At $\sigma_h$ = 0.3 m, only the fixed altitude that clears every plant flies collision-free; every other strategy collides in most missions.** This holds even under the most lenient rule, a point drone with zero clearance. The collision-free missions at $\sigma_h$ = 0.3 m are:
  - clears-all 20/20 with both gain sets;
  - adaptive 5/20 (Tuned) and 7/20 (Tight);
  - fixed 1.5 m 2/20 and 3/20;
  - terrain following 0/20 with both, and 0/10 in every crown-radius and $\sigma_h$ configuration.

  On the safe score, clears-all is resolved ahead of fixed 1.5 m (Tuned **+0.119**, 18/0/2, p 0.0029; Tight **+0.084**, 17/0/3, p 0.039) and of terrain following (20/0/0, p < 0.0001). Clears-all vs adaptive is **not resolved** (adaptive − clears-all Tuned +0.019, p 0.81; Tight +0.139, p 0.17). Neither card strategy is both safe and accurate here.
- **The safe-score comparison of adaptive and clears-all is not resolved under the point-drone rule, and the sign of the mean depends on the clearance and crown-radius assumptions.**
  - With the point-drone rule and 0.5 m crowns, adaptive's mean safe $C_{eff}^{3D}$ (0.190 / 0.305) is above clears-all (0.170 / 0.165), but the difference is not resolved (p 0.81 / 0.17). Its mean is pulled up by its few collision-free missions, which average 0.7585 / 0.8703.
  - With 1.0 m crowns (10 seeds) the Tuned sign reverses: 0.075 vs 0.152, adaptive − clears-all −0.077, 1/0/9, p 0.40, not resolved.
  - With a 0.1 m drone radius and 0.05 m vertical margin, Tuned adaptive is collision-free in 0/20 missions, and clears-all is resolved ahead (adaptive − clears-all **−0.170**, 0/0/20, p < 0.0001). Tight adaptive stays 7/20 (0.305), not resolved against clears-all (p 0.17).
  - With a 0.2 m radius and 0.1 m margin, adaptive is collision-free in 0/20 (Tuned) and 2/20 (Tight) missions, safe $C_{eff}^{3D}$ 0.000 / 0.087. Clears-all is resolved ahead with Tuned gains (p < 0.0001); with Tight gains the difference is not resolved (−0.078, 2/0/18, p 0.23).
  - Clears-all flies at the tallest crown top + 0.5 m, so neither sweep affects it. In the clearance sweep (20 main seeds) it is 20/20 at every clearance, with safe C_eff 0.170 / 0.165. In the crown-radius sweep (10 seeds) it is 10/10 at every radius, with 0.152 / 0.144.
  - Fixed 1.5 m reaches 0/20 at a 0.2 m radius with either gain set.
- **At low height spread ($\sigma_h$ = 0.15 m, 20 seeds) the safe-score difference between adaptive and fixed 1.5 m is not resolved.** No row is resolved by the exact sign-flip test: the means run from −0.016 to +0.128 with p 0.089 to 0.98.
  - Adaptive has the higher dose $C_{eff}^{3D}$ in every seed (20/0/0 in all six rows), and it wins on the safe score in exactly the seeds where it does not collide.
  - It also collides more often than fixed 1.5 m. Seeds where only adaptive collides: 5 / 6 / 6 (Tuned) and 2 / 5 / 6 (Tight). Seeds where only fixed 1.5 m collides: 1 / 2 / 3 and 1 / 1 / 2. This collision difference is not resolved either (McNemar p 0.22 to 1.00).
  - Wins and losses do not balance: the mean margin is +0.188 to +0.292 in the seeds adaptive wins and −0.588 to −0.672 in the seeds it loses. That is why the mean difference stays near 0 although adaptive wins more seeds.
  - The sign test on the win / loss counts gives p 0.0004 for the Tight point-drone row (18 / 2) and p 0.041 for Tuned point drone and Tight 0.1 / 0.05 m (15 / 5). The other three rows are not resolved (p 0.12 to 0.24). These count tests do not say that adaptive scores higher on average, which is not resolved in any row.
  - Against clears-all, adaptive's safe-score lead is resolved in four of six rows: all three Tight rows (+0.237 to +0.411, p 0.0001 to 0.026) and the Tuned point-drone row (**+0.180**, p 0.035). The two other Tuned rows are not resolved (p 0.11 and 0.45). Fixed 1.5 m is resolved ahead of clears-all in five of six rows (p ≤ 0.0005); the Tuned 0.2 / 0.1 m row is not resolved (p 0.18).
  - An earlier 10-seed run (seeds 0–9; output of commit db461b9, not printed now) had means favouring fixed 1.5 m by up to 0.24 with clearance. Those differences were not resolved either, and at 20 seeds no row is resolved in either direction.
  - At $\sigma_h$ = 0 the three planned strategies coincide (0.757, 10-seed sweep).
- **By dose quality alone, per-plant adaptive altitude is the clear winner, as the card expects.** At $\sigma_h$ = 0.3 m it holds the standoff within 4.5 cm (Tuned) and 1.8 cm (Tight) of 0.5 m. It reaches $C_{eff}^{3D}$ = 0.760 / 0.871 and beats both fixed altitudes in 20/20 seeds (+0.332 to +0.706, resolved, p < 0.0001 each).
  - In the 10-seed $\sigma_h$ sweep (Tuned), its $C_{eff}^{3D}$ is flat across $\sigma_h$ (0.757–0.760), but its safety is not: collision-free missions fall from 10/10 at $\sigma_h$ = 0 to 6/10, 4/10 and then 0/10 at $\sigma_h$ ≥ 0.45, so its safe $C_{eff}^{3D}$ falls from 0.757 to 0.
  - Its safe-score difference to fixed 1.5 m is not resolved at any spread. At 0.3 m (20 main seeds): Tuned +0.139, p 0.14; Tight +0.224, p 0.051. At 0.15 m (20 low-spread seeds): Tuned −0.006, p 0.95; Tight +0.128, p 0.089. At 0.45 and 0.6 both score 0, and clears-all is resolved ahead of both (10/10 seeds, p 0.002).
  - Fixed 1.5 m falls from 0.757 to 0.271 in $C_{eff}^{3D}$. In the 10 sweep seeds it collides in every mission from $\sigma_h$ = 0.3 m; in the 20 main-run seeds at 0.3 m it has 2/20 (Tuned) and 3/20 (Tight) collision-free missions. The clears-all altitude stays collision-free but falls to 0.081.
- **Part of the dose gap is built into the card's dose model, but not all of it.** The standoff factor penalises fixed altitudes directly. Without it, adaptive still beats the clears-all altitude in every seed and fixed 1.5 m in 18–20 of 20. The margins are smaller but resolved: Tuned +0.065 / +0.018, Tight +0.063 / +0.125, p < 0.0001 each.
- **Card-style lidar terrain following without look-ahead is unsafe once collisions count.** It tracks the ground (0.5 m) between plants and sees each crown only once it is overhead. It therefore meets every plant from the side: 19.55 (Tuned) / 17.45 (Tight) crown contacts per mission, almost all on the plant it is approaching. Between plants it dives towards the floor so often that the braking barrier is active 379.6 / 423.1 steps per mission.
  - The previous version (commit 1c61058) enforced the floor with an instant stop at up to 2.78 m/s. That stop is physically impossible, and in reality these would have been ground strikes.
  - By dose alone, with Tuned gains, it is not resolved against fixed 1.5 m (+0.018, 10/0/10, p 0.38). With Tight gains it is resolved ahead of fixed 1.5 m (+0.356, p < 0.0001) and resolved slightly below adaptive (adaptive − TF **+0.033**, 19/0/1, p < 0.0001). The dose-only result depends strongly on the crown radius I added (Tuned: 0.020 / 0.459 / 0.708 at 0.25 / 0.5 / 1.0 m).
  - With collisions counted, terrain following scores 0 at every radius and every $\sigma_h$. In the 20-seed main runs (0.5 m crowns) adaptive is resolved ahead on the safe score with Tight gains (**+0.305**, 7/13/0, p 0.016). With Tuned gains it is not resolved (+0.190, 5/15/0, p 0.063). In the 10-seed crown-radius sweep no radius is resolved (p 0.13 to 1).
- **Higher vertical damping removes the adaptive plan's descent contacts with the target plant (probed); its effect on the safe score is resolved with Tight gains only.** With Tuned gains the plan touches 1.25 crowns per mission: 0.65 on the plant being approached and 0.60 on other plants. Of the 0.65, 0.55 happen on descending or level legs and 0.10 on climbs to a taller plant.
  - Raising the vertical derivative gain 2.5× (damping 0.189 → 0.527, Tuned) cuts the target-plant contacts from 0.65 to 0.10 per mission (**−0.550**, 0/11/9, p 0.0039). Collision-free missions go 5/20 → 10/20 and the safe $C_{eff}^{3D}$ 0.190 → 0.382. Neither the safe-score gain (+0.192, 10/9/1, p 0.058) nor the collision split (6 vs 1, McNemar p 0.13) is resolved.
  - With Tight gains: 7/20 → 11/20 collision-free and 0.305 → 0.480 safe. The safe-score gain is resolved (**+0.176**, 8/9/3, p 0.013); the collision split (4 vs 0, McNemar p 0.13) and the change in target-plant contacts (−0.100, p 0.50) are not. The climb contacts (0.10 / 0.20) remain.
  - The change in other-plant contacts is not resolved with either gain set (0.60 → 0.50 and 0.45 → 0.35, both −0.100, 1/16/3, p 0.63). Flying at the endpoints' altitude over a taller plant in between is the likely cause of these contacts (*not probed*).
- **The horizontal gains still matter for the top score.** With the card's Tuned gains, the S063 finding carries over: 40 cm horizontal hover RMS and 19.9 of 20 settle time-outs. That caps adaptive at $C_{eff}^{3D}$ = 0.760; Tight raises it to 0.871 (paired **+0.111**, 20/0/0, p < 0.0001).
- **The resolved parts of the dose ranking do not depend on the added drift parameters.** Adaptive > terrain following and fixed 1.5 m > clears-all are resolved in all nine combinations. Terrain following vs fixed 1.5 m is resolved in only one of the nine, an isolated result. These parameters do not affect the flight, so they do not change the collision counts.

### 3D Field Overview

Seed 0, Tuned PID: adaptive (left) and fixed 1.5 m (right). Green stems are canopy heights, dark-green rings the crowns (0.5 m), black x marks positions inside a crown, and dotted lines the standoff during spraying. Both seed-0 missions are collisions: adaptive touches 1 crown (C_eff 0.754) and fixed 1.5 m touches 2 (C_eff 0.401), so both have a safe C_eff of 0. The panel titles state this:

![Field 3D](./field_3d.png)

### Altitude Time Series

Seed 0, Tuned PID, all four strategies. Brown marks the canopy top of the plant being sprayed; green bands are $h_i$ + 0.5 ± $\sigma_z$:

![Altitude](./altitude_timeseries.png)

### Standoff Deviation per Plant

Mean $|\delta z - 0.5|$ during spraying over 20 seeds, by visit order (log scale):

![Standoff](./standoff_deviation.png)

### Droplet Impacts

Seed 0, Tuned PID: impact points (every 5th spray sample) around each plant, with the $\sigma_{spray}$ circle. The panel titles give this seed's $C_{eff}^{3D}$ (0.754 / 0.401), mean offset (34.7 / 35.6 cm) and collision status (1 / 2 crown contacts, safe $C_{eff}^{3D}$ 0):

![Impacts](./droplet_impacts.png)

### $C_{eff}^{3D}$ vs $\sigma_h$

Left: $C_{eff}^{3D}$ by dose only. Right: safe $C_{eff}^{3D}$, with collision missions scored 0:

![C_eff vs sigma_h](./ceff_vs_sigma_h.png)

### Animation

Lidar terrain following, Tuned PID, seed 0. The drone (red) dives between plants to the 0.305 m barrier floor and climbs at each crown. Dark-green rings are the crowns, black x marks positions inside a crown, and the title counts crown entries. The mission enters crowns 66 times and touches 18 plants. 169 frames, one every 100 steps (2 s):

![Animation](./animation.gif)

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| $t_{fall} = \sqrt{2\delta z/(g + w_{dw})}$ | $t_{fall} = (\sqrt{w_{dw}^2 + 2g\delta z} - w_{dw})/g$, $w_{dw}$ = 2 m/s (swept 0–5) | The card adds a speed to an acceleration. Card corrected. |
| Drift uses a wind *velocity* $\mathbf u_w$, but the wind model is an acceleration | $\mathbf u_w$ = sinusoidal gust acceleration / $c_w$, $c_w$ = 0.3 s⁻¹ (swept 0.15–0.6) | No wind speed is defined on the card. Card updated. |
| Hover starts on the plant (card) | S063 continuous flight with settle test; altitude error < 0.15 m added to it | S063 fix (arrival transients matter). |
| Single horizontal gain set (Tuned) | Tuned and Tight | S063 showed Tuned is poorly damped near the gust frequency; Tight shows the altitude effect without that limit. |
| Fixed altitude only at 1.5 m | Also a fixed altitude that clears every plant | 1.5 m is below 5.8 % of the plants; a 2D plan has to choose between collisions and standoff error. |
| Plants as points | Crowns of radius 0.5 m (swept 0.25–1.0 m); contacts counted; dose 0 when the nozzle is inside the crown | Needed for the lidar surface and for physical sanity (X4). The terrain-following vs fixed-altitude result depends on the radius, so it is swept. |
| Adaptive: "climbs to $z_{cmd,i}$ before hovering" | Leg reference at the higher of the two plant altitudes, then descent over the plant | Climb first; a linear ramp would approach tall plants below their top. |
| Altitude bounds enforced | Enforced by an acceleration-limited braking barrier in the controller (no state clamp, no velocity reset). Asserts check $z \ge$ 0.3 m, $z$ > 0, and thrust acceleration ≤ $A_{max}$ from the logged trajectory | The previous state clamp stopped the drone within one 0.02 s step at up to 2.78 m/s, far beyond the 3 m/s² limit. The asserts can fail (X4): with the barrier disabled the altitude assert fires, and a reinserted velocity reset trips the acceleration assert. Card clarified. |
| Barrier state | The barrier uses the true altitude and vertical speed; the PID uses the noisy altimeter or lidar | An ideal height estimate for the safety layer. A barrier on the noisy 0.05 m measurement would presumably need an extra altitude margin (*not probed*). |
| Broken loop undetected | Assert: barrier active in ≤ 50 % of a mission's steps | Nominal runs reach at most 3.65 %; a sign-flipped vertical loop reaches 99.5 % (separate test, not printed) and trips the assert. |
| Point drone | Main rule: point drone, zero clearance; clearance sweep (0.1 / 0.2 m radius) reported | The ranking of mean safe scores depends on it; only clears-all is unaffected. |
| Collisions not scored (card) | Any crown contact makes the mission a collision; safe $C_{eff}^{3D}$ = 0 for that mission, shown beside $C_{eff}^{3D}$ everywhere | A collision is a failed mission, not just a lower dose (X4). |
| Terrain-following PID as a safe strategy | Implemented as specified; it collides in every mission | Without look-ahead the lidar sees a crown only once it is overhead. Card note added. |
| Single dose model | Also reported without the standoff factor | Separates the built-in standoff penalty from the drift effect (C9). |

---

## Extensions

1. Look-ahead terrain following: command the maximum canopy height within the next 1–2 m of track (card Extension 1). Without it the strategy collides in every mission.
2. Better-damped vertical gains plus a collision-aware path (climb before approach, clear intermediate plants). In the probe, damping alone raised the adaptive plan's collision-free missions from 5/20 to 10/20 (Tuned) and from 7/20 to 11/20 (Tight); the collision split is not resolved (McNemar p 0.13), so a larger seed set is needed to confirm it.
3. A 3D tour cost that includes the canopy along each leg (card Extension 5), to avoid transit contacts.
4. Wind-aware nozzle offset (card Extension 2): the mean drift offset is 23.8–36.7 cm, comparable to the horizontal hover error with Tight gains.

---

## Related Scenarios

- Base 2D version: [S063 Single-Plant Precision Spraying](../../../../scenarios/04_industrial_agriculture/S063_precision_spraying.md)
- 3D card: [S063 3D Upgrade — Precision Per-Plant Spraying](../../../../scenarios/04_industrial_agriculture/3d/S063_3d_precision_spraying.md)
- Spray coverage: [S067 Spray Coverage Overlap Optimization](../../../../scenarios/04_industrial_agriculture/S067_spray_overlap.md)
- Greenhouse flight: [S064 Greenhouse Interior Precision Flight](../../../../scenarios/04_industrial_agriculture/S064_greenhouse.md)
