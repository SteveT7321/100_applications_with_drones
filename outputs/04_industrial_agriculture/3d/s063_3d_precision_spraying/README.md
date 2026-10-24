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

---

## Mathematical Model

**Vertical loop**: $a_z = K_p^{(z)}e_z + K_i^{(z)}\!\int e_z + K_d^{(z)}(0 - \dot z)$, with $K^{(z)} = 1.5\,K^{(xy)}$ and Euler integration at 50 Hz. The altitude bounds $z \in [0.3, 6]$ m are enforced by the controller, not by clamping the state. A braking barrier raises the vertical command whenever the descent speed exceeds what full braking can stop before $z_{min}$ + 5 mm. The plan uses a braking rate of 2.60 m/s² = $A_{max}$ − worst-case downwash (0.15 + 4 × 0.05) − 0.05 margin, with the exact discrete-time stopping distance; the same applies towards the 6 m ceiling. The script asserts at every step that $z \ge$ 0.3 m and $z$ > 0. It also asserts, from the logged trajectory, that the thrust part of the vertical acceleration never exceeds $A_{max}$. The altitude error is measured with 0.05 m noise. For terrain following, the measured quantity is the standoff to the surface below: $e_z = 0.5 - (z - s(x, y) + \eta)$.

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
| Collision rule (added) | any crown contact → collision mission, safe $C_{eff}^{3D}$ = 0 |
| Settle test | S063 (0.3 m, 0.1 m/s, 0.2 s, 10 s time-out) plus altitude error < 0.15 m (added) |
| Wind drag rate $c_w$ (added) | 0.3 s⁻¹ (wind amplitude 1 m/s); sweep 0.15, 0.3, 0.6 |
| Downwash droplet speed $w_{dw}$ (added) | 2 m/s; sweep 0, 2, 5 m/s |
| Seeds | 20 per strategy and gain set; $\sigma_h$ sweep 0–0.6 m with 10 seeds (Tuned) |

---

## Implementation

```
src/04_industrial_agriculture/3d/s063_3d_precision_spraying.py
```

```bash
conda activate drones
python src/04_industrial_agriculture/3d/s063_3d_precision_spraying.py
```

The script imports the S063 module for the layout, tour, gains and noise streams. Runtime ≈ 2.3 min, including the crown-radius sweep and the damping probe (140 s and 138 s with `OMP/OPENBLAS/MKL_NUM_THREADS=1`, 138 s with default threading). The three runs printed byte-identical output and wrote byte-identical PNG and GIF files.

---

## Results

**Checks**:
- Heights over 20 seeds × 20 plants: mean 1.011 m, std 0.298 m, range 0.313–1.935 m.
- 23 plants (5.8 %) are taller than the fixed 1.5 m altitude, and 46 more are within 0.2 m below it.
- Fall time at a 0.5 m standoff: 0.319 / 0.175 / 0.092 s for $w_{dw}$ = 0 / 2 / 5 m/s; at 1.5 m: 0.553 / 0.386 / 0.242 s.
- The highest descent speed from which full braking ($A_{max}$ − mean downwash) can stop between 0.3 m and the ground is 1.31 m/s. The previous version clamped the altitude and zeroed the vertical speed in one step at up to 2.78 m/s. That was a physically impossible stop, and in reality a ground strike. With the barrier, the lowest altitude in any job group is 0.3050 m and no step needs more than $A_{max}$.
- Vertical loop poles:
  - Tuned × 1.5: −0.324 ± 1.682j and −0.102 (damping of the complex pair 0.189);
  - Tight × 1.5: −0.699 ± 2.621j and −0.102 (damping 0.258).

### $\sigma_h$ = 0.3 m, 20 seeds

**Tuned PID (card)**

| Strategy | $C_{eff}^{3D}$ | Collision-free missions | Safe $C_{eff}^{3D}$ | Hover RMS 3D / horiz. | Standoff deviation | Drift offset $d$ | Crown contacts per mission (target / other) | Plants sprayed from inside the crown | Time | Saturated | Floor-barrier steps per mission | z range |
|---|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Fixed 1.5 m | 0.428 ± 0.069 | 2/20 | 0.051 | 49.5 / 40.3 cm | 24.1 cm | 34.8 cm | 1.60 (1.60 / 0.00) | 1.50 | 333.3 s | 0.00 % | 0.0 | 1.404–1.53 m |
| Fixed, clears all | 0.170 ± 0.082 | **20/20** | 0.170 | 70.6 / 40.3 cm | 54.2 cm | 35.6 cm | 0.00 | 0.00 | 333.3 s | 0.00 % | 0.0 | 1.804–2.47 m |
| Per-plant adaptive | **0.760 ± 0.011** | 5/20 | **0.190** | 40.6 / 40.3 cm | **4.5 cm** | 33.9 cm | 1.25 (0.65 / 0.60) | 0.00 | 333.5 s | 0.01 % | 1.5 | 0.305–2.91 m |
| Lidar terrain following | 0.445 ± 0.066 | **0/20** | 0.000 | 52.0 / 40.5 cm | 25.2 cm | 35.3 cm | 19.55 (19.40 / 0.15) | 0.90 | 334.3 s | 9.35 % | 379.6 | 0.305–3.45 m |

**Tight PID**

| Strategy | $C_{eff}^{3D}$ | Collision-free missions | Safe $C_{eff}^{3D}$ | Hover RMS 3D / horiz. | Standoff deviation | Drift offset $d$ | Crown contacts per mission (target / other) | Plants sprayed from inside the crown | Time | Saturated | Floor-barrier steps per mission | z range |
|---|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Fixed 1.5 m | 0.483 ± 0.072 | 3/20 | 0.081 | 27.3 / 10.3 cm | 23.8 cm | 23.8 cm | 1.50 (1.50 / 0.00) | 1.30 | 265.3 s | 0.27 % | 0.0 | 1.438–1.54 m |
| Fixed, clears all | 0.165 ± 0.080 | **20/20** | 0.165 | 58.5 / 10.3 cm | 56.7 cm | 36.7 cm | 0.00 | 0.00 | 265.3 s | 0.27 % | 0.0 | 1.839–2.46 m |
| Per-plant adaptive | **0.871 ± 0.004** | 7/20 | **0.305** | 10.6 / 10.3 cm | **1.8 cm** | 24.7 cm | 0.75 (0.30 / 0.45) | 0.00 | 275.0 s | 0.98 % | 0.1 | 0.350–2.79 m |
| Lidar terrain following | 0.838 ± 0.016 | **0/20** | 0.000 | 12.3 / 10.5 cm | 3.7 cm | 25.5 cm | 17.45 (16.65 / 0.80) | 0.00 | 289.3 s | 9.73 % | 423.1 | 0.305–3.61 m |

Plants receiving a dose > 0.9: at most 4.0 % (adaptive, Tight). Settle time-outs per mission: Tuned 19.9–20.0 of 20 for every strategy; Tight 4.3 / 4.3 / 5.2 / 7.2. Vertical travel per mission: Tuned 6.5 / 6.5 / 24.9 / 151.2 m; Tight 6.9 / 6.9 / 20.8 / 73.2 m.

**Floor barrier over every job group** (main runs, $\sigma_h$ sweep, crown-radius sweep and damping probe). 14 of 38 groups ever use the barrier. All are terrain following, plus adaptive at Tuned $\sigma_h$ ≥ 0.3 and Tight $\sigma_h$ = 0.3. No step needed more than $A_{max}$, and the lowest altitude is 0.3050 m:

| Job group | Barrier steps per mission | Missions using it | Lowest altitude |
|---|---:|:---:|---:|
| TF, Tuned, $\sigma_h$ 0.0 / 0.15 / 0.3 / 0.45 / 0.6 | 339.4 / 350.5 / 379.6 / 438.9 / 480.9 | all | 0.3050 m |
| TF, Tuned, 0.3, $R_{crown}$ 0.25 / 1.0 m | 233.7 / 257.6 | all | 0.3050 m |
| TF, Tight, 0.3, $R_{crown}$ 0.25 / 0.5 / 1.0 m | 494.9 / 423.1 / 356.3 | all | 0.3050 m |
| Adaptive, Tuned, $\sigma_h$ 0.3 / 0.45 / 0.6 | 1.5 / 17.0 / 28.5 | 1/20, 7/10, 9/10 | 0.3051 / 0.3051 / 0.3050 m |
| Adaptive, Tight, 0.3 | 0.1 | 1/20 | 0.3501 m |

**Per-seed comparisons** ($C_{eff}^{3D}$ higher / equal / lower and mean difference; then the same for safe $C_{eff}^{3D}$):

| Gains | Pair | $C_{eff}^{3D}$ | Safe $C_{eff}^{3D}$ |
|---|---|---|---|
| Tuned | Adaptive vs fixed 1.5 m | 20 / 0 / 0, +0.332 | 5 / 13 / 2, +0.139 |
| Tuned | Adaptive vs fixed clears-all | 20 / 0 / 0, +0.590 | 5 / 0 / 15, +0.019 |
| Tuned | Adaptive vs terrain following | 20 / 0 / 0, +0.315 | 5 / 15 / 0, +0.190 |
| Tuned | Terrain following vs fixed 1.5 m | 10 / 0 / 10, +0.018 | 0 / 18 / 2, −0.051 |
| Tuned | Fixed clears-all vs fixed 1.5 m | 0 / 0 / 20, −0.257 | 18 / 0 / 2, +0.119 |
| Tight | Adaptive vs fixed 1.5 m | 20 / 0 / 0, +0.388 | 7 / 11 / 2, +0.224 |
| Tight | Adaptive vs fixed clears-all | 20 / 0 / 0, +0.706 | 7 / 0 / 13, +0.139 |
| Tight | Adaptive vs terrain following | 19 / 0 / 1, +0.033 | 7 / 13 / 0, +0.305 |
| Tight | Terrain following vs fixed 1.5 m | 20 / 0 / 0, +0.356 | 0 / 17 / 3, −0.081 |
| Tight | Fixed clears-all vs fixed 1.5 m | 0 / 0 / 20, −0.318 | 17 / 0 / 3, +0.084 |

**Crown-radius sweep** ($\sigma_h$ = 0.3 m, 10 seeds; $C_{eff}^{3D}$ / collision-free missions / safe $C_{eff}^{3D}$ / crown contacts per mission):

| $R_{crown}$ | Gains | Fixed 1.5 m | Adaptive | Terrain following | TF > fixed 1.5 m ($C_{eff}$ / safe) | Adaptive > TF ($C_{eff}$ / safe) |
|---|---|---|---|---|:---:|:---:|
| 0.25 m | Tuned | 0.407 / 0/10 / 0.000 / 1.70 | 0.760 / 4/10 / 0.302 / 0.70 | 0.020 / 0/10 / 0.000 / 18.80 | 0/10 / 0/10 | 10/10 / 4/10 |
| 0.25 m | Tight | 0.458 / 0/10 / 0.000 / 1.70 | 0.871 / 4/10 / 0.348 / 0.80 | 0.850 / 0/10 / 0.000 / 19.50 | 10/10 / 0/10 | 10/10 / 4/10 |
| 0.50 m | Tuned | 0.407 / 0/10 / 0.000 / 1.80 | 0.760 / 4/10 / 0.302 / 1.20 | 0.459 / 0/10 / 0.000 / 19.40 | 7/10 / 0/10 | 10/10 / 4/10 |
| 0.50 m | Tight | 0.458 / 0/10 / 0.000 / 1.70 | 0.871 / 4/10 / 0.348 / 0.80 | 0.836 / 0/10 / 0.000 / 17.40 | 10/10 / 0/10 | 10/10 / 4/10 |
| 1.00 m | Tuned | 0.407 / 0/10 / 0.000 / 1.80 | 0.760 / 1/10 / 0.075 / 2.20 | 0.708 / 0/10 / 0.000 / 16.70 | 10/10 / 0/10 | 10/10 / 1/10 |
| 1.00 m | Tight | 0.458 / 0/10 / 0.000 / 1.70 | 0.871 / 3/10 / 0.260 / 1.20 | 0.823 / 0/10 / 0.000 / 13.20 | 10/10 / 0/10 | 10/10 / 3/10 |

**Vertical-damping probe** (adaptive, $\sigma_h$ = 0.3 m, 20 seeds). Contacts with the target plant are split by whether the leg to it climbs to a taller plant or descends / stays level:

| Gains | $K_d^{(z)}$ scale (damping) | Target-plant contacts (climb / descent or level) | Other plants | Lowest altitude | $C_{eff}^{3D}$ | Collision-free | Safe $C_{eff}^{3D}$ |
|---|---|---|---:|---:|---:|:---:|---:|
| Tuned | × 1 (0.189) | 0.65 (0.10 / 0.55) | 0.60 | 0.31 m | 0.760 | 5/20 | 0.190 |
| Tuned | × 2.5 (0.527) | 0.10 (0.10 / 0.00) | 0.50 | 0.60 m | 0.765 | 10/20 | 0.382 |
| Tight | × 1 (0.258) | 0.30 (0.20 / 0.10) | 0.45 | 0.35 m | 0.871 | 7/20 | 0.305 |
| Tight | × 2.5 (0.683) | 0.20 (0.20 / 0.00) | 0.35 | 0.72 m | 0.874 | 11/20 | 0.480 |

**Probe: dose without the standoff factor** (drift and hover error only, $C_{eff}^{3D}$; collisions as in the main tables):

| Gains | Fixed 1.5 m | Fixed clears-all | Adaptive | Terrain following | Adaptive vs fixed 1.5 m | Adaptive vs clears-all |
|---|---:|---:|---:|---:|:---:|:---:|
| Tuned | 0.718 ± 0.039 | 0.765 ± 0.016 | 0.783 ± 0.011 | 0.665 ± 0.039 | 20 / 0 / 0 | 20 / 0 / 0 |
| Tight | 0.814 ± 0.035 | 0.751 ± 0.028 | 0.876 ± 0.004 | 0.869 ± 0.004 | 18 / 0 / 2 | 20 / 0 / 0 |

**Sensitivity to the added drift parameters** (Tuned, mean $C_{eff}^{3D}$, dose only; fixed 1.5 m / clears-all / adaptive / terrain following). The drift parameters do not change the flight, so the collision counts above apply unchanged:

| $c_w$ (wind amplitude) | $w_{dw}$ = 0 m/s | $w_{dw}$ = 2 m/s | $w_{dw}$ = 5 m/s |
|---|---|---|---|
| 0.15 s⁻¹ (2.0 m/s) | 0.310 / 0.106 / 0.561 / 0.351 | 0.409 / 0.154 / 0.738 / 0.436 | 0.427 / 0.170 / 0.760 / 0.445 |
| 0.30 s⁻¹ (1.0 m/s) | 0.417 / 0.162 / 0.745 / 0.441 | 0.428 / 0.170 / 0.760 / 0.445 | 0.420 / 0.169 / 0.744 / 0.436 |
| 0.60 s⁻¹ (0.5 m/s) | 0.428 / 0.171 / 0.759 / 0.445 | 0.419 / 0.169 / 0.743 / 0.436 | 0.412 / 0.165 / 0.729 / 0.427 |

By dose alone the ranking is adaptive > terrain following > fixed 1.5 m > clears-all in all nine cases. The mean drift offset ranges from 33.9 to 76.5 cm.

**$C_{eff}^{3D}$ vs canopy-height spread** (Tuned, 10 seeds; $C_{eff}^{3D}$, collision-free missions, safe $C_{eff}^{3D}$):

| $\sigma_h$ | Fixed 1.5 m | Fixed clears-all | Adaptive | Terrain following |
|---|---|---|---|---|
| 0.00 | 0.757 ± 0.010, 10/10, 0.757 | 0.757 ± 0.010, 10/10, 0.757 | 0.757 ± 0.010, 10/10, 0.757 | 0.463 ± 0.067, 0/10, 0.000 |
| 0.15 | 0.593 ± 0.036, 10/10, 0.593 | 0.367 ± 0.070, 10/10, 0.367 | 0.758 ± 0.009, 6/10, 0.454 | 0.457 ± 0.065, 0/10, 0.000 |
| 0.30 | 0.407 ± 0.050, 0/10, 0.000 | 0.152 ± 0.057, 10/10, 0.152 | 0.760 ± 0.012, 4/10, 0.302 | 0.459 ± 0.065, 0/10, 0.000 |
| 0.45 | 0.315 ± 0.055, 0/10, 0.000 | 0.097 ± 0.035, 10/10, 0.097 | 0.760 ± 0.012, 0/10, 0.000 | 0.459 ± 0.063, 0/10, 0.000 |
| 0.60 | 0.271 ± 0.051, 0/10, 0.000 | 0.081 ± 0.025, 10/10, 0.081 | 0.760 ± 0.012, 0/10, 0.000 | 0.453 ± 0.064, 0/10, 0.000 |

**What changed with the physics fix** (main runs, $\sigma_h$ = 0.3 m, 20 seeds; previous version → this version):

| Quantity | Before (state clamp) | After (braking barrier) |
|---|---|---|
| TF Tuned / Tight $C_{eff}^{3D}$ | 0.448 / 0.839 | 0.445 / 0.838 |
| TF Tuned / Tight crown contacts per mission | 19.50 / 17.50 | 19.55 / 17.45 |
| TF Tuned plants sprayed from inside the crown | 0.70 | 0.90 |
| TF floor events | 21.4 / 18.5 clamps per mission; 170 / 218 touchdowns > 1 m/s, up to 2.78 m/s in the sweeps | 0 contacts; barrier active 379.6 / 423.1 steps per mission; lowest altitude 0.3050 m |
| Adaptive, fixed 1.5 m, clears-all | – | unchanged to the printed precision (adaptive Tuned now uses the barrier 1.5 steps per mission) |
| Crown-radius sweep, TF Tuned at 0.25 / 0.5 / 1.0 m | 0.021 / 0.461 / 0.709 | 0.020 / 0.459 / 0.708 |
| Newly reported | – | collision-free missions and safe $C_{eff}^{3D}$ in every table |

**Key findings**:
- **Only the fixed altitude that clears every plant flies collision-free; every other strategy collides in most missions.** Counting any crown contact as a collision, the collision-free missions at $\sigma_h$ = 0.3 m are:
  - clears-all 20/20 with both gain sets;
  - adaptive 5/20 (Tuned) and 7/20 (Tight);
  - fixed 1.5 m 2/20 and 3/20;
  - terrain following 0/20 with both, and 0/10 in every crown-radius and $\sigma_h$ configuration.

  With collisions scored 0, the safe $C_{eff}^{3D}$ is:
  - adaptive 0.190 / 0.305;
  - clears-all 0.170 / 0.165;
  - fixed 1.5 m 0.051 / 0.081;
  - terrain following 0.000.

  Per seed, clears-all beats adaptive on the safe score in 15/20 (Tuned) and 13/20 (Tight) seeds. Adaptive wins only the seeds where it does not collide, and its mean is higher because those missions score 0.760 / 0.871. Neither card strategy is both safe and accurate here.
- **By dose quality alone, per-plant adaptive altitude is the clear winner, as the card expects.** At $\sigma_h$ = 0.3 m it holds the standoff within 4.5 cm (Tuned) and 1.8 cm (Tight) of 0.5 m. It reaches $C_{eff}^{3D}$ = 0.760 / 0.871 and beats both fixed altitudes in 20/20 seeds (+0.332 to +0.706).
  - Its $C_{eff}^{3D}$ is flat across $\sigma_h$ (0.757–0.760), but its safety is not: collision-free missions fall from 10/10 at $\sigma_h$ = 0 to 6/10, 4/10 and then 0/10 at $\sigma_h$ ≥ 0.45, so its safe $C_{eff}^{3D}$ falls from 0.757 to 0.
  - Fixed 1.5 m falls from 0.757 to 0.271 in $C_{eff}^{3D}$ and collides in every mission from $\sigma_h$ = 0.3 m. The clears-all altitude stays collision-free but falls to 0.081.
- **Part of the dose gap is built into the card's dose model, but not all of it.** The standoff factor penalises fixed altitudes directly. Without it, adaptive still wins every seed against the clears-all altitude and 18–20 of 20 against fixed 1.5 m, with smaller margins (0.783 vs 0.718 / 0.765 Tuned; 0.876 vs 0.814 / 0.751 Tight).
- **Card-style lidar terrain following without look-ahead is unsafe and is not a competitive strategy once collisions count.** It tracks the ground (0.5 m) between plants and sees each crown only once it is overhead. It therefore meets every plant from the side: 19.55 (Tuned) / 17.45 (Tight) crown contacts per mission, almost all on the plant it is approaching. Between plants it dives towards the floor so often that the braking barrier is active 379.6 / 423.1 steps per mission.
  - The previous version enforced the floor with an instant stop at up to 2.78 m/s. That stop is physically impossible, and in reality these would have been ground strikes.
  - By dose alone it is level with fixed 1.5 m with Tuned gains (10 / 0 / 10) and close to adaptive with Tight gains (0.838 vs 0.871). That dose-only comparison depends strongly on the crown radius I added (Tuned: 0.020 / 0.459 / 0.708 at 0.25 / 0.5 / 1.0 m).
  - With collisions counted, terrain following scores 0 at every radius and every $\sigma_h$, and loses to adaptive in every seed where adaptive is collision-free.
- **The adaptive plan's collisions are partly a tuning problem (probed).** With Tuned gains it touches 1.25 crowns per mission: 0.65 on the plant being approached and 0.60 on other plants. Of the 0.65, 0.55 happen on descending or level legs and 0.10 on climbs to a taller plant.
  - Raising the vertical derivative gain 2.5× (damping 0.189 → 0.527) removes the descent contacts. Collision-free missions double, 5/20 → 10/20, and the safe $C_{eff}^{3D}$ goes 0.190 → 0.382.
  - With Tight gains: 7/20 → 11/20 collision-free and 0.305 → 0.480 safe. The climb contacts (0.10 / 0.20) remain.
  - Other-plant contacts barely change (0.60 → 0.50, 0.45 → 0.35). Flying at the endpoints' altitude over a taller plant in between is the likely cause (*not probed*).
- **The horizontal gains still matter for the top score.** With the card's Tuned gains, the S063 finding carries over: 40 cm horizontal hover RMS and 19.9 of 20 settle time-outs. That caps adaptive at $C_{eff}^{3D}$ = 0.760; Tight raises it to 0.871.
- **The dose ranking does not depend on the added drift parameters.** It never changes over the nine combinations of wind amplitude and downwash speed. These parameters do not affect the flight, so they do not change the collision counts.

### 3D Field Overview

Seed 0, Tuned PID: adaptive (left) and fixed 1.5 m (right). Green stems are canopy heights; dotted lines are the standoff during spraying:

![Field 3D](./field_3d.png)

### Altitude Time Series

Seed 0, Tuned PID, all four strategies. Brown marks the canopy top of the plant being sprayed; green bands are $h_i$ + 0.5 ± $\sigma_z$:

![Altitude](./altitude_timeseries.png)

### Standoff Deviation per Plant

Mean $|\delta z - 0.5|$ during spraying over 20 seeds, by visit order (log scale):

![Standoff](./standoff_deviation.png)

### Droplet Impacts

Seed 0, Tuned PID: impact points (every 5th spray sample) around each plant, with the $\sigma_{spray}$ circle. The panel titles give this seed's $C_{eff}^{3D}$ and mean offset (figure only, not printed):

![Impacts](./droplet_impacts.png)

### $C_{eff}^{3D}$ vs $\sigma_h$

Left: $C_{eff}^{3D}$ by dose only. Right: safe $C_{eff}^{3D}$, with collision missions scored 0:

![C_eff vs sigma_h](./ceff_vs_sigma_h.png)

### Animation

Lidar terrain following, Tuned PID, seed 0: the drone (red) dips to 0.5 m between plants and climbs at each crown. 169 frames, one every 100 steps (2 s):

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
| Collisions not scored (card) | Any crown contact makes the mission a collision; safe $C_{eff}^{3D}$ = 0 for that mission, shown beside $C_{eff}^{3D}$ everywhere | A collision is a failed mission, not just a lower dose (X4). |
| Terrain-following PID as a safe strategy | Implemented as specified; it collides in every mission | Without look-ahead the lidar sees a crown only once it is overhead. Card note added. |
| Single dose model | Also reported without the standoff factor | Separates the built-in standoff penalty from the drift effect (C9). |

---

## Extensions

1. Look-ahead terrain following: command the maximum canopy height within the next 1–2 m of track (card Extension 1). Without it the strategy collides in every mission.
2. Better-damped vertical gains plus a collision-aware path (climb before approach, clear intermediate plants). The probe shows that damping alone doubles the collision-free missions of the adaptive plan.
3. A 3D tour cost that includes the canopy along each leg (card Extension 5), to avoid transit contacts.
4. Wind-aware nozzle offset (card Extension 2): the mean drift offset is 24–37 cm, comparable to the horizontal hover error with Tight gains.

---

## Related Scenarios

- Base 2D version: [S063 Single-Plant Precision Spraying](../../../../scenarios/04_industrial_agriculture/S063_precision_spraying.md)
- 3D card: [S063 3D Upgrade — Precision Per-Plant Spraying](../../../../scenarios/04_industrial_agriculture/3d/S063_3d_precision_spraying.md)
- Spray coverage: [S067 Spray Coverage Overlap Optimization](../../../../scenarios/04_industrial_agriculture/S067_spray_overlap.md)
- Greenhouse flight: [S064 Greenhouse Interior Precision Flight](../../../../scenarios/04_industrial_agriculture/S064_greenhouse.md)
