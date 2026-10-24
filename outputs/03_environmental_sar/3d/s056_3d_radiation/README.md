# S056 3D Upgrade — Radiation Hotspot Detection

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: The S056 survey is kept: a 500 × 500 m exclusion zone, one point source of known strength, inverse-square dose with air attenuation and background, Gaussian counting noise, a sequential Bayesian posterior, a stop at posterior peak ≥ 0.90 or 80 readings, a 5-waypoint cooldown and a first reading at the launch point. Two things become 3D:
- The source can sit anywhere in $[0, 500]^2 \times [0, 20]$ m (ground to rooftop). It is inferred on a 25 × 25 × 5 voxel posterior (20 × 20 × 4 m voxels, 3125 voxels).
- The drone chooses its altitude. Candidate waypoints are the 625 horizontal cell centres × 5 altitudes {3, 10, 20, 35, 60} m. It flies straight 3D legs at 8 m/s and dwells 5 s per reading.

The absorbed dose is integrated along every leg from the true field. The EIG score is penalised by $\lambda_D(k)\,D_{3D}(\mathbf{w}, \hat{\mathbf{p}}_s^{mean})$ as on the card. Every strategy takes its first reading at the launch point (250, 250, 15) m.

**Strategies** (same 50 sources, same noise draws, same launch reading):

| Strategy | Description |
|----------|-------------|
| **3D EIG** (card 1) | Max of EIG − dose penalty over all 3125 (x, y, z) waypoints, 3D voxel posterior |
| **Fixed-altitude EIG** (card 2) | The same score and posterior, waypoints locked at 15 m |
| **Altitude-sweep lawnmower** (card 3) | A 4 × 4 boustrophedon at each altitude, 60 m down to 3 m (80 readings), 3D voxel posterior |
| **2D baseline** (X5) | The S056 model as written: ground-level source hypotheses on the same 25 × 25 cells, h = 15 m, same EIG rule. Its estimate is taken at z = 0 |
| **Lawnmower at 15 m** (control) | The S056 8 × 10 lawnmower at 15 m, 3D voxel posterior |

Ablations: 3D EIG with $\lambda_0$ = 0 and 0.5 (card 0.05). Sources placed exactly on voxel centres, which removes the model mismatch. Like S056, everything is run at the card's detector noise $\sigma_0$ = 0.04 and at a 10× noisier detector ($\sigma_0$ = 0.4).

**Key question (card)**: Does choosing the altitude localise a 3D source faster or better than a fixed 15 m survey?

---

## Mathematical Model

**Dose rate** (3D slant range, stand-off floor $r_{min}$ = 1 m added):
$$D_{3D}(\mathbf{p}, \mathbf{p}_s) = D_{bg} + \frac{A}{r^2}e^{-\mu r},\qquad r = \max(\|\mathbf{p} - \mathbf{p}_s\|, r_{min})$$

**Measurement** (S056 fix: $D_{3D}$ already contains $D_{bg}$):
$$z_k = D_{3D}(\mathbf{p}_k, \mathbf{p}_s^*) + \varepsilon_k,\qquad \varepsilon_k \sim \mathcal{N}\bigl(0, \sigma_0^2 D_{3D}(\mathbf{p}_k, \mathbf{p}_s^*)\bigr)$$

**Altitude sensitivity** (card formula, checked numerically):
$$\frac{\partial D_{3D}}{\partial z_d} = -A\,(z_d - z_s)\frac{2 + \mu r}{r^4}e^{-\mu r}$$
This is zero at $z_d = z_s$. At $\rho$ = 20 m horizontal offset its magnitude peaks at $|z_d - z_s|$ = 11.54 m (≈ $\rho/\sqrt3$ = 11.55 m). At one fixed altitude $h$, $z_s$ and $2h - z_s$ give identical readings. At 15 m, a source at 12 m and one at 18 m both read 4.970237 µSv/h at 20 m offset.

**Voxel posterior**: uniform prior, $H_0 = \ln 3125$ = 8.047 nats. Log-likelihood update with $\hat\sigma_j = \sigma_0\sqrt{D_{3D}(\mathbf{p}_k, \mathbf{c}_j)}$. MAP voxel estimate.

**EIG and dose penalty**:
$$\text{EIG}(\mathbf{w}) = \tfrac12\log\Bigl(1 + \frac{\sum_j P_k(v_j)[D_{3D}(\mathbf{w},\mathbf{c}_j) - \bar z]^2}{\sigma_0^2\,\bar z(\mathbf{w})}\Bigr),\qquad
\mathbf{w}^* = \arg\max_{\mathbf{w}\notin\mathcal{R}_k}\bigl[\text{EIG}(\mathbf{w}) - \lambda_D(k)\,D_{3D}(\mathbf{w}, \hat{\mathbf{p}}_s^{mean})\bigr]$$
$$\lambda_D(k) = \lambda_0\exp\Bigl(\frac{D_{acc,k} - D_{max}/2}{D_{max}/4}\Bigr)$$
Ties within 10⁻⁹ (relative) are broken by 3D distance and then by index (the S056 fix).

**Dose accumulation**: the true dose rate is sampled every ≤ 2 m along each leg and multiplied by the leg time, plus $D_{3D}\,\Delta t_{int}$ per dwell. A mission stops if $D_{acc} \ge D_{max}$.

**Safe return to home**: the iso-surface $D_{3D} = 1$ µSv/h around the MAP voxel is a sphere of radius 46.92 m. The corridor is a shortest path on the waypoint lattice (plus the launch point) that does not enter this sphere. Inside it, a step may only go to a node of lower estimated dose, so it can leave. It is compared with the straight line home.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Zone / source height range | 500 × 500 m / $z_s \in [0, 20]$ m |
| Voxel grid | 25 × 25 × 5 = 3125 voxels of 20 × 20 × 4 m |
| Altitude set $\mathcal{Z}$ | {3, 10, 20, 35, 60} m; 3125 candidate waypoints |
| Fixed altitude (card 2, 2D baseline) | 15 m |
| Speed / dwell | 8 m/s / 5 s |
| Max readings / stop threshold | 80 / peak ≥ 0.90 |
| Source strength $A$ | 2000 µSv·m²/h |
| $\mu$ / $D_{bg}$ | 2 × 10⁻⁴ m⁻¹ / 0.10 µSv/h |
| $\sigma_0$ | 0.04 (card); 0.4 (10× case, as in S056) |
| Dose budget $D_{max}$ / $\lambda_0$ | 2 mSv / 0.05 (ablations 0 and 0.5) |
| Cooldown | 5 waypoints |
| Target | 3D error ≤ 15 m |
| Monte Carlo (changed) | 50 trials (card 30), source and noise from `default_rng(56000 + trial)` |
| Launch point (added) | (250, 250, 15) m, first reading taken there |
| Stand-off floor $r_{min}$ (added) | 1 m (active in at most 1 reading of 50 missions) |
| Dose integration step (added) | ≤ 2 m along each leg |
| Altitude-sweep lattice (added) | 4 × 4 per altitude layer, layers 60 → 3 m |
| Iso-dose for the corridor | 1 µSv/h (radius 46.92 m) |
| Demo source (added) | (175, 310, 13) m |
| Error band | 95 % bootstrap of the RMS (1000 resamples) |

---

## Implementation

```
src/03_environmental_sar/3d/s056_3d_radiation.py
```

```bash
conda activate drones
python src/03_environmental_sar/3d/s056_3d_radiation.py
```

The dose matrices [candidate waypoint × hypothesis] are computed once (3125 × 3125 for the 3D EIG), so each EIG map costs two matrix–vector products. Runtime ≈ 1–1.5 min (58 s, 62 s, and 79 s single-threaded). Three runs (two default, one with `OMP/OPENBLAS/MKL_NUM_THREADS=1`) printed byte-identical output and wrote byte-identical PNG and GIF files.

---

## Results

### Field checks

| Altitude above a ground source | Dose directly above | Dose 20 m off-centre |
|------|------:|------:|
| 3 m | 222.19 µSv/h | 4.970 µSv/h |
| 10 m | 20.06 µSv/h | 4.082 µSv/h |
| 20 m | 5.08 µSv/h | 2.586 µSv/h |
| 35 m | 1.72 µSv/h | 1.321 µSv/h |
| 60 m | 0.65 µSv/h | 0.594 µSv/h |

### Monte Carlo (50 sources uniform in the volume; same draws for all strategies)

**σ0 = 0.04 (card)**

| Strategy | RMS 3D | Median / max | ≤ 15 m | RMS horiz. | RMS vert. | Readings | Converged | Path | Time | Mean dose |
|----------|------:|------:|:---:|------:|------:|------:|:---:|------:|------:|------:|
| 3D EIG | 17.5 m | 8.7 / 75.9 m | 42/50 | 16.1 m | 6.7 m | 5.6 ± 1.4 | 50/50 | 848 m | 134 s | 0.059 µSv |
| **Fixed-altitude EIG** | **12.3 m** | 10.6 / 35.9 m | 42/50 | 9.8 m | 7.5 m | 5.1 ± 1.5 | 50/50 | 800 m | 126 s | 0.048 µSv |
| Altitude-sweep lawnmower | 12.6 m | 9.2 / 32.9 m | 38/50 | 11.6 m | 5.0 m | 51.6 ± 25.4 | 30/50 | 5957 m | 1003 s | 0.053 µSv |
| 2D baseline (S056 model) | 22.7 m | 14.6 / 121.2 m | 26/50 | 19.5 m | 11.7 m | 4.7 ± 1.3 | 50/50 | 748 m | 117 s | 0.028 µSv |
| Lawnmower at 15 m | 12.2 m | 11.5 / 20.5 m | 37/50 | 8.6 m | 8.7 m | 45.7 ± 24.3 | 45/50 | 2500 m | 541 s | 0.046 µSv |
| 3D EIG, $\lambda_0$ = 0 | 19.8 m | 8.9 / 75.9 m | 40/50 | 18.7 m | 6.6 m | 5.5 ± 1.5 | 50/50 | 849 m | 134 s | 0.196 µSv |
| 3D EIG, $\lambda_0$ = 0.5 | 14.0 m | 10.1 / 39.0 m | 35/50 | 11.1 m | 8.4 m | 5.8 ± 1.6 | 50/50 | 880 m | 139 s | 0.071 µSv |

**σ0 = 0.4 (10×)**

| Strategy | RMS 3D | Median / max | ≤ 15 m | RMS horiz. | RMS vert. | Readings | Converged | Path | Time | Mean dose |
|----------|------:|------:|:---:|------:|------:|------:|:---:|------:|------:|------:|
| 3D EIG | 11.4 m | 8.4 / 23.6 m | 41/50 | 9.2 m | 6.7 m | 15.8 ± 7.9 | 50/50 | 3480 m | 514 s | 0.188 µSv |
| **Fixed-altitude EIG** | **10.3 m** | 9.1 / 20.6 m | 46/50 | 7.8 m | 6.7 m | 23.2 ± 22.1 | 44/50 | 4121 m | 631 s | 0.117 µSv |
| Altitude-sweep lawnmower | 31.7 m | 17.3 / 95.8 m | 20/50 | 30.8 m | 7.6 m | 78.4 ± 7.9 | 2/50 | 9010 m | 1518 s | 0.103 µSv |
| 2D baseline (S056 model) | 16.5 m | 15.8 / 39.1 m | 23/50 | 11.6 m | 11.7 m | 14.2 ± 7.8 | 50/50 | 2956 m | 440 s | 0.074 µSv |
| Lawnmower at 15 m | 16.2 m | 13.7 / 36.3 m | 28/50 | 13.7 m | 8.5 m | 74.2 ± 15.2 | 10/50 | 3940 m | 864 s | 0.066 µSv |
| 3D EIG, $\lambda_0$ = 0 | 10.4 m | 8.5 / 23.3 m | 44/50 | 8.3 m | 6.3 m | 16.0 ± 7.7 | 50/50 | 3418 m | 507 s | 0.258 µSv |
| 3D EIG, $\lambda_0$ = 0.5 | 10.7 m | 8.4 / 22.7 m | 41/50 | 8.5 m | 6.4 m | 19.5 ± 8.8 | 50/50 | 4218 m | 625 s | 0.190 µSv |

**Head-to-head on identical sources** (wins / ties / losses for the first strategy):

| σ0 | Pair | Lower 3D error | Lower horiz. error | Fewer readings | RMS 3D ratio | Mean-readings ratio |
|----|------|:---:|:---:|:---:|:---:|:---:|
| 0.04 | 3D EIG vs fixed-altitude EIG | 24 / 6 / 20 | 7 / 35 / 8 | 6 / 22 / 22 | 1.42 | 1.10 |
| 0.04 | 3D EIG vs altitude-sweep lawnmower | 21 / 13 / 16 | 12 / 32 / 6 | 50 / 0 / 0 | 1.38 | 0.11 |
| 0.04 | Fixed-altitude EIG vs 2D baseline | 43 / 0 / 7 | 12 / 31 / 7 | 8 / 21 / 21 | 0.54 | 1.08 |
| 0.04 | Altitude-sweep vs 15 m lawnmower | 16 / 15 / 19 | 5 / 31 / 14 | 18 / 8 / 24 | 1.03 | 1.13 |
| 0.4 | 3D EIG vs fixed-altitude EIG | 26 / 5 / 19 | 3 / 36 / 11 | 24 / 11 / 15 | 1.10 | 0.68 |
| 0.4 | 3D EIG vs altitude-sweep lawnmower | 32 / 4 / 14 | 28 / 16 / 6 | 50 / 0 / 0 | 0.36 | 0.20 |
| 0.4 | Fixed-altitude EIG vs 2D baseline | 46 / 0 / 4 | 18 / 29 / 3 | 11 / 5 / 34 | 0.63 | 1.64 |
| 0.4 | Altitude-sweep vs 15 m lawnmower | 19 / 4 / 27 | 11 / 14 / 25 | 2 / 38 / 10 | 1.96 | 1.06 |

**Diagnostics**

| σ0 | Quantity | 3D EIG | Fixed-altitude EIG | 2D baseline |
|----|----------|------:|------:|------:|
| 0.04 | Gross errors (> 30 m) | 2 | 1 | 1 |
| 0.04 | Median posterior on the source's own voxel | 6.47 × 10⁻²²¹ | 2.31 × 10⁻¹²⁹ | 1.00 (2D cell) |
| 0.04 | Distinct altitudes per mission | 1.48 | 1.00 | 1.00 |
| 0.4 | Gross errors (> 30 m) | 0 | 0 | 1 |
| 0.4 | Median posterior on the source's own voxel | 3.79 × 10⁻⁷ | 4.78 × 10⁻¹⁰ | 0.987 (2D cell) |
| 0.4 | Distinct altitudes per mission | 1.32 | 1.00 | 1.00 |

**Altitude use of the 3D EIG** (readings per mission after the launch reading):

| σ0 | Variant | 3 m | 10 m | 20 m | 35 m | 60 m | Share at 3 m |
|----|---------|---:|---:|---:|---:|---:|---:|
| 0.04 | $\lambda_0$ = 0.05 (card) | 4.02 | 0.50 | 0.10 | 0.00 | 0.00 | 0.870 |
| 0.04 | $\lambda_0$ = 0 | 3.92 | 0.48 | 0.14 | 0.00 | 0.00 | 0.863 |
| 0.04 | $\lambda_0$ = 0.5 | 4.02 | 0.44 | 0.30 | 0.00 | 0.00 | 0.845 |
| 0.4 | $\lambda_0$ = 0.05 (card) | 14.42 | 0.34 | 0.02 | 0.00 | 0.00 | 0.976 |
| 0.4 | $\lambda_0$ = 0 | 14.32 | 0.58 | 0.06 | 0.00 | 0.00 | 0.957 |
| 0.4 | $\lambda_0$ = 0.5 | 16.90 | 0.72 | 0.60 | 0.16 | 0.14 | 0.913 |

**Vertical probe**: trials where the horizontal voxel is correct, and how many of these miss the height by more than one voxel or land on the mirror side of 15 m:

| σ0 | Strategy | Horizontal voxel correct | Vertical error > 1 voxel | Mirror side of 15 m | RMS vert. (these trials) |
|----|----------|:---:|:---:|:---:|---:|
| 0.04 | 3D EIG | 46/50 | 21 | 5 | 6.60 m |
| 0.04 | Fixed-altitude EIG | 45/50 | 25 | 9 | 7.42 m |
| 0.04 | Altitude-sweep lawnmower | 39/50 | 15 | 1 | 4.83 m |
| 0.4 | 3D EIG | 46/50 | 17 | 8 | 5.84 m |
| 0.4 | Fixed-altitude EIG | 49/50 | 27 | 10 | 6.53 m |
| 0.4 | Altitude-sweep lawnmower | 24/50 | 13 | 6 | 6.70 m |

**On-grid ablation** (each source moved to its voxel centre, so there is no model mismatch):

| σ0 | Strategy | RMS 3D | Exact voxel | Readings | Converged | 3D EIG vs fixed: fewer / equal / more readings | Mean-readings ratio |
|----|----------|------:|:---:|------:|:---:|:---:|:---:|
| 0.04 | 3D EIG | 0.0 m | 50/50 | 5.7 ± 1.3 | 50/50 | 13 / 26 / 11 | 0.89 |
| 0.04 | Fixed-altitude EIG | 0.6 m | 49/50 | 6.4 ± 3.0 | 50/50 | | |
| 0.4 | 3D EIG | 0.8 m | 48/50 | 16.8 ± 8.4 | 50/50 | 23 / 18 / 9 | 0.82 |
| 0.4 | Fixed-altitude EIG | 0.8 m | 48/50 | 20.6 ± 14.8 | 48/50 | | |

**Mean voxel entropy (nats) after $k$ readings**:

| σ0 | Strategy | k = 1 | 2 | 3 | 5 | 10 |
|----|----------|---:|---:|---:|---:|---:|
| 0.04 | 3D EIG | 7.02 | 5.75 | 4.21 | 1.17 | 0.01 |
| 0.04 | Fixed-altitude EIG | 7.02 | 5.63 | 3.92 | 0.89 | 0.03 |
| 0.04 | Altitude-sweep lawnmower | 7.02 | 6.13 | 5.45 | 4.40 | 2.88 |
| 0.04 | Lawnmower at 15 m | 7.02 | 6.25 | 5.96 | 5.25 | 4.10 |
| 0.4 | 3D EIG | 7.94 | 7.76 | 7.63 | 6.61 | 4.16 |
| 0.4 | Fixed-altitude EIG | 7.94 | 7.76 | 7.61 | 6.67 | 4.26 |
| 0.4 | Altitude-sweep lawnmower | 7.94 | 7.82 | 7.54 | 7.23 | 6.57 |
| 0.4 | Lawnmower at 15 m | 7.94 | 7.85 | 7.70 | 7.18 | 6.51 |

**Dose and return to home** (after the 3D EIG missions):

| σ0 | Largest mission dose (any strategy) | Start inside est. iso-surface | Straight line crosses it from outside | Length straight / corridor (ratio) | True dose straight / corridor (mean) | Corridor lower dose | Max true dose after exit, straight / corridor |
|----|------:|:---:|:---:|------:|------:|:---:|------:|
| 0.04 | 2.937 µSv (0.147 % of $D_{max}$) | 49/50 | 0/50 | 207 / 219 m (1.06) | 13.93 / 14.60 nSv | 26/50 | 3.958 / 23.876 µSv/h |
| 0.4 | 3.167 µSv (0.158 % of $D_{max}$) | 50/50 | 0/50 | 205 / 217 m (1.06) | 24.59 / 22.59 nSv | 29/50 | 0.996 / 1.000 µSv/h |

No mission was stopped by the dose budget.

### Demo source (175, 310, 13) m

| σ0 | Strategy | Readings | Peak | MAP voxel | 3D error | Path | Altitudes used |
|----|----------|---:|---:|---|---:|---:|---|
| 0.04 | 3D EIG | 4 | 1.000 | (170, 310, 2) | 12.1 m | 212 m | 3 |
| 0.04 | Fixed-altitude EIG | 4 | 1.000 | (190, 310, 2) | 18.6 m | 292 m | 15 |
| 0.04 | Altitude-sweep lawnmower | 27 | 0.903 | (170, 310, 14) | 5.1 m | 3164 m | 35, 60 |
| 0.04 | 2D baseline | 4 | 1.000 | (190, 310, 0) | 19.8 m | 360 m | 15 |
| 0.04 | Lawnmower at 15 m | 28 | 1.000 | (170, 310, 2) | 12.1 m | 1611 m | 15 |
| 0.4 | 3D EIG | 13 | 1.000 | (170, 310, 2) | 12.1 m | 1271 m | 3, 10 |
| 0.4 | Fixed-altitude EIG | 9 | 0.992 | (190, 310, 2) | 18.6 m | 1142 m | 15 |
| 0.4 | Altitude-sweep lawnmower | 80 | 0.631 | (170, 310, 14) | 5.1 m | 9196 m | all five |
| 0.4 | 2D baseline | 10 | 1.000 | (170, 310, 0) | 13.9 m | 1186 m | 15 |
| 0.4 | Lawnmower at 15 m | 80 | 0.207 | (170, 310, 10) | 5.8 m | 4231 m | 15 |

Return home on the σ0 = 0.4 demo: straight 100 m / 39.62 nSv, corridor 105 m / 33.03 nSv. The drone starts inside the estimated iso-surface.

**Key findings**:
- **Modelling the source height matters most (2D-baseline comparison, X5).** The S056 model (ground source, 2D posterior) and the fixed-altitude EIG fly the same 15 m survey with the same EIG rule. Only the posterior differs. With the 3D voxel posterior the RMS 3D error drops from 22.7 to 12.3 m (ratio 0.54) at the card's noise and from 16.5 to 10.3 m (0.63) at 10× noise; it wins 43 and 46 of 50 trials. Part of that gap is by construction, because the 2D model always answers z = 0 (RMS vertical 11.7 m). The horizontal error is also lower, though: 9.8 vs 19.5 m and 7.8 vs 11.6 m RMS, and per trial it is lower in 12 vs 7 and 18 vs 3 trials. Assuming a ground source distorts the horizontal fit when the source is elevated.
- **Freeing the altitude does not improve accuracy; the card's expectation does not hold.** The 3D EIG has a higher RMS 3D error than the fixed-altitude EIG: 17.5 vs 12.3 m (ratio 1.42) at σ0 = 0.04 and 11.4 vs 10.3 m (1.10) at σ0 = 0.4. Per trial the result is mixed: lower 3D error in 24 and 26 trials, higher in 20 and 19. Horizontally the 3D EIG is worse more often than better (7 vs 8, 3 vs 11). Its entropy also does not fall fastest. At σ0 = 0.04 the fixed-altitude EIG is lower after 2, 3 and 5 readings (0.89 vs 1.17 nats at k = 5); at σ0 = 0.4 the two are within 0.1 nats (6.61 vs 6.67 at k = 5, 4.16 vs 4.26 at k = 10).
- **The altitude choice does save readings with the noisier detector.** At σ0 = 0.4 the 3D EIG needs 15.8 instead of 23.2 readings (ratio 0.68; fewer in 24 trials, more in 15) and converges in 50/50 instead of 44/50 missions. At σ0 = 0.04 it needs slightly more (5.6 vs 5.1; ratio 1.10).
- **The 3D EIG almost never uses the high altitudes.** 87.0 % (σ0 = 0.04) and 97.6 % (σ0 = 0.4) of its readings after launch are at 3 m, and none are at 35 or 60 m with the card's $\lambda_0$. The card's picture of high passes for context followed by a dive does not appear. Close to the ground the spread of predicted doses across hypotheses is largest, which is what the EIG score rewards.
- **The residual 10–20 m errors come from the voxel grid, not from the strategies.** With every source moved to its voxel centre, both EIG strategies find the exact voxel (50/50 and 49/50 at σ0 = 0.04; 48/50 each at σ0 = 0.4; RMS ≤ 0.8 m). With continuous sources the posterior on the source's own voxel ends at a median of 6.47 × 10⁻²²¹ (3D EIG, σ0 = 0.04): near-perfect readings of an off-centre source rule out the correct voxel, as in S056 but with coarser 20 m voxels. Without that mismatch, the altitude freedom saves readings at both noise levels: ratio 0.89 (13 fewer / 11 more) and 0.82 (23 / 9). *Hypothesis, not probed directly*: the 3D EIG is hit harder by the mismatch at σ0 = 0.04 because its 3 m readings sit where the dose changes fastest, so a 10 m off-centre source produces larger model errors there.
- **The mirror ambiguity of a single altitude is real but not the limiting factor.** Among trials with the correct horizontal voxel, the fixed-altitude EIG lands on the mirror side of 15 m in 9 (σ0 = 0.04) and 10 (σ0 = 0.4) trials, against 5 and 8 for the 3D EIG. Voxel centres (2, 6, 10, 14, 18 m) are not symmetric about 15 m, so on-grid sources are resolved exactly even at one altitude.
- **The $\lambda_0$ sweep changes the result by a few trials.** At σ0 = 0.04 the RMS 3D error is 19.8 / 17.5 / 14.0 m for $\lambda_0$ = 0 / 0.05 / 0.5. At σ0 = 0.4 it is 10.4 / 11.4 / 10.7 m. A larger $\lambda_0$ adds readings (19.5 vs 15.8 at σ0 = 0.4) and hardly moves the altitude mix (84.5–97.6 % at 3 m). At none of the three values does the 3D EIG reach the fixed-altitude EIG's RMS; the closest is $\lambda_0$ = 0 at σ0 = 0.4 (10.4 vs 10.3 m).
- **Sweeping the lawnmower through all altitudes hurts it.** Compared with the S056 lawnmower at 15 m, the RMS 3D error is about equal at σ0 = 0.04 (12.6 vs 12.2 m) and much worse at σ0 = 0.4 (31.7 vs 16.2 m, ratio 1.96). Readings at 35 and 60 m carry little information: the dose above a ground source is 1.72 and 0.65 µSv/h there. As in S056, the slow lawnmower is competitive in accuracy at the card's noise (12.6 m, against 17.5 m for the 3D EIG), but the 3D EIG needs only 0.11× its readings (5.6 vs 51.6).
- **The dose budget never binds.** The largest dose of any mission is 2.937 µSv (σ0 = 0.04) and 3.167 µSv (σ0 = 0.4), 0.15–0.16 % of the 2 mSv budget, so $\lambda_D$ stays at its floor $\lambda_0 e^{-2}$. A 5 s dwell 3 m above a ground source costs only 222.19 µSv/h × 5 s.
- **The safe corridor brings no measurable benefit.** The drone ends inside the estimated 1 µSv/h sphere in 49/50 and 50/50 missions, because it finishes near the source. The straight line home never crosses the sphere from outside. The corridor is 6 % longer, and the true dose on the way home is tens of nanosieverts either way; it is lower on the corridor in only 26 and 29 of 50 trials. When the MAP voxel is wrong (σ0 = 0.04), the corridor can pass closer to the true source than the straight line: the maximum true dose after leaving the estimated sphere is 23.876 vs 3.958 µSv/h.

### 3D Trajectory

The 3D EIG on the demo source at both noise levels, coloured by altitude. Gold = true source, red = MAP voxel, orange mesh = true 1 µSv/h iso-surface. The axes are not to scale, so the sphere looks elongated. The σ0 = 0.04 mission flies only at 3 m; the σ0 = 0.4 mission uses 3 m and 10 m:

![3D trajectory](./trajectory_3d.png)

### Altitude Profile and Altitude Use

Left: altitude vs mission time on the demo. Middle and right: mean readings per mission at each altitude over the 50 trials for the 3D EIG and the altitude-sweep lawnmower:

![Altitude profile](./altitude_profile.png)

### Voxel Posterior Slices

3D EIG, demo source, σ0 = 0.4. Top: horizontal slice through the source's voxel layer (12–16 m). Bottom: vertical slice at x = 170 m. The posterior is shown after 1, 10 and 13 readings; the mission stops after 13, so the card's snapshots at $k$ = 30 and 60 do not exist. After 10 readings the column above the source still carries mass at several heights. The final MAP voxel is at z = 2 m (3D error 12.1 m):

![Posterior slices](./posterior_slices.png)

### Entropy and RMS Error Curves

Mean voxel entropy (3125-voxel posteriors only; the 2D baseline has 625 cells) and RMS 3D MAP error with 95 % bootstrap bands, held after a mission stops:

![Entropy and error](./entropy_error_curves.png)

### Cumulative Dose

Mean (solid) and maximum (dotted) cumulative dose vs reading index, against the 2 mSv budget:

![Dose profile](./dose_profile.png)

### Safe Return Corridor

Demo, σ0 = 0.4: straight return vs the corridor around the estimated 1 µSv/h iso-surface:

![Safe corridor](./safe_corridor.png)

### Animation

Rotating view of the top-300 voxels (opacity ∝ posterior) and the 3D EIG path on the demo source at σ0 = 0.4, one frame per reading (13 frames):

![Animation](./animation.gif)

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| $\sigma_{rad} = \sigma_0\sqrt{D_{3D} + D_{bg}}$ (also in $\hat\sigma_j$ and $\bar\sigma^2_{rad}$) | $\sigma_0\sqrt{D_{3D}}$ | $D_{3D}$ already contains $D_{bg}$; same fix as S056. Card corrected. |
| "The gradient $\partial D/\partial z_d$ is maximised when $z_d \approx z_s$" | It is zero at $z_d = z_s$ and peaks near $\|z_d - z_s\| = \rho/\sqrt3$ (printed: 11.54 m at $\rho$ = 20 m) | Follows from the factor $(z_d - z_s)$ in the card's own formula. Card corrected; the mirror ambiguity at a fixed altitude was added to the card. |
| 30 Monte Carlo trials | 50, per-trial seeds | Matches the S056 2D Monte Carlo; win counts need the larger sample. |
| Waypoint chosen before the first reading | First reading at (250, 250, 15) m for every strategy | S056 fix: a common first reading keeps the comparison fair. |
| `argmax` of the score | Relative 10⁻⁹ ties broken by 3D distance, then index | S056 fix for BLAS-dependent tie-breaking (X1). |
| Inverse-square law with no floor | $r \ge r_{min}$ = 1 m | Avoids a singular dose if a waypoint coincides with the source. Active in at most 1 reading per 50 missions. |
| Trapezoid of the two leg end points for the transit dose | True dose rate sampled every ≤ 2 m along the leg | A leg that passes over the source between two distant end points would be under-counted by the trapezoid. |
| "Boustrophedon at each altitude level" | 4 × 4 lattice per layer, 60 → 3 m, 80 readings in total | Fits the 80-reading budget. The S056 8 × 10 lawnmower at 15 m was added as a control. |
| Three strategies | Plus the 2D baseline (S056 model) and the 15 m lawnmower | The X5 2D-baseline comparison and a lawnmower control. |
| — | $\lambda_0$ ablations (0, 0.5) and an on-grid source ablation | Sensitivity of the conclusions to the penalty, and a direct probe of the voxel-mismatch explanation. |
| Corridor "re-routed through the 3D field" | Shortest lattice path that does not enter the estimated 1 µSv/h sphere, except to leave it | The drone almost always ends inside the sphere, so the corridor must allow an exit. |
| Posterior slices at $k$ = 1, 10, 30, 60 | $k$ = 1, 10 and the final reading 13 | The demo mission stops after 13 readings. |
| Dose budget keeps the drone within 2 mSv | Implemented; it never binds (max 0.158 % of the budget) | The card's numbers make the budget 3 orders of magnitude larger than any mission dose. |
| Building-mounted source | Point source in free air, no structure | The card has no geometry for the building; low passes under an elevated source are therefore unobstructed. |

---

## Extensions

1. Fix the voxel-mismatch problem first: inflate the likelihood variance by the within-voxel dose spread, or use a particle filter (card Extension 5). The on-grid ablation shows this mismatch, not the altitude policy, sets the error.
2. Re-run the altitude comparison with that robust likelihood to see whether the 3D EIG's reading savings (ratio 0.82–0.89 on-grid) survive.
3. A realistic dose budget (e.g. a stronger source or a sensitive payload) so that $\lambda_D$ actually rises during the mission.
4. Unknown source strength (card Extension 2): altitude diversity should matter more there, because strength and distance trade off at one altitude.
5. Occluding building geometry for elevated sources (card Extension 1).

---

## Related Scenarios

- Base 2D version: [S056 Nuclear Plant Leak Detection (Radiation Hotspot Localization)](../../../../scenarios/03_environmental_sar/S056_radiation.md)
- 3D card: [S056 3D Upgrade — Radiation Hotspot Detection](../../../../scenarios/03_environmental_sar/3d/S056_3d_radiation.md)
- Bayesian search: [S042 Missing Person Localization](../../../../scenarios/03_environmental_sar/S042_missing_person.md)
- Field mapping: [S045 Chemical Plume Tracing](../../../../scenarios/03_environmental_sar/S045_plume_tracing.md), [S055 Coastline Oil Spill Tracking](../../../../scenarios/03_environmental_sar/S055_oil_spill.md)
