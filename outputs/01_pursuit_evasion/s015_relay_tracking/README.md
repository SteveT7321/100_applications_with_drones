# S015 Relay Tracking

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: Three stationary sensor drones arranged in an equilateral triangle measure bearing-only (angle) observations of a moving target that orbits through the sensor zone. A zone-based handoff selects the two highest-SNR sensors; two-line triangulation estimates the target position. Estimation error spikes at handoff transitions when the two bearing lines become nearly parallel.

**Key question**: How does relay tracking manage continuous coverage across sensor zones, and where does accuracy degrade?

---

## Mathematical Model

### SNR Model (Inverse Distance)

$$SNR_i = \max\!\left(0,\; 1 - \frac{\|\mathbf{p}_T - \mathbf{s}_i\|}{R_{zone}}\right)$$

### Noisy Bearing Measurement

$$\theta_i = \text{atan2}(y_T - s_{iy},\; x_T - s_{ix}) + \mathcal{N}(0, \sigma_\theta)$$

### Two-Line Triangulation

Intersect bearing rays from sensors i and j:

$$\begin{bmatrix} \tan\theta_i & -1 \\ \tan\theta_j & -1 \end{bmatrix} \begin{bmatrix} x \\ y \end{bmatrix} = \begin{bmatrix} \tan\theta_i \cdot s_{ix} - s_{iy} \\ \tan\theta_j \cdot s_{jx} - s_{jy} \end{bmatrix}$$

System is near-singular (det ≈ 0) when bearing lines are nearly parallel → error spikes.

### Target Path

Circular orbit of radius 3 m centred at origin:

$$x(t) = r\cos(\omega t),\quad y(t) = r\sin(\omega t),\quad \omega = V_T / r \approx 0.667\;\text{rad/s}$$

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Sensor positions | (0,4,3), (−3.46,−2,3), (3.46,−2,3) m — equilateral triangle |
| Coverage radius R_zone | 8.0 m |
| Bearing noise σ_θ | 0.05 rad (~3°) |
| Target speed | 2.0 m/s |
| Orbit radius | 3.0 m |
| Orbit period | ≈ 9.4 s |
| Handoff SNR threshold | 0.4 |
| Simulation time | 20 s |
| dt | 0.05 s |

---

## Implementation

```
src/01_pursuit_evasion/s015_relay_tracking.py     # Main simulation (no DroneBase needed)
```

```bash
conda activate drones
python src/01_pursuit_evasion/s015_relay_tracking.py
```

---

## Results

| Metric | Value |
|--------|-------|
| Handoff events | **7** (at t = 0.80, 3.95, 7.10, 10.25, 13.40, 16.50, 19.65 s) |
| Mean triangulation error | **1.74 m** |
| Max triangulation error | **65.38 m** |

**Key Findings**:
- The target completes ~2.1 circular orbits, triggering a handoff every ~3.15 s as it passes between adjacent sensor zones — matching the theoretical 120° spacing of the equilateral sensor triangle.
- Outside handoff transitions, error remains below 0.5 m when two sensors observe the target from well-separated angles.
- Error spikes up to 65 m occur precisely at handoff moments when the newly active sensor pair's bearing lines are nearly parallel, making the triangulation matrix nearly singular.
- The equilateral triangle sensor layout ensures uniform coverage throughout the orbit: SNR never drops below 0.55 at any mid-zone point, eliminating blind spots.

**Top-Down Tracking Overview** (dots = triangulation estimates, coloured by active sensor):

![Tracking Overview](tracking_overview.png)

**Estimation Error & Active Sensor Count vs Time**:

![Error and Coverage](error_and_coverage.png)

**Sensor SNR vs Time** (handoff events marked):

![SNR Over Time](snr_over_time.png)

**Animation**:

![Animation](animation.gif)

---

## Extensions

1. Add Kalman filter to smooth triangulation estimates at handoffs
2. Optimise sensor placement for minimum peak error across the target path
3. Three-sensor triangulation with least-squares to eliminate singularities

---

## Related Scenarios

- Prerequisites: [S008](../../scenarios/01_pursuit_evasion/S008_stochastic_pursuit.md), [S012](../../scenarios/01_pursuit_evasion/S012_relay_pursuit.md)
- Follow-ups: [S016](../../scenarios/01_pursuit_evasion/S016_airspace_defense.md)
