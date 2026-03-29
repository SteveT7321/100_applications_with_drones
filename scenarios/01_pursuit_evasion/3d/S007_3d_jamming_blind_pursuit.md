# S007 3D Upgrade — Jamming & Blind Pursuit

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S007 original](../S007_jamming_blind_pursuit.md)

---

## What Changes in 3D

The original S007 holds both drones at z = 2 m. Dead-reckoning drift accumulates only in x and y, while z stays constant. In 3D, the IMU must integrate all three axes. Vertical dead-reckoning error is especially dangerous because altitude constraints (ground collision, airspace ceiling) mean that uncorrected z-drift can lead to a crash even if horizontal error is tolerable. The evader can exploit jam windows by executing a sudden altitude change — a manoeuvre that the pursuer cannot detect or compensate during the blind period.

---

## Problem Definition

**Setup**: The pursuer's GPS is intermittently jammed. During jam windows, it integrates the last known 3D velocity (dead reckoning) with full 3D IMU noise on all axes. The evader can optionally change altitude during jam intervals to exploit the pursuer's blind period.

**Comparison**:
1. **Dead reckoning (3D)**: integrate last known 3D velocity with 3D drift noise
2. **Barometer-assisted dead reckoning**: z is corrected by an independent altimeter; only x-y drift accumulates
3. **Freeze on jam**: pursuer hovers in place during jam (no motion, no drift)
4. **Baseline**: perfect 3D GPS

**Evader tactics**:
- Straight horizontal escape (same as S007)
- Altitude jump during jam: climb or dive 2 m while pursuer is blind

---

## Mathematical Model

### 3D Dead-Reckoning Estimate

During jam window $t \in [t_j,\; t_j + T_{jam}]$:

$$\hat{\mathbf{p}}_P(t) = \mathbf{p}_P(t_j) + \mathbf{v}_{last} \cdot (t - t_j) + \boldsymbol{\varepsilon}(t)$$

where $\boldsymbol{\varepsilon}(t) = [\varepsilon_x, \varepsilon_y, \varepsilon_z]^T$ with independent components:

$$\varepsilon_k(t) \sim \mathcal{N}\!\left(0,\; \sigma_{drift,k}^2 \cdot (t - t_j)\right)$$

IMU drift is anisotropic: horizontal drift $\sigma_{drift,xy}$ and vertical drift $\sigma_{drift,z}$ (barometric altitude is typically more accurate):

$$\sigma_{drift,xy} = 0.05 \text{ m/s}, \quad \sigma_{drift,z} = 0.02 \text{ m/s (bare IMU)} \text{ or } 0.005 \text{ m/s (baro-aided)}$$

### Altitude Safety Constraint

If the estimated z-position falls below ground level ($\hat{z} < z_{min} = 0.3$ m),
the pursuer commands a hover climb until GPS is restored, regardless of jam status.

### Evader Altitude-Jump Tactic

At the start of each jam window the evader immediately changes altitude:

$$z_E(t_j^+) = z_E(t_j) + \Delta z_{jump}, \quad \Delta z_{jump} \in \{-2, +2\} \text{ m}$$

The pursuer's dead-reckoning estimate maintains $\hat{z}_P \approx z_E(t_j)$,
so the 3D position error on GPS restoration is:

$$\|\hat{\mathbf{p}}_P(t_j + T_{jam}) - \mathbf{p}_P(t_j + T_{jam})\| \approx \sqrt{e_{xy}^2 + \Delta z_{jump}^2}$$

### 3D Position Estimation Error

$$e(t) = \|\hat{\mathbf{p}}_P(t) - \mathbf{p}_P(t)\|$$

---

## Key 3D Additions

- 3D IMU dead-reckoning with anisotropic noise ($\sigma_z \ne \sigma_{xy}$)
- Barometer-assisted variant: altitude corrected independently, reducing z-drift
- Ground collision check during dead-reckoning: trigger hover-climb if $\hat{z} < z_{min}$
- Evader altitude-jump evasion tactic during jam windows
- 3D trajectory visualisation with altitude subplot highlighting blind periods

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Jam period | 3.0 s |
| Jam duration | 1.0 s (33% duty cycle) |
| Horizontal IMU drift $\sigma_{xy}$ | 0.05 m/s |
| Vertical IMU drift $\sigma_z$ (IMU-only) | 0.02 m/s |
| Vertical IMU drift $\sigma_z$ (baro-aided) | 0.005 m/s |
| Pursuer speed | 5 m/s |
| Evader speed | 3 m/s |
| Evader altitude jump $\Delta z$ | ±2 m per jam window |
| Initial pursuer position | (-3, 0, 2) m |
| Initial evader position | (3, 0, 2) m |
| Altitude bounds | [0.3, 8] m |

---

## Expected Output

- **3D trajectory plot**: four strategies with altitude profiles; highlight jam windows as shaded bands
- **3D position error vs time**: spike comparison for IMU-only, baro-aided, and freeze strategies
- **Altitude subplot**: shows pursuer z-drift vs true evader z (especially during altitude-jump tactic)
- **Capture time table**: all strategy combinations (4 pursuit modes × 2 evader tactics)

---

## Extensions

1. Kalman filter with IMU + barometer sensor fusion: reduce drift in all axes
2. Adversarial jammer: activates only when pursuer is closing in (within 1.5 m) — forces worst-case jamming at critical moment
3. 3D terrain below (e.g., sloped ground): altitude safety constraint becomes terrain-following during dead-reckoning

---

## Related Scenarios

- Original: [S007 2D version](../S007_jamming_blind_pursuit.md)
- Truly 3D reference: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
