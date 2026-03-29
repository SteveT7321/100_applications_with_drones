# S002 3D Upgrade — Evasive Maneuver

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S002 original](../S002_evasive_maneuver.md)

---

## What Changes in 3D

The original S002 fixes both drones at z = 2 m throughout the simulation. The evader's perpendicular-escape velocity is computed only in the x-y plane (`r_hat = r / norm(r)` uses only `[:2]`), and the 90° rotation is a 2D matrix applied to `hat_r_xy`. In true 3D, the evader can rotate its velocity vector around any axis perpendicular to the line-of-sight, and altitude changes become an integral part of the evasion strategy — diving, climbing, or spiralling out of the pursuer's horizontal plane.

---

## Problem Definition

Pursuer (1 drone) vs evader (1 drone) in unconstrained 3D space
(bounded arena $[-8,8]^3$ m).

**Pursuer**: Pure Pursuit or Proportional Navigation Guidance (PNG) in full 3D —
velocity always updated toward the current evader position in 3D.

**Evader**: chooses a 3D evasion direction perpendicular to the 3D line-of-sight vector,
with freedom to also vary altitude:
1. **Horizontal perp** (baseline): same as S002, z held constant — control group
2. **3D perp optimal**: rotate line-of-sight by 90° in the plane that maximises
   lateral displacement in 3D
3. **Helix escape**: maintain constant angular rate around the z-axis while climbing
4. **Dive-and-run**: evader dives to low altitude to exploit pursuer's vertical tracking lag

**Objective**: compare how each 3D evasion tactic extends capture time compared to
the 2D baseline; quantify the altitude excursions and total path length.

---

## Mathematical Model

### Pure Pursuit in 3D

$$\mathbf{v}_P = v_P \cdot \frac{\mathbf{p}_E - \mathbf{p}_P}{\|\mathbf{p}_E - \mathbf{p}_P\|}$$

### 3D Perpendicular Evasion

Line-of-sight vector in 3D: $\hat{\mathbf{r}} = (\mathbf{p}_E - \mathbf{p}_P) / \|\mathbf{p}_E - \mathbf{p}_P\|$

Choose the evasion direction orthogonal to $\hat{\mathbf{r}}$ that lies in the plane spanned by $\hat{\mathbf{r}}$ and $\hat{\mathbf{z}}$:

$$\mathbf{e}_{perp} = \hat{\mathbf{z}} - (\hat{\mathbf{z}} \cdot \hat{\mathbf{r}})\hat{\mathbf{r}}$$

If $\|\mathbf{e}_{perp}\| < \varepsilon$ (evader directly above/below pursuer), fall back to:

$$\mathbf{e}_{perp} = \hat{\mathbf{x}} - (\hat{\mathbf{x}} \cdot \hat{\mathbf{r}})\hat{\mathbf{r}}$$

Evader velocity: $\mathbf{v}_E = v_E \cdot \mathbf{e}_{perp} / \|\mathbf{e}_{perp}\|$

### Helix Escape

$$\mathbf{v}_E(t) = v_E \begin{bmatrix} -\sin(\omega t) \\ \cos(\omega t) \\ v_z / v_E \end{bmatrix}, \quad \omega = v_E \cos\alpha / R_{helix}$$

where $\alpha$ is the helix pitch angle and $R_{helix}$ is the helix radius.
The vertical component $v_z = v_E \sin\alpha$ adds altitude gain; optimal $\alpha \approx 30°$ maximises the 3D path curvature seen by a horizontally biased pursuer.

### 3D Capture Condition

$$\|\mathbf{p}_P - \mathbf{p}_E\| < r_{capture} = 0.15 \text{ m}$$

### Analytical Capture Time (3D Pure Pursuit, constant heading evasion)

For pure pursuit in 3D with $v_P / v_E = k$ and initial relative distance $r_0$:

$$T_{cap} \approx \frac{r_0}{v_P - v_E \cos\theta_0}$$

where $\theta_0$ is the 3D angle between the initial LOS and the evader's velocity.
A helix evasion with pitch angle $\alpha$ satisfies $\cos\theta_0 \approx \sin\alpha$,
increasing $T_{cap}$ compared to the 2D perpendicular case where $\cos\theta_0 = 0$.

---

## Key 3D Additions

- Full 3D line-of-sight direction (not just x-y component)
- 3D perpendicular velocity computation via Gram-Schmidt orthogonalisation
- Helix evasion law with adjustable pitch angle $\alpha$
- Dive-and-run altitude strategy (z drops from 2 m to 0.5 m in 1 s, then straight escape)
- Altitude bounds enforcement: $z \in [0.3, 8]$ m
- 3D trajectory visualization with elevation subplot alongside top-down view

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer speed | 5.0 m/s |
| Evader speed | 3.5 m/s |
| Initial pursuer position | (-2, 0, 2) m |
| Initial evader position | (2, 0, 2) m |
| Helix radius $R_{helix}$ | 1.5 m |
| Helix pitch angle $\alpha$ | 30° |
| Altitude bounds | [0.3, 8] m |
| Capture radius | 0.15 m |
| Arena | $[-8, 8]^3$ m |
| Control frequency | 48 Hz |

---

## Expected Output

- **3D trajectory plot**: all four evasion strategies on the same 3D axes
- **Altitude vs time**: shows z excursions for helix and dive-and-run tactics
- **Capture time bar chart**: horizontal-perp (2D baseline), 3D-perp, helix, dive-and-run
- **LOS angle vs time**: $\theta(t)$ between evader velocity and pursuer LOS — helix maintains non-zero $\theta$ longer

---

## Extensions

1. Optimal helix pitch search: sweep $\alpha \in [0°, 90°]$ and find the angle that maximises capture time
2. Energy-aware 3D evasion: climbing costs extra power — model $P = k_h \cdot v_z^2 + k \cdot v^2$ and find the energy-optimal evasion altitude profile
3. 3D proportional navigation pursuer vs helix evader: does PN outperform pure pursuit?

---

## Related Scenarios

- Original: [S002 2D version](../S002_evasive_maneuver.md)
- Truly 3D reference: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
