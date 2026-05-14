# S027 3D Upgrade — Aerial Refueling Relay

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S027 original](../S027_aerial_refueling_relay.md)

---

## What Changes in 3D

The original S027 keeps both drones at the same fixed altitude throughout: the Receiver flies a flat
2-D route from base to target, the Tanker holds a flat loiter orbit, and docking forces only
(x, y) convergence. In the 3D upgrade:

- **Mission drone (Receiver)** flies a full 3-D waypoint route that includes altitude changes
  (climb-out, cruise, descent segments) so its altitude at the refueling trigger differs from the
  Tanker's default loiter altitude.
- **Tanker** holds a 3-D loiter helix (constant altitude $z_T$) at the rendezvous point; the
  Receiver must match **all three** components (x, y, z) to achieve soft-dock.
- **Altitude-matching phase** is added as an explicit FSM state between "fly to RVZ" and
  "docking": the Receiver first climbs/descends to $z_T$ while approaching horizontally.
- **Post-refuel mission** resumes the 3-D waypoint sequence rather than a flat heading.
- Outputs include: 3-D trajectory, altitude vs time profile, energy levels, and docking approach
  detail in all three axes.

---

## Problem Definition

**Setup**: A delivery drone (Receiver) departs from base at ground level, climbs to cruise
altitude, and follows a 3-D waypoint route toward a distant target. Its battery is insufficient
to complete the round trip. A tanker drone launches simultaneously from the same base, flies ahead
to the rendezvous waypoint $\mathbf{w}_{rvz}$, and holds a circular loiter orbit at altitude
$z_T = 15$ m. The Receiver, currently cruising at a different altitude ($z_R = 10$ m), must
simultaneously close horizontal distance *and* match the Tanker's altitude before docking.
After energy transfer the Receiver resumes its 3-D waypoint mission; the Tanker returns to base.

**Roles**:
- **Receiver (red)**: delivery drone, 3-D waypoint mission, battery drains in flight, requests
  refueling when predicted remaining range falls below threshold.
- **Tanker (blue)**: dedicated support drone, full battery, flies 3-D to rendezvous altitude,
  holds circular loiter orbit at $z_T = 15$ m until Receiver docks.

**Objective**: Receiver completes the full 3-D delivery mission without forced landing.
Soft-dock requires:
$$\|\Delta\mathbf{r}\|_3 < \epsilon_p, \quad \|\Delta\mathbf{v}\|_3 < \epsilon_v$$
across all three spatial dimensions.

---

## Mathematical Model

### Battery Drain (same as S027)

$$\dot{E}_i = -k_f \|\mathbf{v}_i\|^2 - k_h$$

Predicted remaining range:

$$d_{rem}(t) = \frac{E_R(t)}{k_f v_R^2 + k_h} \cdot v_R$$

Refueling triggered when:

$$d_{rem}(t) < d_{target}(t) + d_{base} + d_{margin}$$

### 3-D Rendezvous Point

The rendezvous waypoint is placed at fraction $\alpha$ along the base-to-target route and at the
Tanker's loiter altitude $z_T$:

$$\mathbf{w}_{rvz} = \begin{bmatrix}(1-\alpha)\,x_0 + \alpha\,x_{tgt}\\(1-\alpha)\,y_0 + \alpha\,y_{tgt}\\z_T\end{bmatrix}$$

### 3-D Loiter Orbit

The Tanker holds a circular orbit in the horizontal plane at altitude $z_T$:

$$\mathbf{p}_T(t) = \mathbf{w}_{rvz} + \begin{bmatrix}R_{orb}\cos(\omega_{orb}\,t+\phi_0)\\R_{orb}\sin(\omega_{orb}\,t+\phi_0)\\0\end{bmatrix}, \quad \omega_{orb} = \frac{V_{T,orb}}{R_{orb}}$$

### Altitude-Matching Phase

When the Receiver enters the rendezvous capture radius $d_{cap}$ but its altitude differs from
the Tanker's by more than $\delta_z = 1.0$ m, the Receiver applies a vertical velocity command
while maintaining horizontal approach:

$$v_{R,z} = K_{alt} \cdot (z_T - z_R), \quad |v_{R,z}| \leq v_{z,max}$$

Altitude match declared when $|z_R - z_T| < \delta_z$.

### 3-D Docking Control (PD, full 3-D)

Relative state:
$$\Delta\mathbf{r} = \mathbf{p}_R - \mathbf{p}_T, \qquad \Delta\mathbf{v} = \mathbf{v}_R - \mathbf{v}_T$$

**Phase 1 — Approach** ($\|\Delta\mathbf{r}\| > d_{dock}$):
$$\mathbf{u}_R = -K_p\,\Delta\mathbf{r} - K_d\,\Delta\mathbf{v} + \mathbf{a}_T$$

**Phase 2 — Soft Dock** ($\|\Delta\mathbf{r}\| \leq d_{dock}$):
$$\mathbf{u}_R = -K_{p2}\,\Delta\mathbf{r} - K_{d2}\,\Delta\mathbf{v}$$

Docking declared successful when $\|\Delta\mathbf{r}\| < \epsilon_p$ and $\|\Delta\mathbf{v}\| < \epsilon_v$.

### Energy Transfer

$$E_R \leftarrow E_R + \Delta E_{xfer}, \qquad E_T \leftarrow E_T - \Delta E_{xfer}(1 + \eta_{loss})$$

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Cruise speed $V$ | 5.0 m/s |
| Tanker loiter speed $V_{T,orb}$ | 2.0 m/s |
| Loiter radius $R_{orb}$ | 4.0 m |
| Receiver cruise altitude $z_R$ | 10.0 m |
| Tanker loiter altitude $z_T$ | 15.0 m |
| Altitude difference at RVZ entry | 5.0 m |
| Altitude match gain $K_{alt}$ | 1.2 |
| Max vertical speed $v_{z,max}$ | 2.0 m/s |
| Flight power coefficient $k_f$ | 0.3 W·s²/m² |
| Hover drain $k_h$ | 0.5 W |
| Receiver initial battery $E_{0,R}$ | 40 J |
| Tanker initial battery $E_{0,T}$ | 80 J |
| Energy transferred $\Delta E_{xfer}$ | 30 J |
| Transfer loss $\eta_{loss}$ | 5 % |
| Docking radius $d_{dock}$ | 0.5 m |
| Position tolerance $\epsilon_p$ | 0.20 m |
| Velocity tolerance $\epsilon_v$ | 0.08 m/s |
| Approach PD gains $(K_p, K_d)$ | (2.0, 1.5) |
| Soft-dock PD gains $(K_{p2}, K_{d2})$ | (4.0, 2.5) |
| Base-to-target distance (XY) | 60.0 m |
| Rendezvous fraction $\alpha$ | 0.50 |
| DT | 0.05 s |
| T_MAX | 120 s |

---

## Expected Output

- **`trajectory_3d.png`**: Full 3-D trajectories of Receiver (red) and Tanker (blue) with the
  refueling point marked as a green star; docking segment highlighted in yellow.
- **`altitude_profile.png`**: Altitude $z(t)$ for both drones; vertical dashed lines mark the
  altitude-match start, dock success, and energy transfer; shows the 5 m altitude gap closing.
- **`energy_levels.png`**: Battery energy $E(t)$ for both drones; vertical events annotated;
  post-refuel energy jump on Receiver clearly visible.
- **`docking_detail.png`**: Three-panel plot of $\Delta x$, $\Delta y$, $\Delta z$ vs time during
  the docking approach; convergence to zero in all three axes.
- **`animation.gif`**: Real-time 3-D animation showing both drones, the loiter orbit, and the
  energy bar below.

---

## Extensions

1. **Altitude uncertainty**: add GPS altitude noise $\sigma_z = 0.3$ m and re-tune $K_{alt}$
2. **Moving refueling platform**: Tanker flies a constant-speed straight segment instead of
   loitering; Receiver must intercept a 3-D moving target
3. **Optimal rendezvous altitude**: sweep $z_T \in [5, 25]$ m and minimise total energy cost
   including climb/descent
4. **Multi-hop 3D chain**: two Tankers at different waypoints and altitudes; Receiver climbs and
   descends between hops
5. **Turbulence during altitude match**: add vertical gust model and assess docking robustness

---

## Related Scenarios

- Original 2D version: [S027](../S027_aerial_refueling_relay.md)
- Prerequisites: [S021](../S021_point_delivery.md), [S024](../S024_wind_compensation.md)
- Follow-ups: [S028](../S028_cargo_escort_formation.md), [S036](../S036_last_mile_relay.md)
- Cross-domain: [S012 3D relay pursuit](../../01_pursuit_evasion/3d/S012_3d_relay_pursuit.md)
