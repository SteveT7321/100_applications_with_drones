# S039 3D Upgrade — Offshore Platform Exchange

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S039 original](../S039_offshore_platform_exchange.md)

---

## What Changes in 3D

The original S039 models all positions in a flat 2D sea plane (z is never assigned; all groundspeed
and wind calculations operate on 2D vectors). This is a deliberate simplification: offshore platforms
and a support vessel are treated as points in the horizontal plane, and deck landings are merely
a 2D PD convergence check.

In the real offshore environment three effects are irrecoverable from 2D modelling:

1. **Vessel 6-DOF motion**: a supply ship in open sea heaves, pitches, and rolls with wave period
   $T_w \approx 8$–12 s and deck-tip amplitudes of 0.5–2.0 m vertically. A drone approaching a
   vessel deck in 2D sees a stationary target; in 3D it must track a deck point that oscillates
   up and down while also tilting, shifting the safe touch-down point laterally.
2. **Deck-relative coordinate frame**: the drone's guidance law must transform its world-frame
   position into a ship-body frame (heave/pitch/roll applied) to determine whether it is above the
   landing grid and whether the deck tilt angle is within safe limits at touch-down.
3. **3D approach corridor under sea state**: maritime aviation procedures (and the analogous drone
   rules) define a glideslope — typically $6°$–$10°$ descent angle — for the final approach.
   Honouring this corridor requires active altitude control referenced to the instantaneous deck
   height, not a fixed z constant.

---

## Problem Definition

**Setup**: Identical to S039 — $N = 4$ drones, $M = 5$ fixed offshore platforms in a
$6 \times 6$ km area, 4 cargo requests, 1 support vessel sailing a piecewise-linear route at
$V_s = 4$ m/s. Wind is modelled as a 3D Dryden gust field (horizontal gusts as in S039, plus
a smaller vertical component $\sigma_{g,z} = 0.4$ m/s). All drone positions are now
$\mathbf{p}_k \in \mathbb{R}^3$.

**Vessel motion**: the vessel deck reference point $\mathbf{p}_{deck}(t) \in \mathbb{R}^3$ is the
sum of the 2D horizontal trajectory from S039 and a 6-DOF oscillation driven by sea state:

- **Heave** $z_{heave}(t)$: sinusoidal vertical displacement at wave frequency $\omega_w$.
- **Pitch** $\theta_{pitch}(t)$: longitudinal tilt of the vessel about its beam axis.
- **Roll** $\phi_{roll}(t)$: transverse tilt about the vessel's fore-aft axis.

Platform structures are fixed at $z_{platform} = 3.0$ m above sea level (deck height of a
typical jackup rig service pad).

**Roles**: same as S039, with drone altitude now an active control degree of freedom.

**Objective**: same multi-objective as S039 (feasibility, energy minimisation, conflict-free
scheduling), extended with two 3D-specific constraints:

1. **Glideslope constraint**: during the final approach to the vessel, the drone's descent angle
   must satisfy $\gamma \in [\gamma_{min}, \gamma_{max}]$ relative to the instantaneous deck height.
2. **Deck-tilt limit**: touch-down is only declared when the deck's combined tilt
   $|\phi_{combined}| < \phi_{max}$ (deck is momentarily near-level).

---

## Mathematical Model

### Vessel 6-DOF Deck Position

Let $\mathbf{p}_{v,xy}(t)$ be the 2D horizontal vessel trajectory from S039. The full 3D deck
reference point is:

$$\mathbf{p}_{deck}(t) = \begin{bmatrix} p_{v,x}(t) \\ p_{v,y}(t) \\ z_{ref} + z_{heave}(t) \end{bmatrix}$$

where $z_{ref} = 5.0$ m is the mean deck height above sea level, and the heave displacement:

$$z_{heave}(t) = A_h \sin(\omega_w t + \phi_{h})$$

with wave amplitude $A_h$ and wave angular frequency $\omega_w = 2\pi / T_w$.

Deck normal vector in world frame, derived from pitch $\theta_p(t)$ and roll $\phi_r(t)$:

$$\hat{\mathbf{n}}_{deck}(t) = R_x(\phi_r) R_y(\theta_p) \hat{\mathbf{z}}$$

$$\theta_p(t) = \Theta_p \sin(\omega_w t + \phi_p), \qquad
\phi_r(t) = \Phi_r \sin(\omega_w t + \phi_r^{(0)})$$

Combined tilt magnitude used for touch-down gating:

$$|\phi_{combined}(t)| = \sqrt{\theta_p(t)^2 + \phi_r(t)^2}$$

### Deck-Relative Coordinate Frame

Define the ship-body frame $\mathcal{F}_s$ with origin at $\mathbf{p}_{deck}(t)$,
$\hat{\mathbf{x}}_s$ along the vessel heading $\psi_v$, $\hat{\mathbf{z}}_s = \hat{\mathbf{n}}_{deck}$.

Transformation from world frame to ship-body frame:

$$\mathbf{q}_k^s = R_{sb}(t) \bigl(\mathbf{p}_k - \mathbf{p}_{deck}(t)\bigr)$$

$$R_{sb}(t) = \begin{bmatrix}
\cos\psi_v & \sin\psi_v & 0 \\
-\sin\psi_v & \cos\psi_v & 0 \\
0 & 0 & 1
\end{bmatrix} R_x(\phi_r)^T R_y(\theta_p)^T$$

The drone is above the landing grid when $|q_{k,x}^s| < \ell_x / 2$ and
$|q_{k,y}^s| < \ell_y / 2$ (pad half-dimensions $\ell_x = \ell_y = 1.0$ m).

### 3D Glideslope Approach Corridor

At horizontal range $d_{horiz}$ from the deck centre, the commanded altitude is:

$$z_{cmd}(t) = z_{heave}(t) + z_{ref} + d_{horiz} \tan\gamma_{gs}$$

where $\gamma_{gs}$ is the nominal glideslope angle. The drone tracks this reference altitude
while converging horizontally. The total 3D error from the glideslope:

$$\varepsilon_{gs}(t) = \bigl(z_k(t) - z_{cmd}(t)\bigr) / d_{horiz}$$

Glideslope constraint:

$$|\varepsilon_{gs}(t)| \leq \varepsilon_{gs,max} \quad \text{during final approach phase}$$

### 3D Moving-Deck PD Controller

Extend the S039 2D PD law to 3D. Define the 3D relative state:

$$\Delta\mathbf{r}_k = \mathbf{p}_k - \mathbf{p}_{deck}(t), \qquad
\Delta\mathbf{v}_k = \dot{\mathbf{p}}_k - \dot{\mathbf{p}}_{deck}(t)$$

3D approach command (world frame):

$$\mathbf{u}_k = -K_p \Delta\mathbf{r}_k - K_d \Delta\mathbf{v}_k + \ddot{\mathbf{p}}_{deck}(t)$$

where $\ddot{\mathbf{p}}_{deck}$ is the deck acceleration (heave $\ddot{z}_{heave}$, plus the
Coriolis-like terms from pitch/roll rate). In practice $\ddot{\mathbf{p}}_{deck}$ is estimated
by finite differencing $\dot{\mathbf{p}}_{deck}$ over two consecutive steps.

3D touch-down declared successful when:

$$\|\Delta\mathbf{r}_k\| < \varepsilon_p \quad \text{and} \quad
\|\Delta\mathbf{v}_k\| < \varepsilon_v \quad \text{and} \quad
|\phi_{combined}| < \phi_{max}$$

### Vertical Wind Component

Extend the S039 Dryden model to include the vertical gust component $\xi_z(t)$:

$$\dot{\xi}_z = -\frac{V_a}{L_{g,z}} \xi_z + \sigma_{g,z} \sqrt{\frac{2 V_a}{L_{g,z}}}\, \eta_z(t)$$

with vertical gust length scale $L_{g,z} = 100$ m (shorter than horizontal). Drone vertical
groundspeed:

$$v_{g,z}^k = v_{cmd,z}^k + \xi_z(t)$$

### Cruise Altitude and Transition Logic

Drones cruise at $z_{cruise} = 30$ m above sea level between platforms. Altitude state machine:

- **Cruise**: $z_k = z_{cruise}$, horizontal navigation as in S039.
- **Descent to platform**: linear altitude ramp from $z_{cruise}$ to $z_{platform} = 3.0$ m
  over horizontal distance $d_{ramp} = 100$ m.
- **Final approach to vessel**: track $z_{cmd}(t)$ (glideslope + heave feed-forward) while
  converging horizontally inside $d_{dock} = 15$ m.
- **Holding orbit**: circular orbit at $z_{hold} = z_{platform} + 10$ m above the occupied
  platform, $R_{hold} = 30$ m.

### Battery Model in 3D

The power model is extended to include the vertical climb/descent penalty:

$$\dot{E}_k = -P_{cruise} - k_{wind}\|\mathbf{w}_{xy}\|^2 - k_{vert} v_{z,k}^2$$

where $k_{vert}$ is the vertical drag power coefficient and $v_{z,k}$ is the drone's vertical
speed. Climbing at $v_z = 1$ m/s costs an additional $\Delta P = k_{vert} \cdot 1.0$ W;
the heave-following during vessel approach incurs the same cost with $v_z = \dot{z}_{heave}$.

---

## Key 3D Additions

- **Vessel 6-DOF motion**: heave amplitude $A_h = 0.8$ m, pitch amplitude $\Theta_p = 3°$,
  roll amplitude $\Phi_r = 4°$, wave period $T_w = 9$ s.
- **Deck-relative frame**: $R_{sb}(t)$ transforms world-frame drone position into ship-body
  coordinates to evaluate pad-grid alignment and tilt-gate condition.
- **Glideslope corridor**: nominal $\gamma_{gs} = 8°$, tolerance $\varepsilon_{gs,max} = 0.03$
  (3% of horizontal range).
- **Touch-down tilt gate**: $\phi_{max} = 5°$ combined; drone holds position at $z_{cmd}$ until
  the deck passes through its near-level window.
- **Vertical gust**: $\sigma_{g,z} = 0.4$ m/s, $L_{g,z} = 100$ m added to Dryden model.
- **3D holding orbit**: $z_{hold} = z_{platform} + 10$ m to separate vertically from the deck.
- **Vertical power penalty**: $k_{vert} = 5.0$ W·s²/m² for altitude change cost.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| All S039 base parameters | (unchanged — see S039) |
| Cruise altitude $z_{cruise}$ | 30.0 m |
| Platform deck height $z_{platform}$ | 3.0 m |
| Mean vessel deck height $z_{ref}$ | 5.0 m |
| Heave amplitude $A_h$ | 0.8 m |
| Wave period $T_w$ | 9.0 s |
| Pitch amplitude $\Theta_p$ | 3.0° |
| Roll amplitude $\Phi_r$ | 4.0° |
| Touch-down tilt gate $\phi_{max}$ | 5.0° |
| Glideslope angle $\gamma_{gs}$ | 8.0° |
| Glideslope tolerance $\varepsilon_{gs,max}$ | 0.03 (3%) |
| Altitude ramp distance $d_{ramp}$ | 100 m |
| Holding orbit altitude offset | +10 m above platform deck |
| Vertical gust intensity $\sigma_{g,z}$ | 0.4 m/s |
| Vertical gust length scale $L_{g,z}$ | 100 m |
| Vertical power coefficient $k_{vert}$ | 5.0 W·s²/m² |
| Pad half-dimensions $(\ell_x, \ell_y)$ | (1.0, 1.0) m |

---

## Expected Output

- **3D sea-area plot**: platform towers shown as vertical stubs at $z_{platform}$, vessel
  route on the sea surface, drone trajectories in full 3D coloured by drone index; heave
  motion of the vessel deck visible as a wavy line on the vessel track.
- **Altitude time series**: all 4 drones on a shared plot showing cruise, ramp-down,
  glideslope approach, holding orbits, and post-swap re-climb; vessel deck $z_{deck}(t)$
  overlaid as a reference.
- **Heave and tilt time series**: $z_{heave}(t)$, $\theta_p(t)$, $\phi_r(t)$ and
  $|\phi_{combined}(t)|$ with $\phi_{max}$ threshold line; touch-down events marked when
  the tilt-gate condition is satisfied.
- **Deck-relative approach path**: ship-body-frame $q_x^s$ vs $q_z^s$ for each vessel landing,
  with glideslope cone boundary drawn; shows whether the drone tracked within the corridor.
- **Relative 3D distance and velocity to deck** during each vessel approach: convergence of
  $\|\Delta\mathbf{r}_k\|$ and $\|\Delta\mathbf{v}_k\|$ to tolerance bands; indicates waiting
  during tilt-gate hold.
- **Battery SoC vs time**: same Gantt-style comparison as S039, but with vertical-climb
  penalty increments visible as slightly steeper drain segments during altitude transitions.
- **Platform queue Gantt chart**: unchanged from S039 but now annotated with holding-orbit
  altitudes.
- **3D energy comparison bar chart**: greedy direct vs vessel-relay vs rolling-horizon for
  both horizontal and total (including vertical) energy components stacked.

---

## Extensions

1. **Sea-state escalation**: increase $A_h$, $\Theta_p$, $\Phi_r$ during the mission (Beaufort
   3 → 5 transition); observe how the tilt-gate hold time grows and which cargo deadlines are
   missed first.
2. **Heave feed-forward control**: replace finite-difference deck velocity estimation with a
   Kalman filter fusing GPS altitude and an IMU on the vessel; compare touch-down scatter with
   and without feed-forward.
3. **Optimal glideslope angle search**: sweep $\gamma_{gs} \in [4°, 15°]$ and find the angle
   that minimises total approach time while keeping glideslope deviation within tolerance under
   Dryden vertical gusts.
4. **Night / low-visibility landing**: add a constraint that the drone must be within visual
   range of the vessel's landing beacon; route drones to use the vessel only when within 300 m
   of its beacon cone.
5. **Vessel pitch/roll prediction**: fit a short-term AR(2) model to the heave/tilt signal and
   use predicted tilt at intercept time to plan the touch-down window in advance, reducing
   hover-wait time.

---

## Related Scenarios

- Original 2D version: [S039](../S039_offshore_platform_exchange.md)
- 3D moving-deck foundation: [S023 3D](S023_3d_moving_landing_pad.md)
- 3D wind compensation reference: [S024 3D](S024_3d_wind_compensation.md)
- 3D aerial refueling (vessel-relay concept in 3D): [S027 3D](S027_3d_aerial_refueling.md)
