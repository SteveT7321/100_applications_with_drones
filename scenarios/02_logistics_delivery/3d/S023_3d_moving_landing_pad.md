# S023 3D Upgrade — Moving Landing Pad

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S023 original](../S023_moving_landing_pad.md)

---

## What Changes in 3D

The original S023 models the truck pad as a point fixed at $z = 0$ with no vertical or rotational dynamics — the platform is assumed perfectly level and its deck height never varies. The drone's descent is governed by a scalar $\tanh$ flare law that targets a static $z = 0$ ground plane.

In the real-world equivalent (ship deck, barge, moving flatbed on rough terrain) the platform undergoes 6-DOF motion: it heaves vertically ($z_p(t)$), pitches (tilts nose-up/down), and rolls (tilts port/starboard). These motions introduce three coupling effects that are absent in 2D:

1. **Moving deck height** — the drone must track a time-varying touchdown altitude $z_p(t)$ rather than a fixed $z = 0$.
2. **Deck tilt** — pitch/roll create a sloped landing surface; the acceptable contact velocity is no longer purely $-\hat{\mathbf{z}}$ but depends on the instantaneous normal to the deck.
3. **Relative velocity in 3D** — the deck surface velocity has a heave component $\dot{z}_p(t)$ that must be matched at touchdown in addition to the horizontal velocity match already required in S023.

Additionally, the approach trajectory must now incorporate a constrained descent angle so that the drone does not clip the vessel's superstructure, and the Phase 2 descent reference tracks the live deck height rather than a pre-planned fixed altitude.

---

## Problem Definition

**Setup**: A delivery drone departs a fixed shore station and must land on a moving vessel. The vessel travels at constant forward speed $v_V$ along the $x$-axis. Its deck undergoes sinusoidal heave, pitch, and roll: all three motions are periodic with vessel-specific frequencies and amplitudes. The drone receives real-time telemetry of the vessel's full pose (position + heave + pitch + roll) but must compensate for a fixed one-step sensor latency $\tau_s$.

**Roles**:
- **Drone**: quadrotor executing a three-phase guidance law — intercept (PNG), constrained approach along a descent-angle ramp, and active deck-tracking flare.
- **Vessel**: surface platform moving at constant $v_V$ along $x$; deck undergoes 6-DOF heave/pitch/roll with known frequencies and amplitudes; deck corners define the usable pad boundary in the body frame.

**Objective**: achieve touchdown on the pad with:
1. Lateral miss distance $\|\mathbf{p}_{xy,D} - \mathbf{p}_{xy,pad}\| \leq r_{pad}$
2. Relative vertical speed at contact $|\dot{z}_D - \dot{z}_p| \leq v_{land,max}$
3. Relative horizontal velocity mismatch $\|\dot{\mathbf{p}}_{xy,D} - \dot{\mathbf{p}}_{xy,V}\| \leq \Delta v_{max}$
4. Landing only attempted during a **motion window** when deck heave satisfies $|z_p(t) - \bar{z}_p| \leq \delta_{heave}$ (near-crest or near-trough gate)

---

## Mathematical Model

### Vessel 6-DOF Platform Motion

Vessel centre position (world frame):

$$\mathbf{p}_V(t) = \begin{bmatrix} v_V t \\ 0 \\ 0 \end{bmatrix}$$

Deck reference point (pad centre) with heave, pitch, and roll superimposed:

$$z_p(t) = A_h \sin(2\pi f_h t + \phi_h)$$

$$\theta_p(t) = A_\theta \sin(2\pi f_\theta t + \phi_\theta) \quad \text{(pitch, rad)}$$

$$\phi_r(t) = A_\phi \sin(2\pi f_r t + \phi_r^0) \quad \text{(roll, rad)}$$

Deck normal vector in world frame (small-angle approximation):

$$\hat{\mathbf{n}}_{deck}(t) \approx \begin{bmatrix} -\theta_p(t) \\ \phi_r(t) \\ 1 \end{bmatrix} \Big/ \left\| \begin{bmatrix} -\theta_p \\ \phi_r \\ 1 \end{bmatrix} \right\|$$

Pad centre in world frame:

$$\mathbf{p}_{pad}(t) = \begin{bmatrix} v_V t \\ 0 \\ z_p(t) \end{bmatrix} + R_{deck}(t)\,\mathbf{d}_{offset}$$

where $R_{deck}(t)$ is the $3\times3$ rotation matrix from pitch and roll angles, and $\mathbf{d}_{offset}$ is the pad's offset from the vessel centre in the body frame.

### Deck Velocity

$$\dot{\mathbf{p}}_{pad}(t) = \begin{bmatrix} v_V \\ 0 \\ \dot{z}_p(t) \end{bmatrix}, \quad \dot{z}_p(t) = 2\pi f_h A_h \cos(2\pi f_h t + \phi_h)$$

### Phase 1 — Intercept (3D PNG)

Identical to S023. Iterative intercept prediction with 5-iteration fixed-point:

$$\Delta t_{fly}^{(k+1)} = \frac{\|\mathbf{p}_{pad}^*(t + \Delta t_{fly}^{(k)}) - \mathbf{p}_D\|}{v_{D,max}}$$

PNG acceleration:

$$\mathbf{a}_P = N \cdot V_c \cdot \frac{\mathbf{v}_{rel} - (\mathbf{v}_{rel} \cdot \hat{\boldsymbol{\lambda}})\hat{\boldsymbol{\lambda}}}{\|\mathbf{p}_{pad}^* - \mathbf{p}_D\|}$$

where $\hat{\boldsymbol{\lambda}} = (\mathbf{p}_{pad}^* - \mathbf{p}_D)/\|\mathbf{p}_{pad}^* - \mathbf{p}_D\|$ and $V_c = -d\|\mathbf{p}_{pad}^* - \mathbf{p}_D\|/dt$.

### Phase 2 — Constrained Approach (Descent Ramp)

When the drone enters the approach cone (horizontal distance $d_{xy} \leq r_{approach}$, altitude $z_D \leq z_{ramp}$), it flies along a straight-line ramp toward the deck at a constrained descent angle $\gamma$:

$$\mathbf{p}_{ramp}(s) = \mathbf{p}_{D,entry} + s \cdot \hat{\mathbf{d}}_{ramp}, \quad s \in [0, L_{ramp}]$$

$$\hat{\mathbf{d}}_{ramp} = \frac{1}{\sqrt{1+\tan^2\gamma}}\begin{bmatrix} \cos\psi_V \\ \sin\psi_V \\ -\tan\gamma \end{bmatrix}$$

where $\psi_V$ is the vessel heading angle and $\gamma \geq \gamma_{min} = 6°$ is the minimum approach angle (clearance above superstructure).

Commanded ramp position at time $t$ tracks the advancing ramp endpoint:

$$\mathbf{p}_{cmd}^{ramp}(t) = \mathbf{p}_{pad}(t) + L_{ramp}(t)\,\hat{\mathbf{d}}_{ramp}$$

with $L_{ramp}(t) = \|p_D - p_{pad}\|$ decaying as the drone closes.

### Phase 3 — Active Deck-Tracking Flare

Once $d_{xy} \leq r_{flare}$ and $z_D - z_p \leq z_{flare,rel}$, the drone transitions to a PD law that tracks the live deck height:

**Lateral (match pad horizontal velocity)**:

$$\mathbf{a}_{xy} = K_p(\mathbf{p}_{pad,xy} - \mathbf{p}_{D,xy}) + K_v(\dot{\mathbf{p}}_{pad,xy} - \mathbf{v}_{D,xy})$$

**Vertical (track deck height + flare)**:

$$h_{rel}(t) = z_D(t) - z_p(t)$$

$$\dot{h}_{rel,cmd}(t) = -v_{land,max} \cdot \tanh\!\left(\frac{h_{rel}}{z_{flare,rel}}\right) + \dot{z}_p(t)$$

$$a_z = K_{z,r}\!\left(\dot{h}_{rel,cmd} - (\dot{z}_D - \dot{z}_p)\right)$$

The $\dot{z}_p$ feed-forward term is critical: without it the flare controller fights the heaving deck and risks a hard contact during a downward heave cycle.

### Motion Window Gate

A landing attempt is only permitted if the current heave phase satisfies:

$$|z_p(t)| \leq \delta_{heave} \quad \text{and} \quad |\dot{z}_p(t)| \leq \dot{z}_{gate}$$

If the gate is not open, Phase 3 holds the drone in a hover at $h_{rel} = z_{hold}$ above the mean deck height, waiting for the next window.

### Landing Success Criterion

$$\|\mathbf{p}_{xy,D} - \mathbf{p}_{xy,pad}\| \leq r_{pad}$$
$$|\dot{z}_D - \dot{z}_p| \leq v_{land,max}$$
$$\|\dot{\mathbf{p}}_{xy,D} - \dot{\mathbf{p}}_{xy,pad}\| \leq \Delta v_{max}$$

---

## Key 3D Additions

- **6-DOF vessel platform**: deck undergoes heave, pitch, and roll with independent sinusoidal models; deck normal $\hat{\mathbf{n}}_{deck}(t)$ varies continuously.
- **Moving touchdown altitude**: flare law references live $z_p(t)$ instead of a fixed $z = 0$; $\dot{z}_p$ feed-forward cancels heave-induced velocity errors.
- **Constrained approach ramp**: descent angle $\gamma \geq 6°$ enforced geometrically during Phase 2 to simulate superstructure clearance.
- **Relative-velocity touchdown criterion**: success requires matching both $\dot{z}_p$ and $\dot{\mathbf{p}}_{xy,V}$ simultaneously, not just absolute velocity limits.
- **Motion window gate**: Phase 3 entry is gated on heave amplitude and rate so the touchdown is timed to a calm portion of the heave cycle.
- **Sensor latency model**: pad state is fed with a one-step delay $\tau_s$ to reflect the real-time telemetry lag.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Vessel speed $v_V$ | 4.0 m/s |
| Heave amplitude $A_h$ | 0.6 m |
| Heave frequency $f_h$ | 0.1 Hz |
| Pitch amplitude $A_\theta$ | 4° |
| Roll amplitude $A_\phi$ | 6° |
| Platform motion frequency | 0.08 – 0.12 Hz |
| Drone max speed | 8.0 m/s |
| Drone max acceleration | 5.0 m/s² |
| PNG navigation constant $N$ | 3 |
| Min approach angle $\gamma_{min}$ | 6° |
| Phase-switch altitude $z_{ramp}$ | 8.0 m |
| Approach cone radius $r_{approach}$ | 5.0 m |
| Flare entry height (relative) $z_{flare,rel}$ | 2.5 m |
| Motion window heave gate $\delta_{heave}$ | 0.2 m |
| Motion window rate gate $\dot{z}_{gate}$ | 0.3 m/s |
| Landing pad radius $r_{pad}$ | 0.5 m |
| Max relative touchdown vertical speed | 0.8 m/s |
| Max horizontal velocity mismatch | 0.5 m/s |
| Sensor latency $\tau_s$ | 0.04 s (2 steps at 50 Hz) |
| $z$ range | 0.0 – 20.0 m |
| Simulation time step | 0.02 s (50 Hz) |

---

## Expected Output

- **3D trajectory plot**: drone approach arc in red, vessel track in orange, pad trace in green with heave oscillation visible in $z$; Phase 1/2/3 segments colour-coded
- **Altitude time series**: $z_D(t)$ vs $z_p(t)$ on the same axes showing the drone tracking the heaving deck; relative height $h_{rel}(t)$ in a subplot
- **Deck motion time series**: heave $z_p(t)$, pitch $\theta_p(t)$, roll $\phi_r(t)$ vs time; vertical shading for motion window open/closed intervals
- **Relative velocity time series**: $|\dot{z}_D - \dot{z}_p|$ and $\|\dot{\mathbf{p}}_{xy,D} - \dot{\mathbf{p}}_{xy,V}\|$ converging to within success thresholds
- **Lateral miss distance**: $\|\mathbf{p}_{xy,D} - \mathbf{p}_{xy,pad}\|$ vs time with $r_{pad}$ threshold line
- **Motion window timing bar**: horizontal bar on time axis showing gate open (green) / gate closed (red) with touchdown event marked
- **Printed touchdown report**: miss distance, relative vertical speed, horizontal mismatch, heave phase at touchdown, success/fail verdict

---

## Extensions

1. **Irregular sea state**: replace sinusoidal heave with a JONSWAP spectrum realisation to simulate realistic ocean wave forcing.
2. **Predictive deck tracking**: fit a short-window sinusoid to recent $z_p$ samples and extrapolate $\hat{z}_p(t + \tau)$ to pre-compensate sensor latency.
3. **Abort and go-around**: if the motion window closes during Phase 3, trigger a climb-out and re-queue; count fuel cost of aborted attempts.
4. **Deck tilt landing acceptance**: extend the success criterion to include contact-normal alignment — require $|\hat{\mathbf{v}}_{D,rel} \cdot \hat{\mathbf{n}}_{deck}| \geq \cos\alpha_{max}$.
5. **Multi-vessel fleet**: schedule landings on two vessels with different heave phases; find the dispatch order that minimises total flight time.

---

## Related Scenarios

- Original 2D version: [S023](../S023_moving_landing_pad.md)
- Moving pad with wind: [S024](../S024_wind_compensation.md)
- Offshore platform exchange (related 3D context): [S039](../S039_offshore_platform_exchange.md)
- 3D obstacle avoidance delivery: [S022 3D](S022_3d_obstacle_avoidance.md)
