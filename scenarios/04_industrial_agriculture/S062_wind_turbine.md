# S062 Wind Turbine Blade Inspection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete
**Algorithm**: Phase-Locked Orbital Inspection + Safety Margin Controller | **Dimension**: 3D

---

## Problem Definition

**Setup**: A horizontal-axis wind turbine stands at hub height $h_{hub} = 30$ m with three blades of
length $R_{blade} = 15$ m and rotor angular velocity $\omega = 0.5$ rad/s (period $T_{rot} \approx
12.6$ s). The blades are equally spaced at $\phi_i = 2\pi i/3$ ($i = 0, 1, 2$) and rotate in the
rotor plane (the $xz$-plane with the hub as origin). A single inspection drone orbits the rotor hub
in the same plane at radius $r_{orbit} = 17$ m (hub-centred), maintaining a fixed angular phase
offset $\phi_{offset} = 60°$ ahead of each blade tip. GPS positioning carries an isotropic error
$\sigma_{GPS} = 0.1$ m. The minimum safe separation from any blade surface is
$d_{safe} = 1.5$ m (hard constraint), and the target inspection standoff is
$d_{inspect} = 2.0$ m from the blade surface. The drone speed **relative to the imaged blade
section** during an active inspection pass must satisfy $v_{rel} \leq v_{max,inspect} = 1.0$ m/s.

> **Corrections (verified in the implementation)**:
> - The original card set $d_{inspect} = 1.0$ m, below $d_{safe} = 1.5$ m, so no segment could be
>   imaged without violating the safety margin. $d_{inspect}$ is now 2.0 m (> $d_{safe}$).
> - A drone that only orbits at $r_{orbit} = 17$ m with $y_{offset} = 1$ m is never closer than
>   $\sqrt{2^2 + 1^2} = 2.24$ m to any blade, so it can neither violate $d_{safe}$ nor image any
>   segment. And because $\phi_{offset} = 60°$ is exactly midway between two blades, an open-loop
>   orbit at $\dot\theta = \omega$ is identical to the phase-locked one unless something disturbs
>   the drone. The implemented mission therefore **co-rotates** with each blade in turn: it holds
>   1.73 m behind the blade's trailing edge (2.0 m from the blade line at $y = 1$ m), slides from
>   tip ($r = 17$ m) to root ($r = 1$ m) at 0.8 m/s relative to the blade, returns to $r = 17$ m at
>   3 m/s and moves along the 17 m orbit to the next blade. The orbit and the 60° start phase are
>   kept for the transit. A wind-gust disturbance (OU, $\sigma = 0.5$ m/s, $\tau = 2$ s) is added
>   so the three controllers can differ.
> - Motion blur depends on the speed relative to the blade, not on ground speed: co-rotating at
>   $r = 17$ m already means 8.5 m/s over the ground.

**Roles**:
- **Drone**: single inspection agent orbiting the rotor plane; carries a downward- and side-facing
  optical/thermal camera; must complete at least one full inspection pass over every blade section
  at the prescribed standoff distance.
- **Rotor**: three rotating blades modelled as line segments from hub to tip; angular state is
  known to the drone in real time via a turbine SCADA feed (simulated encoder signal).

**Objective**: Complete a full-blade inspection of all three blades — visiting every 1 m segment
along each blade at standoff distance $d_{inspect}$ — while never violating the safety margin
$d_{safe}$ from any blade surface. Total inspection time and safety margin violations are the
primary metrics.

**Controller variants** (three compared):
1. **Open-loop orbit** — drone advances at constant angular rate $\dot{\theta} = \omega$; no
   blade-phase feedback; baseline, expected to accumulate safety violations.
2. **Phase-locked orbit** — drone maintains $\theta_{target}(t) = \omega t + \phi_{offset}$
   relative to the nearest blade; advances through inspection waypoints between blade passes.
3. **Phase-locked + safety margin controller** (proposed) — phase-locked orbit augmented with an
   emergency lateral thrust when $\|\mathbf{p}_{drone} - \mathbf{p}_{tip}(t)\| < d_{safe} + 0.3$ m
   (warning zone); inspection speed reduced to $v_{max,inspect}$ during active blade coverage.

---

## Mathematical Model

### Coordinate Frame and Blade Kinematics

The global frame has its origin at ground level directly below the turbine hub. The hub position
is $\mathbf{p}_{hub} = (0,\, 0,\, h_{hub})^\top$. The rotor plane is the $xz$-plane (turbine
shaft along $y$). The drone orbits at a lateral offset $y_{offset}$ from the rotor plane to avoid
the nacelle and allow camera coverage; a small offset $y_{offset} = 1.0$ m is used.

The tip position of blade $i$ at time $t$ is:

$$\mathbf{p}_{tip,i}(t) = \mathbf{p}_{hub} + R_{blade}
  \begin{bmatrix} \cos(\omega t + \phi_i) \\ 0 \\ \sin(\omega t + \phi_i) \end{bmatrix},
  \quad \phi_i = \frac{2\pi i}{3}, \quad i = 0, 1, 2$$

The full blade $i$ is the line segment parameterised by $s \in [0, 1]$:

$$\mathbf{p}_{blade,i}(s, t) = \mathbf{p}_{hub} + s \cdot R_{blade}
  \begin{bmatrix} \cos(\omega t + \phi_i) \\ 0 \\ \sin(\omega t + \phi_i) \end{bmatrix}$$

### Drone Orbital Parameterisation

The drone moves on a circle of radius $r_{orbit}$ centred on the hub:

$$\mathbf{p}_{drone}(\theta) = \mathbf{p}_{hub} +
  \begin{bmatrix} r_{orbit} \cos\theta \\ y_{offset} \\ r_{orbit} \sin\theta \end{bmatrix}$$

The drone's angular position $\theta(t)$ is the controlled degree of freedom. The nominal
inspection path is to track blade $i$'s angular position with lead angle $\phi_{offset}$:

$$\theta_{target,i}(t) = \omega t + \phi_i + \phi_{offset}$$

The nearest blade index is selected as:

$$i^*(t) = \arg\min_{i \in \{0,1,2\}} \left|\, \theta(t) - (\omega t + \phi_i)\,\right|_{\,2\pi}$$

where $|\cdot|_{2\pi}$ denotes the minimum angular distance modulo $2\pi$.

### Phase-Locked Control Law

The phase error with respect to the target is:

$$e_\theta(t) = \theta_{target,i^*}(t) - \theta(t)$$

A proportional-derivative angular rate command:

$$\dot{\theta}_{cmd}(t) = \omega + K_p \, e_\theta(t) + K_d \, \dot{e}_\theta(t)$$

with gains $K_p = 2.0$ rad/(rad·s) and $K_d = 0.3$ s. This keeps the drone locked $\phi_{offset}$
ahead of the leading blade tip regardless of small perturbations or GPS noise.

The Cartesian velocity commanded to the drone is:

$$\dot{\mathbf{p}}_{cmd} = \dot{\theta}_{cmd}
  \begin{bmatrix} -r_{orbit} \sin\theta \\ 0 \\ r_{orbit} \cos\theta \end{bmatrix}$$

Speed is clamped to $v_{max,inspect}$ when the drone is within $d_{inspect} + \varepsilon$ of any
blade surface ($\varepsilon = 0.2$ m tolerance).

### Safety Margin Controller

The signed safety clearance to blade tip $i$ is:

$$\delta_i(t) = \|\mathbf{p}_{drone}(t) - \mathbf{p}_{tip,i}(t)\| - d_{safe}$$

More precisely, the minimum distance to blade $i$ (modelled as a line segment) is:

$$d_{seg,i}(t) = \min_{s \in [0,1]}
  \|\mathbf{p}_{drone}(t) - \mathbf{p}_{blade,i}(s,t)\|$$

The emergency condition is triggered when $d_{seg,i^*}(t) < d_{safe}$:

$$\mathbf{f}_{safety} = \begin{cases}
  K_{safe} \cdot \dfrac{\mathbf{p}_{drone} - \mathbf{p}_{tip,i^*}}
  {\|\mathbf{p}_{drone} - \mathbf{p}_{tip,i^*}\|}
  & \text{if } d_{seg,i^*} < d_{safe} \\[6pt]
  \mathbf{0} & \text{otherwise}
\end{cases}$$

with $K_{safe} = 3.0$ m/s² (lateral thrust gain). This repulsion is added to the nominal
Cartesian velocity command and takes precedence over the phase-lock term.

### Inspection Quality Metric

Discretise each blade into $N_{seg} = 15$ segments of 1 m each. Segment $k$ of blade $i$ is
considered **inspected** at time $t$ when all three conditions hold simultaneously:

$$\text{inspected}_{i,k}(t) = \mathbf{1}\!\left[
  d_{blade,i,k}(t) \leq d_{inspect} + \varepsilon
  \;\wedge\;
  v_{drone}(t) \leq v_{max,inspect}
  \;\wedge\;
  \cos\angle(\hat{\mathbf{n}}_{camera},\, \hat{\mathbf{r}}_{i,k}) \geq \cos 30°
\right]$$

where $d_{blade,i,k}(t)$ is the drone distance to the midpoint of segment $k$,
$\hat{\mathbf{n}}_{camera}$ is the camera boresight direction, and
$\hat{\mathbf{r}}_{i,k}$ is the unit vector from the drone to the segment midpoint.

Overall inspection coverage:

$$\text{Coverage} = \frac{1}{3 N_{seg}} \sum_{i=0}^{2} \sum_{k=1}^{N_{seg}}
  \mathbf{1}\!\left[\exists\, t: \text{inspected}_{i,k}(t) = 1\right] \times 100\%$$

### GPS Noise Model

The drone's measured position is corrupted by isotropic Gaussian noise:

$$\tilde{\mathbf{p}}_{drone}(t) = \mathbf{p}_{drone}(t) + \boldsymbol{\eta}(t),
  \qquad \boldsymbol{\eta}(t) \sim \mathcal{N}(\mathbf{0},\, \sigma_{GPS}^2 \mathbf{I}_3)$$

The phase-lock controller operates on $\tilde{\mathbf{p}}_{drone}$ while the true position is used
to evaluate inspection coverage and safety violations.

---

## Implementation

```python
# Implementation sketch of the corrected (co-rotating) design.
# Full script: src/04_industrial_agriculture/s062_wind_turbine.py
import numpy as np

HUB = np.array([0.0, 0.0, 30.0]); R_BLADE = 15.0; OMEGA = 0.5
PHASES = 2 * np.pi * np.arange(3) / 3
Y_OFF, D_SAFE, D_WARN, D_INSPECT, EPS = 1.0, 1.5, 1.8, 2.0, 0.2
LAT = np.sqrt(D_INSPECT**2 - Y_OFF**2)          # 1.732 m behind the trailing edge
R_ORBIT, R_IN = 17.0, 1.0
V_SWEEP, V_RETURN, V_TRANSFER = 0.8, 3.0, 4.0   # m/s relative to the blade
KP, KD, K_SAFE, TAU, V_MAX = 2.0, 0.3, 3.0, 0.2, 15.0
DT = 0.05

e_r = lambda a: np.array([np.cos(a), 0.0, np.sin(a)])
e_t = lambda a: np.array([-np.sin(a), 0.0, np.cos(a)])

def reference(t, plan):
    """Plan = list of phases (transfer on the 17 m orbit, sweep_in 17->1 m, sweep_out 1->17 m)
    expressed in the frame of the blade being inspected; returns the world position."""
    kind, b, t0, dur, par = active_phase(plan, t)
    phi = OMEGA * t + PHASES[b]
    if kind == 'transfer':                        # move around the orbit, outside the tips
        rel0, rel1, r_o = par
        ang = phi + rel0 + (rel1 - rel0) * min(t - t0, dur) / dur
        return HUB + r_o * e_r(ang) + Y_OFF * np.array([0, 1, 0])
    a0, a1 = par                                  # radial sweep, co-rotating with the blade
    a = a0 + (a1 - a0) * min(t - t0, dur) / dur
    return HUB + a * e_r(phi) - LAT * e_t(phi) + Y_OFF * np.array([0, 1, 0])

def blade_dists(p, t):
    ang = OMEGA * t + PHASES
    U = np.stack([np.cos(ang), np.zeros(3), np.sin(ang)], axis=1)
    s = np.clip(U @ (p - HUB), 0.0, R_BLADE)
    near = HUB + s[:, None] * U
    return np.linalg.norm(p - near, axis=1), near

def step(strategy, t, pos, v_air, wind, gps, plan):
    p_ref = reference(t, plan)
    v_ff = (reference(t + DT, plan) - p_ref) / DT  # exact plan displacement per step
    v_cmd = v_ff.copy()
    if strategy != 'open_loop':
        meas = pos + gps                          # GPS-measured position
        v_true = v_air + wind                     # ground velocity, assumed measured exactly
        fb = KP * (p_ref - meas) + KD * (v_ref - v_true)
        if strategy == 'phase_locked_safety':
            d, near = blade_dists(meas, t)
            for i in np.where(d < D_WARN)[0]:
                n = (meas - near[i]) / d[i]
                fb -= min(fb @ n, 0.0) * n        # drop feedback that pushes towards the blade
                v_cmd += K_SAFE * (D_WARN - d[i]) * n
        v_cmd += fb
    sp = np.linalg.norm(v_cmd)
    if sp > V_MAX:
        v_cmd *= V_MAX / sp
    v_air = v_air + (v_cmd - v_air) * DT / TAU    # 0.2 s velocity lag
    pos = pos + (v_air + wind) * DT               # wind: OU gust, sigma 0.5 m/s, tau 2 s
    return pos, v_air

# Segment k of blade i is inspected when |p - m_ik| <= D_INSPECT + EPS, the speed relative to
# the segment is <= 1 m/s and the segment lies inside the 30 deg camera cone. A distance below
# 0.3 m to a blade (strike), contact with the tower or the ground ends the flight.
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Hub height | $h_{hub}$ | 30 m |
| Blade length | $R_{blade}$ | 15 m |
| Rotor angular velocity | $\omega$ | 0.5 rad/s |
| Rotor period | $T_{rot}$ | $\approx$ 12.6 s |
| Number of blades | $N_{blades}$ | 3 |
| Blade phase spacing | $\phi_i$ | $2\pi i/3$ |
| Orbit radius (hub-centred) | $r_{orbit}$ | 17 m |
| Lateral rotor-plane offset | $y_{offset}$ | 1.0 m |
| Phase lead angle | $\phi_{offset}$ | 60° ($\pi/3$ rad) |
| Hard safety margin | $d_{safe}$ | 1.5 m |
| Target inspection standoff | $d_{inspect}$ | 2.0 m (was 1.0 m, below $d_{safe}$) |
| Max inspection speed | $v_{max,inspect}$ | 1.0 m/s |
| GPS noise std dev | $\sigma_{GPS}$ | 0.1 m |
| PD gain $K_p$ | — | 2.0 rad/(rad·s) |
| PD gain $K_d$ | — | 0.3 s |
| Safety repulsion gain | $K_{safe}$ | 3.0 m/s² |
| Blade segments per blade | $N_{seg}$ | 15 |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Mission horizon | $T_{max}$ | 120 s ($\approx$ 9.5 revolutions) |

---

## Expected Output

- **Rotor-frame path plot**: drone path in the co-rotating frame (blades static), one panel per
  controller.
- **3D trajectory plot**: rotor plane cross-section showing tower (grey vertical bar), rotor disk
  outline (dashed silver circle), three blade positions at three snapshot times (translucent grey
  segments), and the drone's full orbital trajectory colour-coded by minimum blade clearance (blue
  = safe, gold = warning, red = violation); start marked green, end marked red.
- **Clearance comparison panel**: three stacked subplots (one per strategy) showing minimum blade
  clearance vs. time; horizontal dashed red line at $d_{safe}$; orange dotted line at
  $d_{inspect}$; shaded red band in the violation zone; per-strategy coverage percentage and
  violation count in the subplot title.
- **Inspection coverage heatmap**: three $3 \times 15$ boolean grids (Blade × Segment) rendered
  as a red-yellow-green image; green = inspected, red = not inspected; coverage percentage
  annotated per strategy.
- **3D inspection animation (GIF)**: rotating view of turbine hub, three blades sweeping in
  real time (blue), drone orbit (red dot + trailing red arc); blade clearance displayed as live
  text annotation; frame rate 10 fps; trail length 60 frames.
- **Console metrics** (printed): per-strategy table of coverage %, safety violations (steps and
  fraction of mission), mean blade clearance, and minimum observed clearance.

Expected quantitative results for the proposed strategy over $T_{max} = 120$ s:
- Coverage $\geq 90\%$ across all three blades (target; the implementation reports how far gusts
  pull it below this)
- Zero hard safety violations ($d < d_{safe}$)
- Mean blade clearance $\approx d_{inspect} = 2.0$ m during the imaging sweeps (the original
  $\approx 1.2$ m would lie inside $d_{safe}$); during transit on the 17 m orbit the clearance never
  drops below $\sqrt{2^2 + 1^2} = 2.24$ m

---

## Extensions

1. **Variable rotor speed tracking**: replace the constant $\omega$ assumption with a wind-speed-
   dependent rotor speed model $\omega(t) = \omega_0 + \Delta\omega \sin(0.05 t)$; extend the
   phase-lock controller to use a Kalman filter estimating real-time $\omega$ from the SCADA encoder
   feed, enabling operation during transient speed changes (e.g., gusting conditions).
2. **Inspection path optimisation along the blade**: instead of fixing the drone at $r_{orbit}$,
   plan a spiral radial approach for each blade — sweeping from root ($s = 0$) to tip ($s = 1$) at
   $d_{inspect}$ standoff while the blade rotates through its upper half; optimise dwell time per
   segment to minimise total inspection time subject to the safety constraint.
3. **Thermal anomaly detection**: augment the inspection quality criterion with a thermal camera
   model; simulate delamination hotspots as elevated temperature regions $\Delta T = 5$ K along
   two randomly selected blade segments; require the drone to hover within $d_{inspect} \pm 0.1$ m
   for $\geq 2$ s at each suspected anomaly before flagging it.
4. **Tower shadow turbulence**: model the aerodynamic tower shadow as a periodic reduction in
   effective rotor torque when blades pass $\theta \approx 180°$; introduce a velocity perturbation
   to the drone when it is in the wake corridor ($|x| < 2$ m, $y \approx 0$); test controller
   robustness by quantifying phase error spike amplitude during shadow passages.
5. **Multi-turbine farm inspection**: schedule three identical drones across a $3 \times 3$ wind
   farm grid (9 turbines, 300 m spacing); solve the assignment problem (Hungarian algorithm) to
   minimise total travel time between inspection sequences; enforce no-fly zones within 5 m of
   any non-assigned turbine's rotor disk.

---

## Related Scenarios

- Prerequisites: [S061 Power Line Inspection](S061_power_line.md) (proximity tracking along linear structures), [S071 Bridge Underside Structural Inspection](S071_bridge_inspection.md) (confined-space proximity control)
- Follow-ups: [S079 Offshore Wind Farm Installation Assistance](S079_offshore_wind.md) (multi-turbine coordination and marine environment)
- Algorithmic cross-reference: [S005 Stealth Approach](../01_pursuit_evasion/S005_stealth_approach.md) (phase-based angular positioning), [S023 Moving Landing Pad](../02_logistics_delivery/S023_moving_landing_pad.md) (tracking a moving reference), [S044 Wall Crack Inspection](../03_environmental_sar/S044_wall_crack.md) (close-range surface inspection)
