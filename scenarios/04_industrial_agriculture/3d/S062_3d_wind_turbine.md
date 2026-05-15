# S062 3D Upgrade — Wind Turbine Blade Inspection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S062 original](../S062_wind_turbine.md)

---

## What Changes in 3D

The original S062 fixes the drone at a constant lateral offset $y_{offset} = 1.0$ m from the rotor
plane and restricts it to a circular orbit of fixed radius $r_{orbit} = 17$ m in the hub-centred
$xz$-plane. This captures only a single side of each blade at a single radial standoff — the
leading edge, trailing edge, and blade tip region all receive identical (and geometrically
suboptimal) camera geometry. Three additional limitations follow:

1. The drone cannot vary its radial distance from the hub, so it covers the root and the tip with
   the same standoff even though the tip moves much faster and requires a different approach angle.
2. The single fixed $y_{offset}$ means the pressure-side and suction-side surfaces of every blade
   are never individually targeted.
3. Wake turbulence behind the rotor plane — the dominant aerodynamic hazard during in-service
   inspection — is never modelled.

This 3D upgrade lifts all three restrictions. The drone now executes a **helical radial sweep** per
blade: it spirals inward from root to tip along the leading-edge face, crosses the tip in a tight
arc, and sweeps back along the trailing-edge face, continuously adjusting both orbit radius $r(t)$
and lateral offset $y(t)$. Wake turbulence is modelled as a stochastic velocity disturbance in the
downstream corridor, and the safety margin controller is extended from 1-D angular repulsion to
full 3-D gradient repulsion.

---

## Problem Definition

**Setup**: A horizontal-axis wind turbine stands with hub at $\mathbf{p}_{hub} = (0, 0, 30)$ m.
Three blades of length $R_{blade} = 15$ m rotate in the rotor plane ($y = 0$) at
$\omega = 0.5$ rad/s. The inspection drone must achieve full-surface coverage of every blade
across four surface zones per blade:

| Zone | Description | Nominal $y_{offset}$ | Radial sweep direction |
|------|-------------|----------------------|------------------------|
| LE (leading edge) | upwind face | $-1.5$ m | root $\to$ tip |
| Tip cap | blade tip hemisphere | $0$ m (in-plane arc) | arc around tip |
| TE (trailing edge) | downwind face | $+1.5$ m | tip $\to$ root |
| Root fillet | hub-to-root transition | $0$ m | hover at $s = 0.05$ |

The drone carries a stereo optical + thermal camera pair with a $\pm 30°$ boresight cone. The
minimum safe separation from any blade surface is $d_{safe} = 1.5$ m. Target inspection standoff
on leading/trailing-edge faces is $d_{inspect} = 1.0$ m; at the tip cap it is $d_{inspect} = 0.8$ m.

**Roles**:
- **Drone**: single inspection agent; 3-D position controlled via full Cartesian velocity commands;
  executes a pre-planned zone schedule interleaved with blade-phase gating.
- **Rotor**: three blades whose angular state $\theta_{blade,i}(t) = \omega t + \phi_i$ is
  telemetered to the drone in real time (SCADA encoder).
- **Wake field**: a stochastic wind-disturbance model active in the corridor
  $y \in [+0.5, +4.0]$ m downstream of the rotor plane.

**Objective**: Achieve $\geq 95\%$ surface coverage across all four zones of all three blades
within a single inspection mission of $T_{max} = 180$ s, with zero hard safety violations and
total wake-induced position error $< 0.3$ m RMS.

---

## Mathematical Model

### Blade Kinematics (Full 3D)

Hub position: $\mathbf{p}_{hub} = (0,\, 0,\, 30)^\top$ m. Blade $i$ angle:

$$\alpha_i(t) = \omega t + \phi_i, \qquad \phi_i = \frac{2\pi i}{3}, \quad i = 0, 1, 2$$

The unit radial vector of blade $i$ in the rotor plane:

$$\hat{\mathbf{u}}_i(t) = \begin{bmatrix} \cos\alpha_i(t) \\ 0 \\ \sin\alpha_i(t) \end{bmatrix}$$

Point at fractional arc-length $s \in [0, 1]$ along blade $i$:

$$\mathbf{p}_{blade,i}(s, t) = \mathbf{p}_{hub} + s \cdot R_{blade} \cdot \hat{\mathbf{u}}_i(t)$$

Blade chord direction (perpendicular to $\hat{\mathbf{u}}_i$ within the rotor plane):

$$\hat{\mathbf{c}}_i(t) = \begin{bmatrix} -\sin\alpha_i(t) \\ 0 \\ \cos\alpha_i(t) \end{bmatrix}$$

The blade has aerodynamic thickness $\delta_{chord} = 0.4$ m (tapered; approximated as constant for
safety margin purposes).

### 3D Inspection Waypoint Generation

For each blade $i$ and each surface zone $z \in \{\text{LE},\, \text{Tip},\, \text{TE},\, \text{Root}\}$,
the desired drone position as a function of blade arc-length $s$ is:

$$\mathbf{w}_{i,z}(s,t) = \mathbf{p}_{blade,i}(s,t) + d_{inspect,z}\,\hat{\mathbf{n}}_{i,z}(t)$$

where the standoff normal $\hat{\mathbf{n}}_{i,z}$ depends on the zone:

$$\hat{\mathbf{n}}_{i,\text{LE}}(t) = -\hat{\mathbf{y}} = (0, -1, 0)^\top \quad \text{(upwind)}$$

$$\hat{\mathbf{n}}_{i,\text{TE}}(t) = +\hat{\mathbf{y}} = (0, +1, 0)^\top \quad \text{(downwind)}$$

$$\hat{\mathbf{n}}_{i,\text{Tip}}(t) = \hat{\mathbf{u}}_i(t) \quad \text{(radially outward from tip)}$$

$$\hat{\mathbf{n}}_{i,\text{Root}}(t) = -\hat{\mathbf{u}}_i(t) + (0, -1, 0)^\top \big/ \sqrt{2}
  \quad \text{(inward-upwind diagonal)}$$

The tip-cap arc is parameterised by azimuth $\psi \in [-90°, +90°]$ around the tip:

$$\mathbf{w}_{i,\text{Tip}}(\psi, t) = \mathbf{p}_{blade,i}(1,t)
  + d_{inspect,\text{Tip}} \bigl(\cos\psi\,\hat{\mathbf{u}}_i(t) + \sin\psi\,\hat{\mathbf{y}}\bigr)$$

### Radial Sweep Profile

During a leading-edge pass on blade $i$, the orbit radius varies as:

$$r(s) = R_{blade} \cdot s + d_{inspect,\text{LE}}$$

so that the drone tracks a constant standoff from the blade surface regardless of radial position.
The corresponding sweep speed along the blade must satisfy:

$$\left|\dot{s}\right| \leq \frac{v_{max,inspect}}{R_{blade}} = \frac{1.0}{15} \;\text{s}^{-1}$$

to keep the drone velocity below $v_{max,inspect} = 1.0$ m/s while the blade also rotates.

### Phase-Gated Zone Scheduling

Each zone pass begins only when blade $i$ is in its **inspection window** — the arc of blade
angular positions where the zone is accessible without requiring the drone to cross the rotor plane:

| Zone | Inspection window (blade angle $\alpha_i$) |
|------|---------------------------------------------|
| LE | $\alpha_i \in [80°, 100°]$ (blade near top, upwind side accessible) |
| Tip cap | $\alpha_i \in [85°, 95°]$ (blade at apex) |
| TE | $\alpha_i \in [80°, 100°]$ (blade near top, downwind side accessible) |
| Root | $\alpha_i \in [60°, 120°]$ (wider window, slow hover) |

The gate condition for starting a pass on blade $i$, zone $z$ at time $t$:

$$G_{i,z}(t) = \mathbf{1}\!\left[\alpha_i(t) \bmod 2\pi \in [\alpha_{lo,z},\, \alpha_{hi,z}]\right]
  \;\wedge\; \mathbf{1}\!\left[d_{seg,i}(t) > d_{safe}\right]$$

where $d_{seg,i}(t)$ is the minimum distance from the drone to any point on blade $i$.

### 3D Safety Margin Controller

Minimum distance from drone position $\mathbf{p}$ to blade $i$ (line-segment):

$$d_{seg,i}(\mathbf{p}, t) = \min_{s \in [0,1]} \left\|\mathbf{p} - \mathbf{p}_{blade,i}(s,t)\right\|$$

The closest point parameter:

$$s^*_i = \mathrm{clip}\!\left(\frac{(\mathbf{p} - \mathbf{p}_{hub}) \cdot \hat{\mathbf{u}}_i(t)}{R_{blade}},\; 0,\; 1\right)$$

Full 3D repulsion vector (gradient of the distance field):

$$\mathbf{f}_{rep,i}(\mathbf{p}, t) = \begin{cases}
K_{safe} \cdot \dfrac{d_{safe} - d_{seg,i}}{d_{safe}} \cdot
\dfrac{\mathbf{p} - \mathbf{p}_{blade,i}(s^*_i, t)}{\|\mathbf{p} - \mathbf{p}_{blade,i}(s^*_i, t)\|}
& \text{if } d_{seg,i} < d_{safe} \\[8pt]
\mathbf{0} & \text{otherwise}
\end{cases}$$

Total safety command added to the nominal waypoint-tracking velocity:

$$\mathbf{v}_{safety}(\mathbf{p}, t) = \sum_{i=0}^{2} \mathbf{f}_{rep,i}(\mathbf{p}, t)$$

### Wake Turbulence Model

The rotor wake occupies the corridor $y > 0$ (downstream, $+y$ direction). The turbulence
intensity decays with downstream distance $d_y = \max(y - 0.2, 0)$ m:

$$\sigma_{wake}(y) = \sigma_{w,0} \cdot \exp\!\left(-\frac{d_y}{\ell_{wake}}\right),
  \qquad \sigma_{w,0} = 0.8\;\text{m/s},\quad \ell_{wake} = 3.0\;\text{m}$$

The instantaneous wake disturbance velocity applied to the drone:

$$\mathbf{v}_{wake}(t) = \sigma_{wake}(y_{drone}) \cdot \boldsymbol{\xi}(t),
  \qquad \boldsymbol{\xi}(t) \sim \mathcal{N}(\mathbf{0},\, \mathbf{I}_3)$$

The disturbance is clipped to $\|\mathbf{v}_{wake}\| \leq 2\sigma_{wake}$ to avoid unphysical
instantaneous jumps.

### 3D Inspection Quality Metric

Each blade surface is discretised into a grid of $N_{rad} \times N_{zone} = 15 \times 4 = 60$
cells. Cell $(k, z)$ of blade $i$ is marked inspected when:

$$\text{insp}_{i,k,z}(t) = \mathbf{1}\!\left[
  d_{cell}(t) \leq d_{inspect,z} + \varepsilon
  \;\wedge\;
  v_{drone}(t) \leq v_{max,inspect}
  \;\wedge\;
  \cos\angle(\hat{\mathbf{n}}_{cam},\, \hat{\mathbf{r}}_{i,k,z}) \geq \cos 30°
  \;\wedge\;
  \hat{\mathbf{n}}_{cam} \cdot \hat{\mathbf{n}}_{i,z} \geq \cos 45°
\right]$$

The fourth condition ensures the camera boresight is roughly aligned with the zone normal
(prevents glancing-angle measurements from being credited to the wrong surface).

Overall surface coverage:

$$\text{Coverage}_{3D} = \frac{1}{3 \cdot N_{rad} \cdot N_{zone}}
  \sum_{i=0}^{2}\sum_{k=1}^{N_{rad}}\sum_{z} \mathbf{1}\!\left[\exists\, t:\, \text{insp}_{i,k,z}(t)=1\right]
  \times 100\%$$

---

## Key 3D Additions

- **Zone-differentiated standoff normals**: $\hat{\mathbf{n}}_{i,z}$ changes sign between LE and
  TE passes; tip-cap uses a radially outward normal with azimuth sweep.
- **Radial sweep with varying orbit radius**: $r(s) = R_{blade} \cdot s + d_{inspect}$ tracks the
  blade surface at constant standoff along the full span, not just at the tip.
- **Phase-gated scheduling**: zone passes are interleaved across three blades using the inspection
  window gate $G_{i,z}(t)$ to avoid conflicts and rotor-plane crossings.
- **Full 3D gradient repulsion**: safety vector computed from the nearest point on the line segment
  (not just the tip), with proportional gain scaling on penetration depth.
- **Wake turbulence disturbance**: stochastic velocity perturbation active in the $y > 0$ corridor;
  exponential decay with downstream distance; controller must reject disturbances within
  $\|\Delta\mathbf{p}\| < 0.3$ m RMS.
- **Altitude variation**: drone $z$-coordinate changes continuously during radial sweeps and tip-cap
  arcs; no fixed-altitude constraint.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Hub height | $h_{hub}$ | 30 m |
| Blade length | $R_{blade}$ | 15 m |
| Rotor angular velocity | $\omega$ | 0.5 rad/s |
| Number of blades | $N_{blades}$ | 3 |
| Blade chord thickness (approx.) | $\delta_{chord}$ | 0.4 m |
| LE / TE standoff distance | $d_{inspect,\text{LE/TE}}$ | 1.0 m |
| Tip-cap standoff distance | $d_{inspect,\text{Tip}}$ | 0.8 m |
| Tip-cap azimuth sweep | $\psi$ | $[-90°, +90°]$ |
| Hard safety margin | $d_{safe}$ | 1.5 m |
| Max inspection speed | $v_{max,inspect}$ | 1.0 m/s |
| Radial segments per blade | $N_{rad}$ | 15 |
| Surface zones per blade | $N_{zone}$ | 4 (LE, Tip, TE, Root) |
| Safety repulsion gain | $K_{safe}$ | 3.0 m/s² |
| Wake turbulence intensity | $\sigma_{w,0}$ | 0.8 m/s |
| Wake decay length | $\ell_{wake}$ | 3.0 m |
| GPS noise std dev | $\sigma_{GPS}$ | 0.1 m |
| PD phase-lock gain $K_p$ | — | 2.0 rad/(rad·s) |
| PD phase-lock gain $K_d$ | — | 0.3 s |
| Mission horizon | $T_{max}$ | 180 s |
| Simulation timestep | $\Delta t$ | 0.05 s |

---

## Expected Output

- **3D trajectory plot**: full 3D drone path colour-coded by active zone (LE = blue, Tip = cyan,
  TE = orange, Root = purple, transit = grey); rotor disk outline (dashed), tower (solid grey),
  three blade snapshots at equally-spaced times; view from slightly above and to the side to show
  $y$-offset variation between LE and TE passes.
- **Altitude and lateral offset time series**: two stacked subplots showing $z(t)$ and $y(t)$
  continuously varying during the mission; overlaid shading shows active blade inspection windows.
- **Radial orbit distance vs time**: $r_{hub}(t) = \|\mathbf{p}_{drone} - \mathbf{p}_{hub}\|$
  showing the sweep between root ($r \approx 16$ m) and tip ($r \approx 17$ m) during LE/TE passes,
  and the close approach ($r \approx 15.8$ m) during tip-cap arcs.
- **3D surface coverage heatmap**: three $4 \times 15$ grids (Zone × Radial segment) rendered as
  red-yellow-green images; one grid per blade; coverage percentage annotated per zone and per blade.
- **Wake turbulence effect plot**: drone position error $\|\mathbf{p}_{drone} - \mathbf{p}_{cmd}\|$
  vs time, with $y_{drone}(t)$ overlaid; shaded bands show wake-active periods ($y > 0$).
- **Minimum blade clearance time series**: single plot for the proposed 3D controller; horizontal
  dashed lines at $d_{safe}$ and $d_{inspect}$; colour-coded by active zone to see which zone is
  most demanding.
- **3D inspection animation (GIF)**: rotating 3D view; blades sweep in real time (dark blue); drone
  (red dot) follows the zone schedule with colour-coded trail; text annotation shows active zone,
  blade index, and current clearance; 10 fps.

Expected quantitative results:
- 3D surface coverage $\geq 95\%$ across all zones and all three blades
- Zero hard safety violations ($d_{seg} < d_{safe}$)
- Wake-induced position error $< 0.3$ m RMS during TE passes (worst case, maximum wake exposure)
- Tip-cap arc completed in $< 6$ s per blade (within the $\approx 10°$ inspection window)

---

## Extensions

1. **Chord-resolved blade surface mesh**: replace the line-segment blade model with a flat-plate
   approximation of width $\delta_{chord}(s)$ tapering from root to tip; redefine the standoff
   normal per surface cell and require the camera to resolve surface features at $\leq 2$ mm/pixel
   given a sensor resolution of $4096 \times 3000$ px and field of view $60°$.
2. **Variable rotor speed and shutdown sequencing**: model a blade inspection during controlled
   slow-down from $\omega = 0.5$ rad/s to $\omega = 0$ (parked position); update the phase-gate
   windows dynamically as $\omega(t)$ decreases; park the blade at $\alpha_i = 90°$ to maximise
   inspection window duration.
3. **Dual-drone coordination — leading/trailing arrangement**: assign one drone to the LE face and
   a second to the TE face of the same blade simultaneously; use a shared SCADA clock to maintain
   $\Delta y = 3.0$ m separation across the rotor plane; solve the mutual collision avoidance
   problem as a linear program over the two drones' radial sweep rates.
4. **Wind-field estimation from wake disturbances**: fit a Dryden turbulence model to the observed
   velocity residuals during TE passes; use the fitted parameters to predict the worst-case lateral
   displacement for any future pass and pre-compensate the waypoint position.
5. **Thermal delamination localisation in 3D**: place two delamination hotspots at known $(s, z)$
   positions on different surface zones; require the drone to confirm detection from two independent
   viewing angles (LE and TE) within the mission; plan the confirmation visit as an online
   replanning problem triggered by a threshold exceedance in the thermal image.

---

## Related Scenarios

- Original 2D version: [S062 Wind Turbine Blade Inspection](../S062_wind_turbine.md)
- Structural cross-reference: [S061 Power Line Inspection](../S061_power_line.md) (radial
  standoff tracking along elongated structures), [S071 Bridge Underside Inspection](../S071_bridge_inspection.md) (close-proximity 3D surface coverage)
- Dynamic target tracking: [S023 Moving Landing Pad](../../02_logistics_delivery/S023_moving_landing_pad.md) (tracking a rotating reference frame)
- Multi-turbine follow-up: [S079 Offshore Wind Installation](../S079_offshore_wind.md) (fleet
  scheduling across a wind farm)
- Wake modelling reference: [S058 Typhoon Reconnaissance](../../03_environmental_sar/S058_typhoon.md) (wind-field disturbance rejection)
