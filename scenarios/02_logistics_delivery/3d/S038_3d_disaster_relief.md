# S038 3D Upgrade — Disaster Relief Drop

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S038 original](../S038_disaster_relief_drop.md)

---

## What Changes in 3D

The original S038 treats all flight as planar: drones cruise at a hardcoded `z = 20 m`, descend to
`z = 2 m` at each drop site, and all distance metrics (sortie range, insertion costs, kinematics)
use only the x-y components of position. Two critical effects are omitted:

1. **Ballistic drop physics**: payloads are modelled as instantaneous teleport to ground. In
   reality a package released at altitude $z_r$ with forward velocity $v_k$ follows a ballistic arc
   — the horizontal overshoot $\Delta x_{ballistic}$ must be pre-corrected.
2. **Wind-corrected release-point planning**: a measured wind vector $\mathbf{w} = (w_x, w_y)$
   applies aerodynamic drag to the descending package; the drone must release the payload upwind of
   the target to achieve a precision landing.
3. **3D debris-field avoidance**: post-disaster terrain contains rubble volumes modelled as
   axis-aligned bounding boxes (AABBs) at varying heights. Straight-line 2D routes may pass through
   these obstacles; 3D waypoint corridors must be planned to clear them.

This upgrade extends the position state, kinematics, route feasibility checks, and the drop
sub-routine to full 3D, enabling realistic high-fidelity airdrop simulation.

---

## Problem Definition

**Setup**: Identical disaster scenario to S038 — $M = 8$ relief sites in a $1000 \times 1000$ m
area, $K = 5$ heterogeneous drones departing from a FOB at a known position. The environment now
adds:
- A **steady wind field** $\mathbf{w} \in \mathbb{R}^3$ (horizontal components dominant, small
  vertical component possible) measured at the FOB before mission start.
- $N_{obs} = 6$ **debris volumes** represented as AABBs $\mathcal{O}_j = [x_j^-, x_j^+] \times
  [y_j^-, y_j^+] \times [0, z_j^{max}]$, where $z_j^{max} \leq 15$ m (below cruise altitude in
  clear air).
- Drone altitudes are now decision variables within $z \in [5, 40]$ m.

**Roles**:
- **FOB**: single depot; drones depart and return at $z_{FOB} = 2$ m.
- **Drones** ($K = 5$): same heterogeneous fleet as S038; now carry payload mass $m_p$ (kg) that
  is released at the wind-corrected release point $\mathbf{p}_{rel}$.
- **Relief sites** ($M = 8$): ground targets at $z = 0$ m; each has priority $\pi_i$, demand
  $q_i$, and a ground landing zone radius $r_{LZ} = 3$ m.

**Objective**: Maximise total weighted delivery value $J$ (same formulation as S038) subject to:
- 3D debris-field clearance on every inter-waypoint segment.
- Ballistic drop accuracy: the payload must land within $r_{LZ} = 3$ m of the site centre.
- Payload mass decreases drone aerodynamic range by $\Delta R_k = \alpha_R \cdot m_p$ per kg
  carried (heavier loads drain battery faster, reducing usable range).

---

## Mathematical Model

### 3D Drone Kinematics

Full 3D position state $\mathbf{p}_k = (x_k, y_k, z_k)^\top$. Constant-speed flight along
straight-line segments between 3D waypoints:

$$\dot{\mathbf{p}}_k = v_k \cdot \frac{\mathbf{w}_{next} - \mathbf{p}_k}{\|\mathbf{w}_{next} - \mathbf{p}_k\|}$$

Segment travel time between 3D waypoints $\mathbf{A}$ and $\mathbf{B}$:

$$\Delta t_{seg} = \frac{\|\mathbf{B} - \mathbf{A}\|}{ v_k}$$

### Payload-Adjusted Range

Carrying payload mass $m_p$ reduces effective one-way range by:

$$R_k^{eff}(m_p) = R_k - \alpha_R \cdot m_p, \qquad \alpha_R = 50 \text{ m/kg}$$

Range feasibility for a sortie visiting sites in sequence $\sigma_k^{(r)} = (i_1, \ldots, i_{n_r})$
with cumulative payload drop:

$$\sum_{j=0}^{n_r} \|\mathbf{w}_{j+1}^{3D} - \mathbf{w}_j^{3D}\| \leq R_k^{eff}\!\left(\sum_{i \in \sigma} q_i\right)$$

where $\mathbf{w}_j^{3D}$ are the full 3D cruise waypoints (including obstacle-avoidance detour
nodes) and $\mathbf{w}_0 = \mathbf{w}_{n_r+1} = \mathbf{p}_{FOB}^{3D}$.

### Ballistic Drop Model

At release altitude $z_r$ above ground with forward drone velocity $v_k$ and wind field
$\mathbf{w} = (w_x, w_y, 0)$, the payload free-falls under gravity with aerodynamic drag.
Using a simplified drag model with drag constant $C_d$:

$$\ddot{z} = -g + \frac{C_d}{m_p}(\dot{z}^2)\,\text{sgn}(-\dot{z})$$

The vertical time-of-flight $T_{fall}$ satisfies (terminal velocity $v_t = \sqrt{m_p g / C_d}$):

$$T_{fall} = \frac{v_t}{g} \ln\!\left(\cosh\!\left(\frac{g \cdot z_r}{v_t^2}\right)\right)^{1/2}
\cdot 2 \approx \sqrt{\frac{2 z_r}{g}} \quad \text{(low-drag approximation)}$$

Horizontal displacement during fall (drone forward velocity + wind drift):

$$\Delta \mathbf{p}_{bal} = \left(\mathbf{v}_{k,xy} + \mathbf{w}_{xy}\right) \cdot T_{fall}$$

where $\mathbf{v}_{k,xy}$ is the drone's horizontal velocity at release. Wind-corrected release
point for site $i$ at ground position $\mathbf{s}_i$:

$$\mathbf{p}_{rel}^{(i)} = \begin{pmatrix} s_{i,x} \\ s_{i,y} \\ z_r \end{pmatrix}
  - \begin{pmatrix} \Delta p_{bal,x} \\ \Delta p_{bal,y} \\ 0 \end{pmatrix}$$

The drone flies to $\mathbf{p}_{rel}^{(i)}$ at cruise altitude, releases the payload, then
continues to the next waypoint without descending. Drop dwell time reduces to $\tau_{drop}^{3D}
= 2$ s (release-and-go, no hover descent).

### Landing Accuracy Constraint

The payload lands at $\hat{\mathbf{s}}_i = \mathbf{s}_i + \boldsymbol{\epsilon}_{wind}$ where
$\boldsymbol{\epsilon}_{wind}$ is the residual wind-estimation error (assumed Gaussian,
$\sigma_w = 0.5$ m). Delivery is accepted if:

$$\|\hat{\mathbf{s}}_i - \mathbf{s}_i\| \leq r_{LZ} = 3 \text{ m}$$

The 3-sigma landing radius under estimated wind error:

$$r_{3\sigma} = 3 \sigma_w \cdot T_{fall} \leq r_{LZ}$$

This constrains minimum approach speed and maximum release altitude.

### 3D Debris-Field Avoidance (AABB Clearance)

Each segment $\mathbf{A} \to \mathbf{B}$ is checked against each debris AABB $\mathcal{O}_j$.
Parametric segment: $\mathbf{q}(t) = \mathbf{A} + t(\mathbf{B} - \mathbf{A}),\; t \in [0, 1]$.

AABB intersection test: the segment intersects $\mathcal{O}_j$ if and only if the slab-method
intervals $[t_{min}^{(j)}, t_{max}^{(j)}]$ across all three axes overlap and
$t_{max}^{(j)} \geq 0$, $t_{min}^{(j)} \leq 1$.

If a segment intersects an obstacle, a **vertical detour waypoint** is inserted at the
midpoint of the segment, raised to $z_{detour} = z_j^{max} + \Delta z_{clear}$:

$$\mathbf{w}_{detour} = \frac{\mathbf{A} + \mathbf{B}}{2} + \begin{pmatrix} 0 \\ 0 \\ z_j^{max} + \Delta z_{clear} \end{pmatrix}$$

with clearance margin $\Delta z_{clear} = 5$ m. The modified two-segment route
$\mathbf{A} \to \mathbf{w}_{detour} \to \mathbf{B}$ is re-checked recursively until all
segments are obstacle-free.

### 3D Route Feasibility and Sortie Range

Route length using full 3D waypoints (including detour nodes):

$$L_k^{3D}(\sigma) = \sum_{j} \|\mathbf{w}_{j+1} - \mathbf{w}_j\|_2$$

Sortie feasibility requires:

$$L_k^{3D}(\sigma) \leq R_k^{eff}\!\left(\textstyle\sum_{i \in \sigma} q_i\right)$$

Altitude bounds enforced at each waypoint: $z_k \in [5, 40]$ m during cruise.

### Delivery Time Propagation (3D)

Arrival time at site $i_j$ using 3D segment distances:

$$t_{arrive}(i_j) = t_0^{(r)} + \sum_{l=1}^{j} \frac{\|\mathbf{w}_l^{3D} - \mathbf{w}_{l-1}^{3D}\|}{v_k}
  + (j - 1) \cdot \tau_{drop}^{3D}$$

where $\tau_{drop}^{3D} = 2$ s (release-and-go; no hover descent).

### Weighted Delivery Value (unchanged from S038)

$$J = \sum_{i \in \mathcal{S}_{served}} V_i\!\left(t_{arrive}(i)\right) \cdot q_i
    - \mu \sum_{i \in \mathcal{S}_{unserved}} \frac{\pi_{max} + 1 - \pi_i}{\pi_{max}} \cdot q_i$$

$$V_i(t) = \frac{\pi_{max} + 1 - \pi_i}{\pi_{max}} \cdot \exp\!\left(-\lambda \cdot \max(0,\, t - d_i^{soft})\right)$$

---

## Key 3D Additions

- **Ballistic drop trajectory**: payload free-fall model with drag; release point pre-offset by wind drift and forward velocity.
- **Wind-corrected release point**: $\mathbf{p}_{rel}^{(i)} = \mathbf{s}_i^{3D} - \Delta\mathbf{p}_{bal}$; eliminates need to hover-descend over the site.
- **3D debris-field avoidance**: AABB slab-method intersection test on each inter-waypoint segment; automatic vertical detour insertion.
- **Payload-mass range penalty**: $R_k^{eff}(m_p) = R_k - \alpha_R m_p$; heavier sorties have shorter usable range.
- **Release-and-go drop protocol**: $\tau_{drop}^{3D} = 2$ s instead of 8 s, enabled by accurate ballistic pre-correction.
- **Full 3D route length metric**: sortie range and insertion costs use $\|\cdot\|_2$ in 3D.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Relief sites $M$ | 8 |
| Fleet size $K$ | 5 (heterogeneous) |
| Heavy-lift drones (0–1) | 5.0 kg, 2000 m range, 12 m/s |
| Mid-range drones (2–3) | 2.5 kg, 2800 m range, 16 m/s |
| Scout drone (4) | 1.0 kg, 3500 m range, 20 m/s |
| Range penalty $\alpha_R$ | 50 m/kg |
| Mission horizon $T_{max}$ | 600 s |
| Drop dwell time $\tau_{drop}^{3D}$ | 2 s (release-and-go) |
| Battery swap time $\tau_{turn}$ | 30 s |
| Value decay rate $\lambda$ | 0.01 s$^{-1}$ |
| Unserved penalty $\mu$ | 5.0 |
| Cruise altitude $z_{cruise}$ | 20 m |
| Altitude bounds | 5 – 40 m |
| Release altitude $z_r$ | 18 m (2 m below cruise) |
| Landing zone radius $r_{LZ}$ | 3 m |
| Wind estimation error $\sigma_w$ | 0.5 m |
| Drag constant $C_d$ | 0.3 kg/m |
| Debris obstacles $N_{obs}$ | 6 AABBs, $z^{max} \leq 15$ m |
| Obstacle clearance margin $\Delta z_{clear}$ | 5 m |
| Wind field $\mathbf{w}$ | $(3, 2, 0)$ m/s (example) |
| Area | 1000 × 1000 m |
| FOB position | (500, 100, 2) m |

---

## Expected Output

- **3D mission map**: full 3D trajectory plot (Matplotlib 3D axes) with FOB (black star), relief sites colour-coded by priority, debris AABBs rendered as grey wireframe boxes, and sortie paths drawn as coloured 3D polylines per drone; wind vector shown as an arrow at FOB.
- **Release-point diagram**: per-site 2D inset showing the nominal site centre, the wind-corrected release point, and the simulated payload landing scatter (Monte Carlo over wind estimation error).
- **Altitude profile**: z vs time for each drone over the full mission; highlights cruise segments, obstacle-avoidance climbs, and the constant release altitude.
- **Debris avoidance check**: before-and-after comparison of straight-line vs detour-corrected route segments, annotated with obstacle AABB projections.
- **Delivery value timeline**: cumulative $J(t)$ for the 3D planner vs the original 2D greedy baseline; shows value gain from the faster $\tau_{drop}^{3D} = 2$ s release-and-go protocol.
- **Landing accuracy scatter**: for each served site, the x-y touchdown scatter of the payload under wind-estimation noise; circle overlay at $r_{LZ} = 3$ m.
- **Animation (GIF)**: 3D view with drones flying their corrected 3D routes, payload objects appearing at the release point and following a parabolic arc to the ground, sites turning from hollow to filled upon successful delivery.

---

## Extensions

1. **Variable wind field**: replace the constant wind vector with a spatially varying field $\mathbf{w}(\mathbf{p})$ sampled from a Gaussian process; drones update their release-point correction as they traverse the field and collect local wind measurements.
2. **Terrain-following altitude**: integrate a digital elevation model (DEM) so the cruise altitude adapts to terrain height, maintaining a constant AGL (above-ground-level) clearance rather than a fixed MSL altitude.
3. **Multi-package sequential drop**: one sortie carries $n$ packages and drops them at consecutive release points computed individually per site; total flight time savings from eliminating hover descents enable extra site visits within $T_{max}$.
4. **Cooperative payload escort**: two drones fly in formation — one carries the payload and releases it while the second monitors the ballistic arc with a downward camera and reports touchdown error for real-time guidance correction (closed-loop airdrop).
5. **RL release-timing policy**: train a PPO agent to select the exact release instant given the drone's 3D velocity, altitude, and real-time wind measurement, replacing the open-loop ballistic pre-correction with an adaptive closed-loop drop control.

---

## Related Scenarios

- Original 2D version: [S038](../S038_disaster_relief_drop.md)
- Wind compensation reference: [S024](../S024_wind_compensation.md)
- Relay range extension: [S036](../S036_last_mile_relay.md)
- Cooperative lift (multi-drone payload): [S026](../S026_cooperative_heavy_lift.md)
- Weather rerouting (obstacle-aware replanning): [S034](../S034_weather_rerouting.md)
