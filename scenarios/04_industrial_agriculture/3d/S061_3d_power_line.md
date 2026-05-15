# S061 3D Upgrade — Power Line Inspection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S061 original](../S061_power_line.md)

---

## What Changes in 3D

The original S061 already operates in 3D space in a limited sense: the cable polyline has
genuine z-variation between pole attachment heights (8–12 m), and the Frenet frame uses
$\hat{N}_i = \hat{T}_i \times \hat{z}$ — a construction that always yields a horizontal lateral
normal regardless of the cable's actual tilt. Vertical offset is a fixed additive scalar
($\Delta z = 0.5$ m above cable), and the repulsive potential field acts only in the horizontal
plane (x-y) with no z-component from the pole top structure.

This 3D upgrade replaces those simplifications with:

1. **Catenary cable sag model**: each span between poles is replaced by a 3D catenary arc
   sampled at high resolution; the Frenet frame is derived analytically from the catenary tangent,
   so the lateral normal and binormal genuinely reflect cable curvature in both the sag plane and
   the horizontal.
2. **Multi-conductor bundle**: the transmission line carries three phase conductors arranged in a
   triangular cross-section bundle (equilateral triangle, side $w_{bundle} = 0.8$ m); the drone
   must orbit all three conductors in sequence at a fixed standoff $d_{stand} = 1.5$ m from each
   conductor axis, computing separate inspection passes per conductor.
3. **Tower structure 3D orbit**: at each lattice tower the drone executes a full 3D orbit around
   the tower body (not just horizontal avoidance) — climbing from cable attachment height to tower
   apex height along a helical arc before descending to resume cable following on the far side.
4. **Altitude-clearance hold**: instead of a fixed $\Delta z = 0.5$ m scalar, a clearance
   controller holds a constant 3D standoff distance $d_{clear} = 1.5$ m from the nearest cable
   surface (including bundle sag), correcting for ground proximity using an altitude floor
   $z_{min} = 3.0$ m.

---

## Problem Definition

**Setup**: A $300\ \text{m}$ three-phase transmission line spans four lattice towers, each
$H_{tower} = 20\ \text{m}$ tall. Conductors are attached to cross-arms at heights between
$14$ and $18$ m. Each span between towers follows a catenary curve with sag parameter $a$ chosen
to match a given sag-to-span ratio $\rho_{sag} = 0.03$ (sag depth $\approx 2.3$ m per 75 m
span). The three-phase bundle has conductors at vertices of an equilateral triangle with side
$w_{bundle} = 0.8$ m centred on the attachment point.

The drone must:

- Follow each phase conductor in sequence, maintaining standoff $d_{stand} = 1.5$ m from the
  conductor axis along its full 3D catenary arc.
- At each tower, exit the cable-following mode and execute a helical orbit around the tower
  structure (radius $R_{orbit} = 3.0$ m, altitude ramp from attachment height to apex height in
  $N_{orbit} = 1.5$ turns) before resuming on the far side.
- Maintain 3D clearance $d_{clear} = 1.5$ m from all conductors at all times.
- Keep altitude above the ground clearance floor $z_{min} = 3.0$ m.

**Roles**:
- **Drone**: single UAV with full 3D velocity control; Frenet-frame controller extended to
  catenary geometry; clearance controller as a parallel altitude channel.
- **Cable bundle**: three catenary conductors per span, sampled at $\Delta s_{cat} = 0.1$ m
  arc-length resolution; each conductor has radius $r_{cond} = 0.01$ m (excluded zone
  $r_{excl} = 0.5$ m including corona discharge safety margin).
- **Towers**: four lattice towers modelled as vertical cylinders of radius $r_{tower} = 0.4$ m
  and height $H_{tower} = 20$ m; helical orbit waypoints generated per tower.

**Objective**: Complete inspection of all three conductors across all four spans with zero
collisions ($d_{drone-conductor} > r_{excl}$ at all times), 3D path deviation from the nominal
standoff trajectory $\leq 0.3\ \text{m}$ (RMS), and coverage completeness $\geq 98\%$ per
conductor. Report per-conductor metrics and tower transition times.

---

## Mathematical Model

### Catenary Cable Geometry

For a span between attachment points $\mathbf{A} = (x_A, y_A, z_A)$ and
$\mathbf{B} = (x_B, y_B, z_B)$ with horizontal span $L = \|\mathbf{B} - \mathbf{A}\|_{xy}$
and sag parameter $a$, the catenary in the local span plane is:

$$z_{cat}(\xi) = a \cosh\!\left(\frac{\xi - \xi_0}{a}\right) + z_{offset},
\quad \xi \in \left[-\tfrac{L}{2},\, \tfrac{L}{2}\right]$$

where $\xi_0$ satisfies the boundary conditions at both attachments and $z_{offset}$ aligns
the sag vertex. The sag parameter $a$ is found implicitly from the sag-to-span ratio
$\rho_{sag} = f_{sag}/L$:

$$f_{sag} = a\left(\cosh\!\left(\frac{L}{2a}\right) - 1\right) = \rho_{sag} \cdot L$$

The 3D catenary point at parameter $\xi$ is:

$$\mathbf{c}(\xi) = \mathbf{A} + \frac{\xi + L/2}{L}(\mathbf{B} - \mathbf{A})_{xy}
+ z_{cat}(\xi)\,\hat{z}$$

### Catenary Frenet Frame (Analytic)

The unit tangent at arc-length parameter $s$ along the catenary:

$$\hat{T}(\xi) = \frac{d\mathbf{c}/d\xi}{\|d\mathbf{c}/d\xi\|}, \qquad
\frac{d\mathbf{c}}{d\xi} = \left(\frac{B_x - A_x}{L},\; \frac{B_y - A_y}{L},\;
\sinh\!\!\left(\frac{\xi - \xi_0}{a}\right)\right)$$

The principal normal (pointing toward center of curvature, upward for catenary):

$$\hat{N}_{cat}(\xi) = \frac{d\hat{T}/d\xi}{\|d\hat{T}/d\xi\|}$$

The binormal completing the right-hand frame:

$$\hat{B}_{cat}(\xi) = \hat{T}(\xi) \times \hat{N}_{cat}(\xi)$$

Unlike the original Frenet frame construction $\hat{N}_i = \hat{T}_i \times \hat{z}$, the
catenary normal $\hat{N}_{cat}$ is not confined to the horizontal plane — it tilts with the sag
curvature, allowing the standoff offset to be genuinely perpendicular to the curved conductor.

### Nominal Standoff Trajectory (3D)

For conductor $c \in \{1, 2, 3\}$ with centroid offset $\boldsymbol{\delta}_c$ (bundle
geometry in the local cross-section plane), the nominal drone position along arc-length $s$ is:

$$\mathbf{p}_{nom}^{(c)}(s) = \mathbf{c}^{(c)}(s) + d_{stand}\,\hat{B}_{cat}(s)$$

where $\hat{B}_{cat}(s)$ is chosen as the outward-lateral direction from the bundle centroid,
and $d_{stand} = 1.5\ \text{m}$ is the standoff distance.

### 3D Conductor Bundle Layout

Conductors are placed at vertices of an equilateral triangle in the plane perpendicular to
the span horizontal direction. With bundle side $w_{bundle} = 0.8$ m, the offsets from the
attachment centroid are:

$$\boldsymbol{\delta}_1 = \frac{w_{bundle}}{\sqrt{3}}\,\hat{z}, \qquad
\boldsymbol{\delta}_2 = \frac{w_{bundle}}{2}\,\hat{n}_{lat} - \frac{w_{bundle}}{2\sqrt{3}}\,\hat{z},
\qquad \boldsymbol{\delta}_3 = -\frac{w_{bundle}}{2}\,\hat{n}_{lat} - \frac{w_{bundle}}{2\sqrt{3}}\,\hat{z}$$

where $\hat{n}_{lat} = \hat{T} \times \hat{z} / \|\hat{T} \times \hat{z}\|$ is the
horizontal lateral direction.

### 3D Frenet Error Decomposition

Errors in the local catenary Frenet frame:

$$e_T(t) = \bigl(\mathbf{p}_{drone}(t) - \mathbf{p}_{nom}(t)\bigr) \cdot \hat{T}$$

$$e_N(t) = \bigl(\mathbf{p}_{drone}(t) - \mathbf{p}_{nom}(t)\bigr) \cdot \hat{N}_{cat}$$

$$e_B(t) = \bigl(\mathbf{p}_{drone}(t) - \mathbf{p}_{nom}(t)\bigr) \cdot \hat{B}_{cat}$$

Three-dimensional RMS path deviation:

$$\bar{e}_{path} = \sqrt{\frac{1}{T}\int_0^T \bigl(e_N^2(t) + e_B^2(t)\bigr)\,dt}$$

### PID Controllers in the Catenary Frame

Three independent PID loops in the analytic Frenet directions:

$$a_T(t) = K_p^{(T)}\bigl(v_{insp} - \dot{s}\bigr) - K_d^{(T)}\ddot{s}$$

$$a_N(t) = -K_p^{(N)}\,e_N - K_d^{(N)}\,\dot{e}_N - K_i^{(N)}\int e_N\,dt$$

$$a_B(t) = -K_p^{(B)}\,e_B - K_d^{(B)}\,\dot{e}_B$$

Total commanded acceleration in world frame:

$$\mathbf{a}_{PID} = a_T\,\hat{T} + a_N\,\hat{N}_{cat} + a_B\,\hat{B}_{cat}$$

### 3D Clearance Hold

The 3D clearance from conductor $k$ is:

$$d_k^{3D}(t) = \min_{s} \|\mathbf{p}_{drone}(t) - \mathbf{c}^{(k)}(s)\|$$

A repulsive potential acting in the full 3D gradient direction (not just horizontal):

$$\mathbf{F}_{rep,k}^{3D} = k_{rep}\!\left(\frac{1}{d_k^{3D}} - \frac{1}{d_0}\right)
\frac{1}{\bigl(d_k^{3D}\bigr)^2}\,\nabla d_k^{3D}$$

where $\nabla d_k^{3D}$ is the 3D unit vector from the nearest catenary point to the drone.

Ground clearance enforcement (altitude floor):

$$a_{z,floor}(t) = \max\!\left(0,\; K_{floor}\bigl(z_{min} - z_{drone}(t)\bigr)\right),
\quad z_{min} = 3.0\ \text{m}$$

Combined commanded acceleration:

$$\mathbf{a}_{cmd} = \mathbf{a}_{PID} + \sum_k \mathbf{F}_{rep,k}^{3D} + a_{z,floor}\,\hat{z}$$

### Tower Helical Orbit

At tower $m$ with base center $(x_m, y_m)$, attachment height $z_{att,m}$, and apex height
$H_{tower} = 20\ \text{m}$, the drone transitions to a helical waypoint sequence:

$$x_{hel}(\phi) = x_m + R_{orbit}\cos\phi, \qquad y_{hel}(\phi) = y_m + R_{orbit}\sin\phi$$

$$z_{hel}(\phi) = z_{att,m} + \frac{H_{tower} - z_{att,m}}{2\pi N_{orbit}}\,(\phi - \phi_0)
+ \frac{H_{tower} - z_{att,m}}{2\pi N_{orbit}}\,(\phi_{end} - \phi),
\quad \phi \in [\phi_0, \phi_0 + 2\pi N_{orbit}]$$

This ramps altitude from $z_{att,m}$ to $H_{tower}$ over the first half-turn, holds near the
apex for $\frac{1}{2}$ turn for apex inspection, then descends back to the exit attachment
height over the final turn. The orbit direction $\phi_0$ is chosen so the drone enters on the
inbound cable bearing and exits on the outbound cable bearing.

### Coverage Metric (per Conductor)

Conductor $c$ is discretised into $N_{seg} = 500$ arc-length segments. Segment $j$ is
inspected when the drone has maintained 3D standoff within $[d_{stand} - \epsilon,\,
d_{stand} + \epsilon]$ for at least $\Delta t_{min} = 0.5\ \text{s}$:

$$\text{inspected}^{(c)}(j) = \mathbf{1}\!\left[
\int_0^T \mathbf{1}\!\left[\bigl|d_{drone}^{(c,j)}(t) - d_{stand}\bigr| \leq \epsilon\right]dt
\geq \Delta t_{min}\right]$$

Per-conductor coverage completeness:

$$C^{(c)} = \frac{\#\{\text{inspected segments for conductor } c\}}{N_{seg}} \times 100\%$$

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Total line length | $S_{total}$ | $\approx 300$ m (4 spans) |
| Spans | — | 4 $\times$ 75 m |
| Number of towers | — | 4 |
| Tower height | $H_{tower}$ | 20 m |
| Conductor attachment heights | — | 14–18 m |
| Catenary sag-to-span ratio | $\rho_{sag}$ | 0.03 (sag $\approx 2.25$ m) |
| Conductor bundle side | $w_{bundle}$ | 0.8 m |
| Number of conductors (phases) | — | 3 |
| Conductor exclusion radius | $r_{excl}$ | 0.5 m |
| Standoff distance from conductor | $d_{stand}$ | 1.5 m |
| Altitude floor | $z_{min}$ | 3.0 m |
| Inspection speed | $v_{insp}$ | 1.0 m/s |
| Tower orbit radius | $R_{orbit}$ | 3.0 m |
| Tower orbit turns | $N_{orbit}$ | 1.5 |
| Potential field activation radius | $d_0$ | 2.0 m |
| Repulsive gain | $k_{rep}$ | 1.5 |
| GPS noise std dev | $\sigma_{GPS}$ | 0.1 m |
| Aerodynamic drag coefficient | $c_{drag}$ | 0.5 s$^{-1}$ |
| Along-track PID gains $(K_p, K_d)$ | — | 1.2, 0.4 |
| Cross-track PID gains $(K_p, K_d, K_i)$ | — | 2.0, 0.8, 0.1 |
| Binormal PID gains $(K_p, K_d)$ | — | 2.5, 1.0 |
| Altitude floor gain | $K_{floor}$ | 3.0 |
| Arc-length resolution (catenary) | $\Delta s_{cat}$ | 0.1 m |
| Coverage segments per conductor | $N_{seg}$ | 500 |
| Minimum dwell time per segment | $\Delta t_{min}$ | 0.5 s |
| Simulation timestep | $\Delta t$ | 0.05 s |

---

## Expected Output

- **3D full-line trajectory plot**: `mpl_toolkits.mplot3d` scene showing all three catenary
  conductors in black (with sag curvature visible), the nominal standoff paths for all three
  phases in green/cyan/magenta, the actual drone trajectory for each conductor inspection pass
  colour-mapped by 3D clearance error magnitude, tower cylinders as grey tubes, and helical
  orbit arcs in orange.
- **Altitude time series**: z-coordinate of the drone vs elapsed mission time, with shaded
  bands marking cable-following phases (blue) and tower-orbit phases (orange); horizontal dashed
  lines at $z_{min} = 3.0$ m floor and at each attachment height.
- **Per-conductor error panel**: three stacked subplots (one per phase conductor), each showing
  $e_N(t)$ and $e_B(t)$ vs arc-length $s$ with $\pm 0.3$ m tolerance bands shaded yellow;
  grey vertical bands at tower transition zones.
- **Coverage bar chart (per conductor)**: three side-by-side bar charts of dwell time per
  arc-length segment for phases 1, 2, 3; teal bars for inspected segments, coral for
  under-dwelled; overall completeness $C^{(c)}$ printed per panel.
- **Catenary sag validation plot**: overlay of the analytic catenary vs the original
  piecewise-linear polyline for a single span, showing sag depth and Frenet frame tilt
  discrepancy.
- **Animation** (`FuncAnimation`): 3D animated view rotating around the tower-line scene,
  showing the drone moving along each conductor in turn with a trailing trajectory ribbon and
  instantaneous clearance sphere; saved as
  `outputs/04_industrial_agriculture/s061_3d_power_line/s061_3d_inspection.gif`.

Terminal metrics printed at completion:

```
Total mission time:         910.0 s
Conductor 1 coverage:       98.8 %
Conductor 2 coverage:       98.4 %
Conductor 3 coverage:       98.6 %
RMS path deviation (C1):    0.11 m
RMS path deviation (C2):    0.13 m
RMS path deviation (C3):    0.12 m
Min 3D clearance observed:  0.58 m  (> 0.50 m exclusion zone)
Tower orbit transitions:    8  (entry + exit per tower)
Collision count:            0
```

---

## Extensions

1. **Wind shear across sag**: apply a height-dependent crosswind profile $v_{wind}(z)$ using a
   power-law shear model; the catenary under lateral wind shifts sideways (forming a 3D skewed
   catenary); extend the geometry model and clearance controller to track the wind-deflected
   conductor centerline.
2. **Ice loading and asymmetric sag**: increase the catenary load parameter $a$ for one span to
   simulate ice accumulation; the drone must detect the anomalous sag depth from clearance
   telemetry and flag the over-loaded span autonomously.
3. **Bi-directional bundle orbit**: instead of a single standoff pass per conductor, the drone
   executes a full 360° cross-sectional orbit around each conductor at each arc-length station,
   enabling detection of corrosion on the back face invisible from the nominal standoff direction.
4. **Multi-drone cooperative inspection**: assign one drone per phase conductor flying in
   formation with staggered arc-length offsets; use a consensus protocol to maintain inter-drone
   separation $> 5$ m at all tower crossings; minimise total mission time by maximising parallel
   coverage.
5. **Thermographic sag estimation**: instrument the drone with a simulated IR sensor; use
   measured conductor temperature distribution along arc-length to infer current loading and
   estimate sag parameter $a$ in real time, comparing to the design catenary.

---

## Related Scenarios

- Original 2D/3D-lite version: [S061](../S061_power_line.md)
- 3D orbit reference: [S002 3D Upgrade](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
- Tower structure inspection: [S071 Bridge Inspection](../S071_bridge_inspection.md)
- Pipeline following (similar Frenet approach): [S072 Pipeline Leak](../S072_pipeline_leak.md)
- 3D scan path planning: [S065 Building 3D Scan Path](../S065_3d_scan_path.md)
