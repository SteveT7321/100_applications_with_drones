# S091 3D Upgrade — Acrobatic Maneuvers

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S091 original](../S091_acrobatics.md)

---

## What Changes in 3D

The original S091 fixes the position reference at `HOME_POS = (0, 0, 10)` for the entire sequence
(`p_ref_arr = np.tile(HOME_POS, (n_steps, 1))`). The four manoeuvres are pure attitude sequences
executed in place: the drone spins but translates at most a few metres from home under PD
position recovery. The spatial trajectory is therefore nearly a single point in 3D space, and no
minimum-snap path is planned in translation.

This 3D upgrade replaces the stationary position reference with a **full aerobatic spatial
trajectory**: each manoeuvre is associated with a distinct geometric path through 3D space
(a circular loop, a rolling helix, a split-S dive-and-climb arc, an Immelmann half-loop with
course reversal). Position waypoints are connected by **minimum-snap polynomial segments**
optimised jointly over both translational and rotational axes. Aerobatic envelope constraints
(peak load factor, minimum airspeed, structural g-limits, altitude floor) are enforced as
inequality constraints on the optimised trajectory. The geometric controller on SO(3) is retained
but its position reference now tracks the planned 3D path rather than a hover point.

---

## Problem Definition

**Setup**: A single quadrotor executes a four-manoeuvre aerobatic sequence in the volume
$[{-15}, 15] \times [{-15}, 15] \times [5, 20]$ m. The sequence proceeds along a closed oval
flight path; each manoeuvre occupies a distinct spatial segment so that the trajectory is
visually distinguishable in a 3D plot.

Manoeuvre sequence:

1. **Inside Loop** — the drone climbs from $z = 8$ m, traces a full vertical circle of radius
   $r_{loop} = 4$ m in the $x$-$z$ plane, exits at the same altitude and heading as entry.
   Duration $T_{loop} = 4.0$ s.
2. **Aileron Roll** — executing a full 360° roll about the body $x$-axis while translating at
   constant forward speed $v_{fwd} = 5$ m/s along the $y$-axis. Duration $T_{roll} = 2.0$ s.
3. **Split-S** — starting inverted, the drone performs a half-loop downward ($-z$ half-circle of
   radius $r_{split} = 3$ m), reversing direction and recovering at altitude floor $z_{floor} = 6$ m.
   Duration $T_{split} = 3.5$ s.
4. **Immelmann Turn** — the drone performs a half-loop upward ($+z$ half-circle of radius
   $r_{imm} = 3.5$ m), then executes a 180° roll about the body $x$-axis to restore upright
   attitude with course reversed. Duration $T_{imm} = 3.5$ s.

Between each manoeuvre a 1.5-second minimum-snap transit segment connects the exit state of one
manoeuvre to the entry state of the next, enforcing $C^4$ continuity (position, velocity,
acceleration, and jerk match at knot points).

**Roles**:
- **Drone**: single quadrotor, mass $m = 0.5$ kg, inertia $I = \mathrm{diag}(I_{xx}, I_{yy},
  I_{zz})$; full 6-DOF rigid-body dynamics; four-rotor thrust allocation.
- **Spatial reference**: minimum-snap polynomial trajectory in $\mathbb{R}^3$, piecewise degree-7
  polynomials whose coefficients are found by solving a constrained QP.
- **Attitude reference**: coupled to the translational trajectory — at each waypoint the desired
  body $z$-axis is aligned with the centripetal acceleration direction of the loop/arc, then slerp-
  interpolated along the SO(3) geodesic between coupled waypoint attitudes.
- **Geometric controller**: tracks both the spatial minimum-snap position trajectory and the
  coupled attitude reference simultaneously on SO(3).

**Objective**: Complete all four manoeuvres while satisfying:

1. **Path tracking**: RMS position error $\bar{e}_{pos} \leq 0.4$ m over the full sequence.
2. **Attitude tracking**: RMS geometric attitude error $\bar{e}_R \leq 0.20$ rad.
3. **Load factor**: normalised specific force $n = \|m\dot{v} + mg\hat{z}\| / (mg) \leq n_{max} = 3.0$ g.
4. **Altitude floor**: $z(t) \geq z_{floor} = 5.5$ m (safety margin above ground).
5. **Angular rate bound**: $\|\Omega\|_\infty \leq 15$ rad/s.
6. **Manoeuvre completion angle**: geodesic residual $\delta_k \leq 8°$ at end of each manoeuvre.

---

## Mathematical Model

### Minimum-Snap Trajectory (Position)

The position trajectory on segment $i$ is represented as a degree-7 polynomial in time:

$$\mathbf{p}(t) = \sum_{j=0}^{7} \mathbf{c}_{i,j} \left(\frac{t - t_i}{T_i}\right)^j, \quad t \in [t_i, t_{i+1}]$$

where $T_i = t_{i+1} - t_i$ is the segment duration and $\mathbf{c}_{i,j} \in \mathbb{R}^3$ are
the coefficient vectors. The snap (fourth derivative) cost is minimised:

$$J_{snap} = \sum_{i} \int_{t_i}^{t_{i+1}} \left\|\frac{d^4 \mathbf{p}}{dt^4}\right\|^2 dt = \mathbf{c}^\top Q_{snap} \mathbf{c}$$

where $Q_{snap}$ is the block-diagonal Hessian of the snap energy, assembled analytically for
degree-7 polynomials. The QP is solved subject to:

- **Boundary conditions** at $t = t_0$ and $t = t_N$: $\mathbf{p}, \dot{\mathbf{p}},
  \ddot{\mathbf{p}}, \dddot{\mathbf{p}}$ prescribed.
- **Continuity constraints** at interior knots $t_i$ ($i = 1, \ldots, N-1$): $C^4$ matching
  (position through jerk continuous).
- **Waypoint passage**: $\mathbf{p}(t_i) = \mathbf{w}_i$ for each aerobatic waypoint.

For the loop and arc segments, the waypoints are sampled on the geometric circle at equal arc-
length intervals; the number of interior waypoints per manoeuvre is $n_{wp} = 8$.

### Centripetal Acceleration and Coupled Attitude

On a circular arc of radius $r$ traversed at speed $v_{arc}$, the centripetal acceleration is:

$$\mathbf{a}_{c}(t) = \frac{v_{arc}^2}{r} \hat{\mathbf{n}}(t)$$

where $\hat{\mathbf{n}}(t)$ is the inward unit normal to the arc. The desired body $z$-axis (thrust
direction) must provide both lift and centripetal force; the total desired acceleration is:

$$\mathbf{a}_{des}(t) = \ddot{\mathbf{p}}_{ref}(t) + g\,\mathbf{e}_3$$

The nominal desired attitude $R_{des}(t)$ is constructed by aligning the body $z$-axis with
$\hat{\mathbf{a}}_{des} = \mathbf{a}_{des}/\|\mathbf{a}_{des}\|$ and choosing the body $x$-axis
to point forward along the trajectory tangent, projected onto the plane perpendicular to
$\hat{\mathbf{a}}_{des}$:

$$\mathbf{b}_3 = \hat{\mathbf{a}}_{des}, \quad \mathbf{b}_2 = \frac{\mathbf{b}_3 \times \dot{\mathbf{p}}_{ref}}{\|\mathbf{b}_3 \times \dot{\mathbf{p}}_{ref}\|}, \quad \mathbf{b}_1 = \mathbf{b}_2 \times \mathbf{b}_3$$

$$R_{des}(t) = \begin{bmatrix} \mathbf{b}_1 & \mathbf{b}_2 & \mathbf{b}_3 \end{bmatrix}$$

For the roll and inverted segments where $\mathbf{a}_{des}$ is aligned with $-\mathbf{e}_3$,
a prescribed roll angle $\phi_{roll}(t)$ is overlaid by composing:

$$R_{ref}(t) = R_{des}(t)\, R_x(\phi_{roll}(t))$$

where $R_x(\phi) = \exp(\phi\,\hat{\mathbf{e}}_1)$ is a rotation about the body $x$-axis. The roll
profile $\phi_{roll}(t)$ follows the cubic-Hermite smooth-step from $0$ to $2\pi$ (aileron roll)
or $0$ to $\pi$ (half-roll for Immelmann recovery).

### Full Quadrotor Dynamics (unchanged from S091)

$$\dot{\mathbf{p}} = \mathbf{v}, \quad m\dot{\mathbf{v}} = f\,R\mathbf{e}_3 - mg\,\mathbf{e}_3$$

$$\dot{R} = R\,\hat{\Omega}, \quad I\dot{\Omega} = \tau - \Omega \times I\Omega$$

### Geometric Controller on SO(3) with Feedforward (unchanged from S091)

Attitude error:

$$e_R = \frac{1}{2}\!\left(R_{ref}^\top R - R^\top R_{ref}\right)^\vee$$

Angular velocity error:

$$e_\Omega = \Omega - R^\top R_{ref}\,\Omega_{ref}$$

Control torque:

$$\tau = -K_R\,e_R - K_\Omega\,e_\Omega + \Omega \times I\Omega - I\!\left(\hat{\Omega}\,R^\top R_{ref}\,\Omega_{ref} - R^\top R_{ref}\,\dot{\Omega}_{ref}\right)$$

Position-tracking thrust (now tracking a time-varying $\mathbf{p}_{ref}(t)$ from the polynomial):

$$\mathbf{a}_{des} = -K_x\,(\mathbf{p} - \mathbf{p}_{ref}) - K_v\,(\mathbf{v} - \mathbf{v}_{ref}) + g\,\mathbf{e}_3 + \ddot{\mathbf{p}}_{ref}$$

$$f = m\,\mathbf{a}_{des} \cdot R\mathbf{e}_3$$

### Aerobatic Envelope Constraints

**Load factor** (specific force normalised by gravity):

$$n(t) = \frac{\|f\,R\mathbf{e}_3\|}{mg} = \frac{f}{mg}$$

Constraint: $n(t) \leq n_{max} = 3.0$.

**Altitude floor safety**:

$$z(t) \geq z_{floor} = 5.5 \text{ m}$$

Enforced during trajectory optimisation by adding a lower-bound constraint on the $z$-component
of all waypoints and checking all polynomial segment minima post-optimisation.

**Structural margin metric** (peak acceleration normalised):

$$g_{peak} = \max_t \left\|\dot{\mathbf{v}}(t)\right\| / g$$

This metric is logged per manoeuvre and compared against the structural design limit of 3 g.

### Snap Energy Hessian

For a single scalar axis, the snap cost matrix for a degree-7 polynomial over $[0, T]$ is:

$$[Q_{snap}]_{jk} = \frac{j!\, k!}{(j-4)!\,(k-4)!\,(j+k-7)} T^{j+k-7}, \quad j,k \geq 4$$

with $[Q_{snap}]_{jk} = 0$ for $j < 4$ or $k < 4$. The full 3D Hessian is block-diagonal with
three copies of the scalar $Q_{snap}$.

### Performance Metrics

**RMS position tracking error** over the full sequence:

$$\bar{e}_{pos} = \sqrt{\frac{1}{T_{total}} \int_0^{T_{total}} \|\mathbf{p}(t) - \mathbf{p}_{ref}(t)\|^2\, dt}$$

**Manoeuvre completion angle** (geodesic residual on SO(3), degrees):

$$\delta_k = \frac{180}{\pi}\arccos\!\left(\frac{\mathrm{tr}(R_{ref}(t_{e,k})^\top R(t_{e,k})) - 1}{2}\right)$$

**Snap cost** (offline trajectory quality measure):

$$J_{snap,total} = \sum_i \int_{t_i}^{t_{i+1}} \left\|\frac{d^4\mathbf{p}_{ref}}{dt^4}\right\|^2 dt$$

---

## Key 3D Additions

- **Spatial aerobatic paths**: each manoeuvre has a prescribed geometric path (circle, helix, arc)
  through 3D space rather than a stationary hover point.
- **Minimum-snap polynomial trajectory**: degree-7 piecewise polynomial optimised via QP with $C^4$
  continuity at knots; snap energy Hessian assembled analytically.
- **Coupled attitude from trajectory geometry**: desired $R_{ref}(t)$ derived from $\ddot{\mathbf{p}}_{ref}$
  plus roll overlay rather than from a purely kinematic slerp schedule.
- **Load factor and altitude floor constraints**: aerobatic envelope enforced during trajectory
  planning and monitored at simulation run time.
- **Altitude range**: $z \in [5.5, 20]$ m with full excursions during loop ($\Delta z \approx 8$ m)
  and split-S dive ($\Delta z \approx 6$ m).

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Arena volume | $[\pm 15, \pm 15, 5\text{–}20]$ m |
| Loop radius $r_{loop}$ | 4.0 m |
| Split-S radius $r_{split}$ | 3.0 m |
| Immelmann radius $r_{imm}$ | 3.5 m |
| Forward speed during roll | 5.0 m/s |
| Altitude floor $z_{floor}$ | 5.5 m |
| Cruise altitude $z_0$ | 10.0 m |
| Loop duration $T_{loop}$ | 4.0 s |
| Aileron roll duration $T_{roll}$ | 2.0 s |
| Split-S duration $T_{split}$ | 3.5 s |
| Immelmann duration $T_{imm}$ | 3.5 s |
| Transit segment duration | 1.5 s each |
| Polynomial degree | 7 |
| Interior waypoints per manoeuvre | 8 |
| Max load factor $n_{max}$ | 3.0 g |
| Max angular rate | 15 rad/s |
| RMS position error target | $\leq 0.4$ m |
| RMS attitude error target | $\bar{e}_R \leq 0.20$ rad |
| Manoeuvre completion threshold $\delta_k$ | $\leq 8°$ |
| Drone mass $m$ | 0.5 kg |
| Inertia $I_{xx}, I_{yy}$ | 4.9 × 10⁻³ kg·m² |
| Inertia $I_{zz}$ | 8.8 × 10⁻³ kg·m² |
| Attitude gain $K_R$ | diag(8, 8, 4) |
| Angular velocity gain $K_\Omega$ | diag(0.6, 0.6, 0.4) |
| Position gain $K_x$ | 6.0 (isotropic) |
| Velocity gain $K_v$ | 4.0 (isotropic) |
| Integration timestep $\Delta t$ | 0.005 s |

---

## Expected Output

- **3D aerobatic path plot** (`s091_3d_path.png`): `mpl_toolkits.mplot3d` view of the full spatial
  trajectory; drone path in red; minimum-snap reference path overlaid as a dashed black line;
  coloured circle/arc markers showing the geometric shape of each manoeuvre (loop: orange circle,
  aileron roll: purple helix cylinder, split-S: cyan downward arc, Immelmann: gold upward arc);
  altitude floor shown as a translucent horizontal plane at $z = 5.5$ m; entry and exit waypoints
  of each manoeuvre marked with distinct symbols.

- **Altitude and load factor time series** (`s091_altitude_loadfactor.png`): top panel shows
  $z(t)$ with the $z_{floor} = 5.5$ m hard limit as a dotted red line and the reference $z_{ref}(t)$
  as dashed black; bottom panel shows $n(t) = f/(mg)$ with the 3 g ceiling as a dotted line; both
  panels annotated with manoeuvre segment labels on a shared time axis.

- **Metrics panel** (`s091_metrics_panel.png`): six-subplot grid showing attitude error $\|e_R(t)\|$,
  angular rate $\|\Omega(t)\|$, position tracking error $\|\mathbf{p} - \mathbf{p}_{ref}\|$, total
  thrust $f(t)$, Euler roll/pitch angles over time, and per-manoeuvre completion error bar chart
  $\delta_k$; all targets shown as dotted reference lines.

- **Minimum-snap trajectory visualisation** (`s091_snap_trajectory.png`): three-row subplot showing
  the reference position $\mathbf{p}_{ref}(t)$, velocity $\dot{\mathbf{p}}_{ref}(t)$, and snap
  (fourth derivative) magnitude per axis; snap peaks at knot transitions visible; $C^4$ continuity
  confirmed by smooth velocity and acceleration profiles.

- **Acrobatic animation** (`s091_animation.gif`): 3D view animating the drone traversing its
  spatial aerobatic path; three body-axis arrows (red/green/blue for $x$/$y$/$z$) rotate in real
  time showing full attitude including inverted flight; the minimum-snap reference path drawn as a
  static dashed line in the background; current position indicated by a bright marker; frame title
  shows time and current manoeuvre name.

- **Terminal metric summary**: printed table of $\bar{e}_{pos}$, $\bar{e}_R$, $g_{peak}$,
  $J_{snap,total}$, and per-manoeuvre $\delta_k$.

---

## Extensions

1. **Time-optimal aerobatic planning**: replace the fixed segment durations with decision variables
   in the snap QP; minimise total sequence time subject to the load factor constraint and angular
   rate bound; compare against the fixed-time schedule using the snap cost ratio $J_{snap,opt} /
   J_{snap,fixed}$.
2. **Wind disturbance rejection during loops**: add a constant horizontal wind field $\mathbf{w} =
   w_0\,\hat{\mathbf{x}}$ inside the loop volume; augment the geometric controller with a
   disturbance observer (DOB) that estimates aerodynamic drag from the thrust residual; evaluate
   loop-closure error (position at loop exit vs entry) with and without the DOB at $w_0 \in
   \{1, 3, 5\}$ m/s.
3. **Motor saturation during snap peaks**: compute the instantaneous rotor allocation margin
   $\rho = 1 - \max_i(\omega_i^2) / \omega_{max}^2$ throughout the sequence; identify segments
   where the snap trajectory drives the motor to saturation and re-optimise those segments with a
   tighter jerk limit.
4. **Multi-drone synchronised aerobatics**: extend to three drones each executing the same spatial
   loop/roll/split-S/Immelmann sequence offset by $\pm 5$ m laterally; add a formation-keeping
   term to the position controller penalising deviation from the formation centroid; synchronise
   manoeuvre start times and animate all three body-axis triples simultaneously.
5. **Hardware-in-the-loop validation**: export the minimum-snap coefficients as a time-stamped
   waypoint file compatible with the Crazyflie `high_level_commander`; replay on physical
   hardware and overlay the on-board state estimate against the simulated trajectory to identify
   unmodelled aerodynamic effects at peak load factor.
6. **Differential flatness cross-check**: use the differential flatness of the quadrotor model
   (Mellinger & Kumar 2011) to analytically derive the required $R_{ref}(t)$, $\Omega_{ref}(t)$,
   and $f(t)$ directly from $\mathbf{p}_{ref}(t)$ and $\psi_{ref}(t)$ without the iterative
   SO(3) slerp; compare the coupled attitude trajectory against the slerp-based construction and
   measure the resulting difference in $\|e_R\|$.

---

## Related Scenarios

- Original 2D version: [S091](../S091_acrobatics.md)
- Minimum-snap trajectory reference: [S084 Wind Endurance](../S084_wind_endurance.md),
  [S090 Racing Optimal](../S090_racing_optimal.md)
- Truly 3D attitude control reference: [S001 Basic Intercept](../../01_pursuit_evasion/S001_basic_intercept.md),
  [S003 Low-Altitude Tracking](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Formation extension: [S088 Formation Morphing](../S088_formation_morphing.md),
  [S098 Synchronized Dance](../S098_synchronized_dance.md)
