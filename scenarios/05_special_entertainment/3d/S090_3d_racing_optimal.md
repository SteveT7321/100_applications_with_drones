# S090 3D Upgrade — Racing Optimal Path

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S090 original](../S090_racing_optimal.md)

---

## What Changes in 3D

The original S090 already operates in 3D space (gate z-coordinates are hard-coded as scalar
constants: 1.5, 2.5, 1.8, 2.2, 1.5, 1.0 m). However, it treats altitude as a passive waypoint
coordinate — the polynomial planner simply interpolates through the given z-values with no awareness
of vertical dynamics, bank-angle feasibility, or the aerodynamic cost of sustained climbs and dips.
Gate headings include a small z-component but the heading constraint enforces only the passage
velocity direction; there is no attitude model and no coupling between the commanded trajectory
curvature and the drone's physical roll-pitch-yaw state.

This 3D upgrade introduces three major additions:

1. **Banked-turn racecourse geometry**: gates are designed as a true 3D circuit with deliberate
   climbs, descents, and banked arcs. Each gate carries a full 6-DOF frame (position, approach
   heading, and a bank-angle reference $\phi_{ref,i}$) that the trajectory planner must satisfy at
   passage time.

2. **Minimum-time 3D trajectory with dynamic feasibility constraints**: rather than holding
   $T_{total}$ fixed and redistributing, the optimiser minimises the augmented cost
   $J_{aug} = J_{snap} + \lambda T_{total}$ subject to hard per-sample bounds on speed, net
   thrust-to-weight ratio, and roll/pitch angle derived from the flatness mapping of quadrotor
   dynamics.

3. **6-DOF attitude optimisation through gates**: the differential-flatness property of the
   quadrotor is used to convert the trajectory's acceleration profile into a reference attitude
   sequence $(\phi(t), \theta(t), \psi(t))$. The planner is extended to penalise attitude rate
   exceedance near gate passages and to enforce that the roll angle at each banked gate matches
   $\phi_{ref,i}$ within a tolerance $\delta\phi = 5°$.

---

## Problem Definition

**Setup**: A racing drone must navigate a 3D racecourse of $N_{gates} = 8$ gates arranged to include
two climbing sections, one diving chicane, and two banked flat turns. Each gate is defined by:

- Centre position $\mathbf{g}_i \in \mathbb{R}^3$
- Required passage velocity direction $\hat{\mathbf{h}}_i \in \mathbb{R}^3$ (unit vector)
- Required bank angle at passage $\phi_{ref,i} \in [-60°, 60°]$ (positive = right bank)

The drone is modelled as a quadrotor with differential-flatness outputs
$\sigma(t) = [x(t),\, y(t),\, z(t),\, \psi(t)]^\top$, so that the full attitude and rotor thrusts
can be recovered from the trajectory's first four time derivatives. The trajectory is parameterised
as $N_{gates}$ piecewise degree-7 polynomial segments with $C^3$ continuity, identical to S090.

Three trajectory strategies are compared:

1. **Fixed-time minimum-snap (baseline)**: segment durations set proportional to inter-gate
   distance; QP solved once; no dynamic feasibility check.
2. **Time-redistributed minimum-snap (S090 method)**: gradient descent on $T_i$ while holding
   $T_{total}$ fixed; penalises snap cost only.
3. **Full 3D minimum-time with attitude constraints (this upgrade)**: minimises
   $J_{snap} + \lambda T_{total}$ with additional per-sample penalties for thrust bound violation and
   bank-angle mismatch at each gate; attitude is recovered at every sample via the flatness map and
   feasibility is verified explicitly.

**Roles**:
- **Drone**: single racing agent; quadrotor with mass $m = 0.5$ kg; maximum collective thrust
  $F_{max} = 2mg$; maximum tilt angle $\theta_{max} = 45°$; maximum angular rate
  $\dot{\phi}_{max} = 3$ rad/s; maximum speed $v_{max} = 12$ m/s.
- **Gates**: $N_{gates} = 8$ fixed 3D frames; each imposes position, heading, and bank-angle
  constraints; gate radius $r_{gate} = 0.3$ m; positional tolerance $r_{tol} = 0.2$ m.

**Objective**: generate a trajectory that passes all 8 gates with correct heading and bank angle,
minimises total lap time $T_{total}$, and satisfies the quadrotor thrust and tilt constraints at
every trajectory sample. Report gate pass rate, total lap time, snap cost, maximum tilt angle
achieved, and bank-angle error per gate.

---

## Mathematical Model

### Polynomial Trajectory (identical basis to S090)

For segment $i$ and axis $\alpha \in \{x, y, z\}$:

$$p_i^\alpha(t) = \sum_{k=0}^{7} c_{ik}^\alpha \, t^k, \quad t \in [0, T_i]$$

The snap cost matrix $\mathbf{Q}(T_i)$ and $C^3$ continuity constraints are unchanged from S090.
The added complexity is entirely in the constraint set and the objective augmentation.

### Differential-Flatness Attitude Recovery

Given the polynomial trajectory $\mathbf{p}(t)$ and yaw schedule $\psi(t)$, the thrust vector
required by the quadrotor is:

$$\mathbf{f}(t) = m\bigl(\ddot{\mathbf{p}}(t) + g\hat{\mathbf{z}}\bigr)$$

The collective thrust magnitude and body z-axis are:

$$f(t) = \|\mathbf{f}(t)\|, \qquad \hat{\mathbf{z}}_B(t) = \frac{\mathbf{f}(t)}{f(t)}$$

The tilt angle (deviation from hover) is:

$$\theta_{tilt}(t) = \arccos\!\bigl(\hat{\mathbf{z}}_B(t) \cdot \hat{\mathbf{z}}\bigr)
= \arccos\!\!\left(\frac{g + \ddot{z}(t)}{f(t)/m}\right)$$

The roll angle in the velocity frame is extracted via the intermediate body frame construction.
Define the yaw-axis unit vector $\hat{\mathbf{x}}_C = [\cos\psi,\, \sin\psi,\, 0]^\top$ and the
body y-axis $\hat{\mathbf{y}}_B = \hat{\mathbf{z}}_B \times \hat{\mathbf{x}}_C /
\|\hat{\mathbf{z}}_B \times \hat{\mathbf{x}}_C\|$. Then:

$$\hat{\mathbf{x}}_B = \hat{\mathbf{y}}_B \times \hat{\mathbf{z}}_B$$

$$\phi(t) = \arctan2\!\bigl(-(\hat{\mathbf{z}}_B \cdot \hat{\mathbf{y}}),\;
                              \hat{\mathbf{z}}_B \cdot \hat{\mathbf{x}}\bigr)$$

where $\hat{\mathbf{x}}$, $\hat{\mathbf{y}}$ are the world axes, giving the body-frame roll angle
in the range $(-\pi, \pi]$.

### 3D Gate Constraints

**Position** (identical to S090):

$$\|\mathbf{p}(t_i^*) - \mathbf{g}_i\| \leq r_{tol}$$

**Heading alignment**:

$$\frac{\dot{\mathbf{p}}(t_i^*) \cdot \hat{\mathbf{h}}_i}{\|\dot{\mathbf{p}}(t_i^*)\|}
\geq \cos\delta_h, \quad \delta_h = 20°$$

**Bank-angle matching** (new in 3D upgrade):

$$\bigl|\phi(t_i^*) - \phi_{ref,i}\bigr| \leq \delta\phi = 5°$$

where $t_i^* = \sum_{k=1}^{i} T_k$ is the scheduled passage time for gate $i$.

### Dynamic Feasibility Constraints

For every evaluation sample $t_j \in \{0, \Delta t, 2\Delta t, \ldots, T_{total}\}$:

**Thrust bound**:

$$\frac{F_{min}}{m} \leq \|\ddot{\mathbf{p}}(t_j) + g\hat{\mathbf{z}}\| \leq \frac{F_{max}}{m}$$

with $F_{min} = 0.05 mg$ (prevents free-fall) and $F_{max} = 2mg$.

**Tilt bound**:

$$\theta_{tilt}(t_j) \leq \theta_{max} = 45°$$

**Speed bound**:

$$\|\dot{\mathbf{p}}(t_j)\| \leq v_{max} = 12 \text{ m/s}$$

### Augmented Objective with Feasibility Penalty

The full minimisation objective is:

$$J_{aug} = \underbrace{\sum_{i=1}^{N_{gates}} \mathbf{c}_i^\top \mathbf{Q}(T_i) \mathbf{c}_i}_{J_{snap}}
+ \lambda \sum_{i=1}^{N_{gates}} T_i
+ \mu_{tilt} \sum_j \max\!\bigl(0,\; \theta_{tilt}(t_j) - \theta_{max}\bigr)^2
+ \mu_{\phi} \sum_{i=1}^{N_{gates}} \bigl(\phi(t_i^*) - \phi_{ref,i}\bigr)^2$$

where $\lambda = 0.5$ trades snap cost against lap time, $\mu_{tilt} = 10^3$ penalises tilt
violations, and $\mu_{\phi} = 50$ penalises bank-angle mismatch at gates.

### Gradient with Attitude Penalty

The gradient of the attitude penalty terms with respect to segment duration $T_i$ is computed via
finite differences in the outer optimisation loop (same finite-difference scheme as S090 for
$\partial J_{snap} / \partial T_i$), evaluating the full flatness map at each perturbation:

$$\frac{\partial J_{aug}}{\partial T_i} \approx
\frac{J_{aug}(\ldots, T_i + \varepsilon, \ldots) - J_{aug}(\ldots, T_i - \varepsilon, \ldots)}{2\varepsilon}$$

### Climb and Dive Altitude Profile

The 3D gate layout defines three altitude zones:

- **Climb section** (gates 2–3): $\Delta z = +2.0$ m over one inter-gate segment; average vertical
  velocity $\bar{v}_z = \Delta z / T_i$; adds vertical component to required thrust.
- **Dive chicane** (gate 5): $z_5 = 0.8$ m, lowest point; dive approach angle $\gamma_{dive}$
  relative to horizontal satisfies:

$$\gamma_{dive} = \arctan\!\left(\frac{z_4 - z_5}{d_{45}}\right)$$

  where $d_{45} = \|\mathbf{g}_4 - \mathbf{g}_5\|_{xy}$ is the horizontal distance between gates 4
  and 5.

- **Banked turns** (gates 6–7): required bank angle $\phi_{ref} = \pm 45°$; the centripetal
  acceleration needed to sustain the bank satisfies:

$$\phi_{ref} = \arctan\!\left(\frac{v^2}{g \, R_{turn}}\right)$$

  implying a minimum turn radius $R_{turn} = v^2 / (g \tan\phi_{ref})$ that constrains the gate
  lateral spacing.

---

## Key 3D Additions

- **Bank-angle constraint at gates**: flatness map recovers $\phi(t_i^*)$ and enforces
  $|\phi - \phi_{ref,i}| \leq 5°$ via a quadratic penalty in the objective.
- **Tilt feasibility check**: at every trajectory sample, $\theta_{tilt}$ is extracted from the
  recovered thrust vector and penalised if it exceeds $\theta_{max} = 45°$.
- **Climb/dive altitude sections**: gate z-coordinates span 0.8–3.5 m with deliberate 2 m climbs
  and a shallow dive chicane; vertical velocity reaches up to $\sim 3$ m/s.
- **Differential-flatness attitude recovery**: from $\ddot{\mathbf{p}}(t)$ and $\psi(t)$, the full
  rotation matrix and collective thrust are reconstructed at every evaluation sample.
- **Banked-turn minimum-radius enforcement**: gate lateral spacing is pre-validated against the
  kinematic minimum turn radius at $v_{gate}$; gates that violate the radius are shifted outward.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of gates | 8 |
| Gate physical radius $r_{gate}$ | 0.3 m |
| Gate passage tolerance $r_{tol}$ | 0.2 m |
| Bank-angle tolerance $\delta\phi$ | 5° |
| Required gate passage speed $v_{gate}$ | 5.0 m/s |
| Maximum drone speed $v_{max}$ | 12.0 m/s |
| Drone mass $m$ | 0.5 kg |
| Maximum thrust-to-weight ratio | 2 (i.e. $F_{max} = 2mg$) |
| Maximum tilt angle $\theta_{max}$ | 45° |
| Maximum angular rate $\dot{\phi}_{max}$ | 3 rad/s |
| Polynomial degree | 7 (minimum-snap) |
| Continuity order | $C^3$ (pos / vel / acc / jerk) |
| Altitude range | 0.8 – 3.5 m |
| Climb section $\Delta z$ | +2.0 m |
| Dive gate altitude | 0.8 m |
| Banked-turn reference angles | $\pm 45°$ |
| Snap-vs-time weight $\lambda$ | 0.5 |
| Tilt penalty weight $\mu_{tilt}$ | $10^3$ |
| Bank-angle penalty weight $\mu_{\phi}$ | 50 |
| Gradient-descent step size $\eta$ | $10^{-3}$ |
| Gradient-descent iterations | 300 |
| Minimum segment duration $T_{min}$ | 0.2 s |
| Trajectory evaluation timestep $\Delta t$ | 0.02 s |

---

## Expected Output

- **3D trajectory comparison plot**: all three strategies (fixed-time, S090 redistribution, full 3D
  minimum-time) rendered on the same 3D axes; gates coloured by bank angle (blue = left bank, red =
  right bank, green = level); climb and dive sections highlighted by altitude-coloured path segments.
- **Altitude time series**: $z(t)$ for all three strategies with gate passage times marked; climb
  gradient and dive profile visible; $z_{min} = 0.8$ m lower bound shown as a dashed line.
- **Tilt angle time series**: $\theta_{tilt}(t)$ recovered from the flatness map for all three
  strategies; $\theta_{max} = 45°$ shown as a dashed limit; violation regions highlighted in orange
  for the fixed-time baseline.
- **Bank-angle error per gate**: bar chart of $|\phi(t_i^*) - \phi_{ref,i}|$ for each gate and each
  strategy; $\delta\phi = 5°$ tolerance line shown.
- **Snap cost convergence**: semi-logarithmic plot of $J_{aug}$ vs gradient-descent iteration,
  showing the contribution of the attitude penalty terms separately from $J_{snap} + \lambda T$.
- **Speed profile**: instantaneous speed $\|\dot{\mathbf{p}}(t)\|$ for all three strategies with
  $v_{max}$ limit; dive section shows speed increase; climb section shows speed decrease.
- **Segment time allocation**: bar chart comparing the three strategies' $T_i$ distributions across
  8 segments; climb and banked-turn segments should receive more time in the 3D optimiser.
- **Race trajectory animation (GIF)**: 3D animation with drone attitude indicator (a small
  triad showing roll and pitch); gate frames drawn as rectangles rotated to the required bank angle;
  altitude colour-coding on the growing trail.

**Expected metric targets**:

| Metric | Fixed-time | S090 Redistrib. | 3D Min-time |
|--------|-----------|-----------------|-------------|
| Gates passed (miss $\leq r_{tol}$) | 8 / 8 | 8 / 8 | 8 / 8 |
| Max tilt violation | may exceed 45° | may exceed 45° | 0 violations |
| Mean bank-angle error | unconstrained | unconstrained | $\leq 5°$ |
| Total snap cost $J_{snap}$ | baseline | reduced | lowest |
| Total lap time $T_{total}$ | 12.0 s | 12.0 s | $\leq 10.5$ s |

---

## Extensions

1. **Yaw-rate optimisation**: treat the yaw profile $\psi(t)$ as a free variable (currently fixed to
   the heading direction); add a yaw-rate penalty $\mu_\psi \int \dot{\psi}^2 \, dt$ to the
   objective and co-optimise $\psi(t)$ with the segment times; measure the effect on attitude
   feasibility near banked turns.
2. **Actuator-aware polynomial degree sweep**: repeat the minimum-time optimisation for polynomial
   degrees 5, 7, 9, and 11; compare the achievable lap time and tilt feasibility rate; determine
   the lowest degree that satisfies all dynamic constraints on the 3D circuit.
3. **Iterative corridor constraint**: replace the soft tilt penalty with a hard half-space constraint
   per trajectory sample; implement the successive convexification (SCvx) outer loop that linearises
   the constraints around the current trajectory and solves a sequence of QPs until convergence;
   compare the converged solution to the penalty-based approach.
4. **Wind-perturbed 3D circuit**: add a steady horizontal wind field $\mathbf{w} = [w_x, w_y, 0]^\top$
   in the drone's equations of motion; re-plan the trajectory to compensate (the heading constraint
   at each gate must now account for air-relative vs ground-relative velocity); measure lap time
   increase vs wind speed.
5. **Dynamic gate difficulty scoring**: define a difficulty score per gate as a weighted sum of
   approach speed, bank-angle magnitude, and altitude change; use the scores to pre-allocate more
   gradient-descent budget to high-difficulty segments; compare convergence speed against uniform
   step-size gradient descent.
6. **Multi-drone formation race**: extend to two racing drones sharing the same 3D circuit;
   add inter-drone minimum separation $d_{sep} = 1.0$ m as a soft penalty; co-optimise both
   trajectories jointly; visualise how the time-offset between drones affects optimal segment
   durations.

---

## Related Scenarios

- Original 2D/3D version: [S090 Racing Optimal Path](../S090_racing_optimal.md)
- Attitude dynamics reference: [S001 Basic Intercept](../../01_pursuit_evasion/S001_basic_intercept.md),
  [S003 Low Altitude Tracking](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Acrobatic attitude planning: [S091 FPV Acrobatic Manoeuvre](../S091_acrobatics.md)
- Multi-drone extension: [S099 Obstacle Relay](../S099_obstacle_relay.md)
- Polynomial trajectory cross-reference: [S077 Precision Pollination](../../04_industrial_agriculture/S077_pollination.md)
