# S089 3D Upgrade — Large-Scale Collision Avoidance

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S089 original](../S089_collision_avoidance.md)

---

## What Changes in 3D

The original S089 already operates in $\mathbb{R}^3$ for state representation but its collision
avoidance logic does not exploit the full vertical dimension as an independent strategic axis.
The ORCA velocity obstacle cone geometry is correct in 3D, yet the original scenario initialises
all drones on a sphere of fixed radius and assigns antipodal goals — trajectories therefore remain
symmetric around the origin with no layered altitude structure. Two important 3D capabilities are
absent:

1. **Altitude-layer separation**: assigning each drone a distinct cruise altitude band so that
   the central congestion zone is vertically stratified rather than all traffic competing for the
   same horizontal plane through the origin.
2. **Full 3D bounding-sphere collision detection**: the original uses Euclidean distance in
   $\mathbb{R}^3$ (correct) but does not model the drone body as a sphere with independently
   verified clearance in the vertical axis versus the horizontal plane. This upgrade makes the
   vertical safety margin explicit and separately reportable.

This variant adds: altitude-layer assignment at initialisation, a vertical separation term in the
preferred-velocity law, 3D bounding-sphere clearance accounting, altitude-time series plots, and
a second metrics panel comparing planar vs layered initialisation.

---

## Problem Definition

**Setup**: $N = 50$ drones are placed at random positions on the surface of a sphere of radius
$R_{sphere} = 20$ m. Each drone is assigned its antipodal goal. Three initialisation modes are
compared:

- **Mode A — Flat (baseline)**: all drones placed with $z \in [-2, 2]$ m (near the equatorial
  belt of the sphere), forcing all trajectories through the same horizontal slab.
- **Mode B — Full sphere**: uniform random placement over the full sphere surface (original
  S089 setup).
- **Mode C — Layered**: drones are sorted by their goal's $z$-coordinate and assigned altitude
  layers $z_{layer}^{(i)} \in [z_{min}, z_{max}]$ with spacing $\Delta z = (z_{max}-z_{min})/(N-1)$;
  the preferred velocity includes a vertical correction term that steers each drone toward its
  target altitude before executing the horizontal avoidance manoeuvre.

All three modes use the **ORCA** solver operating in full $\mathbb{R}^3$ velocity space.

**Roles**:
- **Drones** ($N = 50$): point-mass quadrotors, body radius $r = 0.25$ m (bounding sphere
  diameter $2r = 0.50$ m); maximum speed $v_{max} = 3.0$ m/s; preferred speed
  $v_{pref} = 2.0$ m/s; neighbourhood sensing radius $R_{nb} = 10$ m; vertical speed cap
  $v_{z,max} = 1.5$ m/s (realistic climb/descent limit for small quadrotors).
- **Targets**: fixed antipodal waypoints; no dynamics.

**Objective**: zero collisions (inter-drone Euclidean separation never below $2r = 0.50$ m)
across all 50 drones for all three modes. Secondary objectives:

- Quantify how Mode C (layered) reduces ORCA constraint activations relative to Mode B.
- Report vertical clearance separately from horizontal clearance.
- Show altitude-vs-time traces for all three modes to confirm layering is maintained.

---

## Mathematical Model

### State Space

Drone $i$ has position $\mathbf{p}_i = (x_i, y_i, z_i)^\top \in \mathbb{R}^3$ and velocity
$\mathbf{v}_i \in \mathbb{R}^3$. Altitude is unconstrained except for the soft bounds
$z \in [z_{floor}, z_{ceil}] = [0.5, 30.0]$ m.

### Preferred Velocity with Altitude Layer Correction (Mode C)

The preferred velocity in Mode C is a weighted sum of the goal-directed component and an
altitude-correction component:

$$\mathbf{v}_{i}^{pref} = v_{pref} \cdot \frac{\alpha_{h} \, \mathbf{d}_{h}^{(i)} + \alpha_{z} \, \mathbf{d}_{z}^{(i)}}{\|\alpha_{h} \, \mathbf{d}_{h}^{(i)} + \alpha_{z} \, \mathbf{d}_{z}^{(i)}\|}$$

where:
- $\mathbf{d}_{h}^{(i)} = (\mathbf{g}_i - \mathbf{p}_i) \odot (1, 1, 0)^\top$ is the horizontal
  component of the goal vector (zeroed $z$-component),
- $\mathbf{d}_{z}^{(i)} = (0, 0, z_{layer}^{(i)} - z_i)^\top$ is the altitude correction,
- $\alpha_h = 0.85$ and $\alpha_z = 0.15$ are blending weights,
- $z_{layer}^{(i)}$ is the pre-assigned cruise altitude for drone $i$.

When drone $i$ is within $\delta_z = 0.5$ m of its target altitude, $\alpha_z \to 0$ and the
velocity reverts to purely goal-directed.

### Altitude Layer Assignment

Drones are sorted by their initial $z$-coordinate in ascending order and assigned layers:

$$z_{layer}^{(i)} = z_{min} + \frac{i}{N-1}(z_{max} - z_{min}), \quad i = 0, \ldots, N-1$$

with $z_{min} = 2.0$ m and $z_{max} = 18.0$ m, giving $\Delta z = 16/(N-1) \approx 0.33$ m
per layer for $N = 50$. This ensures that any two adjacent-layer drones have a vertical
separation of at least $\Delta z$ even before ORCA applies horizontal adjustments.

### 3D Bounding-Sphere Collision Detection

Drone bodies are modelled as spheres of radius $r = 0.25$ m. A collision between drones $A$
and $B$ is confirmed when:

$$\|\mathbf{p}_A(t) - \mathbf{p}_B(t)\|_2 < 2r = 0.50 \; \text{m}$$

The full Euclidean distance is decomposed into horizontal and vertical components for reporting:

$$d_{horiz}^{AB} = \sqrt{(x_A - x_B)^2 + (y_A - y_B)^2}, \qquad
d_{vert}^{AB} = |z_A - z_B|$$

Near-miss statistics are reported separately for $d_{horiz}$ and $d_{vert}$, allowing
identification of whether close encounters are predominantly horizontal (lateral traffic) or
vertical (altitude layer violations).

### ORCA Velocity Obstacle in 3D (Truncated Cone)

The velocity obstacle of agent $A$ induced by agent $B$ with time horizon $\tau$ is:

$$\mathrm{VO}_{A|B}^{\tau}(\mathbf{v}_B) = \left\{ \mathbf{v}_A \;\middle|\; \exists\, t \in [0, \tau],\;
\bigl\|\bigl(\mathbf{p}_A - \mathbf{p}_B\bigr) + \bigl(\mathbf{v}_A - \mathbf{v}_B\bigr)t\bigr\| < 2r \right\}$$

In $\mathbb{R}^3$, this is a truncated cone with apex at
$\mathbf{v}_B + (\mathbf{p}_A - \mathbf{p}_B)/\tau$, axis along $-(\mathbf{p}_A - \mathbf{p}_B)$,
and half-angle:

$$\theta_{VO} = \arcsin\!\left(\frac{2r}{\|\mathbf{p}_A - \mathbf{p}_B\|}\right)$$

The cone is three-dimensional: a velocity in any direction (including vertical) that enters the
cone is forbidden. The ORCA half-plane for pair $(A, B)$ is:

$$\mathrm{ORCA}_{A|B}^{\tau} = \left\{ \mathbf{v} \;\middle|\;
\bigl(\mathbf{v} - \mathbf{v}_A^{cur} - \tfrac{1}{2}\mathbf{u}_{A|B}\bigr) \cdot \hat{\mathbf{n}}_{A|B} \geq 0 \right\}$$

where $\hat{\mathbf{n}}_{A|B} \in \mathbb{R}^3$ is the outward unit normal to the cone surface at
the closest point to $\mathbf{v}_A^{cur}$, and
$\mathbf{u}_{A|B}$ is the minimum-norm velocity change that exits the cone.

### Quadratic Program per Drone (3D)

$$\mathbf{v}_A^* = \arg\min_{\mathbf{v} \in \mathbb{R}^3} \;\|\mathbf{v} - \mathbf{v}_A^{pref}\|^2$$

$$\text{subject to:} \quad \hat{\mathbf{n}}_{A|B}^\top \mathbf{v} \geq b_{A|B} \quad \forall B \in \mathcal{N}(A)$$

$$\|\mathbf{v}\| \leq v_{max}, \qquad |v_z| \leq v_{z,max}$$

The additional constraint $|v_z| \leq v_{z,max}$ encodes the quadrotor's limited climb/descent
rate. It is added to the LP as two half-planes:
$(0, 0, 1)^\top \mathbf{v} \leq v_{z,max}$ and $(0, 0, -1)^\top \mathbf{v} \leq v_{z,max}$.

### Vertical Clearance Metric

At each timestep $k$, for every pair $(A, B)$ within $R_{nb}$, record:

$$\Delta z_{AB}^{(k)} = |z_A^{(k)} - z_B^{(k)}|$$

The **vertical clearance violation rate** is:

$$\rho_{vert} = \frac{|\{(A, B, k) : \Delta z_{AB}^{(k)} < 2r\}|}{N(N-1)/2 \cdot T_{steps}}$$

For Mode C (layered), $\rho_{vert}$ should be significantly lower than for Mode A (flat)
because layer spacing $\Delta z \gg 2r$.

### Performance Metrics

**Time-to-goal** for drone $i$:

$$T_{goal}^{(i)} = \min\{ t \geq 0 \mid \|\mathbf{p}_i(t) - \mathbf{g}_i\| \leq d_{goal} \}$$

**Mission completion time**: $T_{mission} = \max_i T_{goal}^{(i)}$.

**Mean altitude deviation from assigned layer** (Mode C only):

$$\bar{\varepsilon}_z = \frac{1}{N \cdot T_{steps}} \sum_{i=1}^{N} \sum_{k=1}^{T_{steps}}
\left|z_i^{(k)} - z_{layer}^{(i)}\right|$$

**ORCA constraint activity rate**:

$$\rho_{ORCA} = \frac{\text{total half-plane activations}}{N \cdot T_{steps}}$$

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones | $N$ | 50 |
| Sphere radius (initial placement) | $R_{sphere}$ | 20.0 m |
| Drone body radius | $r$ | 0.25 m |
| Collision diameter | $2r$ | 0.50 m |
| Preferred cruise speed | $v_{pref}$ | 2.0 m/s |
| Maximum speed cap | $v_{max}$ | 3.0 m/s |
| Vertical speed cap | $v_{z,max}$ | 1.5 m/s |
| ORCA time horizon | $\tau$ | 3.0 s |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Mission time limit | $T_{max}$ | 60.0 s |
| Goal arrival radius | $d_{goal}$ | 0.5 m |
| Neighbourhood sensing radius | $R_{nb}$ | 10.0 m |
| Layer altitude range | $[z_{min}, z_{max}]$ | [2.0, 18.0] m |
| Layer spacing (N=50) | $\Delta z$ | ~0.33 m |
| Altitude floor / ceiling | $[z_{floor}, z_{ceil}]$ | [0.5, 30.0] m |
| Horizontal blend weight | $\alpha_h$ | 0.85 |
| Altitude correction weight | $\alpha_z$ | 0.15 |
| Altitude snap threshold | $\delta_z$ | 0.5 m |
| Random seed | — | 42 |

---

## Expected Output

- **3D trajectory plot — Mode A (flat)**: all 50 drones crowded into a thin horizontal slab; severe
  congestion visible at the origin; ORCA must work almost entirely in the horizontal plane; altitude
  traces nearly flat.
- **3D trajectory plot — Mode B (full sphere)**: trajectories distributed across all elevations;
  natural vertical separation reduces congestion; baseline for comparison.
- **3D trajectory plot — Mode C (layered)**: clear stratification of paths into altitude bands;
  trajectories weave horizontally but remain near their assigned layer; qualitatively the least
  congested appearance.
- **Altitude vs time (all three modes)**: three subplots showing $z_i(t)$ for all drones; Mode A
  shows all traces compressed near zero altitude; Mode C shows evenly spaced horizontal bands.
- **Vertical vs horizontal clearance scatter**: for each near-miss event (pair within $R_{nb}$),
  scatter $d_{horiz}$ vs $d_{vert}$; Mode C points cluster away from the $d_{vert} < 0.5$ m
  zone; Mode A points scatter along a vertical strip at small $d_{vert}$.
- **Metrics comparison bar chart (3 modes x 4 metrics)**: collision count, mission time, ORCA
  constraint activity rate, mean altitude deviation (Mode C only); Mode C expected to show the
  lowest constraint activation rate.
- **Animated GIF (3D rotating view)**: 120-frame animation with slow azimuth rotation showing
  all drones simultaneously; colour-coded by altitude layer in Mode C; clearly demonstrates the
  vertical stratification.

### Expected Numerical Results

| Metric | Mode A (Flat) | Mode B (Full sphere) | Mode C (Layered) |
|--------|--------------|----------------------|-----------------|
| Collision count | 0 (ORCA) | 0 (ORCA) | 0 (ORCA) |
| Min separation (m) | > 0.50 | > 0.50 | > 0.50 |
| Mission time (s) | ~35–45 | ~25–35 | ~28–38 |
| ORCA activations / (N·step) | highest | medium | lowest |
| Mean altitude deviation (m) | N/A | N/A | < 1.0 |
| Vertical clearance violation rate | highest | medium | ~0 |

---

## Extensions

1. **Vertical speed asymmetry**: quadrotors descend faster than they climb (motor saturation
   on the way up). Model asymmetric caps $v_{z,up} = 1.5$ m/s and $v_{z,down} = 2.5$ m/s;
   update the LP constraints accordingly and measure whether asymmetric caps reduce mission time.
2. **Dynamic layer reassignment**: if a drone falls more than $2 \Delta z$ behind its layer
   schedule (blocked by ORCA), reassign it to the next available empty layer; implement as a
   priority-based handoff and measure the number of reassignments vs swarm density.
3. **Wind disturbance in 3D**: add a spatially varying wind field
   $\mathbf{w}(\mathbf{p}) = w_0 \hat{\mathbf{x}} + w_z \sin(k_z z) \hat{\mathbf{z}}$;
   measure how vertical gusts degrade layer separation and whether the altitude correction term
   in $\mathbf{v}^{pref}$ is sufficient to maintain layering without additional integral control.
4. **Density scaling with layers**: sweep $N \in \{20, 50, 100, 200\}$ with the same
   $[z_{min}, z_{max}]$ range; the layer spacing $\Delta z = 16/(N-1)$ shrinks with $N$; find
   the critical $N$ at which $\Delta z < 2r$ and layers can no longer provide passive vertical
   separation, forcing ORCA to handle full 3D avoidance.
5. **Heterogeneous drone sizes**: mix two drone classes — small ($r = 0.15$ m, $v_{max} = 4$ m/s)
   and large ($r = 0.40$ m, $v_{max} = 2$ m/s); assign large drones to the middle altitude
   layers and small drones to the outer layers; verify ORCA combined-radius $r_A + r_B$ accounts
   for heterogeneous pairs.

---

## Related Scenarios

- Original 2D/3D version: [S089](../S089_collision_avoidance.md)
- Altitude-layer coordination reference: [S085 Light Matrix Positioning](../S085_light_matrix.md),
  [S088 Formation Shape Morphing](../S088_formation_morphing.md)
- Multi-agent velocity reasoning: [S009 Differential Game Pursuit](../../01_pursuit_evasion/S009_differential_game.md)
- Dense swarm coordination: [S070 Swarm Weeding](../../04_industrial_agriculture/S070_swarm_weeding.md)
- 3D evasion reference (ORCA geometry analogy): [S002 3D Upgrade](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
