# S098 3D Upgrade — Swarm Synchronized Dance

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S098 original](../S098_synchronized_dance.md)

---

## What Changes in 3D

The original S098 treats altitude as a simple sinusoidal decoration applied uniformly on top of a
fixed 50 m show altitude — the z-axis carries no independent choreographic intent, all PTP
correction logic operates only on the scalar clock offset, and the animation is displayed as a 2D
top-down XY scatter. Critically, collision avoidance is never evaluated because the z-separation
between ring layers is treated as guaranteed.

This variant elevates altitude to a first-class choreographic axis. Keyframes are authored as full
$(x, y, z, t)$ tuples with deliberate altitude-layer structure. Inter-keyframe interpolation is
replaced by **minimum-snap trajectories** (4th-order polynomial per axis) that enforce zero velocity
and acceleration at each keyframe boundary. Altitude-layer synchronisation is separately tracked as
a KPI distinct from horizontal XY fidelity. A real-time **3D collision-avoidance** check runs at
every timestep and forces a repulsion impulse whenever any two drones approach within the safety
radius — something the original entirely omits because the two-ring geometry at a single altitude
passively separated the drones.

---

## Problem Definition

**Setup**: Ten drones perform a 60-second pre-scripted aerial dance spanning a true 3D volume
$[-20, 20]\,\text{m} \times [-20, 20]\,\text{m} \times [30, 70]\,\text{m}$. The choreography is
divided into four acts, each using a different altitude-layer configuration:

| Act | Duration | Altitude layers | Formation |
|-----|----------|-----------------|-----------|
| I | 0–15 s | Low 35 m / High 55 m | Two horizontal rings |
| II | 15–30 s | 35–65 m sweep (interleaved) | Rising double helix |
| III | 30–45 s | All at 50 m | Flat star burst |
| IV | 45–60 s | 30–70 m | 3D sphere lattice |

Each drone reads its pre-authored keyframes through a minimum-snap polynomial and queries the
trajectory at its local clock. A PTP consensus corrects clock offsets every 0.5 s. In addition, a
3D collision-avoidance layer monitors inter-drone distances and injects a velocity repulsion term
whenever two drones come within $d_{safe} = 2.0$ m of each other.

**Roles**:
- **Drones** ($N = 10$): identical quadrotors following individual 3D minimum-snap trajectories
  derived from per-drone keyframe tables. Each maintains a drifting local clock corrected by PTP.
- **Choreography script**: per-drone 3D keyframes $\{(t_n^{kf},\, x_{k,n},\, y_{k,n},\, z_{k,n})\}$
  authored so that altitude layers change across the four acts.
- **PTP consensus network**: fully connected broadcast graph, identical in structure to S098; now
  applied to a 3D trajectory evaluation.
- **Collision-avoidance layer**: runs independently of the choreography playback; adds a short
  repulsion velocity impulse that is blended back to the choreography reference within 0.2 s.

**Objective**: maintain the swarm synchronisation error $\varepsilon_{sync}(t) < 10$ ms, altitude
layer synchronisation error $\varepsilon_{alt}(t) < 0.5$ m, and pairwise 3D separation
$d_{kj}(t) > d_{safe} = 2.0$ m at all times. Compare three clock-sync strategies (no sync, PTP
standard, PTP aggressive) on all three KPIs simultaneously.

---

## Mathematical Model

### Local Clock Model (unchanged from S098)

$$t_{local,k}(t) = t_{global}(t) + \delta_k(t)$$

$$\delta_k(t + \Delta t_{ptp}) = \delta_k(t) + \eta_k, \qquad \eta_k \sim \mathcal{N}(0,\,\sigma_{drift}^2)$$

### Minimum-Snap Trajectory Segment

Between consecutive keyframes $n$ and $n{+}1$ (time interval $[t_n^{kf},\, t_{n+1}^{kf}]$), each
axis $q \in \{x, y, z\}$ of drone $k$ is governed by a degree-7 polynomial in the normalised time
$\tau = (t - t_n^{kf}) / (t_{n+1}^{kf} - t_n^{kf}) \in [0, 1]$:

$$q_k(\tau) = \sum_{i=0}^{7} c_{k,q,n,i}\, \tau^i$$

The eight coefficients $\mathbf{c}_{k,q,n}$ are determined by eight boundary conditions per
segment: position, velocity, acceleration, and jerk match at both endpoints (zero velocity,
acceleration, and jerk at keyframes). Minimising the integral of squared snap (4th derivative) over
the segment is equivalent to the standard minimum-snap LP formulation:

$$\min_{\mathbf{c}} \int_0^1 \left(\frac{d^4 q_k}{d\tau^4}\right)^2 d\tau$$

subject to the eight boundary conditions. The solution is obtained by solving an $8 \times 8$ linear
system per axis per segment per drone.

The full 3D commanded position at query time $t$ for drone $k$:

$$\mathbf{p}_k^{cmd}(t) = \begin{bmatrix} x_k(\tau(t)) \\ y_k(\tau(t)) \\ z_k(\tau(t)) \end{bmatrix}$$

### Trajectory Evaluation Under Clock Offset

$$\mathbf{p}_k^{actual}(t) = \mathbf{p}_k^{cmd}\!\left(t_{local,k}(t)\right)$$

$$\mathbf{p}_k^{ideal}(t) = \mathbf{p}_k^{cmd}\!\left(t_{global}(t)\right)$$

### Altitude Layer Synchronisation Error

Let $z_k^{layer}(t)$ denote the target layer altitude for drone $k$ at time $t$ (read from the act
schedule). The altitude synchronisation error of the swarm is:

$$\varepsilon_{alt}(t) = \frac{1}{N} \sum_{k=1}^{N} \bigl|z_k^{actual}(t) - z_k^{layer,ideal}(t)\bigr|$$

**Target**: $\varepsilon_{alt}(t) < 0.5$ m throughout the performance.

### 3D Swarm Synchronisation Error

$$\varepsilon_{sync}(t) = \max_{k \in \{1,\ldots,N\}} \bigl|\delta_k(t)\bigr| \times 10^3 \quad [\text{ms}]$$

### Visual Fidelity (3D)

The 3D position error for drone $k$ now includes altitude deviation:

$$\varepsilon_{3D,k}(t) = \bigl\|\mathbf{p}_k^{actual}(t) - \mathbf{p}_k^{ideal}(t)\bigr\|$$

The 3D choreography speed is:

$$d_{move,k}(t) = \left\|\frac{d\,\mathbf{p}_k^{cmd}}{dt}\bigg|_{t_{global}}\right\| \cdot \Delta t_{ptp}$$

Per-drone visual fidelity:

$$v_k(t) = \mathrm{clip}\!\left(1 - \frac{\varepsilon_{3D,k}(t)}{d_{move,k}(t)},\; 0,\; 1\right)$$

Swarm average: $V(t) = \frac{1}{N} \sum_{k} v_k(t)$.

### 3D Collision Avoidance (Repulsion Layer)

At each simulation step, for every ordered pair $(k, j)$ with $k < j$, compute:

$$\mathbf{r}_{kj}(t) = \mathbf{p}_k^{actual}(t) - \mathbf{p}_j^{actual}(t), \qquad d_{kj} = \|\mathbf{r}_{kj}\|$$

If $d_{kj} < d_{safe}$, add a repulsion impulse to both drones:

$$\Delta\mathbf{v}_k^{rep} = +\frac{v_{rep}\,(d_{safe} - d_{kj})}{d_{kj}}\,\mathbf{r}_{kj}$$

$$\Delta\mathbf{v}_j^{rep} = -\frac{v_{rep}\,(d_{safe} - d_{kj})}{d_{kj}}\,\mathbf{r}_{kj}$$

where $v_{rep} = 0.5\,\text{m/s}$ is the repulsion gain. The position update then blends the
repulsion velocity back to the choreography reference over a recovery time $t_{rec} = 0.2$ s via an
exponential filter:

$$\mathbf{p}_k(t + \Delta t) = (1 - \alpha_{rec})\,\mathbf{p}_k^{cmd}(t + \Delta t) + \alpha_{rec}\,\left[\mathbf{p}_k(t) + \Delta\mathbf{v}_k^{rep}\,\Delta t\right]$$

with $\alpha_{rec} = e^{-\Delta t / t_{rec}}$.

### PTP Clock Correction (unchanged structure, same EMA)

$$\hat{\delta}_k \leftarrow (1 - \beta)\,\hat{\delta}_k + \beta\,\Delta_k, \qquad \delta_k \leftarrow \delta_k - \hat{\delta}_k$$

---

## Key 3D Additions

- **3D choreography keyframes**: each waypoint is $(x, y, z, t)$; altitude is independently
  choreographed per act rather than added as a uniform decoration.
- **Minimum-snap trajectory**: degree-7 polynomial per axis per segment enforces zero velocity,
  acceleration, and jerk at keyframe boundaries; replaces the natural cubic spline used in S098.
- **Altitude-layer synchronisation KPI**: $\varepsilon_{alt}(t)$ tracks how accurately drones
  achieve their intended layer altitude; target $< 0.5$ m.
- **3D collision-avoidance during transitions**: repulsion impulse layer active throughout Act II
  (rising double helix) and Act IV (3D sphere lattice) where drones cross altitude layers.
- **3D visual fidelity**: error metric now includes z-axis displacement, not just xy separation.
- **3D trajectory visualisation**: actual flight paths rendered on 3D axes with per-act colouring;
  altitude time-series shown as a separate subplot per drone cluster.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones | $N$ | 10 |
| Performance duration | $T_{show}$ | 60 s |
| Altitude volume | $z$ range | 30–70 m |
| Simulation timestep | $\Delta t_{sim}$ | 0.05 s (20 Hz) |
| PTP correction interval | $\Delta t_{ptp}$ | 0.5 s |
| Clock drift std dev | $\sigma_{drift}$ | 5 ms per correction interval |
| Initial offset range | $\delta_k(0)$ | $\mathcal{U}(-5, +5)$ ms |
| PTP gain (standard) | $\beta$ | 0.5 |
| PTP gain (aggressive) | $\beta$ | 0.9 |
| Synchronisation target | $\varepsilon_{sync}$ | < 10 ms |
| Altitude layer target | $\varepsilon_{alt}$ | < 0.5 m |
| Visual fidelity target | $V$ | > 90% |
| Trajectory polynomial degree | — | 7 (minimum-snap) |
| Keyframes per drone | $N_{kf}$ | 9 (act boundaries + midpoints) |
| Safety separation | $d_{safe}$ | 2.0 m |
| Repulsion gain | $v_{rep}$ | 0.5 m/s |
| Collision recovery time | $t_{rec}$ | 0.2 s |
| Inner ring radius | $r_1$ | 8 m |
| Outer ring radius | $r_2$ | 16 m |
| Sphere lattice radius (Act IV) | $r_{sphere}$ | 12 m |

---

## Expected Output

- **3D choreography map** (`choreography_3d.png`): full 3D trajectories of all 10 drones coloured
  by act (four colour bands); altitude variation clearly visible in the z-axis; sphere lattice
  formation evident in Act IV.

- **Sync error plot** (`sync_error.png`): $\varepsilon_{sync}(t)$ over 60 s for all three strategies
  with the 10 ms target line; identical qualitative result to S098 but now driven by a 3D trajectory
  with larger velocity magnitudes, making clock errors more costly.

- **Altitude layer error plot** (`altitude_error.png`): $\varepsilon_{alt}(t)$ over 60 s for all
  three strategies with the 0.5 m target line; no-sync strategy breaches the target during Act II
  and Act IV altitude transitions; PTP strategies remain below threshold.

- **Visual fidelity plot** (`visual_fidelity.png`): $V(t)$ (%) for all three strategies; 3D fidelity
  degradation under no-sync is more severe than in S098 because z-axis errors add to the norm.

- **Minimum separation plot** (`min_separation.png`): time series of $\min_{k < j} d_{kj}(t)$ for
  all three strategies; red dashed line at $d_{safe} = 2.0$ m; violations occur under no-sync
  during Act II altitude crossings; PTP strategies avoid violations.

- **Altitude time series** (`altitude_timeseries.png`): z-coordinate of all 10 drones vs. time
  (ideal vs. actual under PTP-standard); vertical bands marking act transitions; drones coloured by
  cluster.

- **Synchronised dance animation** (`sync_dance_animation.gif`): side-by-side 3D view (no-sync vs.
  PTP-standard) at 10 fps; both 3D scatter and a live altitude bar chart rendered per frame.

**Typical numerical targets**:

| Strategy | Max sync error | Mean visual fidelity | Max altitude error | Min separation |
|----------|---------------|----------------------|--------------------|----------------|
| No sync | ~50 ms | ~70% | ~2.0 m | < 1.0 m (violation) |
| PTP $\beta=0.5$ | < 10 ms | > 95% | < 0.5 m | > 2.0 m (safe) |
| PTP $\beta=0.9$ | < 8 ms | > 97% | < 0.4 m | > 2.0 m (safe) |

---

## Extensions

1. **Optimal keyframe timing**: given minimum-snap polynomials, the segment duration $T_n$ affects
   peak jerk and velocity; formulate the keyframe timing as an optimisation over total snap cost
   subject to maximum thrust constraints per drone.
2. **Music beat-aligned altitude layers**: snap act transitions to beat onsets of a backing audio
   track; the PTP correction interval is aligned to beat period so residual clock jitter is masked
   by the percussion transient.
3. **Fault-tolerant re-choreography**: one drone drops out mid-performance; the remaining nine
   drones receive an updated keyframe table via broadcast and seamlessly close the gap in the
   sphere lattice formation without breaching $d_{safe}$.
4. **Wind-disturbance rejection**: add a horizontal wind field $\mathbf{w}(z, t)$ that varies with
   altitude; the minimum-snap trajectory is recomputed on-board using a receding-horizon MPC that
   accounts for the predicted wind over the next keyframe interval.
5. **Perceptual quality model**: replace the geometric visual fidelity $V(t)$ with a
   just-noticeable-difference model based on viewer distance and angular resolution; compute the
   maximum tolerable clock offset as a function of audience standoff distance.
6. **3D light-trail visualisation**: attach LED colour to altitude via a hue map
   $H(z) = (z - z_{min})/(z_{max} - z_{min})$; evaluate whether altitude synchronisation errors
   produce perceptible colour-band mismatches in long-exposure photography.

---

## Related Scenarios

- Original 2D version: [S098](../S098_synchronized_dance.md)
- Formation in 3D: [S083 3D Light Show](S083_3d_light_show.md), [S085 3D Light Matrix](S085_3d_light_matrix.md)
- Clock sync reference: [S095 Swarm Communication](../S095_swarm_communication.md)
- Collision avoidance at scale: [S089 Collision Avoidance](../S089_collision_avoidance.md)
- Pursuit & evasion 3D reference: [S002 3D Evasive Maneuver](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
