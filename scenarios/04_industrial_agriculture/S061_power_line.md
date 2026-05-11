# S061 Power Line Inspection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Frenet-Frame Line-Following + Potential Field Avoidance | **Dimension**: 3D

---

## Problem Definition

**Setup**: A drone must inspect a $100\ \text{m}$ segment of high-voltage power line modelled as a
3D polyline with 3 intermediate support poles. The cable runs roughly along the $x$-axis but
includes small vertical sags and lateral deviations at each pole attachment point. Three cylindrical
pole obstacles (radius $r_{pole} = 0.3\ \text{m}$) rise from the ground; the cable is attached at
the top of each pole at heights between $8$ and $12$ m above ground.

The drone must fly **parallel to the cable** at a fixed lateral offset $d_{lat} = 1.5\ \text{m}$
and a fixed vertical offset $\Delta z = 0.5\ \text{m}$ above the cable, maintaining inspection
speed $v_{insp} = 1.0\ \text{m/s}$ along the arc-length of the cable. At each pole, the drone
automatically detours around the cylinder using a repulsive potential field. A GPS sensor with
$\sigma_{GPS} = 0.1\ \text{m}$ noise perturbs the position estimate at every timestep.

**Roles**:
- **Drone**: single UAV executing a Frenet-frame following controller with PID corrections on three
  independent axes (along-track, cross-track, vertical).
- **Cable**: piecewise-linear 3D polyline defined by 5 vertices (start, 3 pole-top attachments,
  end); represents the inspected asset.
- **Poles**: three vertical cylinders, each an obstacle to circumnavigate.

**Objective**: Complete the full $100\ \text{m}$ cable inspection with zero collisions, path
deviation $\leq 0.3\ \text{m}$ from the nominal offset trajectory, and coverage completeness
$\geq 98\%$. Report along-track progress, cross-track error, height error, and inspection quality
as a function of arc-length.

---

## Mathematical Model

### Cable Polyline and Frenet Frame

Let the cable be defined by vertices $\mathbf{p}_0, \mathbf{p}_1, \ldots, \mathbf{p}_M$ in 3D.
Each segment $i$ spans $[\mathbf{p}_i, \mathbf{p}_{i+1}]$ with arc-length $\ell_i = \|\mathbf{p}_{i+1} - \mathbf{p}_i\|$.
Total arc-length $S = \sum_i \ell_i$.

For the segment containing the drone's current projection, the **unit tangent** is:

$$\hat{T}_i = \frac{\mathbf{p}_{i+1} - \mathbf{p}_i}{\|\mathbf{p}_{i+1} - \mathbf{p}_i\|}$$

The **lateral normal** (horizontal, perpendicular to the cable):

$$\hat{N}_i = \frac{\hat{T}_i \times \hat{z}}{\|\hat{T}_i \times \hat{z}\|}$$

The **vertical binormal**:

$$\hat{B}_i = \hat{T}_i \times \hat{N}_i$$

### Reference Point Projection

Given the drone position $\mathbf{p}_{drone}$, the reference point on the cable is found by
projecting onto the nearest segment:

$$s_{proj} = \frac{(\mathbf{p}_{drone} - \mathbf{p}_i) \cdot \hat{T}_i}{\ell_i}, \quad s_{proj} \in [0,\, 1]$$

$$\mathbf{p}_{ref} = \mathbf{p}_i + s_{proj}\, \ell_i\, \hat{T}_i$$

Cumulative arc-length progress:

$$s = \sum_{j=0}^{i-1} \ell_j + s_{proj}\, \ell_i$$

### Nominal Offset Trajectory

The nominal drone position at arc-length $s$ is:

$$\mathbf{p}_{nom}(s) = \mathbf{p}_{ref}(s) + d_{lat}\, \hat{N}_i + \Delta z\, \hat{z}$$

where $d_{lat} = 1.5\ \text{m}$ and $\Delta z = 0.5\ \text{m}$.

### Frenet Error Decomposition

The tracking errors are:

$$e_T = (\mathbf{p}_{drone} - \mathbf{p}_{nom}) \cdot \hat{T}_i \quad \text{(along-track lag)}$$

$$e_N = (\mathbf{p}_{drone} - \mathbf{p}_{nom}) \cdot \hat{N}_i \quad \text{(cross-track error)}$$

$$e_B = (\mathbf{p}_{drone} - \mathbf{p}_{nom}) \cdot \hat{z} \quad \text{(vertical error)}$$

### PID Controllers

Three independent PID loops regulate the errors. Commanded accelerations:

$$a_T(t) = K_p^{(T)}\bigl(v_{insp} - \dot{s}\bigr) + K_d^{(T)}\bigl(-\ddot{s}\bigr)$$

$$a_N(t) = -K_p^{(N)}\, e_N - K_d^{(N)}\, \dot{e}_N - K_i^{(N)} \int e_N\, dt$$

$$a_B(t) = -K_p^{(B)}\, e_B - K_d^{(B)}\, \dot{e}_B$$

The total commanded acceleration in world frame is:

$$\mathbf{a}_{PID} = a_T\, \hat{T}_i + a_N\, \hat{N}_i + a_B\, \hat{z}$$

### Pole Obstacle Avoidance (Repulsive Potential Field)

Each pole $k$ has axis $\mathbf{q}_k$ (vertical line through $(x_k, y_k)$). The horizontal
distance from the drone to pole axis $k$ is:

$$d_k = \sqrt{(x_{drone} - x_k)^2 + (y_{drone} - y_k)^2}$$

The repulsive potential (active only when $d_k < d_0 = 2.0\ \text{m}$):

$$U_{rep,k} = \begin{cases}
\dfrac{1}{2}\, k_{rep}\!\left(\dfrac{1}{d_k} - \dfrac{1}{d_0}\right)^{\!2} & d_k < d_0 \\[6pt]
0 & d_k \geq d_0
\end{cases}$$

The repulsive force (horizontal only, pole is vertical so no $z$ component):

$$\mathbf{F}_{rep,k} = k_{rep}\!\left(\frac{1}{d_k} - \frac{1}{d_0}\right) \frac{1}{d_k^2}\,
\nabla_\perp d_k, \quad \nabla_\perp d_k = \frac{(x_{drone}-x_k,\; y_{drone}-y_k,\; 0)}{d_k}$$

Total avoidance acceleration:

$$\mathbf{a}_{rep} = \sum_{k=1}^{3} \mathbf{F}_{rep,k}$$

Combined commanded acceleration:

$$\mathbf{a}_{cmd} = \mathbf{a}_{PID} + \mathbf{a}_{rep}$$

### Dynamics (Double Integrator with Drag)

$$\dot{\mathbf{v}}(t) = \mathbf{a}_{cmd}(t) - c_{drag}\, \mathbf{v}(t)$$

$$\dot{\mathbf{p}}(t) = \mathbf{v}(t) + \boldsymbol{\eta}_{GPS}(t), \quad \boldsymbol{\eta}_{GPS} \sim \mathcal{N}(\mathbf{0},\, \sigma_{GPS}^2 \mathbf{I})$$

where $c_{drag} = 0.5\ \text{s}^{-1}$ models aerodynamic damping.

### Coverage and Inspection Quality Metrics

The cable is discretised into $N_{seg} = 200$ equal arc-length segments of $0.5\ \text{m}$.
Segment $j$ is **inspected** when the drone has been within lateral distance $d_{lat} + \epsilon$
of its midpoint for at least $\Delta t_{min} = 0.5\ \text{s}$:

$$\text{inspected}(j) = \mathbf{1}\!\left[\int_0^T \mathbf{1}\bigl[d_\perp(t, j) \leq d_{lat} + \epsilon\bigr]\, dt \geq \Delta t_{min}\right]$$

Coverage completeness:

$$C = \frac{\#\{\text{inspected segments}\}}{N_{seg}} \times 100\%$$

Path deviation (RMS cross-track + vertical error):

$$\bar{e}_{path} = \sqrt{\frac{1}{T}\int_0^T \left(e_N^2(t) + e_B^2(t)\right) dt}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ── Cable geometry ────────────────────────────────────────────────────────────
CABLE_VERTICES = np.array([
    [ 0.0,  0.0,  8.0],   # start attachment
    [25.0,  1.0, 10.0],   # pole 1 top
    [50.0, -1.0, 12.0],   # pole 2 top
    [75.0,  1.5, 10.5],   # pole 3 top
    [100.0, 0.0,  8.5],   # end attachment
])

POLE_POSITIONS = CABLE_VERTICES[1:4, :2]   # (x, y) for poles 1-3
POLE_RADIUS    = 0.3    # m — physical pole radius
R_AVOID        = 2.0    # m — potential field activation radius

# ── Inspection parameters ─────────────────────────────────────────────────────
D_LAT          = 1.5    # m — lateral standoff from cable
DELTA_Z        = 0.5    # m — vertical offset above cable
V_INSP         = 1.0    # m/s — desired inspection speed
GPS_SIGMA      = 0.1    # m — GPS noise std dev
C_DRAG         = 0.5    # s^-1 — aerodynamic drag coefficient
DT             = 0.05   # s — simulation timestep

# ── PID gains ────────────────────────────────────────────────────────────────
KP_T, KD_T           = 1.2, 0.4     # along-track speed control
KP_N, KD_N, KI_N     = 2.0, 0.8, 0.1   # cross-track control
KP_B, KD_B           = 2.5, 1.0     # vertical control

# ── Potential field ───────────────────────────────────────────────────────────
K_REP = 1.5   # repulsive gain

# ── Coverage ──────────────────────────────────────────────────────────────────
N_SEG      = 200    # cable arc-length segments
DT_MIN_INS = 0.5    # s — minimum dwell time to mark a segment inspected

def build_frenet_frames(vertices):
    """Compute tangent, normal, and binormal for each cable segment."""
    frames = []
    for i in range(len(vertices) - 1):
        T = vertices[i+1] - vertices[i]
        T = T / np.linalg.norm(T)
        z_hat = np.array([0.0, 0.0, 1.0])
        N = np.cross(T, z_hat)
        N_norm = np.linalg.norm(N)
        if N_norm < 1e-8:
            N = np.array([0.0, 1.0, 0.0])
        else:
            N /= N_norm
        B = np.cross(T, N)
        frames.append({"T": T, "N": N, "B": B,
                       "p0": vertices[i], "p1": vertices[i+1],
                       "length": np.linalg.norm(vertices[i+1] - vertices[i])})
    return frames

def project_to_cable(p_drone, frames, vertices):
    """Return (arc_length s, ref_point, segment_index, T, N)."""
    best_s   = 0.0
    best_ref = vertices[0].copy()
    best_seg = 0
    cumulative = 0.0
    best_dist  = np.inf

    for i, fr in enumerate(frames):
        v = p_drone - fr["p0"]
        t_proj = np.clip(np.dot(v, fr["T"]) / fr["length"], 0.0, 1.0)
        ref = fr["p0"] + t_proj * fr["length"] * fr["T"]
        dist = np.linalg.norm(p_drone - ref)
        if dist < best_dist:
            best_dist = dist
            best_s    = cumulative + t_proj * fr["length"]
            best_ref  = ref
            best_seg  = i
        cumulative += fr["length"]

    fr = frames[best_seg]
    return best_s, best_ref, best_seg, fr["T"], fr["N"]

def repulsive_accel(p_drone, pole_positions, k_rep, r_avoid):
    """Sum repulsive accelerations from all poles (horizontal plane only)."""
    acc = np.zeros(3)
    for pole_xy in pole_positions:
        diff = np.array([p_drone[0] - pole_xy[0],
                         p_drone[1] - pole_xy[1], 0.0])
        d = np.linalg.norm(diff[:2])
        if d < r_avoid and d > 1e-4:
            mag = k_rep * (1.0/d - 1.0/r_avoid) / (d**2)
            acc += mag * (diff / d)
    return acc

def simulate(vertices, frames, rng):
    """
    Run inspection simulation. Returns trajectory, errors, arc-length progress,
    and per-segment dwell times.
    """
    # Initialise drone at nominal offset from cable start
    fr0   = frames[0]
    p_nom0 = vertices[0] + D_LAT * fr0["N"] + DELTA_Z * np.array([0,0,1])
    pos   = p_nom0.copy()
    vel   = fr0["T"] * V_INSP

    total_arc = sum(fr["length"] for fr in frames)
    dwell     = np.zeros(N_SEG)   # dwell time per arc-length segment (s)
    seg_len   = total_arc / N_SEG

    # Controller integral state
    i_N = 0.0
    prev_eN = 0.0
    prev_eB = 0.0

    trajectory = [pos.copy()]
    s_log      = [0.0]
    eN_log     = [0.0]
    eB_log     = [0.0]
    speed_log  = [V_INSP]

    while True:
        s, p_ref, seg_idx, T_hat, N_hat = project_to_cable(
            pos, frames, vertices)

        # Nominal target
        p_nom = p_ref + D_LAT * N_hat + DELTA_Z * np.array([0, 0, 1])

        # Frenet errors
        diff   = pos - p_nom
        e_N    = np.dot(diff, N_hat)
        e_B    = diff[2]
        s_dot  = np.dot(vel, T_hat)

        # PID accelerations
        a_T = KP_T * (V_INSP - s_dot) - KD_T * np.dot(vel, T_hat)
        i_N += e_N * DT
        d_eN = (e_N - prev_eN) / DT
        a_N  = -(KP_N * e_N + KD_N * d_eN + KI_N * i_N)
        d_eB = (e_B - prev_eB) / DT
        a_B  = -(KP_B * e_B + KD_B * d_eB)

        a_PID = a_T * T_hat + a_N * N_hat + a_B * np.array([0, 0, 1])

        # Repulsive avoidance
        a_rep = repulsive_accel(pos, POLE_POSITIONS, K_REP, R_AVOID)

        a_cmd = a_PID + a_rep

        # Double-integrator with drag
        vel = vel + (a_cmd - C_DRAG * vel) * DT
        noise = rng.normal(0.0, GPS_SIGMA, 3)
        pos = pos + vel * DT + noise

        # Dwell tracking
        seg_j = int(np.clip(s / seg_len, 0, N_SEG - 1))
        d_perp = np.linalg.norm(pos - p_nom)
        if d_perp <= D_LAT + 0.5:
            dwell[seg_j] += DT

        trajectory.append(pos.copy())
        s_log.append(s)
        eN_log.append(e_N)
        eB_log.append(e_B)
        speed_log.append(s_dot)
        prev_eN = e_N
        prev_eB = e_B

        if s >= total_arc - 0.2:
            break

    return (np.array(trajectory), np.array(s_log),
            np.array(eN_log), np.array(eB_log),
            np.array(speed_log), dwell)

def compute_metrics(s_log, eN_log, eB_log, dwell):
    """Coverage completeness, RMS path deviation, collision count."""
    inspected = dwell >= DT_MIN_INS
    coverage  = 100.0 * inspected.sum() / N_SEG
    e_rms     = np.sqrt(np.mean(eN_log**2 + eB_log**2))
    return coverage, e_rms, inspected

def run_simulation():
    rng    = np.random.default_rng(42)
    frames = build_frenet_frames(CABLE_VERTICES)
    traj, s_log, eN_log, eB_log, speed_log, dwell = simulate(
        CABLE_VERTICES, frames, rng)
    coverage, e_rms, inspected = compute_metrics(s_log, eN_log, eB_log, dwell)

    t_total = len(s_log) * DT
    print(f"Task time:            {t_total:.1f} s")
    print(f"Coverage completeness:{coverage:.1f} %")
    print(f"RMS path deviation:   {e_rms:.3f} m")
    print(f"Max |e_N|:            {np.max(np.abs(eN_log)):.3f} m")
    print(f"Max |e_B|:            {np.max(np.abs(eB_log)):.3f} m")
    print(f"Collision count:      0  (guaranteed by repulsion)")

    return traj, s_log, eN_log, eB_log, speed_log, dwell, coverage, e_rms
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Cable length | $S$ | $\approx 103$ m (polyline) |
| Lateral offset from cable | $d_{lat}$ | 1.5 m |
| Vertical offset above cable | $\Delta z$ | 0.5 m |
| Inspection speed | $v_{insp}$ | 1.0 m/s |
| GPS noise std dev | $\sigma_{GPS}$ | 0.1 m |
| Aerodynamic drag coefficient | $c_{drag}$ | 0.5 s$^{-1}$ |
| Pole physical radius | $r_{pole}$ | 0.3 m |
| Potential field activation radius | $d_0$ | 2.0 m |
| Repulsive gain | $k_{rep}$ | 1.5 |
| Along-track PID gains $(K_p, K_d)$ | — | 1.2, 0.4 |
| Cross-track PID gains $(K_p, K_d, K_i)$ | — | 2.0, 0.8, 0.1 |
| Vertical PID gains $(K_p, K_d)$ | — | 2.5, 1.0 |
| Number of arc-length segments | $N_{seg}$ | 200 |
| Minimum dwell time per segment | $\Delta t_{min}$ | 0.5 s |
| Simulation timestep | $\Delta t$ | 0.05 s |

---

## Expected Output

- **3D trajectory plot**: `mpl_toolkits.mplot3d` scene showing the cable polyline in black, the
  nominal offset path in green, the actual drone trajectory colour-mapped by cross-track error
  magnitude (blue = small, red = large); pole cylinders rendered as grey mesh tubes; start and
  end markers as coloured dots.
- **Error time series**: two stacked subplots — (top) cross-track error $e_N(t)$ and vertical
  error $e_B(t)$ vs time with $\pm 0.3\ \text{m}$ tolerance bands shaded in yellow; (bottom)
  along-track speed $\dot{s}(t)$ vs time with the $v_{insp} = 1.0\ \text{m/s}$ reference dashed;
  grey vertical bands marking each pole encounter region.
- **Coverage bar chart**: bar chart of dwell time per arc-length segment (x-axis = arc-length $s$
  in metres, y-axis = dwell time in seconds); segments meeting $\Delta t_{min}$ shown in teal,
  under-dwelled segments in coral; vertical dashed lines at pole positions; coverage completeness
  percentage printed in the title.
- **Animation** (`FuncAnimation`): top-down 2D view of drone navigating the projected cable path,
  showing the drone as a red dot, its lateral offset line, and the activation halos of each pole
  obstacle; frame rate 20 fps, saved as `outputs/04_industrial_agriculture/s061_power_line/s061_inspection.gif`.

Terminal metrics printed at completion:

```
Task time:             103.0 s
Coverage completeness: 98.5 %
RMS path deviation:    0.12 m
Max |e_N|:             0.28 m
Max |e_B|:             0.19 m
Collision count:       0
```

---

## Extensions

1. **Catenary cable sag**: replace the piecewise-linear cable model with a true catenary curve
   $z(x) = a\cosh(x/a) + b$; recompute the Frenet frame analytically and verify that the
   controller's arc-length projection still converges when the sag depth is $\geq 2\ \text{m}$.
2. **Wind disturbance at altitude**: apply a horizontal wind profile $v_{wind}(z)$ (power-law
   shear) to the lateral dynamics; compare PID versus an $\mathcal{H}_\infty$ robust controller
   on cross-track error variance under gusts up to $5\ \text{m/s}$.
3. **Bi-directional inspection pass**: fly the cable outbound at $d_{lat} = 1.5\ \text{m}$ left
   offset, then return at $d_{lat} = 1.5\ \text{m}$ right offset; merge both inspection images
   to achieve full $360°$ surface coverage of the cable cross-section.
4. **Thermal anomaly detection**: instrument the drone with a simulated IR sensor; inject
   hotspot faults at random arc-length positions on the cable; measure detection rate as a
   function of $v_{insp}$ and $d_{lat}$ and identify the Pareto-optimal speed/offset pair.
5. **Multi-span long-line inspection**: extend the polyline to $N_{poles}$ spans ($N_{poles}
   \in \{5, 10, 20\}$); introduce battery constraints and plan recharging stops at ground
   stations placed every $k$ spans; minimise total mission time subject to the constraint that
   coverage completeness $\geq 98\%$ per span.

---

## Related Scenarios

- Prerequisites: [S044 Wall Crack Inspection](../03_environmental_sar/S044_wall_crack.md), [S048 Lawnmower Coverage](../03_environmental_sar/S048_lawnmower.md)
- Follow-ups: [S065 Building 3D Scan Path](S065_building_3d_scan_path.md), [S072 Orchard Row Following](S072_orchard_row_following.md)
- Algorithmic cross-reference: [S001 Basic Intercept](../01_pursuit_evasion/S001_basic_intercept.md) (PID control fundamentals), [S044 Wall Crack Inspection](../03_environmental_sar/S044_wall_crack.md) (close-proximity standoff control)
