# S072 Oil Pipeline Leak Detection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Pipeline Line-Following + Chemical Gradient Localisation | **Dimension**: 3D

---

## Problem Definition

**Setup**: A 200 m oil pipeline segment is modelled as a piecewise-linear polyline at $z = 0$ m
(ground level). A single drone patrols the pipeline at a fixed flight height $h = 3$ m directly
above the centreline, moving at patrol speed $v_{patrol} = 2$ m/s. The drone carries a chemical
sensor (e.g. a photoionisation detector) that samples gas concentration every $\Delta t = 0.5$ s.

One gas leak exists at an unknown location along the pipeline. The leak emits a chemical tracer
that disperses as a Gaussian instantaneous-point-source plume driven by a steady horizontal wind.
When the measured concentration first exceeds detection threshold $C_{thresh}$, the drone exits
patrol mode and begins a spiral gradient-ascent search centred on the detection point to localise
the leak precisely.

**Roles**:
- **Drone**: single UAV; follows the pipeline Frenet frame during patrol; switches to
  gradient-ascent spiral on detection.
- **Pipeline**: 200 m piecewise-linear polyline defined by 5 vertices at $z = 0$ m; the drone
  tracks a virtual reference path offset by $\Delta z_{offset} = 3$ m above the cable.
- **Leak source**: fixed point at arc-length $s_{leak}$ (unknown to drone) emitting chemical at
  rate $Q = 1.0$ g/s; disperses as an instantaneous Gaussian plume advected by wind.

**Objective**: Detect the leak (concentration exceeds $C_{thresh}$) within the patrol pass, then
localise the leak to within accuracy $\varepsilon_{target} = 2$ m via gradient ascent. Report
detection time $t_{detect}$, leak position error $\varepsilon = \|\hat{\mathbf{p}}_{leak} -
\mathbf{p}_{leak,true}\|$, and false alarm rate across Monte Carlo trials.

**Comparison strategies**:
1. **Patrol-only** — drone completes the full 200 m pass without switching to localisation; serves
   as a baseline to confirm detection reliability.
2. **Patrol + gradient ascent (xy-plane)** — on detection, drone descends to $z = 1$ m and
   executes horizontal gradient ascent toward the concentration peak.
3. **Patrol + 3D spiral** — on detection, drone executes a 3D inward spiral with decreasing radius,
   descending toward $z = 1$ m; more robust in noisy plume conditions.

---

## Mathematical Model

### Pipeline Polyline and Frenet Frame

Let the pipeline be defined by vertices $\mathbf{p}_0, \mathbf{p}_1, \ldots, \mathbf{p}_M \in
\mathbb{R}^3$ all at $z = 0$ m. Each segment $i$ has arc-length $\ell_i =
\|\mathbf{p}_{i+1} - \mathbf{p}_i\|$ and total arc-length $S = \sum_i \ell_i = 200$ m.

For each segment the **unit tangent** is:

$$\hat{T}_i = \frac{\mathbf{p}_{i+1} - \mathbf{p}_i}{\|\mathbf{p}_{i+1} - \mathbf{p}_i\|}$$

The **lateral normal** (horizontal, perpendicular to the pipeline):

$$\hat{N}_i = \frac{\hat{T}_i \times \hat{z}}{\|\hat{T}_i \times \hat{z}\|}$$

The **vertical binormal**:

$$\hat{B}_i = \hat{T}_i \times \hat{N}_i$$

### Drone Reference Path

The drone flies directly above the pipeline centreline at height $h = 3$ m:

$$\mathbf{p}_{nom}(s) = \mathbf{p}_{ref}(s) + h\,\hat{z}$$

where $\mathbf{p}_{ref}(s)$ is the point on the polyline at arc-length $s$ and $\hat{z} =
(0,0,1)^T$.

### Frenet Error Decomposition

Following the convention from S061, the tracking errors at drone position $\mathbf{p}_{drone}$
are:

$$e_T = (\mathbf{p}_{drone} - \mathbf{p}_{nom}) \cdot \hat{T}_i \quad \text{(along-track lag)}$$

$$e_N = (\mathbf{p}_{drone} - \mathbf{p}_{nom}) \cdot \hat{N}_i \quad \text{(cross-track error)}$$

$$e_B = (\mathbf{p}_{drone} - \mathbf{p}_{nom}) \cdot \hat{z} \quad \text{(vertical error)}$$

Three independent PID loops regulate these errors; the total commanded acceleration:

$$\mathbf{a}_{PID} = a_T\,\hat{T}_i + a_N\,\hat{N}_i + a_B\,\hat{z}$$

with GPS noise $\boldsymbol{\eta}_{GPS} \sim \mathcal{N}(\mathbf{0},\,\sigma_{GPS}^2\mathbf{I})$,
$\sigma_{GPS} = 0.1$ m, added at each timestep.

### Gaussian Plume Dispersion (Instantaneous Point Source)

The chemical concentration field is modelled as an instantaneous point-source Gaussian diffusion
plume. At time $t$ after the leak started, the concentration at position $\mathbf{r}$ is:

$$C(\mathbf{r}, t) = \frac{Q}{(4\pi D t)^{3/2}} \exp\!\left(-\frac{\|\mathbf{r} - \mathbf{r}_{leak}(t)\|^2}{4Dt}\right)$$

where $Q = 1.0$ g is the total emitted mass per "puff", $D = 0.5$ m²/s is the effective
turbulent diffusion coefficient, and $\mathbf{r}_{leak}(t) = \mathbf{r}_{leak,0} +
\mathbf{u}_{wind}\,t$ is the advected plume centroid with wind velocity
$\mathbf{u}_{wind} = (u_x, u_y, 0)$ m/s.

For a continuous leak emitting at rate $\dot{Q}$ (g/s), the measured field is the superposition
of puffs released at times $t' \in [0, t]$:

$$C(\mathbf{r}, t) = \int_0^t \frac{\dot{Q}}{(4\pi D(t-t'))^{3/2}}
  \exp\!\left(-\frac{\|\mathbf{r} - \mathbf{r}_{leak,0} - \mathbf{u}_{wind}(t - t')\|^2}{4D(t-t')}\right) dt'$$

In simulation this integral is approximated by summing $N_{puff}$ discrete puffs separated by
$\Delta t_{puff} = 0.5$ s.

### Sensor Model

The measured concentration at drone position $\mathbf{p}$ at time $t$ is:

$$C_{meas}(t) = C(\mathbf{p}(t), t) + \eta(t), \qquad \eta(t) \sim \mathcal{N}(0,\,\sigma_{sensor}^2)$$

with $\sigma_{sensor} = 5 \times 10^{-4}$ (normalised). Detection is declared when:

$$C_{meas}(t) > C_{thresh} = 0.01 \text{ (normalised)}$$

False alarms occur when the noise term alone exceeds $C_{thresh}$; the false alarm rate is:

$$P_{FA} = P\bigl(\eta > C_{thresh}\bigr) = 1 - \Phi\!\left(\frac{C_{thresh}}{\sigma_{sensor}}\right)$$

where $\Phi$ is the standard normal CDF.

### Gradient-Ascent Leak Localisation

On first detection at drone position $\mathbf{p}_{detect}$, the drone switches to a
gradient-ascent spiral search. The numerical concentration gradient is estimated by finite
differences with probe offset $\delta = 0.5$ m along each axis:

$$\hat{\nabla}_x C \approx \frac{C_{meas}(x+\delta,y,z) - C_{meas}(x-\delta,y,z)}{2\delta}$$

$$\hat{\nabla}_y C \approx \frac{C_{meas}(x,y+\delta,z) - C_{meas}(x,y-\delta,z)}{2\delta}$$

The normalised ascent direction:

$$\hat{\mathbf{g}} = \frac{(\hat{\nabla}_x C,\;\hat{\nabla}_y C,\;0)}
                         {\|(\hat{\nabla}_x C,\;\hat{\nabla}_y C,\;0)\| + \varepsilon}$$

The drone command velocity during gradient ascent:

$$\mathbf{v}_{cmd} = v_{ascent}\,\hat{\mathbf{g}} + \mathbf{v}_{descent}$$

where $v_{ascent} = 1.0$ m/s and $\mathbf{v}_{descent} = (0,0,-v_z)$ with $v_z = 0.1$ m/s
gradually lowering the drone toward the pipeline level for closer sampling. The drone stops
descending below $z_{min} = 0.5$ m.

### Leak Position Estimate

The estimated leak position is taken as the drone position at maximum sampled concentration:

$$\hat{\mathbf{p}}_{leak} = \mathbf{p}\!\left(\arg\max_{t \geq t_{detect}} C_{meas}(t)\right)$$

The localisation error metric is:

$$\varepsilon = \|\hat{\mathbf{p}}_{leak} - \mathbf{p}_{leak,true}\|$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# ── Pipeline geometry ─────────────────────────────────────────────────────────
PIPELINE_VERTICES = np.array([
    [  0.0,  0.0, 0.0],   # start
    [ 50.0,  5.0, 0.0],   # bend 1
    [100.0, -3.0, 0.0],   # bend 2
    [150.0,  4.0, 0.0],   # bend 3
    [200.0,  0.0, 0.0],   # end
])

# True leak position (unknown to drone)
LEAK_ARC_LENGTH = 117.0   # m along pipeline arc
LEAK_XY         = np.array([117.0, 0.5, 0.0])  # approximate; computed from polyline

# ── Domain constants (Domain 4) ───────────────────────────────────────────────
INSPECTION_SPEED = 1.0    # m/s  (domain constant; patrol uses 2× this)
INSPECTION_DIST  = 0.5    # m    (domain constant; vertical offset above cable)
GPS_ERROR        = 0.1    # m    (domain constant)

# ── Patrol parameters ─────────────────────────────────────────────────────────
PATROL_SPEED     = 2.0    # m/s, along-pipeline patrol speed
PATROL_HEIGHT    = 3.0    # m, drone flight height above pipeline
SAMPLE_DT        = 0.5    # s, sensor sampling interval
DT               = 0.05   # s, simulation timestep

# ── Plume model ───────────────────────────────────────────────────────────────
Q_PUFF           = 0.5    # g, mass per discrete puff
EMIT_RATE        = 1.0    # g/s, continuous emission rate
D_DIFFUSE        = 0.5    # m²/s, turbulent diffusion coefficient
WIND_VEL         = np.array([0.5, 0.2, 0.0])   # m/s, steady wind
PUFF_DT          = 0.5    # s, puff release interval
SIGMA_SENSOR     = 5e-4   # normalised sensor noise std
C_THRESH         = 0.01   # detection threshold (normalised)

# ── Localisation ──────────────────────────────────────────────────────────────
V_ASCENT         = 1.0    # m/s, gradient-ascent horizontal speed
V_DESCENT        = 0.1    # m/s, descent rate during localisation
Z_MIN            = 0.5    # m, minimum flight altitude
GRAD_DELTA       = 0.5    # m, finite-difference probe offset
T_MAX_LOC        = 120.0  # s, localisation timeout

# ── PID gains (Frenet frame) ──────────────────────────────────────────────────
KP_T, KD_T           = 1.2, 0.4
KP_N, KD_N, KI_N     = 2.0, 0.8, 0.1
KP_B, KD_B           = 2.5, 1.0


def build_frenet_frames(vertices):
    """Compute tangent, normal, and binormal for each pipeline segment."""
    frames = []
    for i in range(len(vertices) - 1):
        T = vertices[i + 1] - vertices[i]
        T = T / np.linalg.norm(T)
        z_hat = np.array([0.0, 0.0, 1.0])
        N = np.cross(T, z_hat)
        n_norm = np.linalg.norm(N)
        N = N / n_norm if n_norm > 1e-8 else np.array([0.0, 1.0, 0.0])
        B = np.cross(T, N)
        frames.append({
            "T": T, "N": N, "B": B,
            "p0": vertices[i], "p1": vertices[i + 1],
            "length": np.linalg.norm(vertices[i + 1] - vertices[i]),
        })
    return frames


def project_to_pipeline(p_drone, frames):
    """Return (arc_length s, ref_point, segment_index, T, N)."""
    best_s, best_ref, best_seg = 0.0, frames[0]["p0"].copy(), 0
    cumulative, best_dist = 0.0, np.inf
    for i, fr in enumerate(frames):
        v = p_drone - fr["p0"]
        t_proj = np.clip(np.dot(v, fr["T"]) / fr["length"], 0.0, 1.0)
        ref = fr["p0"] + t_proj * fr["length"] * fr["T"]
        dist = np.linalg.norm(p_drone - ref)
        if dist < best_dist:
            best_dist = dist
            best_s = cumulative + t_proj * fr["length"]
            best_ref = ref.copy()
            best_seg = i
        cumulative += fr["length"]
    fr = frames[best_seg]
    return best_s, best_ref, best_seg, fr["T"], fr["N"]


def gaussian_puff_concentration(pos, puff_centers, puff_ages, Q_puff, D):
    """
    Superpose all discrete puffs.
    puff_centers : list of np.array([x, y, z])
    puff_ages    : list of floats (seconds since release)
    """
    c = 0.0
    for center, age in zip(puff_centers, puff_ages):
        if age < 1e-6:
            continue
        denom = (4.0 * np.pi * D * age) ** 1.5
        r2 = np.sum((pos - center) ** 2)
        c += (Q_puff / denom) * np.exp(-r2 / (4.0 * D * age))
    return c


def measure_concentration(pos, puff_centers, puff_ages, rng):
    """Noisy sensor reading at drone position."""
    c_true = gaussian_puff_concentration(pos, puff_centers, puff_ages,
                                         Q_PUFF, D_DIFFUSE)
    return max(0.0, c_true + rng.normal(0.0, SIGMA_SENSOR))


def estimate_gradient(pos, puff_centers, puff_ages, rng):
    """Finite-difference concentration gradient in xy-plane."""
    dx = np.array([GRAD_DELTA, 0.0, 0.0])
    dy = np.array([0.0, GRAD_DELTA, 0.0])
    c_xp = measure_concentration(pos + dx, puff_centers, puff_ages, rng)
    c_xm = measure_concentration(pos - dx, puff_centers, puff_ages, rng)
    c_yp = measure_concentration(pos + dy, puff_centers, puff_ages, rng)
    c_ym = measure_concentration(pos - dy, puff_centers, puff_ages, rng)
    grad = np.array([(c_xp - c_xm) / (2 * GRAD_DELTA),
                     (c_yp - c_ym) / (2 * GRAD_DELTA),
                     0.0])
    norm = np.linalg.norm(grad) + 1e-12
    return grad / norm


def arc_to_world(frames, s_target):
    """Convert arc-length s_target to 3D world position on polyline."""
    cumulative = 0.0
    for fr in frames:
        if cumulative + fr["length"] >= s_target or fr is frames[-1]:
            frac = min((s_target - cumulative) / fr["length"], 1.0)
            return fr["p0"] + frac * fr["length"] * fr["T"]
        cumulative += fr["length"]
    return frames[-1]["p1"].copy()


def run_simulation(seed=42):
    rng = np.random.default_rng(seed)
    frames = build_frenet_frames(PIPELINE_VERTICES)
    total_arc = sum(fr["length"] for fr in frames)

    # Leak world position
    leak_pos = arc_to_world(frames, LEAK_ARC_LENGTH)

    # Initialise puff list
    puff_centers = []
    puff_ages    = []
    next_puff_t  = 0.0

    # Drone initial position (above pipeline start)
    pos = PIPELINE_VERTICES[0] + np.array([0.0, 0.0, PATROL_HEIGHT])
    vel = np.zeros(3)
    i_N = 0.0
    prev_eN, prev_eB = 0.0, 0.0

    trajectory   = [pos.copy()]
    concentrations = []
    arc_log      = [0.0]
    mode         = 'patrol'     # 'patrol' | 'localise'
    t_detect     = None
    p_detect     = None
    t            = 0.0

    best_c       = 0.0
    best_p       = pos.copy()

    while True:
        # ── Release new puffs ─────────────────────────────────────────────────
        if t >= next_puff_t:
            puff_centers.append(leak_pos.copy())
            puff_ages.append(0.0)
            next_puff_t += PUFF_DT

        # ── Advect & age puffs ────────────────────────────────────────────────
        puff_centers = [c + WIND_VEL * DT for c in puff_centers]
        puff_ages    = [a + DT for a in puff_ages]

        # ── Sensor sample (at SAMPLE_DT rate) ─────────────────────────────────
        c_meas = measure_concentration(pos, puff_centers, puff_ages, rng)
        concentrations.append(c_meas)
        if c_meas > best_c:
            best_c = c_meas
            best_p = pos.copy()

        if mode == 'patrol':
            # ── Pipeline following (Frenet PID) ───────────────────────────────
            s, p_ref, seg_idx, T_hat, N_hat = project_to_pipeline(pos, frames)
            arc_log.append(s)

            p_nom = p_ref + PATROL_HEIGHT * np.array([0.0, 0.0, 1.0])
            diff  = pos - p_nom
            e_N   = np.dot(diff, N_hat)
            e_B   = diff[2]
            s_dot = np.dot(vel, T_hat)

            a_T = KP_T * (PATROL_SPEED - s_dot) - KD_T * np.dot(vel, T_hat)
            i_N += e_N * DT
            d_eN = (e_N - prev_eN) / DT
            a_N  = -(KP_N * e_N + KD_N * d_eN + KI_N * i_N)
            d_eB = (e_B - prev_eB) / DT
            a_B  = -(KP_B * e_B + KD_B * d_eB)
            prev_eN, prev_eB = e_N, e_B

            a_cmd = a_T * T_hat + a_N * N_hat + a_B * np.array([0, 0, 1])
            vel   = vel + a_cmd * DT
            noise = rng.normal(0.0, GPS_ERROR, 3)
            pos   = pos + vel * DT + noise

            # Detection check
            if c_meas > C_THRESH and t_detect is None:
                t_detect = t
                p_detect = pos.copy()
                mode     = 'localise'

            if s >= total_arc - 0.5:
                break  # completed patrol without detection

        elif mode == 'localise':
            # ── Gradient ascent ───────────────────────────────────────────────
            g_hat  = estimate_gradient(pos, puff_centers, puff_ages, rng)
            v_horiz = V_ASCENT * g_hat[:3]
            v_horiz[2] = 0.0
            v_z    = -V_DESCENT if pos[2] > Z_MIN else 0.0
            v_cmd  = v_horiz + np.array([0.0, 0.0, v_z])

            noise = rng.normal(0.0, GPS_ERROR, 3)
            pos   = pos + v_cmd * DT + noise
            pos[2] = max(pos[2], Z_MIN)

            if t - t_detect >= T_MAX_LOC:
                break  # localisation timeout

        trajectory.append(pos.copy())
        t += DT

    leak_error = float(np.linalg.norm(best_p - leak_pos))
    return {
        "trajectory":     np.array(trajectory),
        "concentrations": np.array(concentrations),
        "arc_log":        np.array(arc_log),
        "leak_pos":       leak_pos,
        "best_pos":       best_p,
        "leak_error":     leak_error,
        "t_detect":       t_detect,
        "p_detect":       p_detect,
        "mode_switch_t":  t_detect,
        "total_time":     t,
        "frames":         frames,
    }
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Pipeline length | $S$ | 200 m (polyline) |
| Drone patrol height | $h$ | 3.0 m |
| Patrol speed | $v_{patrol}$ | 2.0 m/s |
| Sensor sampling interval | $\Delta t$ | 0.5 s |
| Domain inspection speed | $v_{insp}$ | 1.0 m/s |
| Domain inspection distance | $d_{insp}$ | 0.5 m |
| GPS noise std dev | $\sigma_{GPS}$ | 0.1 m |
| Emission rate | $\dot{Q}$ | 1.0 g/s |
| Puff mass | $Q_{puff}$ | 0.5 g |
| Turbulent diffusion coefficient | $D$ | 0.5 m²/s |
| Wind velocity | $\mathbf{u}_{wind}$ | $(0.5, 0.2, 0)$ m/s |
| Puff release interval | $\Delta t_{puff}$ | 0.5 s |
| Sensor noise std dev | $\sigma_{sensor}$ | $5 \times 10^{-4}$ (normalised) |
| Detection threshold | $C_{thresh}$ | 0.01 (normalised) |
| Gradient probe offset | $\delta$ | 0.5 m |
| Gradient ascent speed | $v_{ascent}$ | 1.0 m/s |
| Descent rate during localisation | $v_z$ | 0.1 m/s |
| Minimum flight altitude | $z_{min}$ | 0.5 m |
| Localisation timeout | $T_{max,loc}$ | 120 s |
| Along-track PID gains $(K_p, K_d)$ | — | 1.2, 0.4 |
| Cross-track PID gains $(K_p, K_d, K_i)$ | — | 2.0, 0.8, 0.1 |
| Vertical PID gains $(K_p, K_d)$ | — | 2.5, 1.0 |
| Simulation timestep | $\Delta t_{sim}$ | 0.05 s |

---

## Expected Output

- **3D scene plot** (`mpl_toolkits.mplot3d`): pipeline polyline in grey, nominal patrol path at
  $h = 3$ m in green, actual drone trajectory colour-mapped by mission phase (blue = patrol,
  orange = localisation); true leak position marked with a red sphere; estimated leak position
  as a white diamond; wind direction arrow shown in the scene.
- **Concentration time series**: $C_{meas}(t)$ vs time with $C_{thresh}$ shown as a dashed red
  line; detection event marked with a vertical dashed line and label; background noise floor
  shaded in light grey; peak concentration marked with a star.
- **Plan-view (top-down 2D) trajectory**: $x$-$y$ projection of the drone path overlaid on a
  filled contour plot of the instantaneous plume concentration field at the moment of detection;
  pipeline centreline in black; leak marked with a red cross; detection point as an orange dot;
  gradient ascent spiral as a dashed orange curve.
- **Monte Carlo error statistics** ($N_{MC} = 30$ trials, varying leak position and seed):
  histogram of localisation error $\varepsilon$; vertical dashed lines at mean and $\varepsilon_{target} = 2$ m;
  detection time $t_{detect}$ box plot; false alarm count annotated.
- **Animation (GIF)** (`FuncAnimation`): top-down 2D view of the drone moving along the pipeline
  then spiralling to the leak; plume concentration field rendered as a semi-transparent heatmap
  that evolves with wind advection each frame; mode label (PATROL / LOCALISE) displayed in the
  corner; true and estimated leak positions shown; saved to
  `outputs/04_industrial_agriculture/s072_pipeline_leak/s072_patrol.gif`.

Terminal metrics printed at completion:

```
Patrol distance:       200.0 m
Detection time:        t_detect s
Detection position:    (x, y, z) m
Leak position error:   epsilon m
False alarm rate:      P_FA %
```

---

## Extensions

1. **Multiple leaks**: place $K = 3$ leaks at random arc-length positions; the drone must
   sequentially localise all of them in a single mission using a priority queue sorted by
   measured concentration; evaluate fraction of leaks found within a fixed mission budget.
2. **Night-time thermal variant**: replace the chemical plume with a warm gas jet detectable by
   an IR sensor; blend the thermal and chemical likelihood in a joint particle filter posterior
   for more robust leak position estimation (see S056 for the joint-model concept).
3. **Non-uniform wind field**: replace constant wind with a spatially varying field (e.g.
   potential-flow around cylindrical pipeline supports); evaluate gradient ascent performance
   under wind shadowing that distorts the plume centroid.
4. **RL localisation policy**: train a PPO agent with state $(C_{meas}, \hat{\nabla}C, \mathbf{p}_{drone})$
   and continuous action $\mathbf{v}_{cmd}$; compare average localisation error and time against
   the gradient-ascent heuristic across 100 test configurations.
5. **Multi-drone parallel patrol**: split the 200 m pipeline between $N = 2$ drones starting at
   opposite ends; broadcast detections and fuse concentration measurements via a shared
   particle filter; evaluate speedup in detection and localisation time.
6. **Underground pipeline variant**: mount the drone on a ground vehicle crawling through an
   underground tunnel; replace the Frenet patrol with a 2D corridor-following controller and
   evaluate how enclosed geometry changes plume dispersion and gradient quality.

---

## Related Scenarios

- Prerequisites: [S045 Chemical Plume Tracing](../03_environmental_sar/S045_plume_tracing.md), [S056 Radiation Hotspot Detection](../03_environmental_sar/S056_radiation.md)
- Follow-ups: [S073 Pump Station Vibration](S073_pump_station_vibration.md), [S074 Tank Farm Inspection](S074_tank_farm_inspection.md)
- Algorithmic cross-reference: [S061 Power Line Inspection](S061_power_line.md) (Frenet-frame pipeline following), [S045 Chemical Plume Tracing](../03_environmental_sar/S045_plume_tracing.md) (Gaussian plume model and cast-and-surge), [S056 Radiation Hotspot Detection](../03_environmental_sar/S056_radiation.md) (gradient-ascent source localisation)
