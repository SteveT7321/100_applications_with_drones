# S083 3D Upgrade — Light Show Single Drone Test

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S083 original](../S083_light_show_single.md)

---

## What Changes in 3D

The original S083 fixes the drone at $z_{show} = 50$ m throughout the entire mission — the
figure-8 reference is a flat Lissajous in the $(x, y)$ plane and `pz_wp = np.full(n_wp, LED_SHOW_HEIGHT)`.
The z-spline is therefore a constant, the feedforward z-acceleration is identically zero, and the
vertical PID loop does nothing but hold a fixed altitude against wind.

This 3D upgrade replaces the flat figure-8 with a **fully volumetric choreography path** that
weaves through three altitude bands. Three new ingredients are added:

1. **3D choreography path** — the reference trajectory combines horizontal figure-8 lobes with
   sinusoidal altitude variation, producing distinct vertical arcs (ascent during outer lobes,
   descent at the crossing point) that are visible from every audience viewing angle.
2. **Altitude-varying LED intensity** — the commanded LED brightness $\ell(t)$ is a function of
   instantaneous altitude, creating a brightness-altitude coupling that audiences perceive as the
   drone "glowing brighter as it climbs."
3. **3D audience viewing-angle optimisation** — the choreography waypoints are shifted iteratively
   to maximise the minimum apparent angular separation of consecutive motion segments as seen from
   a fixed audience arc on the ground, making the spatial choreography legible from all seats.

---

## Problem Definition

**Setup**: A single drone executes a 3D pre-scripted light show over $T_{total} = 60$ s. The
reference trajectory is a **lifted figure-8** parameterised in all three axes. A **cubic spline**
interpolates 200 waypoints into a $C^2$-continuous path. A **PID + feedforward** controller
drives the drone; stochastic wind $\mathbf{w}(t) \sim \mathcal{N}(\mathbf{0}, \sigma_w^2 \mathbf{I}_3)$
acts on all three axes. Point-mass dynamics are integrated with `scipy.integrate.odeint`.

**Roles**:

- **Drone**: single UAV commanded by a full 3D position controller, with LED brightness output
  driven by the altitude profile.
- **Reference path**: 3D lifted figure-8 — horizontal Lissajous lobes combined with a vertical
  sinusoid, forming a saddle-shaped closed curve at altitudes between $z_{low} = 40$ m and
  $z_{high} = 60$ m.
- **Audience arc**: a semi-circular ring of $N_{aud} = 36$ observer positions at ground level,
  radius $R_{aud} = 100$ m, used to score viewing-angle quality of the choreography.
- **Wind disturbance**: independent Gaussian noise on all three axes, modelling atmospheric
  turbulence at show altitude.

**Objective**: For each control variant, simulate the full 60 s mission and report:

1. **RMS tracking error** $\varepsilon_{RMS}$ (m) — must be $\leq 0.15$ m (3D spec, relaxed from 2D
   because the path now uses the full vertical axis).
2. **Maximum instantaneous deviation** $\varepsilon_{max}$ (m).
3. **Path completion** — fraction of time steps within 0.5 m of the 3D reference.
4. **Mean minimum viewing angle** $\bar{\Phi}_{min}$ (°) — audience legibility score.
5. **LED brightness profile** $\ell(t)$ — time series of normalised intensity $\in [0, 1]$.

**Comparison configurations**:

1. **PID-only** — $K_p = 2.0$, $K_i = 0.1$, $K_d = 0.8$; no feedforward.
2. **PID + feedforward** — same gains plus $\mathbf{a}_{ff} = \mathbf{s}''(t)$ from the spline
   second derivative, now non-trivial on the z-axis because altitude is no longer constant.

---

## Mathematical Model

### 3D Choreography Reference Path

The lifted figure-8 reference at normalised time $\tau = t / T_{total} \in [0, 1)$ is:

$$\mathbf{p}_{ref}(\tau) = \begin{pmatrix}
  A_x \sin(2\pi\tau) \\
  A_y \sin(4\pi\tau) \\
  z_c + A_z \sin(4\pi\tau)
\end{pmatrix}$$

where $z_c = (z_{high} + z_{low})/2 = 50$ m is the central altitude, and
$A_z = (z_{high} - z_{low})/2 = 10$ m is the vertical semi-amplitude.
The z-component shares the same $4\pi\tau$ frequency as $y$, coupling the vertical arcs to the
outer lobes of the figure-8: the drone climbs to 60 m on each outer lobe and descends to 40 m
at the central crossing. Horizontal semi-amplitudes remain $A_x = A_y = 20$ m.

### Cubic Spline Interpolation (3D)

Waypoints $\{\mathbf{p}_{ref,k}\}_{k=0}^{N_{wp}-1}$ sampled at $\tau_k = k/N_{wp}$ are fitted
with independent not-a-knot cubic splines on all three axes:

$$\mathbf{p}_{ref}(t) = \mathbf{s}(t), \quad
\mathbf{v}_{ref}(t) = \mathbf{s}'(t), \quad
\dot{\mathbf{v}}_{ref}(t) = \mathbf{s}''(t)$$

The z-spline $s_z(t)$ is now genuinely non-constant; its second derivative provides a non-zero
feedforward acceleration on the vertical axis, which is the primary advantage of the 3D upgrade.

### PID + Feedforward Controller (Full 3D)

Position error and its derivative:

$$\mathbf{e}(t) = \mathbf{p}_{ref}(t) - \mathbf{p}(t), \qquad
\dot{\mathbf{e}}(t) = \mathbf{v}_{ref}(t) - \mathbf{v}(t)$$

PID command (scalar gains applied per-axis):

$$\mathbf{a}_{PID}(t) = K_p\,\mathbf{e}(t) + K_i\int_0^t \mathbf{e}(\tau)\,d\tau + K_d\,\dot{\mathbf{e}}(t)$$

3D feedforward term:

$$\mathbf{a}_{ff}(t) = \mathbf{s}''(t) = \bigl[s_x''(t),\; s_y''(t),\; s_z''(t)\bigr]^\top$$

Unlike the 2D case where $s_z'' \equiv 0$, here $s_z''(t) \ne 0$ and the feedforward directly
cancels the centripetal demand of the vertical arcs.

Total command:

$$\mathbf{a}_{cmd}(t) = \mathbf{a}_{PID}(t) + \mathbf{a}_{ff}(t)$$

### Point-Mass Dynamics with 3D Wind

State $\mathbf{x} = [\mathbf{p}^\top, \mathbf{v}^\top]^\top \in \mathbb{R}^6$:

$$\dot{\mathbf{p}} = \mathbf{v}, \qquad
\dot{\mathbf{v}} = \mathbf{a}_{cmd}(t) + \mathbf{w}(t),
\qquad \mathbf{w}(t) \sim \mathcal{N}(\mathbf{0},\, \sigma_w^2 \mathbf{I}_3)$$

Altitude is soft-bounded to $z \in [z_{low} - 5,\; z_{high} + 5]$ m by a saturation on $\mathbf{a}_{cmd,z}$
when the drone approaches the boundary.

### Altitude-Varying LED Intensity

The normalised LED brightness is a smooth map from altitude to intensity:

$$\ell(t) = \ell_{min} + (\ell_{max} - \ell_{min}) \cdot
  \frac{p_z(t) - z_{low}}{z_{high} - z_{low}}$$

clamped to $[\ell_{min}, \ell_{max}] = [0.3, 1.0]$. A higher drone therefore produces a brighter
light output, yielding a visually salient altitude cue for ground observers.

The apparent colour temperature may also be modulated; a simple linear model maps intensity to
a warm-to-cold hue shift:

$$H(t) = H_{warm} + (H_{cold} - H_{warm}) \cdot \ell(t)$$

where $H_{warm} = 30°$ (orange) and $H_{cold} = 200°$ (sky-blue) in HSV hue space.

### 3D Audience Viewing-Angle Score

Place $N_{aud}$ observer positions uniformly on a ground-level semi-circle of radius $R_{aud}$:

$$\mathbf{q}_j = \bigl(R_{aud}\cos\phi_j,\; R_{aud}\sin\phi_j,\; 0\bigr),
\qquad \phi_j = \frac{\pi j}{N_{aud}-1}, \quad j = 0, \ldots, N_{aud}-1$$

For each consecutive pair of waypoints $(\mathbf{p}_{ref,k}, \mathbf{p}_{ref,k+1})$, compute the
unit direction of drone motion $\hat{\mathbf{d}}_k$ and the unit line-of-sight from observer $j$
to the midpoint of the segment:

$$\hat{\mathbf{r}}_{jk} = \frac{(\mathbf{p}_{ref,k} + \mathbf{p}_{ref,k+1})/2 - \mathbf{q}_j}
  {\|(\mathbf{p}_{ref,k} + \mathbf{p}_{ref,k+1})/2 - \mathbf{q}_j\|}$$

The **apparent angular size** of the motion segment as seen by observer $j$ is the angle between
the segment direction and the LOS:

$$\Phi_{jk} = \arcsin\!\bigl(\|\hat{\mathbf{d}}_k \times \hat{\mathbf{r}}_{jk}\|\bigr)$$

The audience legibility score over the full path is:

$$\bar{\Phi}_{min} = \frac{1}{N_{wp}-1} \sum_{k=0}^{N_{wp}-2} \min_{j}\, \Phi_{jk}$$

A higher $\bar{\Phi}_{min}$ means that every motion segment appears visually distinct from even
the worst viewing position. The baseline flat figure-8 score can be compared against the 3D
lifted version and against a gradient-ascent optimised variant that adjusts $A_z$ and the
vertical phase offset to maximise $\bar{\Phi}_{min}$.

### Tracking Error Metrics

Let $\mathbf{e}_k = \mathbf{p}_{ref}(t_k) - \mathbf{p}(t_k)$:

$$\varepsilon_{RMS} = \sqrt{\frac{1}{N_{steps}} \sum_{k=1}^{N_{steps}} \|\mathbf{e}_k\|^2}, \qquad
\varepsilon_{max} = \max_k \|\mathbf{e}_k\|$$

$$\text{Path completion} = \frac{|\{k : \|\mathbf{e}_k\| \leq 0.5\}|}{N_{steps}} \times 100\%$$

---

## Key 3D Additions

- **Vertical arc path**: $z_{ref}(\tau) = z_c + A_z \sin(4\pi\tau)$ replaces the constant
  $z_{show}$ of S083; altitude now varies 40 – 60 m over each figure-8 lobe.
- **Non-zero z feedforward**: $s_z''(t) \ne 0$ is the key numerical difference — the vertical
  centripetal demand is cancelled directly rather than left entirely to the PID D-term.
- **LED brightness coupling**: $\ell(t) \propto p_z(t)$, producing altitude-legible brightness
  variation visible to ground observers.
- **Audience scoring**: $\bar{\Phi}_{min}$ quantifies choreography legibility from a 180° audience
  arc; used to compare flat vs. lifted vs. optimised path shapes.
- **Altitude bounds enforcement**: soft saturation at $z \in [35, 65]$ m prevents runaway vertical
  excursions under strong wind.
- **3D trajectory visualisation**: `Axes3D` with real z variation (not a flat ribbon); separate
  altitude time-series panel; LED intensity colour-coded along the 3D path.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Central show altitude | $z_c$ | 50 m |
| Altitude band (low / high) | $z_{low},\, z_{high}$ | 40 m / 60 m |
| Vertical semi-amplitude | $A_z$ | 10 m |
| Horizontal semi-amplitudes | $A_x = A_y$ | 20 m |
| Mission duration | $T_{total}$ | 60 s |
| Number of scripted waypoints | $N_{wp}$ | 200 |
| Simulation timestep | $\Delta t$ | 0.05 s (20 Hz) |
| Wind disturbance std dev (per axis) | $\sigma_w$ | 0.5 m/s |
| Proportional gain | $K_p$ | 2.0 |
| Integral gain | $K_i$ | 0.1 |
| Derivative gain | $K_d$ | 0.8 |
| RMS tracking spec (3D) | $\varepsilon_{RMS}^*$ | 0.15 m |
| Path completion threshold | — | 0.5 m |
| LED intensity range | $[\ell_{min}, \ell_{max}]$ | [0.3, 1.0] |
| LED hue range (HSV) | $[H_{warm}, H_{cold}]$ | 30° – 200° |
| Audience arc radius | $R_{aud}$ | 100 m |
| Number of observer positions | $N_{aud}$ | 36 |

---

## Expected Output

- **3D trajectory plot** (`Axes3D`): the full lifted figure-8 reference in green with real altitude
  variation; PID-only drone trajectory in red; PID+FF trajectory in blue; LED intensity
  colour-mapped along the reference path using a warm-to-cool colormap; legend, axis labels,
  $z$-axis range 35–65 m.
- **Altitude time series**: $p_z(t)$ for the reference (green), PID-only (red), and PID+FF (blue)
  over 60 s; horizontal dashed lines at $z_{low} = 40$ m and $z_{high} = 60$ m; displays the
  vertical tracking fidelity of the feedforward term.
- **LED brightness profile**: $\ell(t)$ for each variant, showing the brightness variation that
  the audience perceives; comparison between the reference profile and the distorted profile under
  wind.
- **Tracking error time series**: $\|\mathbf{e}(t)\|$ in centimetres for both variants; horizontal
  dashed line at the 3D RMS spec (15 cm).
- **Audience legibility score bar chart**: $\bar{\Phi}_{min}$ for (a) the flat 2D reference
  (baseline from S083), (b) the 3D lifted path, and (c) gradient-ascent optimised path; shows
  quantitative improvement in choreography legibility.
- **3D animation** (`FuncAnimation`, saved as GIF): drone dot moving along the 3D lifted
  figure-8; dot colour coded by LED intensity; current altitude and brightness displayed as a
  heads-up text overlay; 20 fps playback.

---

## Extensions

1. **Optimal vertical amplitude sweep**: vary $A_z \in [0, 20]$ m and evaluate the joint
   objective $J = w_1 \cdot \bar{\Phi}_{min} - w_2 \cdot \varepsilon_{RMS}$; find the Pareto
   front between choreography legibility and tracking difficulty.
2. **Vertical phase offset optimisation**: allow the z-sinusoid to carry an independent phase
   $\phi_z$ — $z_{ref}(\tau) = z_c + A_z \sin(4\pi\tau + \phi_z)$ — and use gradient ascent on
   $\bar{\Phi}_{min}$ to find the optimal phase for a given audience geometry.
3. **Dryden turbulence model**: replace white Gaussian wind with a coloured-noise Dryden spectrum
   ($L_w = 50$ m, $\sigma_w = 1.5$ m/s at 50 m altitude); assess whether the z-axis feedforward
   advantage erodes when the disturbance bandwidth overlaps the reference signal bandwidth.
4. **Rigid-body attitude dynamics**: replace the point-mass model with a quadrotor roll/pitch/yaw
   model; the vertical arcs generate pitch/roll commands that introduce cross-axis coupling absent
   in the point-mass case.
5. **Multi-drone 3D formation extension**: scale to $K = 5$ drones flying phase-shifted copies of
   the 3D lifted figure-8; compute formation-keeping error and LED synchronisation quality under
   correlated wind fields (see S085).
6. **Audience-driven path re-planning**: given a measured audience distribution (not a uniform
   arc), solve the constrained optimisation for waypoint positions that maximise the
   area-weighted $\bar{\Phi}_{min}$ while keeping $\varepsilon_{RMS} \leq 0.15$ m.

---

## Related Scenarios

- Original 2D version: [S083](../S083_light_show_single.md)
- Multi-drone follow-up: [S085 Light Matrix Formation](../S085_light_matrix.md)
- Formation morphing: [S088 Formation Morphing](../S088_formation_morphing.md)
- Algorithmic cross-reference: [S063 Precision Spraying](../../04_industrial_agriculture/S063_precision_spraying.md) (PID + feedforward under wind), [S048 Lawnmower Coverage](../../03_environmental_sar/S048_lawnmower.md) (pre-scripted path following)
