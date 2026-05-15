# S047 3D Upgrade — Signal Relay Enhancement

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S047 original](../S047_signal_relay.md)

---

## What Changes in 3D

The original S047 optimises relay positions exclusively in the horizontal plane — each relay drone
has a fixed, implicit altitude and all link distances are computed as 2D Euclidean norms in $(x,
y)$. Terrain obstructions are modelled as axis-aligned shadow rectangles whose penalty is
triggered by a 2D midpoint test, entirely ignoring whether a drone at sufficient altitude could
clear a ridge by climbing above it. Three key limitations follow:

1. **Shadow penalty is all-or-nothing in 2D**: a relay positioned in the $(x, y)$ footprint of a
   ridge always pays the full $\Delta_{shadow} = 15$ dB penalty, even if flying at 80 m altitude
   would clear the ridge crest entirely.
2. **Link distance ignores vertical separation**: two relays separated by 400 m horizontally
   but at very different altitudes have a true 3D hop distance that is longer, which matters for
   the log-distance path-loss model.
3. **No altitude optimisation degree of freedom**: the energy-versus-SNR trade-off of climbing
   above terrain obstructions cannot be explored.

This 3D upgrade adds a full altitude dimension $z_i$ to each relay position, a physically
motivated terrain elevation model that gates the shadow penalty on whether the relay-to-relay
line-of-sight clears the ridge crest, and an energy penalty term that discourages unnecessarily
high hover altitudes. The result is a 9-variable optimisation ($(x, y, z)$ per relay) over a
non-convex objective that requires both horizontal repositioning and altitude-optimised hover
height selection.

---

## Problem Definition

**Setup**: A ground base station (BS) at $\mathbf{p}_0 = (100, 500, 1.5)$ m (mast tip) and a
rescue team at $\mathbf{p}_4 = (900, 500, 1.5)$ m are separated by a 800 m mountain corridor
containing three terrain ridges. Each ridge has a known crest elevation $h_{ridge,j}$ (m above
ground). Three relay drones may hover at any position $(x_i, y_i, z_i)$ within the permitted 3D
operating volume. A link between two chain nodes at 3D positions $\mathbf{p}_k$ and
$\mathbf{p}_{k+1}$ is terrain-shadowed if and only if the straight line-of-sight between them
passes below the crest of any intervening ridge.

**Roles**:
- **Base Station (BS)**: fixed 3D position $\mathbf{p}_0$; transmit power $P_{tx}$.
- **Relay Drone 1 (R1)**: free to hover at $(x_1, y_1, z_1)$; repeats signal from BS to R2.
- **Relay Drone 2 (R2)**: free to hover at $(x_2, y_2, z_2)$; repeats signal from R1 to R3.
- **Relay Drone 3 (R3)**: free to hover at $(x_3, y_3, z_3)$; repeats signal from R2 to the rescue team.
- **Rescue Team (T)**: fixed 3D endpoint $\mathbf{p}_4$.

**Objective**: Find relay positions $\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3 \in \mathbb{R}^3$
that maximise the minimum per-link SNR across the four-hop chain while penalising unnecessary
altitude above the terrain clearance minimum:

$$\max_{\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3} \left[
  \min_{k \in \{0,1,2,3\}} \text{SNR}_k
  \;-\; \lambda \sum_{i=1}^{3} \max(0,\; z_i - z_{clear,i})^2
\right]$$

subject to:
- $\|\mathbf{p}_{k+1} - \mathbf{p}_k\|_3 \leq d_{max}$ for each hop $k$
- $z_{min} \leq z_i \leq z_{max}$ for each relay $i$
- $(x_i, y_i)$ within the 2D operating bounding box $[0, 1000]^2$ m

**Comparison strategies**:
1. **2D uniform baseline** — relay altitudes fixed at 20 m, horizontal positions equally spaced.
2. **Altitude-only optimisation** — horizontal positions fixed at uniform spacing; optimise $z_i$
   only to minimise terrain shadowing.
3. **Full 3D gradient ascent** — joint $(x_i, y_i, z_i)$ optimisation of the bottleneck SNR
   minus the altitude penalty.

---

## Mathematical Model

### 3D Hop Distance

For consecutive chain nodes $\mathbf{p}_k = (x_k, y_k, z_k)$ and
$\mathbf{p}_{k+1} = (x_{k+1}, y_{k+1}, z_{k+1})$:

$$d_k = \|\mathbf{p}_{k+1} - \mathbf{p}_k\|_3
       = \sqrt{(x_{k+1}-x_k)^2 + (y_{k+1}-y_k)^2 + (z_{k+1}-z_k)^2}$$

### Log-Distance Path Loss (3D)

The path loss model is identical in form to S047 but uses the true 3D hop distance $d_k$:

$$L(d_k) = L_0 + 10\, n \log_{10}\!\left(\frac{d_k}{d_0}\right) \quad \text{(dB)}$$

$$L_0 = 20 \log_{10}\!\left(\frac{4\pi d_0 f_c}{c}\right)$$

where $n = 2.8$ (mixed terrain), $d_0 = 1$ m, $f_c = 2.4$ GHz.

### 3D Terrain Shadowing via Line-of-Sight Clearance

Each ridge $j$ is modelled as a Gaussian elevation profile:

$$h_{terrain}(x, y) = \sum_{j=1}^{N_s} H_j \exp\!\left(
  -\frac{(x - c_{x,j})^2}{2\sigma_{x,j}^2}
  -\frac{(y - c_{y,j})^2}{2\sigma_{y,j}^2}
\right)$$

where $H_j$ is the peak ridge height, $(c_{x,j}, c_{y,j})$ is the ridge centre, and
$(\sigma_{x,j}, \sigma_{y,j})$ controls ridge width.

For a link from $\mathbf{p}_k$ to $\mathbf{p}_{k+1}$, parameterise the LOS ray as:

$$\mathbf{r}(t) = (1-t)\,\mathbf{p}_k + t\,\mathbf{p}_{k+1}, \quad t \in [0, 1]$$

The LOS altitude at parameter $t$ is $z_{los}(t) = (1-t)z_k + t\, z_{k+1}$.
The terrain height at that point is $h_{terrain}(x(t), y(t))$.

The link is **terrain-shadowed** if the LOS dips below the terrain at any sampled point:

$$\text{shadowed}_k = \mathbf{1}\!\left[\min_{t \in \mathcal{T}} \bigl(z_{los}(t) - h_{terrain}(x(t), y(t))\bigr) < 0\right]$$

where $\mathcal{T} = \{0, \tfrac{1}{N_s-1}, \ldots, 1\}$ is a discrete set of $N_s = 50$ samples
along the ray.

The effective path loss with terrain penalty:

$$L_{eff,k} = L(d_k) + \Delta_{shadow} \cdot \text{shadowed}_k$$

### Per-Link SNR in 3D

$$\text{SNR}_k \;[\text{dB}] = P_{tx} - L_{eff,k} - N_{floor}$$

### Altitude-Optimised Hover Height

For relay $i$ at horizontal position $(x_i, y_i)$, the minimum clearance altitude — the
lowest $z_i$ that guarantees the two adjacent LOS segments are both terrain-free — is:

$$z_{clear,i} = \max\!\Bigl(
  \min_{t \in [0,1]} \text{required altitude to clear ridge on link } k=i-1,\;
  \min_{t \in [0,1]} \text{required altitude to clear ridge on link } k=i
\Bigr)$$

In practice this is computed by sweeping the terrain profile along each LOS segment and finding
the minimum $z_i$ such that the LOS never dips below terrain.

### Altitude Energy Penalty

Hovering at altitude $z_i$ above the clearance minimum $z_{clear,i}$ incurs unnecessary energy
cost. The penalty term added to the objective:

$$E_{penalty}(\mathbf{z}) = \lambda \sum_{i=1}^{3} \max\!\bigl(0,\; z_i - z_{clear,i}\bigr)^2$$

with penalty weight $\lambda = 0.05$ dB/m$^2$, biasing the optimiser toward the minimum
viable hover height rather than arbitrarily high altitudes.

### 3D Bottleneck SNR Objective

$$J_{3D}(\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3)
  = \min_{k=0}^{3} \text{SNR}_k(\mathbf{p}_k, \mathbf{p}_{k+1})
  - \lambda \sum_{i=1}^{3} \max(0,\; z_i - z_{clear,i})^2$$

### Smooth Approximation (Soft-Min) in 3D

The non-differentiable $\min$ is replaced by the soft-min to enable gradient-based optimisation:

$$J_{soft}^{3D} = -\frac{1}{\alpha} \ln\!\left(\sum_{k=0}^{3} e^{-\alpha\, \text{SNR}_k}\right)
                 - \lambda \sum_{i=1}^{3} \max(0,\; z_i - z_{clear,i})^2$$

with temperature $\alpha = 0.5$.

### Gradient of SNR with Respect to 3D Relay Position

For relay $i$ participating in links $k = i-1$ and $k = i$, the partial derivative of
$\text{SNR}_k$ with respect to the full 3D relay position vector $\mathbf{p}_i$:

$$\frac{\partial \text{SNR}_k}{\partial \mathbf{p}_i}
  = -\frac{10\, n}{\ln 10} \cdot \frac{\mathbf{p}_i - \mathbf{p}_{k'}}{d_k^2}$$

where $\mathbf{p}_{k'}$ is the opposite endpoint of link $k$ from relay $i$, and all three
components $(x, y, z)$ of the relay position are included in the gradient. This is the 3D
extension of the 2D gradient used in S047, with $d_k$ now the true 3D hop distance.

### Max-Comm-Range Constraint (3D)

$$d_k = \|\mathbf{p}_{k+1} - \mathbf{p}_k\|_3 \leq d_{max}, \quad k = 0, 1, 2, 3$$

Enforced by projecting over-range steps back to the sphere of radius $d_{max}$ centred on the
preceding node, operating on the full 3D displacement vector.

---

## Key 3D Additions

- **Altitude as a free variable**: each relay position is $(x_i, y_i, z_i) \in \mathbb{R}^3$;
  the optimiser can lift a relay above a ridge crest to eliminate the shadowing penalty on both
  adjacent links simultaneously.
- **Gaussian ridge terrain model**: replaces the 2D axis-aligned shadow rectangles of S047 with
  a smooth, physically motivated elevation surface $h_{terrain}(x,y)$, enabling gradient
  computation and continuous shadow-clearance evaluation.
- **LOS ray-terrain intersection test**: discretised ray sampling over 50 points per link
  replaces the 2D midpoint-in-rectangle test; the shadow penalty is triggered only when the 3D
  LOS actually passes below terrain.
- **Altitude clearance minimum $z_{clear,i}$**: computed per relay as the lowest altitude that
  places both adjacent LOS segments fully above terrain, giving a principled lower bound for
  hover height.
- **Altitude energy penalty**: $\lambda \sum_i \max(0, z_i - z_{clear,i})^2$ discourages
  over-climbing and drives the optimiser to the energy-efficient clearance altitude rather than
  maximum altitude.
- **3D hop distance in path-loss model**: link lengths computed as $\|\cdot\|_3$ rather than
  $\|\cdot\|_2$, so altitude separation between relay and BS/team contributes to path loss.
- **Altitude time-series output**: plot of $z_1(t), z_2(t), z_3(t)$ during gradient ascent,
  showing convergence to the ridge-clearance altitude from an arbitrary initial hover height.
- **3D scatter visualisation**: relay positions rendered in a 3D axes with the terrain surface
  and LOS links shown as 3D line segments, colour-coded by shadowed/clear status.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Carrier frequency $f_c$ | 2.4 GHz |
| Reference distance $d_0$ | 1 m |
| Path loss exponent $n$ | 2.8 (mixed terrain) |
| Transmit power $P_{tx}$ | 30 dBm |
| Noise floor $N_{floor}$ | -95 dBm |
| Minimum viable SNR | 10 dB |
| Max comm range $d_{max}$ | 700 m (3D; slightly larger to allow altitude clearance paths) |
| Terrain shadow penalty $\Delta_{shadow}$ | 15 dB |
| Number of relay drones | 3 |
| Ridge count $N_s$ | 3 |
| Ridge peak heights $H_j$ | 40 m, 55 m, 45 m |
| Ridge $\sigma$ (horizontal spread) | 80 m each axis |
| LOS ray sample count | 50 points |
| Altitude bounds $[z_{min}, z_{max}]$ | 5 m – 120 m |
| Soft-min temperature $\alpha$ | 0.5 |
| Altitude penalty weight $\lambda$ | 0.05 dB m$^{-2}$ |
| Gradient ascent step $\eta$ | 5 m / iteration |
| Gradient ascent iterations | 3000 |
| Operating 2D area | 1000 x 1000 m |
| Base station position | (100, 500, 1.5) m |
| Rescue team position | (900, 500, 1.5) m |
| Initial relay altitude (baseline) | 20 m |

---

## Expected Output

- **3D terrain and relay map**: 3D surface plot of $h_{terrain}(x,y)$ with the three relay
  positions for each strategy rendered as coloured spheres; LOS links drawn as 3D line segments
  coloured green (clear) or red (shadowed); BS and team shown as ground-level markers.
- **Top-down 2D projection**: horizontal relay positions overlaid on a filled-contour terrain
  height map, mirroring the S047 terrain-map plot but with ridge contours replacing rectangles.
- **Altitude convergence plot**: $z_1(t), z_2(t), z_3(t)$ vs gradient ascent iteration; shows
  each relay climbing from 20 m to its ridge-clearance altitude, then stabilising.
- **SNR chain bar chart (3 strategies)**: grouped bars for all four links under (1) 2D uniform
  baseline, (2) altitude-only optimisation, (3) full 3D optimisation; dashed line at
  $\text{SNR}_{min} = 10$ dB; bottleneck link highlighted.
- **Bottleneck SNR comparison bar chart**: single-number $\min_k \text{SNR}_k$ for each
  strategy, illustrating the gain from altitude freedom.
- **Optimisation convergence curve**: bottleneck SNR vs iteration for the 3D gradient ascent,
  compared against the 2D converged value as a horizontal reference line.
- **LOS clearance margin plot**: for the optimised solution, plot the terrain clearance margin
  $z_{los}(t) - h_{terrain}(x(t),y(t))$ along each of the four links, confirming all margins
  are non-negative.

---

## Extensions

1. **Continuous terrain DEM**: replace the Gaussian ridge model with a sampled Digital
   Elevation Model (DEM) loaded from GeoTIFF; requires bilinear interpolation for
   gradient-compatible terrain queries.
2. **Wind-affected altitude feasibility**: at higher altitudes wind speed increases; add an
   altitude-dependent drag model and cap hover altitude where wind exceeds drone endurance
   threshold, creating a feasible altitude band rather than a simple upper bound.
3. **Multi-frequency relay**: assign different frequency bands to adjacent hops to eliminate
   inter-hop interference; co-optimise carrier frequency assignment and 3D position jointly.
4. **Dynamic rescue team tracking in 3D**: the rescue team moves through a valley floor at
   variable altitude; implement a receding-horizon 3D replanner that adjusts relay altitudes
   and positions every 30 s to maintain chain integrity.
5. **Energy budget constraint**: each relay drone has a finite hover endurance; the altitude
   penalty becomes a hard constraint $z_i \leq z_{endurance,i}(E_i)$ where $E_i$ is the
   remaining battery; optimise relay replacement scheduling alongside position.
6. **Relay count vs altitude trade-off**: compare using 2 relays at high altitude vs 3 relays
   at low altitude; quantify the SNR-versus-drone-count Pareto frontier in 3D.

---

## Related Scenarios

- Original 2D version: [S047](../S047_signal_relay.md)
- 3D geometry reference: [S046 3D Trilateration](../S046_3d_trilateration.md)
- Network extension: [S051 Post-Disaster Comm Network](../S051_post_disaster_comm.md)
- Relay chain in a different medium: [S059 Sonar Buoy Relay](../S059_sonar_relay.md)
- Multi-agent positioning: [S049 Dynamic Zone Assignment](../S049_dynamic_zone_assignment.md)
