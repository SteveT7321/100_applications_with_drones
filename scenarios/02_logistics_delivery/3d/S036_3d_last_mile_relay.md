# S036 3D Upgrade — Last-Mile Relay

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S036 original](../S036_last_mile_relay.md)

---

## What Changes in 3D

The original S036 places all relay stations and waypoints at a fixed 2D plane (`z = 0` implied throughout; all positions are `np.array([x, y])` 2-component vectors). The leg-distance metric is the 2D Euclidean norm, handoff positions carry no altitude information, and terrain elevation is entirely ignored.

In the 3D upgrade:

- **Relay stations have individual altitudes** $z_k$ that reflect real terrain elevation plus a fixed clearance, so each station sits at a physically meaningful height above the ground rather than a flat datum.
- **Legs are 3D geodesics**: leg distance uses the full 3D Euclidean norm $\|\mathbf{w}_k - \mathbf{w}_{k-1}\|_3$, which is always $\geq$ the 2D ground distance; the range constraint and battery model must account for the additional energy cost of climbing.
- **Terrain-following between relay stations**: each drone descends or climbs along the leg to follow a digital elevation model (DEM) profile, maintaining a constant clearance $h_{clr}$ above terrain rather than a flat cruise altitude.
- **Altitude-varying handoff**: the incoming drone and outgoing drone must rendezvous at the same 3D point; a vertical alignment phase is required if the two drones arrive from different altitudes.
- **3D bottleneck minimisation**: the optimisation objective is extended to 3D leg lengths, so relay station altitudes become free variables alongside their horizontal positions.

---

## Problem Definition

**Setup**: A single parcel must travel from source depot $D_0 = (x_0, y_0, z_0)$ to destination $D_K = (x_K, y_K, z_K)$ over terrain with elevation profile $h_{terrain}(x, y)$. The straight-line 3D distance $L_3 = \|D_K - D_0\|$ exceeds the effective range $R_{eff}$ of any single drone. A chain of $K-1$ relay stations $\{W_1, \ldots, W_{K-1}\}$ is placed along the route; each station $W_k = (x_k, y_k, z_k)$ sits at terrain elevation plus clearance:

$$z_k = h_{terrain}(x_k, y_k) + h_{clr}$$

Each drone carries the parcel for one leg, navigates a terrain-following altitude profile, arrives at the relay station, hands off the parcel, and waits for recharging. A fresh drone departs from the relay station for the next leg.

**Roles**:
- **Relay drones** ($N \geq K$ units): each executes one 3D leg with terrain-following altitude guidance, then recharges at the relay station.
- **Parcel**: single cargo unit handed off at each relay station in 3D space.
- **Relay stations** ($K-1$ intermediate + 2 terminals): 3D rendezvous points at terrain-following altitude.

**Objective**:

1. **Feasibility**: every 3D leg distance $d_k^{3D} = \|\mathbf{w}_k - \mathbf{w}_{k-1}\|_3$ satisfies the range constraint with altitude energy penalty:

$$d_k^{3D} \leq R_{eff}, \qquad R_{eff} = R \cdot (1 - B_{reserve}) \cdot \eta_{alt}(z_k - z_{k-1})$$

2. **3D Bottleneck Minimisation**: place relay stations (horizontal position and altitude) to minimise the worst-case 3D leg length:

$$\min_{\{\mathbf{w}_k\}} \max_{k=1}^{K} \|\mathbf{w}_k - \mathbf{w}_{k-1}\|_3$$

3. **Timing**: minimise end-to-end delivery time including terrain-following detour cost and handoff dwell time $\tau_h$.

**Comparison**: flat 2D relay (equal horizontal spacing, fixed altitude) vs 3D terrain-following relay; compare bottleneck leg length, total 3D path length, battery usage, and delivery time.

---

## Mathematical Model

### 3D Leg Distance and Range Constraint

Let relay station waypoints be $\mathbf{w}_k = (x_k, y_k, z_k)^T \in \mathbb{R}^3$.

3D leg distance:

$$d_k^{3D} = \|\mathbf{w}_k - \mathbf{w}_{k-1}\|_3 = \sqrt{(x_k - x_{k-1})^2 + (y_k - y_{k-1})^2 + (z_k - z_{k-1})^2}$$

Altitude energy penalty factor (climbing costs extra power; descending is free beyond glide):

$$\eta_{alt}(\Delta z) = 1 + \alpha_{climb} \cdot \max(0,\, \Delta z) / d_k^{3D}$$

where $\alpha_{climb} \approx 0.3$ is the climb energy surcharge coefficient. The altitude-adjusted effective range per leg:

$$R_{eff,k} = \frac{R \cdot (1 - B_{reserve})}{\eta_{alt}(\Delta z_k)}$$

### Terrain-Following Altitude Profile

Given a terrain elevation function $h_{terrain}(x, y)$ sampled at resolution $\delta_{DEM}$, the commanded altitude at horizontal position $(x, y)$ along the leg is:

$$z_{cmd}(x, y) = h_{terrain}(x, y) + h_{clr}$$

The drone follows a piecewise linear altitude trajectory between waypoints while also staying above the terrain-clearance floor at every intermediate point along the leg.

For a leg from $\mathbf{w}_{k-1}$ to $\mathbf{w}_k$, parameterised by $s \in [0, 1]$:

$$\mathbf{p}(s) = (1 - s)\,\mathbf{w}_{k-1} + s\,\mathbf{w}_k$$

The terrain-safe altitude at parameter $s$:

$$z_{safe}(s) = h_{terrain}(p_x(s),\, p_y(s)) + h_{clr}$$

Actual commanded altitude:

$$z_{fly}(s) = \max\!\bigl(z_{safe}(s),\; (1-s)\,z_{k-1} + s\,z_k\bigr)$$

### 3D Optimal Relay Station Placement

For a collinear horizontal route of ground length $L_{2D}$ with terrain elevation change $\Delta H$, the optimal equal-spacing solution in 3D gives each leg a ground component $L_{2D}/K$ and an altitude component $\Delta H / K$:

$$d_k^{3D,*} = \sqrt{\left(\frac{L_{2D}}{K}\right)^2 + \left(\frac{\Delta H}{K}\right)^2}$$

Minimum number of legs to cover the 3D route:

$$K^* = \left\lceil \frac{\max_k d_k^{3D}}{R_{eff}} \right\rceil$$

For general terrain-following routes, relay station altitudes $\{z_k\}$ are jointly optimised with horizontal positions $\{(x_k, y_k)\}$ via the terrain constraint:

$$z_k = h_{terrain}(x_k, y_k) + h_{clr} \quad \forall\, k$$

which makes altitude a dependent variable once horizontal positions are fixed.

### 3D Greedy Segmentation

Extended greedy segmentation over the ordered 3D via-point sequence $\{\mathbf{v}_i\} \subset \mathbb{R}^3$:

```
legs = []
start_idx = 0
while start_idx < len(waypoints_3d) - 1:
    reachable = [
        i for i in range(start_idx + 1, len(waypoints_3d))
        if norm3(waypoints_3d[i] - waypoints_3d[start_idx]) <= R_eff_k
    ]
    if not reachable:
        raise InfeasibleRouteError
    end_idx = reachable[-1]
    legs.append((start_idx, end_idx))
    start_idx = end_idx
```

where `norm3` is the 3D Euclidean norm and `R_eff_k` accounts for the altitude energy penalty for that candidate leg.

### Battery State with Altitude Penalty

Normalised battery consumption for leg $k$:

$$B_k = \frac{d_k^{3D}}{R} \cdot \eta_{alt}(\Delta z_k) \in (0, 1]$$

Reserve constraint:

$$B_k \leq 1 - B_{reserve} = 0.90$$

### 3D Altitude-Varying Handoff

At relay station $W_k = (x_k, y_k, z_k)$, the incoming drone (leg $k$) and outgoing drone (leg $k+1$) must both be at the same 3D position. The outgoing drone pre-positions by climbing or descending to $z_k$ while approaching horizontally. Vertical alignment phase completes when:

$$|z_{out} - z_k| < \delta_z = 0.5 \text{ m}$$

Vertical alignment control law:

$$\dot{z}_{out} = K_{z} \cdot (z_k - z_{out}), \quad |\dot{z}_{out}| \leq v_{z,max}$$

Full 3D handoff condition (both drones within capture radius in 3D):

$$\|\mathbf{p}_{in} - \mathbf{p}_{out}\|_3 < r_{handoff} = 0.5 \text{ m}$$

### Delivery Time Model in 3D

Flight time for leg $k$ at cruise speed $v_c$ (measured along the 3D terrain-following path):

$$t_k^{flight} = \frac{d_k^{3D,path}}{v_c}$$

where $d_k^{3D,path} \geq d_k^{3D}$ accounts for the terrain detour. Total delivery time:

$$T_{delivery} = \sum_{k=1}^{K} t_k^{flight} + (K - 1) \cdot \tau_h$$

### Terrain-Following Path Length

The actual flown path length for leg $k$ sampled at $N_s$ steps along the horizontal segment:

$$d_k^{3D,path} = \sum_{j=1}^{N_s} \left\|\mathbf{p}_{fly}(s_j) - \mathbf{p}_{fly}(s_{j-1})\right\|_3$$

where $\mathbf{p}_{fly}(s) = (p_x(s),\, p_y(s),\, z_{fly}(s))$.

---

## Key 3D Additions

- **3D relay station positions**: each station carries an altitude $z_k = h_{terrain}(x_k, y_k) + h_{clr}$ derived from a synthetic terrain DEM.
- **Terrain-following altitude profile**: commanded altitude along each leg is the maximum of the straight-line interpolation and the terrain-clearance floor, sampled at DEM resolution.
- **Altitude energy penalty**: range budget $R_{eff,k}$ is reduced when a leg climbs; the climb surcharge coefficient $\alpha_{climb}$ modulates the effective range shortfall.
- **3D greedy segmentation**: the furthest-reachable segmentation uses the full 3D norm with per-leg altitude-adjusted range, not the 2D ground distance.
- **3D handoff alignment**: outgoing drone must match altitude before the handoff capture condition is declared; a proportional vertical controller closes the altitude gap during the pre-positioning phase.
- **3D trajectory visualisation**: full 3D plot with terrain mesh surface, relay station markers at their true altitudes, and per-leg altitude profiles.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Source position $D_0$ | (0, 0, 2) m |
| Destination position $D_K$ | (120, 0, 8) m |
| Ground distance $L_{2D}$ | 120 m |
| Total elevation change $\Delta H$ | 6 m |
| Terrain peak height | 5 m (Gaussian hill at centre) |
| Terrain clearance $h_{clr}$ | 2.0 m |
| Drone max range $R$ | 35 m |
| Battery reserve margin $B_{reserve}$ | 10 % |
| Effective range (flat) $R_{eff}$ | 31.5 m |
| Climb energy surcharge $\alpha_{climb}$ | 0.30 |
| Cruise speed $v_c$ | 5.0 m/s |
| Max vertical speed $v_{z,max}$ | 2.0 m/s |
| Altitude alignment gain $K_z$ | 1.5 |
| Handoff capture radius $r_{handoff}$ | 0.5 m |
| Altitude match tolerance $\delta_z$ | 0.5 m |
| Handoff dwell time $\tau_h$ | 3.0 s |
| Minimum legs required $K^*$ | 4 (3D), vs 4 (2D flat) |
| Altitude bounds $z \in$ | [0.5, 20] m |
| DEM resolution $\delta_{DEM}$ | 1.0 m |
| Simulation timestep $\Delta t$ | 0.05 s |

---

## Expected Output

- **`trajectory_3d.png`**: 3D axes showing the terrain mesh surface (grey), relay stations as orange spheres at their true altitudes, per-leg flight paths in distinct colours; source (green cube) and destination (red star) annotated.
- **`altitude_profile.png`**: Altitude $z$ vs horizontal distance along the route for each leg; terrain-clearance floor shown as a grey filled area; relay station altitudes marked as horizontal tick marks.
- **`battery_usage.png`**: Bar chart of $B_k$ (altitude-adjusted) for each leg drone; horizontal dashed lines at the 0.90 reserve limit; comparison bars for flat-2D and 3D terrain-following strategies side by side.
- **`delivery_time_breakdown.png`**: Stacked bar chart showing 3D flight time per leg vs handoff dwell time; comparison of total delivery time between flat relay and terrain-following relay.
- **`handoff_detail.png`**: Per-handoff 3-panel plot of $\Delta x$, $\Delta y$, $\Delta z$ between incoming and outgoing drone during the alignment and capture phase; shows altitude gap closing before handoff trigger.
- **`animation.gif`**: Real-time 3D animation of the parcel progressing along the relay chain; terrain mesh visible; drone marker climbs and descends with terrain; handoff events highlighted with a flash at the relay station.

---

## Extensions

1. **Free relay station altitude**: relax the terrain-anchoring constraint and let $z_k$ be a free variable bounded by $[h_{terrain}(x_k, y_k) + h_{clr},\; z_{max}]$; minimise total 3D path length subject to battery feasibility — some legs may trade a higher altitude for a shorter straight-line distance.
2. **Heterogeneous terrain types**: assign higher $h_{clr}$ over urban or forest zones and lower over open water; the effective range shortfall varies spatially.
3. **Wind-aware 3D legs**: add altitude-varying wind field $\mathbf{w}(z)$ (wind shear model); the drone selects the altitude layer that minimises headwind drag per leg.
4. **Parallel 3D chains**: two parcels dispatched simultaneously on adjacent terrain-following corridors sharing physical relay infrastructure; analyse collision-free altitude separation requirements.
5. **Relay station placement on ridge lines**: constrain relay station horizontal positions to ridge-line candidates (local terrain maxima) so that the station itself is easily serviceable; benchmark against unrestricted placement.

---

## Related Scenarios

- Original 2D version: [S036](../S036_last_mile_relay.md)
- Prerequisites: [S021 3D point delivery](S021_3d_point_delivery.md), [S024 3D wind compensation](S024_3d_wind_compensation.md)
- Follow-ups: [S037](../S037_reverse_logistics.md) (return path), [S040](../S040_fleet_load_balancing.md) (load balancing)
- Related 3D logistics: [S027 3D aerial refueling](S027_3d_aerial_refueling.md), [S030 3D multi-depot](S030_3d_multi_depot.md)
