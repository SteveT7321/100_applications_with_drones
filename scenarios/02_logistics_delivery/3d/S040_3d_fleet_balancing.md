# S040 3D Upgrade — Fleet Load Balancing

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S040 original](../S040_fleet_load_balancing.md)

---

## What Changes in 3D

The original S040 fixes every drone and every delivery location at `z = 2.0 m`. All distances in
the load index, the battery feasibility check, and the queue-swap criterion are computed from
2D Euclidean norms; the DEPOT is a flat point at $(300, 300, 2)$ m. In a realistic urban
deployment drones operate across a vertical airspace that is partitioned into discrete **altitude
zones** (low / mid / high) tied to delivery density and obstacle clearance. This variant
introduces three structural changes:

1. **Altitude-zone task assignment** — each incoming order carries a mandatory delivery altitude
   $z_o \in \{5, 15, 25\}$ m (ground approach, building mid-floor, rooftop), and the dispatcher
   uses a 3D arc cost rather than a horizontal distance when selecting the best drone.
2. **3D energy model** — the battery discharge per leg now includes a gravitational potential
   term for altitude change; climbing costs extra energy while descending is partial-recovery.
3. **Vertical transition cost in reassignment** — the queue-swap criterion adds the altitude
   mismatch penalty between the swapped order's delivery altitude and the receiving drone's
   current operating zone, preventing energetically wasteful cross-zone transfers.

---

## Problem Definition

**Setup**: A fleet of $N = 8$ delivery drones departs from a single base depot at
$\mathbf{p}_{depot} = (300, 300, 0)$ m and operates over a $600 \times 600 \times 30$ m urban
volume. A stream of $M = 40$ delivery orders arrives dynamically over a horizon $T_{sim} = 400$ s.
Each order specifies a horizontal delivery location $(x_o, y_o)$ and a **delivery altitude**
$z_o$ drawn from three urban tiers:

| Tier | Altitude $z_o$ | Typical context |
|------|----------------|-----------------|
| Low | 5 m | Ground-floor handoff, courtyard |
| Mid | 15 m | Building mid-floor, balcony |
| High | 25 m | Rooftop, penthouse |

Each drone maintains a normalised state-of-charge $s_i \in [0,1]$, a pending queue of orders
$w_i$, and a cumulative 3D flight distance $f_i^{3D}$. A central load-balancer assigns new orders
and may swap queued orders between drones using a 3D-aware load index that accounts for the
altitude transition cost of each candidate assignment.

**Roles**:
- **Drones** ($N = 8$): execute a private FIFO queue; fly 3D straight-line legs between their
  current position and the next waypoint's full $(x, y, z)$ coordinates; return to the depot
  (descending to $z = 0$ m) when battery falls below the 3D return-feasibility threshold.
- **Load-balancer**: scores each drone with a 3D composite load index and selects the
  battery-feasible drone with minimum post-assignment index at every dispatch event.
- **Depot**: single base at $(300, 300, 0)$ m; offers 2 recharging pads.

**Objective**: Minimise the peak-to-mean load imbalance $\Lambda(t)$ across all dispatch epochs
(identical objective form as S040), but with the load index and battery feasibility now computed
on 3D arc costs. Secondary objective: maximise total completed deliveries within $T_{sim}$.

**Comparison strategies**:
1. **Round-Robin** — cyclic assignment ignoring 3D geometry
2. **Least-Queue** — assign to the drone with fewest pending orders, no 3D awareness
3. **3D Load-Index Balancer** (proposed) — composite load index using 3D distances, SoC, and
   altitude transition penalty
4. **Zone-Affinity Dispatcher** (extension) — pre-assign each drone to a preferred altitude
   zone; cross-zone orders trigger higher marginal load scores

---

## Mathematical Model

### 3D Arc Distance

For a leg from position $\mathbf{p}_a = (x_a, y_a, z_a)$ to $\mathbf{p}_b = (x_b, y_b, z_b)$:

$$d_{ab}^{3D} = \|\mathbf{p}_b - \mathbf{p}_a\|_2
  = \sqrt{(x_b - x_a)^2 + (y_b - y_a)^2 + (z_b - z_a)^2}$$

### 3D Energy Model

Battery discharge flying a 3D leg of length $d^{3D}$ with altitude change $\Delta z = z_b - z_a$:

$$\Delta s = \frac{\alpha \cdot (m_{body} + m_p) \cdot g \cdot d^{3D}}{E_{cap}}
             + \frac{\gamma \cdot (m_{body} + m_p) \cdot g \cdot \max(\Delta z,\, 0)}{E_{cap}}$$

The first term is the horizontal-cruise discharge (same structure as S040). The second term
($\gamma \geq 0$, dimensionless climb penalty factor) adds extra energy consumption for altitude
gain. Descents ($\Delta z < 0$) provide partial regeneration at rate $\eta_{regen}$:

$$\Delta s_{regen} = \frac{\eta_{regen} \cdot (m_{body} + m_p) \cdot g \cdot |\Delta z|}{E_{cap}}
  \quad \text{if } \Delta z < 0$$

Net discharge for one leg:

$$\Delta s_{net} = \Delta s - \Delta s_{regen}$$

### 3D Drone Load Index

$$L_i(t) = w_i(t) \cdot \phi_w
           + \frac{f_i^{3D}(t)}{F_{norm}^{3D}} \cdot \phi_f
           + \bigl(1 - s_i(t)\bigr) \cdot \phi_s$$

where $f_i^{3D}(t)$ is the cumulative 3D path length flown by drone $i$, and
$F_{norm}^{3D}$ normalises it against the maximum expected 3D flight distance per drone.

### Fleet Imbalance Metric (unchanged form)

$$\Lambda(t) = \frac{L_{max}(t) - L_{min}(t)}{\bar{L}(t)}, \qquad
  \bar{L}(t) = \frac{1}{N} \sum_{i=1}^{N} L_i(t)$$

### 3D Order Assignment Rule

When order $o = (x_o, y_o, z_o)$ arrives at $t_o$, assign it to drone $k^*$:

$$k^* = \arg\min_{i \in \mathcal{A}^{3D}(t_o)} \bigl[ L_i(t_o) + \Delta L_i^{3D}(o) \bigr]$$

The 3D marginal load increment is:

$$\Delta L_i^{3D}(o) = \phi_w \cdot 1
  + \phi_f \cdot \frac{d^{3D}(\mathbf{p}_{last,i},\, \mathbf{q}_o^{3D})}{F_{norm}^{3D}}
  + \phi_z \cdot \frac{|z_o - z_i^{zone}|}{z_{max}}$$

where:
- $\mathbf{q}_o^{3D} = (x_o, y_o, z_o)$ — full 3D delivery waypoint
- $\mathbf{p}_{last,i}$ — last 3D waypoint in drone $i$'s queue (or current position if idle)
- $z_i^{zone}$ — altitude of drone $i$'s most recent delivery (proxy for its current operating zone)
- $z_{max} = 25$ m — maximum tier altitude
- $\phi_z \geq 0$ — altitude-mismatch weight; penalises cross-zone transfers

### 3D Battery Feasibility for Assignment

The feasibility set $\mathcal{A}^{3D}(t_o)$ restricts assignment to drones able to complete their
current queue, fly to order $o$, and return to depot with SoC above $\beta_{safe}$:

$$s_i(t_o) - \frac{\alpha (m_{body} + m_p) g \bigl(L_i^{dist,3D}(t_o) + d_{extra}^{3D}(i,o) + d_{home}^{3D}(o)\bigr)}{E_{cap}}
  - \frac{\gamma (m_{body} + m_p) g \cdot \Delta z_{climb}^{total}}{E_{cap}} \geq \beta_{safe}$$

where $L_i^{dist,3D}$ is the remaining 3D queue distance, $d_{extra}^{3D}$ is the 3D distance
added by appending order $o$ to drone $i$'s queue, $d_{home}^{3D}(o)$ is the 3D distance from
order $o$'s location to the depot, and $\Delta z_{climb}^{total}$ is the total altitude gain
accumulated over the projected route.

### Vertical Transition Cost in Queue-Swap Rebalancing

At rebalancing epochs the load-balancer may transfer a pending order from the most-loaded
drone $i^+$ to the least-loaded feasible drone $i^-$. The swap is accepted only if both the
standard imbalance-reduction condition and the vertical transition feasibility hold:

$$\Lambda\bigl(\text{after swap}\bigr) < \Lambda\bigl(\text{before swap}\bigr)$$

$$\text{and} \quad \Delta s_{net}^{extra}(i^-, o_{swap}) \leq s_{i^-}(t) - \beta_{safe}$$

where $\Delta s_{net}^{extra}$ now uses the 3D energy model including the altitude climb from
$z_{i^-}^{zone}$ to $z_{o_{swap}}$. An additional altitude-mismatch veto is applied:

$$|z_{o_{swap}} - z_{i^-}^{zone}| \leq \Delta z_{veto}$$

Orders requiring a cross-zone climb that exceeds $\Delta z_{veto}$ are not transferred even if
the imbalance reduction condition is satisfied, to prevent energetically wasteful swaps that
would deplete the receiving drone's battery disproportionately.

### 3D Return-Feasibility Threshold (Dynamic)

$$s_{min}^{3D}(i) = \frac{\alpha (m_{body} + m_p) g \cdot d^{3D}(\mathbf{p}_i, \mathbf{p}_{depot})}{E_{cap}}
  + \frac{\gamma (m_{body} + m_p) g \cdot \max(z_{depot} - z_i,\, 0)}{E_{cap}} + \beta_{safe}$$

### Jain's Fairness Index (unchanged)

$$\mathcal{J} = \frac{\bigl(\sum_{i=1}^{N} D_i\bigr)^2}{N \cdot \sum_{i=1}^{N} D_i^2}$$

---

## Key 3D Additions

- **Altitude-zone task assignment**: each order specifies $z_o \in \{5, 15, 25\}$ m; assignment
  scores include a $\phi_z$-weighted altitude-mismatch penalty
- **3D energy model**: discharge computed on full 3D arc length with an additional climb penalty
  $\gamma$ and partial descent regeneration at rate $\eta_{regen}$
- **Vertical transition cost in reassignment**: cross-zone queue swaps are vetoed when the
  altitude gap exceeds $\Delta z_{veto}$ or violates 3D battery feasibility
- **3D marginal load increment**: extends $\Delta L_i(o)$ to use 3D Euclidean distance and zone
  mismatch term $\phi_z \cdot |z_o - z_i^{zone}| / z_{max}$
- **3D return-feasibility**: dynamic $s_{min}^{3D}$ accounts for descent energy toward depot
- **3D trajectory visualisation**: full $(x, y, z)$ paths with altitude time-series per drone

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Fleet size $N$ | 8 drones |
| Charging pads | 2 |
| Total orders $M$ | 40 |
| Simulation horizon $T_{sim}$ | 400 s |
| Operational volume | $600 \times 600 \times 30$ m |
| Delivery altitudes $z_o$ | 5 m / 15 m / 25 m (equal probability) |
| Depot position $\mathbf{p}_{depot}$ | $(300, 300, 0)$ m |
| Cruise speed $v$ | 8.0 m/s (3D vector magnitude) |
| Discharge coefficient $\alpha$ | $1.5 \times 10^{-4}$ m$^{-1}$ |
| Climb penalty factor $\gamma$ | 0.5 (extra fraction of cruise energy per m climbed) |
| Descent regeneration $\eta_{regen}$ | 0.2 (20 % of gravitational potential recovered) |
| Body mass $m_{body}$ | 1.5 kg |
| Payload mass $m_p$ | 0.3 kg |
| Full-battery 3D range $D_{range}^{3D}$ | ~3560 m (horizontal; shorter with altitude) |
| Charging rate $P_{chg}$ | 0.20 $E_{cap}$/s |
| Safety SoC reserve $\beta_{safe}$ | 0.05 |
| Load-index weights $(\phi_w, \phi_f, \phi_s, \phi_z)$ | (0.4, 0.25, 0.2, 0.15) |
| Altitude-mismatch veto $\Delta z_{veto}$ | 15 m |
| Normalisation $F_{norm}^{3D}$ | 1800 m (3D path) |
| Rebalancing period $\Delta t_r$ | 20 s |
| Max queue swaps per epoch $S_{max}$ | 2 |
| Simulation timestep $\Delta t$ | 0.2 s |

---

## Expected Output

- **3D trajectory plot**: all 8 drone paths in a 3D axes coloured by drone index; delivery
  waypoints marked as spheres at their true $(x, y, z)$ altitude; depot shown at $(300, 300, 0)$ m
- **Altitude time-series**: $z_i(t)$ for each drone — reveals zone-affinity clustering and the
  dips/climbs incurred by cross-zone orders
- **Imbalance time series**: $\Lambda(t)$ vs time for Round-Robin, Least-Queue, and 3D
  Load-Index Balancer; shaded bands indicate rebalancing epochs
- **Jain's fairness bar chart**: $\mathcal{J}$ per strategy, comparing 2D baseline values from
  S040 against the 3D-aware dispatcher
- **Energy breakdown bar chart**: per-drone total energy split into horizontal cruise, climb
  penalty, and descent regeneration components
- **3D vs 2D cost comparison**: side-by-side bars showing how ignoring altitude increases
  imbalance $\bar{\Lambda}$ and reduces throughput $D_{total}$
- **Queue-swap veto log**: count of altitude-veto rejections vs imbalance-accepted swaps per
  rebalancing epoch — shows how often cross-zone transfers are blocked
- **Load index heatmap**: $L_i(t)$ colour grid (drones × time) for the 3D balancer vs Round-Robin

---

## Extensions

1. **Zone-affinity dispatcher**: pre-assign each pair of drones to a preferred altitude tier;
   cross-zone orders incur a fixed latency penalty; compare zone-locked vs zone-free dispatching
   on imbalance and total energy
2. **Altitude-adaptive weights**: tune $\phi_z$ online using a running estimate of cross-zone
   order frequency; if high-tier orders dominate, reduce $\phi_z$ to avoid starving high-altitude
   delivery capacity
3. **Glide-slope energy model**: replace instantaneous altitude changes at waypoints with a
   continuous climb/descent profile; add the horizontal distance consumed by the glide slope to
   the 3D arc length in the battery feasibility check
4. **Wind-layer interaction**: assign a horizontal wind vector $\mathbf{w}(z)$ that varies with
   altitude; low-zone drones face less wind drag while high-zone drones benefit from tailwind;
   extend the energy model to include $\pm \mathbf{w}(z) \cdot \hat{\mathbf{v}}_{drone}$ terms
5. **RL dispatcher with altitude state**: extend the PPO state vector from S040 to include
   $z_i^{zone}$ for each drone and $z_o$ for the pending order; reward includes a term
   penalising cross-zone energy waste as well as imbalance

---

## Related Scenarios

- Original 2D version: [S040](../S040_fleet_load_balancing.md)
- Other 3D logistics upgrades: [S030 3D](S030_3d_multi_depot.md), [S029 3D](S029_3d_urban_logistics.md)
- Algorithmic cross-reference: [S019](../../01_pursuit_evasion/S019_dynamic_reassignment.md)
  (real-time Hungarian reassignment), [S032](../S032_charging_queue.md) (charging pad contention),
  [S033](../S033_online_order_insertion.md) (online order insertion)
