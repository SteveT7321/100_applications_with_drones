# S032 3D Upgrade — Charging Queue

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S032 original](../S032_charging_queue.md)

---

## What Changes in 3D

The original S032 locks all drones and pads at a fixed altitude of z = 2.0 m. The base position is
`(0, 0, 2)` and every waypoint is generated as `[x, y, 2.0]`, so the simulation is entirely planar.
In a realistic multi-drone logistics hub, charging infrastructure is stacked vertically across
multiple levels (rooftop decks, mezzanine platforms, ground-level pads), approach and departure
corridors are separated by altitude band to prevent mid-air conflicts, and a drone's 3D return path
— including the descent or climb to the correct pad level — consumes additional energy that the
flat-world model ignores.

This variant introduces:
- **Multi-level charging pads**: $L = 3$ altitude levels ($z \in \{1.5, 3.0, 4.5\}$ m), each
  hosting $K_l = 1$ pad, replacing the single flat layer of $K = 2$ pads.
- **3D approach and departure lanes**: drones fly to an altitude holding point before descending
  vertically to dock, and climb back to cruise altitude before heading to a delivery waypoint.
- **Altitude-based priority queuing**: a drone's queue position is determined not just by arrival
  time but by its current altitude relative to the assigned pad level, rewarding drones that are
  already near the correct flight corridor.

---

## Problem Definition

**Setup**: A fleet of $N = 6$ delivery drones operates over a 3D city volume
$[-30, 30]^2 \times [1.0, 6.0]$ m. The base hub provides $L = 3$ stacked charging levels at
altitudes $z_l \in \{1.5, 3.0, 4.5\}$ m, each with one pad. Drones cruise at $z_{cruise} = 5.0$ m
during delivery and descend to the assigned pad level for charging. The simulation horizon is
$T_{sim} = 300$ s.

**Roles**:
- **Drones** ($\times 6$): execute deliveries at 3D waypoints, monitor SoC, return to base when
  the return-flight feasibility check triggers, and request a pad assignment.
- **Charging pads** ($\times 3$, one per level): each pad serves one drone at a time; charge time
  depends on SoC deficit on arrival.
- **Pad dispatcher**: maintains one FIFO queue per level and assigns an incoming drone to the level
  whose queue minimises the drone's expected total wait — accounting for 3D flight time to reach
  that level.
- **Mission dispatcher**: maintains a pool of pending 3D deliveries and assigns idle drones.

**Objective**: maximise $D_{total}$ over $T_{sim}$ by optimising the return-to-charge threshold
$\beta^*$ under the extended 3D energy model that includes vertical flight costs. Compare:

1. **Flat-world threshold policy** — ignores vertical energy; returns when SoC $\leq \beta$
2. **3D-aware threshold policy** — includes vertical leg energy in $s_{min}^{3D}(i)$
3. **Altitude-priority queue** — assigns pad level based on minimum 3D travel cost, not pure FIFO
4. **Predictive 3D policy** — returns early only when predicted SoC on full 3D return path
   (horizontal + vertical) falls below $\beta_{safe}$

---

## Mathematical Model

### 3D Battery Discharge Model

Power draw depends on both horizontal airspeed $v_h$ and vertical rate $v_z$:

$$P_i = \alpha \left(m_{body} + m_p\right) g \cdot v_h + \gamma \left(m_{body} + m_p\right) g \cdot |v_z|$$

where $\gamma > \alpha$ captures the additional cost of climbing ($v_z > 0$; descent at $v_z < 0$
is cheaper but non-zero due to induced-drag changes). For a 3D flight leg from
$\mathbf{p}_a$ to $\mathbf{p}_b$:

$$\Delta E_{a \to b} = \alpha \left(m_{body} + m_p\right) g \cdot d_{xy} + \gamma \left(m_{body} + m_p\right) g \cdot |z_b - z_a|$$

where $d_{xy} = \sqrt{(x_b - x_a)^2 + (y_b - y_a)^2}$ is the horizontal distance component.

SoC update:

$$s' = s - \frac{\Delta E_{a \to b}}{E_{cap}}$$

### 3D Return-Flight Feasibility

Let $\mathbf{p}_i = (x_i, y_i, z_i)$ be the drone's current 3D position and $\mathbf{p}_{base} =
(0, 0, z_{cruise})$ its initial cruise re-entry point. The return path has two legs:

1. **Horizontal + altitude transit** to the base overhead point at cruise altitude
2. **Vertical descent** to the assigned pad level $z_l$

Minimum SoC required to complete the full 3D return to pad level $l$ with safety reserve:

$$s_{min}^{3D}(i, l) = \frac{\alpha (m_{body} + m_p) g \cdot d_{xy}(i, base)}{E_{cap}}
+ \frac{\gamma (m_{body} + m_p) g \cdot |z_i - z_{cruise}|}{E_{cap}}
+ \frac{\gamma (m_{body} + m_p) g \cdot |z_{cruise} - z_l|}{E_{cap}}
+ \beta_{safe}$$

Drone $i$ should return if:

$$s_i \leq \min_{l \in \{1,\ldots,L\}} s_{min}^{3D}(i, l) \quad \text{(predictive 3D policy)}$$

or simply $s_i \leq \beta$ (threshold policy).

### Approach and Departure Lane Protocol

Each drone follows a **two-segment** return profile:

1. **Cruise segment**: fly at $z_{cruise}$ from current position to the base overhead waypoint
   $\mathbf{p}_{hold} = (0, 0, z_{cruise})$.
2. **Descent segment**: descend vertically from $\mathbf{p}_{hold}$ to the assigned pad at
   $\mathbf{p}_{pad,l} = (0, 0, z_l)$.

Departure mirrors this in reverse:

1. **Ascent segment**: climb vertically from pad level $z_l$ to $z_{cruise}$.
2. **Cruise segment**: fly horizontally to the delivery waypoint at cruise altitude.

This lane separation ensures that ascending and descending drones occupy distinct altitude bands,
preventing horizontal conflicts near the hub.

### Altitude-Based Queue Assignment

When drone $i$ arrives at $\mathbf{p}_{hold}$ and requests a pad, the dispatcher computes the
**3D access cost** for each pad level $l$:

$$C(i, l) = t_{queue}(l) + \frac{|z_{cruise} - z_l|}{v_z^{max}}$$

where $t_{queue}(l)$ is the current expected waiting time at level $l$ and $v_z^{max}$ is the
maximum vertical speed. The drone is assigned to:

$$l^* = \arg\min_{l} C(i, l)$$

### Queue Waiting Time per Level

With one pad per level and $n_l$ drones ahead in the queue for level $l$:

$$t_{queue}(l) = n_l \cdot \bar{t}_{chg}$$

where $\bar{t}_{chg} = \mathbb{E}[(1 - s^{arrive})] / P_{chg}$ is the mean charge time given the
typical arrival SoC.

### Charging Dynamics

Unchanged from S032:

$$t_{chg}(i) = \frac{(1 - s_i^{arrive}) \cdot E_{cap}}{P_{chg}}$$

### Fleet Utilisation

$$\bar{u} = \frac{1}{N} \sum_{i=1}^{N} \frac{t_{fly,i}}{T_{sim}}$$

### Total Deliveries

$$D_{total}(\beta) = \sum_{i=1}^{N} D_i(\beta), \quad \beta^* = \arg\max_{\beta \in \mathcal{B}} D_{total}(\beta)$$

---

## Key 3D Additions

- **Multi-level pad layout**: three charging levels at $z \in \{1.5, 3.0, 4.5\}$ m replace the
  single flat pair of pads.
- **Vertical energy term**: climbing and descending now draw battery proportional to $|v_z|$
  scaled by $\gamma > \alpha$, making the feasibility check altitude-dependent.
- **Two-leg return path**: drones transit at cruise altitude, then descend vertically — lane
  separation prevents hub conflicts.
- **3D approach cost assignment**: pad-level selection minimises queue wait plus vertical transit
  time, not pure FIFO across a flat layer.
- **Altitude time series**: z-profiles for all drones reveal descent-to-pad and ascent-to-cruise
  patterns not visible in the 2D model.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Fleet size $N$ | 6 drones |
| Pad levels $L$ | 3 |
| Pad altitudes $z_l$ | 1.5, 3.0, 4.5 m |
| Cruise altitude $z_{cruise}$ | 5.0 m |
| Altitude bounds | [0.5, 6.0] m |
| Horizontal cruise speed $v_h$ | 8.0 m/s |
| Max vertical speed $v_z^{max}$ | 2.0 m/s |
| Battery capacity $E_{cap}$ | 1.0 (normalised) |
| Charging rate $P_{chg}$ | 0.25 $E_{cap}$/s |
| Horizontal discharge $\alpha$ | $1.8 \times 10^{-4}$ m$^{-1}$ |
| Vertical discharge $\gamma$ | $3.6 \times 10^{-4}$ m$^{-1}$ |
| Body mass $m_{body}$ | 1.5 kg |
| Payload mass $m_p$ | 0.5 kg |
| Safety reserve $\beta_{safe}$ | 0.05 |
| Delivery volume | $[-30, 30]^2 \times [1.0, 6.0]$ m |
| Simulation horizon $T_{sim}$ | 300 s |
| Threshold sweep $\mathcal{B}$ | 0.10 – 0.50 in steps of 0.05 |

---

## Expected Output

- **3D trajectory plot**: all drone paths including vertical descent/ascent legs at the hub, with
  pad levels shown as horizontal planes
- **Altitude time series**: $z_i(t)$ for all six drones, clearly showing cruise-to-pad descent
  and pad-to-cruise ascent transitions
- **Threshold sweep curve**: $D_{total}(\beta)$ vs $\beta$ for both flat-world and 3D-aware
  policies — quantifies the penalty of ignoring vertical energy
- **Policy comparison bar chart**: deliveries completed under flat-threshold, 3D-threshold,
  altitude-priority, and predictive-3D policies
- **Per-level queue length time series**: $n_l(t)$ for each of the three pad levels — reveals
  whether load is balanced across levels
- **SoC traces**: all six drones, with pad-level assignments annotated on charging events
- **Fleet utilisation $\bar{u}$** printed per policy

---

## Extensions

1. **Level-count sweep**: vary $L \in \{1, 2, 3, 4\}$ pad levels and plot $D_{total}(L)$ —
   find the point of diminishing returns versus hub structural cost
2. **Heterogeneous cruise altitudes**: assign different $z_{cruise}$ values per drone class
   (heavy-payload drones fly low, light drones fly high) and analyse level-affinity patterns
3. **Wind shear by altitude layer**: add horizontal wind that varies with $z$, coupling the
   vertical positioning decision to aerodynamic efficiency
4. **Dynamic pad scheduling**: allow a pad to shift altitude within $[1.0, 5.0]$ m to meet
   an incoming drone halfway, minimising total descent time
5. **RL pad-assignment agent**: replace the cost-minimisation heuristic with a PPO agent whose
   state is $(s_i, \mathbf{p}_i, n_1, n_2, n_3, t_{chg,1}, t_{chg,2}, t_{chg,3})$ and reward
   is deliveries per unit time

---

## Related Scenarios

- Original 2D version: [S032](../S032_charging_queue.md)
- 3D logistics references: [S021](../S021_point_delivery.md), [S027](../S027_aerial_refueling_relay.md)
- Follow-ups: [S033](../S033_online_order_insertion.md), [S040](../S040_fleet_load_balancing.md)
