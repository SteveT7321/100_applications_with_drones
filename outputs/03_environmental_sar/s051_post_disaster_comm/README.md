# S051 Post-Disaster Communication Network Restoration

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: An earthquake has destroyed the cellular towers across a 2000 × 2000 m city. The area is divided into a 40 × 40 grid (50 m cells), and each cell has a known population. Five drones take off from a single charge station at (1000, 100) m and hover at 50 m as temporary airborne cell towers. Each drone serves every cell within a ground radius of 300 m.

**Constraints**:
1. **Connectivity**: the base and the five drones must form a connected radio graph (link range 600 m), so every drone can relay traffic back to the charge station.
2. **Battery range**: each drone must fly out, hover for 1500 s and fly back within an 1800 s battery budget at 15 m/s.

**Objective**: choose the five hover positions that maximise the total population served.

**Comparison Strategies**:
1. **Greedy (no connectivity)**: place each drone in turn at the cell centroid that adds the most uncovered population; connectivity is ignored.
2. **Greedy + Repair**: greedy placement, then move disconnected drones toward the connected network until every drone links back to base.
3. **Simulated Annealing (SA)**: optimise continuous positions from a random start, with connectivity and range violations as a hard penalty.

---

## Mathematical Model Summary

### Coverage objective

$$C\left(\{\mathbf{p}_{d_k}\}\right) = \sum_{u \in \mathcal{U}} \text{pop}_u \cdot \mathbf{1}\left[\min_{k} \|\mathbf{c}_u - \mathbf{p}_{d_k}\| \leq r_{cov}\right]$$

### Connectivity constraint

The graph $\mathcal{G}$ has node 0 (base) and nodes $1..K$ (drones), with an edge whenever two nodes are within $r_{comm}$. A breadth-first search from node 0 marks the drones that can relay back to base. All $K$ drones must be reachable (equivalently, $\lambda_2(\mathbf{L}) > 0$ for the graph Laplacian).

### Battery range

$$\frac{2\,\|\mathbf{p}_{d_k} - \mathbf{p}_{base}\|}{v} + T_{hover} \leq T_{max} \;\Rightarrow\; D_{max} = \frac{v\,(T_{max} - T_{hover})}{2} = 2250 \text{ m}$$

Every point in the area is within 2250 m of the base, so the range constraint never binds in this layout.

### Simulated annealing (normalised cost)

$$E(\mathbf{x}) = -\frac{C(\mathbf{x})}{C_{total}} + \alpha \cdot n_{bad}(\mathbf{x}), \qquad \alpha = 1$$

where $n_{bad}$ counts drones that are disconnected from base or out of range. One move displaces a random drone by $\mathcal{U}([-\delta, \delta]^2)$, with $\delta = \max(200 \cdot T/T_0,\, 10)$ m. A move is accepted with probability $\min(1, e^{-\Delta E / T})$, and the temperature cools geometrically from $T_0 = 0.02$ to $10^{-5}$ over 80 000 iterations.

**Changes from the scenario card**, made so the method does what the card intends:
- **Penalty weight.** The card's $\alpha = 10^6$ persons does not dominate: one drone over Downtown already covers about 1.2 M persons, so a disconnected placement could still win. The cost is normalised by total population, and $\alpha = 1$ makes one bad drone cost as much as the whole city.
- **Cooling schedule.** With the card's $T_0 = 500$ and $\beta = 0.995$, the temperature is effectively zero after about 2000 steps, so the remaining 78 000 steps would be pure hill-climbing. Here $\beta = (T_{end}/T_0)^{1/N} \approx 0.99992$ spreads the cooling over all 80 000 iterations.
- **Repair rule.** The card moves a disconnected drone to the midpoint with its nearest connected neighbour. This implementation moves it only as far as needed: to $r_{comm} - 1$ m from its nearest connected anchor (the base or a connected drone), which restores the link with the least loss of coverage.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Drone fleet size $K$ | 5 |
| Area | 2000 × 2000 m |
| Population grid | 40 × 40 cells (50 m) |
| Population clusters (centre, peak, σ) | NW (400, 1600) 8000 σ350; Downtown (1200, 1200) 12000 σ400; East (1700, 700) 6000 σ250 |
| Background population | U(0, 200) per cell, seed 42 |
| Total population | 7 468 626 persons |
| Coverage radius $r_{cov}$ | 300 m |
| Communication radius $r_{comm}$ | 600 m |
| Hover altitude | 50 m |
| Transit speed $v$ | 15 m/s |
| Battery budget $T_{max}$ / hover time $T_{hover}$ | 1800 s / 1500 s |
| Max reachable distance $D_{max}$ | 2250 m |
| Charge station | (1000, 100) m |
| SA iterations / $T_0$ / $T_{end}$ | 80 000 / 0.02 / 1e-5 |
| SA perturbation $\delta$ | 200 m at $T_0$, floor 10 m |
| Penalty weight $\alpha$ | 1 (normalised cost) |
| Sensitivity runs | 20 000 SA iterations each |

---

## Implementation

```
src/03_environmental_sar/s051_post_disaster_comm.py   # Main simulation script
```

```bash
conda activate drones
python src/03_environmental_sar/s051_post_disaster_comm.py
```

---

## Results

| Strategy | Coverage | Connected drones | Max distance to base | Round trip |
|----------|---------:|:----------------:|---------------------:|-----------:|
| Greedy (no connectivity) | 58.97% | 0 / 5 | 1630 m | 217 s |
| Greedy + Repair | 57.37% | 5 / 5 | 1630 m | 217 s |
| **Simulated Annealing** | **62.98%** | **5 / 5** | 1530 m | 204 s |

**Key findings**:
- **Greedy alone is unusable**: none of its five drones can relay to base. The nearest one, at (825, 925) m, is 843 m from the station, beyond the 600 m radio range.
- **Repair is cheap here**: two moves reconnect the network. Drone D4 slides from (825, 925) to (876, 686), 599 m from the base, as the first relay hop. Coverage drops by only 1.6 percentage points.
- **SA beats the unconstrained greedy**, 62.98% vs 58.97%, even though SA must also stay connected. Greedy picks cell centroids one at a time and leaves overlapping circles. SA moves all five drones continuously, so it packs the circles into a chain that starts near the base and spreads across the three clusters.
- **SA connects within 194 iterations** from a random start, because each disconnected drone costs as much as the whole city. The remaining iterations trade coverage, which climbs from about 50% to 63% as the temperature falls.
- **The (1 − 1/e) bound** applies to greedy on the unconstrained discrete problem: greedy is guaranteed at least 63.2% of the unconstrained optimum. The best solution found here (SA, 62.98%) is 1.07× greedy, well inside that bound.

**Sensitivity of the connected SA solution** (20 000 iterations each):

| $r_{cov}$ (m) | 250 | 300 | 350 | 400 | 450 |
|---|---|---|---|---|---|
| Coverage | 47.8% | 62.2% | 74.9% | 84.3% | 90.7% |

| $K$ | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| Coverage | 9.3% | 26.5% | 40.6% | 52.5% | 62.2% | 70.7% | 78.0% | 82.0% |

With $K = 1$ the single drone must stay within 600 m of the charge station, far from every cluster, so it serves only 9.3%. Each added drone extends the relay chain, and returns diminish beyond about 6 drones as the clusters fill up.

### Population Map

Synthetic population density with the three clusters labelled. The black star is the charge station and the dotted circle its 600 m radio range:

![Population Map](./population_map.png)

### Coverage Comparison

Coloured cells are served population; dashed lines are radio links (including to the base); red X marks a disconnected drone. The purple arrow in the middle panel shows the repair move:

![Coverage Comparison](./coverage_comparison.png)

### 3D Relay Network

The SA solution in 3D: drones at 50 m above the population surface, coverage circles projected on the ground, and the relay chain back to the base:

![3D Network](./network_3d.png)

### Coverage Fraction

Hatched bar = violates connectivity. Dashed line = (1 − 1/e) × greedy coverage; dotted line = best connected solution:

![Coverage Bar](./coverage_bar.png)

### SA Convergence

Top: cost $E$ (symmetric log scale). Bottom: coverage and penalty against iteration on a log scale. The penalty drops to zero at iteration 194 and stays there:

![SA Convergence](./sa_convergence.png)

### Sensitivity Analysis

![Sensitivity](./sensitivity.png)

### Animation

SA progress: drones start at random positions (red = disconnected), join the network, then spread out to cover the clusters:

![Animation](./animation.gif)

---

## Extensions

1. **Time-varying population**: daytime crowds in commercial zones and nighttime crowds in residential zones; optimise a hover schedule with at most one repositioning per drone per hour.
2. **Heterogeneous coverage radii**: drones with $r_{cov,k} \in \{200, 300, 400\}$ m antennas; use the per-drone disk model in the SA cost.
3. **Relay bandwidth**: model link throughput $B_{link} \propto r_{comm}^{-2}$ and require the bottleneck link to exceed $B_{min}$.
4. **Rotating recharge**: one drone at a time returns to base while the other $K - 1$ keep coverage; minimise the coverage gap during each swap.
5. **RL placement policy**: train PPO to place drones from the population map, and compare with SA on held-out city layouts.
6. **Building shadowing**: block line of sight with a city height map and adjust the coverage indicator.

---

## Related Scenarios

- Prerequisites: [S047 Base Station Signal Relay](../../../scenarios/03_environmental_sar/S047_signal_relay.md), [S048 Full-Area Coverage Scan (Lawnmower)](../../../scenarios/03_environmental_sar/S048_lawnmower.md), [S049 Dynamic Zone Search](../../../scenarios/03_environmental_sar/S049_dynamic_zone.md)
- Follow-ups: [S052 Glacier Melt Area Measurement](../../../scenarios/03_environmental_sar/S052_glacier.md)
- Cross-reference: [S029 Urban Logistics Scheduling](../../../scenarios/02_logistics_delivery/S029_urban_logistics_scheduling.md), [S015 Relay Tracking](../../../scenarios/01_pursuit_evasion/S015_relay_tracking.md)
