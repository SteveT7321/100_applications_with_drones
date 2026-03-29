# S006 3D Upgrade — Energy Race

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S006 original](../S006_energy_race.md)

---

## What Changes in 3D

The original S006 fixes both drones at z = 2 m and uses a flat power model $P = k v^2$. In reality, multirotor power consumption is altitude-dependent and strongly asymmetric: climbing requires significantly more power than level flight, while descending can partially recover energy. In 3D, the pursuer must trade off horizontal closure speed against the vertical energy cost of manoeuvring to cut off the evader.

---

## Problem Definition

**Setup**: Pursuer and evader carry limited batteries. Power consumption depends on both speed and vertical rate. The pursuer must capture the evader before either battery runs dry, and can choose 3D trajectories that balance closure speed against energy cost.

**New dimension**: the evader can **climb** to drain the pursuer's battery faster (forcing a more expensive vertical chase) or **descend** to exploit reduced power requirements. The pursuer must decide whether to match altitude or take a more energy-efficient horizontal intercept.

**Strategies compared**:
1. **Level pursuit** (z = 2 m fixed): ignores altitude — baseline from S006
2. **3D direct pursuit**: always fly straight toward the evader in 3D, including vertical
3. **Energy-optimal pursuit**: solve for the speed/altitude profile that minimises energy while guaranteeing capture

---

## Mathematical Model

### 3D Power Model

Multirotor power in hover is $P_{hover}$. In flight:

$$P(\mathbf{v}) = P_{hover} + k_{h} v_{xy}^2 + k_v \max(0, v_z)^2$$

where $v_{xy} = \sqrt{v_x^2 + v_y^2}$ is horizontal speed, $v_z$ is vertical speed,
$k_h$ is the horizontal drag coefficient, and $k_v > k_h$ accounts for extra power during climb.
Descent ($v_z < 0$) is modelled as free (no extra power; gravity assists).

Energy consumed:

$$E(t) = E_0 - \int_0^t P(\mathbf{v}(\tau))\, d\tau$$

### Maximum Horizontal Range for a Given Altitude Budget

If the pursuer climbs $\Delta z$ to match the evader's altitude, it spends:

$$\Delta E_{climb} = k_v (\Delta z / T_{climb})^2 \cdot T_{climb}$$

Remaining energy for horizontal pursuit: $E_{horiz} = E_0 - \Delta E_{climb} - P_{hover} \cdot T_{climb}$

Maximum horizontal flight time at speed $v_P$: $T_{horiz} = E_{horiz} / (P_{hover} + k_h v_P^2)$

### Feasibility Condition (3D)

Capture is feasible if the pursuer can close the 3D distance before energy runs out:

$$\frac{\|\mathbf{p}_E - \mathbf{p}_P\|}{v_P - v_E \cos\theta} \leq T_{horiz}$$

where $\theta$ is the 3D angle between the initial LOS and the evader velocity.

### Evader Altitude Drain Tactic

Evader climbs at vertical rate $v_{z,E}$, forcing the pursuer to expend extra climb energy:

$$\mathbf{v}_E = \begin{bmatrix} v_{E} \cos\beta \\ 0 \\ v_{E} \sin\beta \end{bmatrix}$$

where $\beta$ is the climb angle. Optimal $\beta$ for the evader maximises pursuer energy consumption per unit of evader altitude gain.

---

## Key 3D Additions

- Altitude-asymmetric power model: climbing costs more than descending
- Evader altitude-drain tactic: forced climb imposes extra energy burden on pursuer
- Energy-optimal 3D pursuit: minimise $\int P\, dt$ subject to capture constraint
- 3D feasibility boundary on an $(r_0, \Delta z)$ grid: when is capture energetically possible?

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Hover power $P_{hover}$ | 10 W |
| Horizontal coefficient $k_h$ | 0.4 W·s²/m² |
| Climb coefficient $k_v$ | 1.2 W·s²/m² |
| Battery capacity | 80 J |
| Pursuer speed options | 3.0, 4.0, 5.0 m/s |
| Evader speed | 3.0 m/s |
| Evader climb angle $\beta$ | 0°, 15°, 30° |
| Initial horizontal distance | 6 m |
| Initial altitude difference | 0 m, 1 m, 2 m |
| Capture radius | 0.15 m |

---

## Expected Output

- **3D trajectory plot**: pursuer and evader paths for each speed and altitude tactic
- **Energy vs time curves**: 3D direct pursuit vs level pursuit — when 3D routing is more/less efficient
- **Feasibility grid**: heatmap of whether capture succeeds at various $(r_0, \Delta z_0)$ starting conditions
- **Optimal climb-angle curve**: pursuer speed that minimises energy consumption as a function of evader climb angle

---

## Extensions

1. Wind field: headwind increases horizontal power; pursuer can use altitude layers with different wind speeds to reduce drag
2. Regenerative descent: assign negative energy cost to controlled descent — find the downhill intercept path that saves most energy
3. Multi-hop energy budget: relay pursuer (S012) combined with 3D power-aware handoff

---

## Related Scenarios

- Original: [S006 2D version](../S006_energy_race.md)
- Truly 3D reference: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
