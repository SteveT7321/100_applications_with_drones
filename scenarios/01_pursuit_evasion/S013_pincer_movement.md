# S013 Pincer Movement

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: Two pursuers approach the evader from opposite directions (~180° apart), limiting its escape options. The evader uses the perpendicular-escape strategy relative to the mean pursuer direction.

**Objective**: measure how the pincer angle (angular separation between pursuers as seen from the evader) affects capture time.

**Comparison**: coordinated pincer formation vs two independent pure-pursuit drones.

---

## Mathematical Model

### Desired Pincer Positions

Each pursuer $i$ targets an offset position on the opposite side of the evader:

$$\mathbf{p}_{P_i}^{target} = \mathbf{p}_E + R \cdot \begin{bmatrix}\cos\phi_i \\ \sin\phi_i \\ 0\end{bmatrix}$$

where $\phi_1 = 0°$, $\phi_2 = 180°$ (anti-parallel), and $R$ is a shrinking offset radius:

$$R(t) = \max\!\left(R_0 - v_{shrink} \cdot t,\; R_{min}\right)$$

### Pincer Angle

$$\theta_{pincer}(t) = \arccos\!\left(\frac{(\mathbf{p}_{P_1} - \mathbf{p}_E) \cdot (\mathbf{p}_{P_2} - \mathbf{p}_E)}{\|\mathbf{p}_{P_1} - \mathbf{p}_E\|\;\|\mathbf{p}_{P_2} - \mathbf{p}_E\|}\right)$$

Ideal pincer: $\theta_{pincer} = 180°$. As $\theta \to 180°$, evader's escape cone narrows.

### Evader Escape Direction

Evader flies perpendicular to the centroid direction:

$$\mathbf{v}_E = v_E \cdot \mathbf{R}_{90°} \cdot \frac{\mathbf{p}_E - \bar{\mathbf{p}}_P}{\|\mathbf{p}_E - \bar{\mathbf{p}}_P\|}$$

where $\bar{\mathbf{p}}_P = \tfrac{1}{2}(\mathbf{p}_{P_1} + \mathbf{p}_{P_2})$.

---

## Implementation

```python
R0       = 3.0    # m — initial offset radius
R_MIN    = 0.5    # m — minimum offset
V_SHRINK = 0.3    # m/s — shrink rate

def pincer_target(pos_e, phi, R):
    return pos_e + R * np.array([np.cos(phi), np.sin(phi), 0.0])

# Pursuer 1 targets from phi=0, pursuer 2 from phi=pi
R_cur = max(R0 - V_SHRINK * t, R_MIN)
target_1 = pincer_target(evader.pos, 0,     R_cur)
target_2 = pincer_target(evader.pos, np.pi, R_cur)

v_p1 = PURSUER_SPEED * (target_1 - pursuer1.pos) / (norm(...) + 1e-8)
v_p2 = PURSUER_SPEED * (target_2 - pursuer2.pos) / (norm(...) + 1e-8)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer 1 start | (-4, -2, 2) m |
| Pursuer 2 start | (-4, 2, 2) m |
| Evader start | (2, 0, 2) m |
| Pursuer speed (each) | 5 m/s |
| Evader speed | 3.5 m/s |
| Initial offset radius R0 | 3.0 m |
| Shrink rate | 0.3 m/s |
| Capture radius | 0.15 m |

---

## Expected Output

- Top-down 2D trajectory showing pincer closing in
- Pincer angle vs time (should approach 180° as formation establishes)
- Distance from each pursuer to evader vs time
- Capture time: coordinated pincer vs two independent pure-pursuit drones

---

## Extensions

1. Three-way pincer (120° separation, $N=3$ pursuers)
2. Evader uses optimal escape: break toward the weaker (farther) pursuer
3. Pincer in cluttered environment (combine with S004 APF obstacle avoidance)

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md), [S011](S011_swarm_encirclement.md)
- Next: [S016](S016_airspace_defense.md), [S017](S017_swarm_vs_swarm.md)
