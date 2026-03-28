# S019 Dynamic Target Reassignment

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: 3 pursuers vs 3 evaders. Initial assignment by Hungarian algorithm. Every $T_{check}$ seconds, recompute the optimal assignment. If the improvement exceeds a switching cost $\Delta C_{switch}$ (to avoid oscillation), reassign.

**Evader strategies are mixed**: one uses straight escape, one uses spiral, one uses random — unpredictable behaviour forces reassignment.

**Comparison**:
1. **Static** — Hungarian at $t=0$, never updated
2. **Dynamic** — re-optimise every $T_{check}$ seconds
3. **Greedy** — each pursuer always chases its nearest evader, no coordination

---

## Mathematical Model

### Assignment Cost Matrix

Time-to-intercept estimate for pursuer $i$ vs evader $j$:

$$C[i,j] = \frac{\|\mathbf{p}_{P_i} - \mathbf{p}_{E_j}\|}{v_P}$$

### Hungarian Algorithm

$$\sigma^* = \arg\min_{\sigma \in S_M} \sum_i C[i, \sigma(i)]$$

### Reassignment Condition

$$\text{improvement} = \sum_i C[i, \sigma_{current}(i)] - \sum_i C[i, \sigma^*_{new}(i)]$$

Reassign if:

$$\text{improvement} > \Delta C_{switch}$$

---

## Implementation

```python
from scipy.optimize import linear_sum_assignment

T_CHECK   = 1.0    # s — reassignment check period
DC_SWITCH = 0.5    # s — switching cost (time equivalent)

assignment = initial_hungarian(pursuers, evaders)

for step in range(max_steps):
    # Check reassignment every T_CHECK seconds
    if step * DT % T_CHECK < DT:
        C = cost_matrix(pursuers, evaders)            # shape (M, N)
        row_ind, col_ind = linear_sum_assignment(C)
        new_assign = dict(zip(row_ind, col_ind))

        current_cost = sum(C[i, assignment[i]] for i in range(M))
        new_cost     = sum(C[i, new_assign[i]]  for i in range(M))
        if current_cost - new_cost > DC_SWITCH:
            assignment = new_assign

    # Each pursuer pursues its assigned evader
    for i, p in enumerate(pursuers):
        target = evaders[assignment[i]]
        p.step(pure_pursuit(p.pos, target.pos, V_PURSUER))
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuers | 3, start at (-5, -2, 2), (-5, 0, 2), (-5, 2, 2) m |
| Evaders | 3, start at (2, -2, 2), (2, 0, 2), (2, 2, 2) m |
| Pursuer speed | 5 m/s |
| Evader speed | 3.5 m/s |
| Evader strategies | straight, spiral, random |
| Re-check period | 1.0 s |
| Switching cost | 0.5 s equivalent |

---

## Expected Output

- 3D trajectories colour-coded by assignment (dashed lines for reassignments)
- Assignment matrix over time (heatmap)
- Total mission time: static vs dynamic vs greedy
- Number of reassignment events

---

## Extensions

1. Auction algorithm — decentralised, scalable alternative to Hungarian
2. Market-based assignment: pursuers bid for evaders based on expected capture value
3. RL policy for reassignment decisions (replace rule-based switching cost)

---

## Related Scenarios

- Prerequisites: [S011](S011_swarm_encirclement.md), [S017](S017_swarm_vs_swarm.md)
- Next: [S020](S020_pursuit_evasion_game.md)

## References

- Kuhn, H.W. (1955). "The Hungarian Method for the Assignment Problem." *Naval Research Logistics*.
