# S010 Asymmetric Speed — Bounded Arena Trapping

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Completed

---

## Problem Definition

**Setup**: The evader is **faster** than the pursuer ($v_E > v_P$). In open space, capture is impossible. However, in a **bounded square arena** the pursuer can exploit walls and corners to trap the evader. Analyse which pursuit strategies succeed at speed ratio $r = v_P / v_E < 1$.

**Strategies compared**:
1. **Direct pursuit** (pure pursuit) — fails when $r < 1$
2. **Wall-herding** — blend pursuit direction with nearest-wall direction
3. **Corner-targeting** — always aim at the corner closest to the evader

**Roles**:
- **Pursuer**: speed 3 m/s, uses one of the three strategies
- **Evader**: speed 4.5 m/s, straight escape away from pursuer

---

## Mathematical Model

### Apollonius Circle

The set of points equidistant in travel time from pursuer and evader defines the Apollonius circle with radius:

$$R_{apo} = \frac{r \cdot d}{1 - r^2}, \quad \text{centre offset} = \frac{d}{1 - r^2}$$

where $d = \|\mathbf{p}_E - \mathbf{p}_P\|$ and $r = v_P / v_E$.

Points inside the Apollonius circle are reachable by the pursuer first; the evader is "safe" outside.

### Wall-Herding Velocity

$$\mathbf{v}_{herd} = \alpha \cdot \hat{\mathbf{r}}_{PE} + (1-\alpha) \cdot \hat{\mathbf{r}}_{wall}$$

where $\hat{\mathbf{r}}_{PE}$ points toward the evader and $\hat{\mathbf{r}}_{wall}$ points toward the arena wall nearest to the evader. The blend $\alpha \in [0,1]$ controls aggression vs herding.

### Corner-Targeting

$$\mathbf{p}_{target} = \arg\min_{\mathbf{c} \in \text{corners}} \|\mathbf{c} - \mathbf{p}_E\|$$

$$\mathbf{v}_{cmd} = v_P \cdot \frac{\mathbf{p}_{target} - \mathbf{p}_P}{\|\mathbf{p}_{target} - \mathbf{p}_P\|}$$

---

## Implementation

```python
ARENA  = 5.0       # ±5 m square
ALPHA  = 0.5       # wall-herding blend factor
CORNERS = np.array([[-5,-5,2],[5,-5,2],[-5,5,2],[5,5,2]])

def wall_herd(pos_p, pos_e, v_max):
    r_pe   = (pos_e - pos_p); r_pe /= (norm(r_pe) + 1e-8)
    # nearest wall to evader
    walls  = np.array([[-5,0,0],[5,0,0],[0,-5,0],[0,5,0]])
    nearest_w = walls[np.argmin([norm(pos_e - w) for w in walls])]
    r_wall = (nearest_w - pos_p); r_wall /= (norm(r_wall) + 1e-8)
    v = ALPHA * r_pe + (1-ALPHA) * r_wall
    return v_max * v / (norm(v) + 1e-8)

def corner_target(pos_p, pos_e, v_max):
    corner = CORNERS[np.argmin([norm(pos_e - c) for c in CORNERS])]
    r = corner - pos_p
    return v_max * r / (norm(r) + 1e-8)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Arena | 10 x 10 m square, z = 2 m fixed |
| Pursuer speed | 3.0 m/s |
| Evader speed | 4.5 m/s |
| Speed ratio r | 0.67 |
| Blend factor alpha | 0.5 |
| Capture radius | 0.15 m |
| Max simulation time | 60 s |

---

## Expected Output

- 2D top-down trajectory: herding behaviour driving evader into corners
- Apollonius circle snapshots at t = 0, 5, 10, 20 s
- Capture result table: which strategies succeed within 60 s?
- Capture time vs blend factor alpha (sensitivity plot)

---

## Extensions

1. Grid-search over $\alpha$ to find optimal blend factor
2. Circular arena — different Apollonius geometry
3. Two slower pursuers cooperating to trap a faster evader

---

## Related Scenarios

- Prerequisites: [S002](S002_evasive_maneuver.md), [S009](S009_differential_game.md)
- Next: [S011](S011_swarm_encirclement.md), [S013](S013_pincer_movement.md)
