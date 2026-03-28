# S009 Differential Game 1v1 (Differential Game — Lion & Man)

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[x]` Completed

---

## Problem Definition

**Lion & Man Problem**: in a bounded space, the Lion (pursuer) and Man (evader) both move at speed $v$. Can the Lion capture the Man?

**This scenario**:
- Pursuer and evader have equal speeds ($v_P = v_E = 4$ m/s)
- Bounded space ($[-5,5]^3$ m cube)
- Find the value function $V(x)$ under both players' optimal strategies

---

## Mathematical Model

### State Space

Relative state: $\mathbf{s} = \mathbf{p}_E - \mathbf{p}_P \in \mathbb{R}^3$

Pursuer minimizes capture time $T$; evader maximizes it.

### Hamilton-Jacobi-Isaacs (HJI) Equation

$$\frac{\partial V}{\partial t} + H\left(\mathbf{s}, \nabla V\right) = 0$$

Hamiltonian:

$$H(\mathbf{s}, p) = \min_{\hat{u}_P} \max_{\hat{u}_E} \left[ p^T (v_E \hat{u}_E - v_P \hat{u}_P) \right] - 1$$

where $\hat{u}_P, \hat{u}_E \in \mathbb{S}^2$ (unit sphere direction vectors).

Optimal strategies:

$$\hat{u}_P^* = \frac{p}{\|p\|}, \quad \hat{u}_E^* = \frac{p}{\|p\|}$$

(both agents move along the $p = \nabla V$ direction)

### Analytical Solution for Equal Speeds

When speeds are equal, the Lion cannot guarantee capture in finite time (unless the initial distance is zero).

**Numerical verification**: solve HJI using finite difference method.

---

## Implementation (Numerical HJI Solver)

```python
import numpy as np
from scipy.interpolate import RegularGridInterpolator

# Grid setup (2D simplified version, then extend to 3D)
N = 50
x = np.linspace(-5, 5, N)
y = np.linspace(-5, 5, N)
X, Y = np.meshgrid(x, y)

# Initial value function (terminal condition)
# V(s) = 0 when |s| < capture_radius
V = np.sqrt(X**2 + Y**2)   # initial guess: straight-line distance

# Value iteration (simplified)
dt = 0.01
for iteration in range(200):
    # Compute gradient
    dVdx = np.gradient(V, x, axis=1)
    dVdy = np.gradient(V, y, axis=0)
    grad_norm = np.sqrt(dVdx**2 + dVdy**2) + 1e-8

    # HJI right-hand side: H = (vE - vP) * |∇V| - 1 (equal speeds)
    vP, vE = 4.0, 4.0
    H = (vE - vP) * grad_norm - 1.0

    # Update
    V_new = V - dt * H
    V_new[np.sqrt(X**2 + Y**2) < 0.15] = 0.0   # terminal condition

    if np.max(np.abs(V_new - V)) < 1e-4:
        print(f"Converged at iteration {iteration}")
        break
    V = V_new
```

### Extracting Optimal Strategy from Value Function

```python
def optimal_pursuer_action(pos_p, pos_e, V_interp, grid):
    """Optimal pursuer action derived from value function gradient"""
    s = pos_e - pos_p
    grad = V_interp.gradient(s[:2])   # approximate gradient
    return grad / (np.linalg.norm(grad) + 1e-8)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer speed | 4.0 m/s |
| Evader speed | 4.0 m/s (equal!) |
| Arena | $[-5,5]^2$ m (2D) → extend to 3D |
| Numerical grid | 50×50 (2D), 30×30×30 (3D, slower) |
| Iterations | up to 500 |

---

## Expected Output

- 2D value function contour plot (showing capture time distribution)
- Optimal trajectory plot: at equal speeds the Man can maintain distance indefinitely
- Value function convergence curve

---

## Extensions

1. Unequal speeds ($v_P > v_E$): Lion guarantees capture; analyze capture time
2. Extend to 3D (requires coarser grid)
3. Approximate value function with RL (PPO); compare against numerical solution

---

## Related Scenarios

- Prerequisites: [S002](S002_evasive_maneuver.md) (understand strategy concepts)
- Follow-ups: [S010](S010_asymmetric_speed.md) (asymmetric speeds), [S020](S020_pursuit_evasion_game.md)

## References

- Isaacs, R. (1965). *Differential Games*. Wiley.
- Mitchell, I. (2008). *A Toolbox of Level Set Methods*. UBC Tech Report.
- [MATH_FOUNDATIONS.md §4.3](../../MATH_FOUNDATIONS.md)
