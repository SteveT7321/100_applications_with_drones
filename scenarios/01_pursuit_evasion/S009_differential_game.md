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

For the minimum-time game the value $V(\mathbf{s})$ is the capture time under optimal play. It satisfies the **stationary** HJI equation (along optimal play $dV/dt = -1$):

$$\min_{\hat{u}_P} \max_{\hat{u}_E} \left[ \nabla V^T (v_E \hat{u}_E - v_P \hat{u}_P) \right] + 1 = 0, \qquad V = 0 \text{ on } \|\mathbf{s}\| \le r_{capture}$$

where $\hat{u}_P, \hat{u}_E \in \mathbb{S}^2$ (unit sphere direction vectors). Evaluating the min-max gives

$$(v_P - v_E)\,\|\nabla V\| = 1 \quad\Rightarrow\quad V(\mathbf{s}) = \frac{\|\mathbf{s}\| - r_{capture}}{v_P - v_E} \quad (v_P > v_E,\ \text{no walls})$$

Optimal strategies:

$$\hat{u}_P^* = \frac{\nabla V}{\|\nabla V\|}, \quad \hat{u}_E^* = \frac{\nabla V}{\|\nabla V\|}$$

(both agents move along the $\nabla V$ direction: the pursuer toward the evader, the evader straight away)

### Analytical Solution for Equal Speeds

In open space (relative coordinates, no walls) with $v_P = v_E$ the equation has no finite solution: $V = \infty$ outside the capture set, and the Man escapes by running straight away. In a **bounded** arena the walls matter: with capture radius 0 the Man can avoid capture forever (Besicovitch), but with any positive capture radius the Lion can capture in finite time (e.g. with the radius strategy).

**Numerical verification**: solve the HJI equation with a finite-difference scheme.

---

## Implementation (Numerical HJI Solver)

```python
import numpy as np

# Grid setup (2D simplified version, then extend to 3D);
# relative state s = p_E - p_P spans twice the arena
N = 50
x = np.linspace(-10, 10, N)
dx = x[1] - x[0]
X, Y = np.meshgrid(x, x, indexing='ij')
R = np.sqrt(X**2 + Y**2)
vP, vE, r_c = 5.0, 3.0, 0.15

# Start from V = 0 and let the capture-time front grow outward.
# Nodes next to the (sub-grid) capture circle are fixed to the exact value.
fixed = R < r_c + dx
V = np.zeros_like(R)
V[fixed] = np.maximum(R[fixed] - r_c, 0) / (vP - vE)

tau = 0.9 * dx / (2 * (vP - vE))           # CFL-limited pseudo-time step
for iteration in range(2000):
    # Upwind (Godunov) gradient norm
    Vp = np.pad(V, 1, mode='edge')
    gx = np.maximum(np.maximum((V - Vp[:-2, 1:-1]) / dx, -(Vp[2:, 1:-1] - V) / dx), 0)
    gy = np.maximum(np.maximum((V - Vp[1:-1, :-2]) / dx, -(Vp[1:-1, 2:] - V) / dx), 0)
    grad_norm = np.sqrt(gx**2 + gy**2)

    # Pseudo-time update toward (vP - vE)|grad V| = 1
    V_new = V + tau * (1.0 - (vP - vE) * grad_norm)
    V_new[fixed] = V[fixed]

    if np.max(np.abs(V_new - V)) < 1e-6:
        print(f"Converged at iteration {iteration}")
        break
    V = V_new
# For vP == vE the update is V += tau everywhere: V grows without bound (V = inf).
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
- Optimal trajectory plot: at equal speeds in open space the Man keeps his distance indefinitely; in the bounded arena with a positive capture radius the Lion captures in finite time
- Value function convergence curve

---

## Extensions

1. Unequal speeds ($v_P > v_E$): Lion guarantees capture; analyze capture time
2. Extend to 3D (requires coarser grid)
3. Approximate value function with RL (PPO); compare against numerical solution

---

## Related Scenarios

- Prerequisites: [S002 Evasive Maneuver](S002_evasive_maneuver.md) (understand strategy concepts)
- Follow-ups: [S010 Asymmetric Speed — Bounded Arena Trapping](S010_asymmetric_speed.md) (asymmetric speeds), [S020 Pursuit-Evasion Game (RL Capstone)](S020_pursuit_evasion_game.md)

## References

- Isaacs, R. (1965). *Differential Games*. Wiley.
- Mitchell, I. (2008). *A Toolbox of Level Set Methods*. UBC Tech Report.
- [MATH_FOUNDATIONS.md §4.3](../../MATH_FOUNDATIONS.md)
