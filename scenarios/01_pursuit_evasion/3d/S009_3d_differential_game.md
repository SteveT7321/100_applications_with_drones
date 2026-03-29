# S009 3D Upgrade — Differential Game (Lion & Man in 3D)

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[x]` Complete
**Based on**: [S009 original](../S009_differential_game.md)

---

## What Changes in 3D

The original S009 solves the HJI equation on a 2D grid ($\mathbf{s} = [s_x, s_y]$). The code explicitly notes "2D simplified version, then extend to 3D." In 3D, the relative state becomes $\mathbf{s} = [s_x, s_y, s_z] \in \mathbb{R}^3$, the HJI PDE lives on a 3D grid, and the value function is a scalar field $V(s_x, s_y, s_z)$. The optimal strategies are now 3D unit vectors on $\mathbb{S}^2$. The symmetry of equal speeds in 3D is the same (capture impossible in infinite space at equal speeds), but the bounded arena is now a cube and the corner/wall trapping geometry is fundamentally different.

---

## Problem Definition

**Setup**: Lion (pursuer) and Man (evader) both move at speed $v$ in a bounded 3D cube $[-5,5]^3$ m. Find the 3D value function $V(\mathbf{s})$ under both players' optimal strategies.

**Relative state**: $\mathbf{s} = \mathbf{p}_E - \mathbf{p}_P \in [-10,10]^3$ m (relative to the pursuer).

**Goal**: verify that at equal speeds, the value function diverges (no finite capture time) in the interior of the relative-state space; show how boundary wall effects modify the game geometry compared to 2D.

---

## Mathematical Model

### 3D Relative State Dynamics

$$\dot{\mathbf{s}} = v_E \hat{\mathbf{u}}_E - v_P \hat{\mathbf{u}}_P, \quad \hat{\mathbf{u}}_P, \hat{\mathbf{u}}_E \in \mathbb{S}^2$$

where $\mathbb{S}^2$ is the unit sphere in $\mathbb{R}^3$.

### 3D Hamilton-Jacobi-Isaacs Equation

$$\frac{\partial V}{\partial t} + H(\mathbf{s}, \nabla_{\mathbf{s}} V) = 0$$

3D Hamiltonian:

$$H(\mathbf{s}, p) = \min_{\hat{u}_P \in \mathbb{S}^2} \max_{\hat{u}_E \in \mathbb{S}^2} \left[ p^T (v_E \hat{u}_E - v_P \hat{u}_P) \right] - 1$$

Closed-form min-max (saddle point on unit sphere):

$$\hat{u}_E^* = \frac{p}{\|p\|}, \quad \hat{u}_P^* = \frac{p}{\|p\|}$$

For equal speeds $v_P = v_E$:

$$H(\mathbf{s}, p) = (v_E - v_P)\|p\| - 1 = -1$$

This means $\partial V / \partial t = 1$, i.e., capture time grows linearly — confirming no finite-time capture in open space.

### 3D Numerical Solver (Finite Difference)

Grid: $N_x \times N_y \times N_z$ with $N = 30$ per axis (27 000 cells — feasible on a workstation).

Gradient approximation using central differences:

$$\nabla V \approx \left[\frac{V_{i+1,j,k} - V_{i-1,j,k}}{2\Delta s},\; \frac{V_{i,j+1,k} - V_{i,j-1,k}}{2\Delta s},\; \frac{V_{i,j,k+1} - V_{i,j,k-1}}{2\Delta s}\right]$$

Value iteration update:

$$V^{n+1}_{ijk} = V^n_{ijk} - \Delta t \cdot H^n_{ijk}$$

Terminal condition: $V(\mathbf{s}) = 0$ for $\|\mathbf{s}\| \leq r_{capture} = 0.15$ m.

### Apollonius Sphere (Equal Speeds, 3D)

The 3D Apollonius set at equal speeds is the entire complement of the capture sphere — confirming the Man can always maintain nonzero distance in infinite space. In the bounded cube, the wall acts as a constraint; the pursuer strategy is to drive the relative state toward a wall corner.

### 3D Optimal Simulation Strategy

Extract approximate policy by gradient of value function:

$$\hat{u}_P^* = \nabla_{\mathbf{s}} V(\mathbf{s}) / \|\nabla_{\mathbf{s}} V(\mathbf{s})\|$$
$$\hat{u}_E^* = \nabla_{\mathbf{s}} V(\mathbf{s}) / \|\nabla_{\mathbf{s}} V(\mathbf{s})\|$$

Trilinear interpolation of the 3D grid to evaluate $\nabla V$ at arbitrary $\mathbf{s}$.

---

## Key 3D Additions

- HJI PDE extended to 3D relative state space (3D finite-difference solver)
- Optimal strategies are 3D unit vectors on $\mathbb{S}^2$ (full 3D steered, not just x-y)
- 3D value function isosurface visualisation (level sets of $V$ are spheroids)
- Comparison of 2D vs 3D value function: z-slice $s_z = 0$ of the 3D $V$ should match 2D result
- Bounded 3D cube arena: face, edge, and corner capture analysis in 3D

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer speed $v_P$ | 4.0 m/s |
| Evader speed $v_E$ | 4.0 m/s (equal) |
| Bounded arena | $[-5,5]^3$ m cube |
| Relative-state grid | $30 \times 30 \times 30$ |
| Grid spacing $\Delta s$ | 0.33 m |
| Value iteration $\Delta t$ | 0.005 |
| Max iterations | 500 |
| Capture radius | 0.15 m |

---

## Expected Output

- **3D value function isosurface plot**: level surfaces $V = \{5, 10, 20\}$ s in the relative-state cube
- **z=0 slice**: 2D contour to verify match with S009 2D result
- **Trajectory simulation**: using gradient-derived policy in 3D bounded arena
- **Convergence plot**: $\max |V^{n+1} - V^n|$ vs iteration number for 3D solver

---

## Extensions

1. Asymmetric speeds ($v_P > v_E$): finite-time capture guaranteed; 3D value function shows capture-time gradient toward origin
2. 3D arena with cylindrical obstacle: obstacle boundary modifies the HJI BCs and changes the capture funnel shape
3. Approximate 3D value function with a neural network (physics-informed NN) to avoid the $O(N^3)$ grid cost

---

## Related Scenarios

- Original: [S009 2D version](../S009_differential_game.md)
- Truly 3D reference: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
