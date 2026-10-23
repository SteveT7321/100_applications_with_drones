# S009 3D Upgrade — Differential Game (Lion & Man in 3D)

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[x]` Completed
**Based on**: [S009 original](../S009_differential_game.md)

---

## What Changes in 3D

The original S009 solves the HJI equation on a 2D grid ($\mathbf{s} = [s_x, s_y]$). In 3D, the relative state becomes $\mathbf{s} = [s_x, s_y, s_z] \in \mathbb{R}^3$, the HJI PDE lives on a 3D grid, and the value function is a scalar field $V(s_x, s_y, s_z)$. The optimal strategies are 3D unit vectors on $\mathbb{S}^2$. At equal speeds the open-space conclusion is the same as in 2D (no finite capture time), but the bounded arena is now a cube with faces, edges and corners, and the Man has a third direction to run in.

---

## Problem Definition

**Setup**: Lion (pursuer) and Man (evader) move in the bounded 3D cube $[-5,5]^3$ m.

**Relative state**: $\mathbf{s} = \mathbf{p}_E - \mathbf{p}_P \in [-10,10]^3$ m.

**Goals**:
1. Solve the 3D HJI value function in relative coordinates (no walls). For $v_P > v_E$ it must converge to the analytic capture time; for $v_P = v_E$ it must diverge ($V = \infty$, no finite capture in open space).
2. Play the equal-speed game ($v = 4$ m/s) in the bounded cube with the walls enforced, and show how faces, edges and corners and the third dimension change the outcome compared to the 2D square.

---

## Mathematical Model

### 3D Relative State Dynamics

$$\dot{\mathbf{s}} = v_E \hat{\mathbf{u}}_E - v_P \hat{\mathbf{u}}_P, \quad \hat{\mathbf{u}}_P, \hat{\mathbf{u}}_E \in \mathbb{S}^2$$

### 3D Hamilton-Jacobi-Isaacs Equation (minimum-time game)

The value $V(\mathbf{s})$ is the capture time under optimal play, so $dV/dt = -1$ along optimal trajectories. It satisfies the **stationary** HJI equation

$$\min_{\hat{u}_P \in \mathbb{S}^2} \max_{\hat{u}_E \in \mathbb{S}^2} \left[ \nabla V^T (v_E \hat{u}_E - v_P \hat{u}_P) \right] + 1 = 0, \qquad V = 0 \text{ on } \|\mathbf{s}\| \le r_{capture}$$

The saddle point on the unit sphere is

$$\hat{u}_E^* = \hat{u}_P^* = \frac{\nabla V}{\|\nabla V\|}$$

(the pursuer flies toward the evader, the evader straight away), which gives the Eikonal equation

$$(v_P - v_E)\,\|\nabla V\| = 1 \quad\Rightarrow\quad V(\mathbf{s}) = \frac{\|\mathbf{s}\| - r_{capture}}{v_P - v_E} \quad (v_P > v_E,\ \text{no walls})$$

The level sets of $V$ are spheres. For $v_P = v_E$ the equation has no finite solution: $V = \infty$ outside the capture set, so in open space the Man escapes by running straight away.

### 3D Numerical Solver (pseudo-time iteration, Godunov upwind)

Grid: $N_x \times N_y \times N_z$ with $N = 30$ per axis over $[-10, 10]^3$ (27 000 nodes, $\Delta s = 20/29 = 0.69$ m).

Start from $V = 0$ and iterate toward the steady state:

$$V^{n+1}_{ijk} = V^n_{ijk} + \tau\left(1 - (v_P - v_E)\,\|\nabla V^n\|_{God}\right), \qquad \tau = 0.9\,\frac{\Delta s}{3\,(v_P - v_E)}$$

$\|\nabla V\|_{God}$ is the monotone Godunov upwind gradient: per axis $\max(D^- V,\, -D^+ V,\, 0)^2$, summed and square-rooted. Central differences are not monotone for this equation. The 0.15 m capture sphere is smaller than a grid cell, so nodes with $\|\mathbf{s}\| < r_{capture} + \Delta s$ are fixed to their exact value. Converged when $\max|\Delta V| < 10^{-6}$ s. For $v_P = v_E$ the update reduces to $V \mathrel{+}= \tau$ everywhere, so $V$ grows without bound.

### Bounded Cube at Equal Speeds

The relative-state HJI ignores the walls. In the cube the full state is both positions (6-D), which is not solved on a grid. Instead, strategy pairs are simulated with the walls enforced:

- Lion **radius strategy**: stay on the segment centre → Man, as close to the Man as possible (works in any dimension)
- Lion **pure pursuit** (the open-space HJI control)
- Man **flee** (the open-space HJI control), sliding along walls
- Man **circle**: the 2D strategy, a horizontal 4.5 m circle
- Man **sphere**: run tangent to a 4.5 m sphere about the centre, away from the Lion

With a positive capture radius in a bounded arena the Lion can always capture (e.g. with the radius strategy). Indefinite evasion needs zero capture radius or open space.

### 3D Optimal Simulation Strategy (open space, $v_P > v_E$)

$$\hat{u}_P^* = \hat{u}_E^* = \nabla_{\mathbf{s}} V(\mathbf{s}) / \|\nabla_{\mathbf{s}} V(\mathbf{s})\|$$

Trilinear interpolation of the 3D grid gradient evaluates $\nabla V$ at arbitrary $\mathbf{s}$.

---

## Key 3D Additions

- HJI PDE extended to 3D relative state space (3D Godunov finite-difference solver)
- Optimal strategies are 3D unit vectors on $\mathbb{S}^2$
- 3D value function isosurfaces (level sets of $V$ are spheres)
- Comparison of 2D vs 3D value function: the middle plane of the 3D $V$ against a 2D solve on the same x-y nodes
- Bounded 3D cube arena: where (face, edge, corner, interior) the Man is caught

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| HJI speeds | $v_P = 5$, $v_E = 3$ m/s (converging case); $v_P = v_E = 4$ m/s (divergence) |
| Arena game speeds | $v_P = v_E = 4.0$ m/s (equal) |
| Bounded arena | $[-5,5]^3$ m cube |
| Relative-state grid | $30 \times 30 \times 30$ over $[-10, 10]^3$ m |
| Grid spacing $\Delta s$ | 0.69 m |
| Pseudo-time step τ | $0.9\,\Delta s / (3(v_P - v_E)) = 0.103$ s; 0.05 s for the equal-speed demonstration |
| Max iterations | 500 (equal-speed demonstration: 200) |
| Capture radius | 0.15 m |

---

## Expected Output

- **3D value function isosurfaces**: level surfaces $V = \{1, 2.5, 4\}$ s ($v_P = 5$, $v_E = 3$), close to spheres of radius $2V + 0.15$ m
- **Middle-plane slice**: compare with the S009 2D result on the same nodes
- **Trajectory simulation**: the asymmetric game played from the gradient policy in 3D; equal-speed Lion & Man in the bounded cube
- **Convergence plot**: $\max |V^{n+1} - V^n|$ vs iteration (asymmetric converges; equal speeds stay at τ)

---

## Extensions

1. Speed ratio close to 1 in the cube: minimum ratio for guaranteed capture against the sphere strategy
2. 3D arena with cylindrical obstacle: obstacle boundary modifies the HJI BCs and changes the capture funnel shape
3. Approximate 3D value function with a neural network (physics-informed NN) to avoid the $O(N^3)$ grid cost

---

## Related Scenarios

- Original: [S009 Differential Game 1v1 (Differential Game — Lion & Man)](../S009_differential_game.md)
- Truly 3D reference: [S001 Basic Intercept](../S001_basic_intercept.md), [S003 Low-Altitude Tracking](../S003_low_altitude_tracking.md)
