# S011 3D Upgrade — Swarm Encirclement

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S011 original](../S011_swarm_encirclement.md)

---

## What Changes in 3D

The original places $N$ pursuers evenly on a **2D ring** at $z = 2.0$ m fixed, forming a flat horizontal encirclement. The evader can only attempt to break through the ring horizontally. In 3D the ring becomes a **sphere** — pursuers are distributed over the surface of a shrinking sphere using Fibonacci sphere sampling. The evader can now attempt breakout in any direction including straight up or straight down, requiring the encirclement to maintain genuine 3D coverage.

---

## Problem Definition

**Setup**: $N = 6$ pursuer drones are distributed over the surface of a sphere centred on the evader. The sphere radius $R(t)$ shrinks over time. The evader attempts to break through the gap between any two adjacent pursuers — now in 3D, the gap calculation uses solid-angle geometry rather than arc-length on a circle.

**Objective**: determine the minimum $N$ required to prevent 3D breakout as a function of speed ratio $v_E / v_P$.

---

## Mathematical Model

### Fibonacci Sphere Distribution

$N$ points approximately evenly distributed on a unit sphere using the golden-ratio spiral:

$$\mathbf{q}_i = \begin{bmatrix}
\sqrt{1 - y_i^2}\cos\phi_i \\
\sqrt{1 - y_i^2}\sin\phi_i \\
y_i
\end{bmatrix}, \quad y_i = 1 - \frac{2i}{N-1}, \quad \phi_i = i \cdot \frac{2\pi}{\varphi^2}$$

where $\varphi = (1 + \sqrt{5})/2$ is the golden ratio.

### 3D Pursuer Target Positions

Each pursuer $i$ targets a point on the shrinking sphere centred on the evader:

$$\mathbf{p}_i^{target}(t) = \mathbf{p}_E(t) + R(t) \cdot \mathbf{q}_i$$

where the shrinking radius is:

$$R(t) = \max\!\left(R_0 - v_{shrink} \cdot t,\; R_{capture}\right), \quad v_{shrink} = \frac{R_0 - R_{capture}}{T}$$

### 3D Breakout Condition

The nearest-neighbour angular gap on the sphere determines the breakout window. For $N$ points on a sphere of radius $R$, the expected nearest-neighbour arc distance (average gap half-angle) is approximately:

$$\delta_{gap} \approx \arccos\left(1 - \frac{2}{N}\right)$$

The evader can break out if its speed allows it to escape through the gap before the nearest pursuer closes:

$$v_E > v_P \sin(\delta_{gap})$$

For 3D the critical $N$ to prevent breakout at speed ratio $r = v_E/v_P$:

$$N_{critical} = \left\lceil \frac{2}{1 - \cos(\arcsin(r))} \right\rceil$$

### Altitude Strategy

The encirclement sphere naturally includes top and bottom pursuit drones. Two dedicated drones ($i = 0$ for top, $i = N-1$ for bottom) maintain positions:

$$\mathbf{p}_{top}^{target} = \mathbf{p}_E + R(t) \cdot [0, 0, 1]^T$$
$$\mathbf{p}_{bot}^{target} = \mathbf{p}_E + R(t) \cdot [0, 0, -1]^T$$

The remaining $N - 2$ drones fill the equatorial band.

### Evader 3D Breakout Strategy

The evader targets the largest angular gap in the current pursuer distribution:

$$\mathbf{v}_E = v_E \cdot \arg\max_{\hat{\mathbf{u}} \in S^2} \min_{i} \arccos(\hat{\mathbf{u}} \cdot \hat{\mathbf{q}}_i)$$

A practical approximation: evaluate $K = 200$ random candidate directions and pick the one maximising the minimum angular distance to any pursuer.

---

## Key 3D Additions

- **Altitude strategy**: dedicated top/bottom drones seal vertical escape; remaining drones fill equatorial band
- **3D guidance law**: pursuers target Fibonacci sphere points rather than ring angles; velocity command toward 3D target
- **Vertical evasion / geometry**: breakout condition uses solid-angle gap; evader searches full $S^2$ for maximum angular gap

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of pursuers | 6 |
| Initial sphere radius R₀ | 4.0 m |
| Shrink time T | 15 s |
| Capture radius | 0.3 m |
| Pursuer speed | 5 m/s |
| Evader speed | 3.5 m/s |
| Evader start | (0, 0, 2) m |
| z range | 0.5 – 6.0 m |
| Fibonacci gap candidates K | 200 |
| dt | 0.05 s |

---

## Implementation

```python
import numpy as np
from numpy.linalg import norm

N_PURSUERS = 6
R_INIT     = 4.0
T_CONVERGE = 15.0
R_CAPTURE  = 0.3
V_PURSUER  = 5.0
V_EVADER   = 3.5
Z_MIN, Z_MAX = 0.5, 6.0
DT = 0.05
PHI_GOLDEN = (1 + np.sqrt(5)) / 2

def fibonacci_sphere(n):
    """Return n unit vectors approximately uniformly distributed on S²."""
    pts = []
    for i in range(n):
        y = 1 - 2 * i / max(n - 1, 1)
        r = np.sqrt(max(1 - y * y, 0))
        phi = i * 2 * np.pi / PHI_GOLDEN ** 2
        pts.append(np.array([r * np.cos(phi), r * np.sin(phi), y]))
    return np.array(pts)

SPHERE_DIRS = fibonacci_sphere(N_PURSUERS)  # fixed orientation vectors

def encirclement_targets_3d(pos_evader, t):
    R = max(R_INIT - (R_INIT - R_CAPTURE) * t / T_CONVERGE, R_CAPTURE)
    return pos_evader[np.newaxis, :] + R * SPHERE_DIRS

def evader_breakout_3d(pos_e, pursuer_positions, v_max):
    """Find the direction with maximum angular gap from all pursuers."""
    dirs_to_pursuers = pursuer_positions - pos_e[np.newaxis, :]
    norms = norm(dirs_to_pursuers, axis=1, keepdims=True) + 1e-8
    dirs_to_pursuers /= norms

    # Sample K random unit vectors on the sphere
    K = 200
    theta = np.arccos(1 - 2 * np.random.rand(K))
    phi   = 2 * np.pi * np.random.rand(K)
    candidates = np.column_stack([
        np.sin(theta) * np.cos(phi),
        np.sin(theta) * np.sin(phi),
        np.cos(theta)
    ])  # (K, 3)

    # Min angular distance from each candidate to any pursuer
    dots = candidates @ dirs_to_pursuers.T       # (K, N)
    dots = np.clip(dots, -1, 1)
    min_angles = np.arccos(dots).min(axis=1)     # (K,)
    best_idx = int(np.argmax(min_angles))
    return v_max * candidates[best_idx]

def simulate_swarm_3d():
    pos_e = np.array([0.0, 0.0, 2.0])
    # Initial pursuer positions on Fibonacci sphere
    pos_p = pos_e[np.newaxis, :] + R_INIT * SPHERE_DIRS.copy()
    pos_p[:, 2] = np.clip(pos_p[:, 2], Z_MIN, Z_MAX)

    traj_e = [pos_e.copy()]
    traj_p = [pos_p.copy()]
    R_history = []

    for step in range(int(30 / DT)):
        t = step * DT
        R = max(R_INIT - (R_INIT - R_CAPTURE) * t / T_CONVERGE, R_CAPTURE)
        R_history.append(R)

        # Compute targets
        targets = encirclement_targets_3d(pos_e, t)
        targets[:, 2] = np.clip(targets[:, 2], Z_MIN, Z_MAX)

        # Move pursuers toward their targets
        for i in range(N_PURSUERS):
            d = targets[i] - pos_p[i]
            pos_p[i] += V_PURSUER * DT * d / (norm(d) + 1e-8)
            pos_p[i, 2] = np.clip(pos_p[i, 2], Z_MIN, Z_MAX)

        # Evader seeks max-gap direction
        v_e = evader_breakout_3d(pos_e, pos_p, V_EVADER)
        pos_e = pos_e + v_e * DT
        pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)

        traj_e.append(pos_e.copy())
        traj_p.append(pos_p.copy())

        dists = norm(pos_p - pos_e[np.newaxis, :], axis=1)
        if dists.min() < R_CAPTURE:
            print(f"Captured at t={t:.2f}s by pursuer {int(np.argmin(dists))}")
            break

    return np.array(traj_e), np.array(traj_p), np.array(R_history)
```

---

## Expected Output

- 3D sphere visualisation: Fibonacci distribution of pursuers on shrinking sphere
- Altitude time series: $z$ for all pursuers and evader — should show top/bottom drone tracking
- Encirclement radius $R(t)$ vs time
- Breakout success rate vs $N$ (sweep $N = 4, 6, 8, 10, 12$)
- Angular gap vs time: minimum angular gap between adjacent pursuers

---

## Extensions

1. Dynamic Fibonacci rebalancing: re-compute sphere orientations each step to account for pursuer positional errors
2. Heterogeneous altitude tiers: two pursuers at z = 1 m, two at z = 3 m, two at z = 5 m (layer-based formation)
3. Weighted breakout: evader also accounts for pursuer speeds when selecting escape direction

---

## Related Scenarios

- Original 2D version: [S011](../S011_swarm_encirclement.md)
- Truly 3D references: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
