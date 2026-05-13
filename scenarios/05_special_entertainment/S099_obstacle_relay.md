# S099 Cross-Obstacle Relay Pass

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started

**Algorithm**: RRT* Gate Planning + Velocity-Matched Mid-Air Handoff | **Dimension**: 3D

---

## Problem Definition

**Setup**: Two drones must transport a virtual baton through a 5-gate obstacle course placed at random 3D positions. Drone A carries the baton through gates 1–3, performs a velocity-matched mid-air handoff near gate 3, and Drone B then carries the baton through gates 4–5. Each drone independently plans an RRT* path that satisfies gate-passage constraints, and the combined mission is completed in minimum total time.

**Roles**:
- **Drone A (carrier)**: Launches from start, threads gates 1–3, initiates handoff rendezvous near gate 3 exit.
- **Drone B (receiver)**: Holds a staging position, intercepts Drone A's trajectory, matches velocity at handoff, then threads gates 4–5 to the finish.

**Objective**: Minimise total mission time $T_\text{total}$ while ensuring all gates are passed within radius $r_\text{gate}$ and the handoff velocity error is below threshold $\epsilon_v$.

---

## Mathematical Model

### Gate Passage Constraint

Gate $k$ is defined by centre $\mathbf{g}_k \in \mathbb{R}^3$ and unit normal $\hat{\mathbf{n}}_k$ (gate plane orientation). The trajectory $\mathbf{p}(t)$ must satisfy two simultaneous conditions at crossing time $t_k$:

$$\|\mathbf{p}(t_k) - \mathbf{g}_k\| \leq r_\text{gate}$$

$$|(\mathbf{p}(t_k) - \mathbf{g}_k) \cdot \hat{\mathbf{n}}_k| \leq \delta_\text{tol}$$

where $r_\text{gate} = 0.4$ m is the gate disc radius and $\delta_\text{tol} = 0.05$ m is the axial tolerance.

### RRT* Path Planning with Gate Waypoints

The RRT* tree is grown in free space $\mathcal{C}_\text{free}$. Gate centres act as mandatory waypoints inserted into the tree. The steering function biases samples toward the next gate centre with probability $p_\text{bias} = 0.2$. The cost-to-come is Euclidean arc length:

$$c(\mathbf{x}_\text{new}) = c(\mathbf{x}_\text{parent}) + \|\mathbf{x}_\text{new} - \mathbf{x}_\text{parent}\|$$

The rewiring radius shrinks with tree size $n$:

$$r_\text{rewire}(n) = \gamma \left(\frac{\log n}{n}\right)^{1/3}$$

where $\gamma$ is a domain-dependent constant chosen so $r_\text{rewire}$ covers the typical gate spacing.

### Minimum-Snap Polynomial Trajectory

Between each consecutive pair of RRT* waypoints, a degree-7 polynomial segment $\mathbf{p}_i(t)$ is fitted by minimising the integral of squared snap:

$$J_\text{snap} = \int_0^{T_i} \left\|\frac{d^4 \mathbf{p}_i}{dt^4}\right\|^2 dt$$

subject to boundary conditions on position, velocity, acceleration, and jerk at segment endpoints. The per-segment time $T_i$ is allocated proportionally to Euclidean distance.

### Velocity-Matched Handoff (Rendezvous)

At handoff time $t_h$, both drones must be co-located with matched velocity:

$$\|\mathbf{p}_A(t_h) - \mathbf{p}_B(t_h)\| \leq \epsilon_p$$

$$\|\dot{\mathbf{p}}_A(t_h) - \dot{\mathbf{p}}_B(t_h)\| \leq \epsilon_v$$

Drone B plans its intercept trajectory backwards from the predicted handoff state $(\mathbf{p}_A(t_h),\, \dot{\mathbf{p}}_A(t_h))$, solving a boundary-value problem with its own minimum-snap polynomial.

### Combined Cost Function

The planner minimises a penalised total time objective:

$$J = T_\text{total} + \lambda \sum_{k=1}^{5} \max\!\left(0,\; \|\mathbf{p}(t_k) - \mathbf{g}_k\| - r_\text{gate}\right)^2$$

where $\lambda = 50$ is the gate-miss penalty weight. A zero-penalty solution corresponds to all gates being cleanly threaded.

---

## Implementation

```python
# --- RRT* with gate waypoints ---
def rrt_star_gate(start, gates, goal, obstacles, n_iter=3000):
    tree = Tree(start)
    mandatory = list(gates) + [goal]
    current_target_idx = 0

    for _ in range(n_iter):
        # Biased sampling toward next mandatory waypoint
        if random() < P_BIAS:
            q_rand = mandatory[current_target_idx] + randn(3) * 0.3
        else:
            q_rand = sample_free(obstacles)

        q_near = nearest(tree, q_rand)
        q_new  = steer(q_near, q_rand, step=0.2)
        if collision_free(q_near, q_new, obstacles):
            Q_near = near(tree, q_new, rewire_radius(len(tree)))
            q_min, c_min = choose_parent(Q_near, q_near, q_new)
            tree.add(q_new, parent=q_min, cost=c_min)
            rewire(tree, Q_near, q_new)

        if dist(q_new, mandatory[current_target_idx]) < 0.05:
            current_target_idx = min(current_target_idx + 1, len(mandatory) - 1)

    return extract_path(tree, goal)

# --- Minimum-snap segment between two waypoints ---
def min_snap_segment(p0, pf, v0, vf, T):
    # 8 coefficients for degree-7 polynomial per axis
    # Returns callable trajectory p(t), v(t)
    coeffs = solve_min_snap_bvp(p0, pf, v0, vf, T)
    return PolyTrajectory(coeffs, T)

# --- Handoff rendezvous ---
def plan_handoff(drone_a_traj, t_h, gates_b):
    p_h = drone_a_traj.pos(t_h)
    v_h = drone_a_traj.vel(t_h)
    # Drone B intercept: minimum-snap from staging to (p_h, v_h)
    t_intercept = t_h - T_INTERCEPT_LEAD
    drone_b_traj = min_snap_segment(
        p0=B_STAGING, pf=p_h,
        v0=np.zeros(3), vf=v_h,
        T=T_INTERCEPT_LEAD
    )
    return drone_b_traj

# --- Cost evaluation ---
def mission_cost(path_a, path_b, gates, lam=50.0):
    T_total = path_a.duration + path_b.duration
    penalty = sum(
        max(0, np.linalg.norm(path.pos(t_k) - g) - R_GATE)**2
        for path, t_k, g in zip([path_a]*3 + [path_b]*2,
                                  crossing_times, gates)
    )
    return T_total + lam * penalty
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Gate disc radius | r_gate | 0.4 m |
| Gate axial tolerance | delta_tol | 0.05 m |
| Number of gates | K | 5 |
| Gate positions | g_k | Random 3D, x in [2,12] m, y in [-3,3] m, z in [1,4] m |
| Gate normal | n_hat_k | Random unit vector, z-component > 0.5 |
| RRT* iterations | n_iter | 3000 |
| RRT* bias probability | p_bias | 0.2 |
| RRT* step size | step | 0.20 m |
| Rewiring constant | gamma | 2.5 |
| Polynomial degree | — | 7 (minimum-snap) |
| Gate-miss penalty weight | lambda | 50.0 |
| Handoff position tolerance | epsilon_p | 0.10 m |
| Handoff velocity tolerance | epsilon_v | 0.15 m/s |
| Drone max speed | v_max | 3.0 m/s |
| Drone max acceleration | a_max | 5.0 m/s^2 |
| Intercept lead time | T_intercept_lead | 2.5 s |
| RACE_GATE_RADIUS (domain) | — | 0.3 m (domain default; scenario uses 0.4 m) |

---

## Expected Output

**Plot 1 — 3D Gate Course Overview**

Top-down and isometric 3D view showing:
- Gate discs (grey rings) at random 3D positions
- RRT* tree nodes (light grey scatter) for both drones
- Drone A trajectory (red) threading gates 1–3
- Drone B trajectory (blue) threading gates 4–5
- Handoff rendezvous point (gold star marker)
- Start, staging, and goal markers

**Plot 2 — Gate Miss Distance vs Gate Index**

Bar chart of $\|\mathbf{p}(t_k) - \mathbf{g}_k\|$ for each of the 5 gates, with the $r_\text{gate} = 0.4$ m threshold marked as a horizontal dashed line. Bars exceeding the threshold are coloured red.

**Plot 3 — Animation (gate_relay_animation.gif)**

3D animated playback at 30 fps showing both drones moving simultaneously, baton colour switching from red to blue at the handoff frame, gate discs flashing green when cleanly passed.

**Metrics table**

| Metric | Value |
|--------|-------|
| Total mission time T_total | — s |
| Gate miss count | — / 5 |
| Max gate miss distance | — m |
| Handoff position error | — m |
| Handoff velocity error | — m/s |
| RRT* path length (Drone A) | — m |
| RRT* path length (Drone B) | — m |

---

## Extensions

1. **Dynamic gate positions**: gates drift slowly with sinusoidal motion, requiring online re-planning every 0.5 s.
2. **Three-drone chain relay**: add a third drone for a two-handoff sequence through a 9-gate course.
3. **Adversarial blocker drone**: a third drone attempts to obstruct the relay path, coupling this scenario with pursuit-evasion (S082, S094).
4. **Hardware-in-the-loop**: replace the virtual baton with a physical payload modelled as a pendulum; add swing-damping constraints to the handoff BVP.
5. **Multi-lap relay**: both drones complete a second lap in reverse order, testing robustness of repeated gate-threaded trajectories.

---

## Related Scenarios

- Prerequisites: [S082](S082_fpv_racing.md), [S090](S090_racing_optimal.md), [S096](S096_relay_race.md)
- Next: [S100](S100_grand_challenge.md)

## References

- Karaman, S. & Frazzoli, E. (2011). *Sampling-based algorithms for optimal motion planning*. IJRR 30(7).
- Mellinger, D. & Kumar, V. (2011). *Minimum snap trajectory generation and control for quadrotors*. ICRA.
- Spica, R. et al. (2020). *A real-time game theoretic planner for autonomous two-player drone racing*. IEEE T-RO 36(5).
