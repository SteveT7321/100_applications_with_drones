# S099 3D Upgrade — Obstacle Relay Pass

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S099 original](../S099_obstacle_relay.md)

---

## What Changes in 3D

The original S099 already specifies gate centres in 3D (`z in [1, 4] m`) but the structural obstacles — walls and tunnel segments — have no explicit vertical extent or cross-sectional shape, and the altitude-varying corridor navigation is not modelled. The gate normal vectors are biased toward the horizontal plane (z-component constraint `> 0.5`), so gates are never steeply tilted or oriented as vertical hoops. The handoff rendezvous is treated as a 3D point constraint, but no corridor geometry constrains the approach vector.

This variant makes three things fully three-dimensional:

1. **Obstacle geometry**: rectangular wall panels, circular hoops tilted at arbitrary angles, and axis-aligned cylindrical tunnels are all defined with explicit 3D bounding volumes. Collision checking uses signed-distance queries against these volumes.
2. **Altitude-varying corridor navigation**: each tunnel segment defines a cylindrical free-space corridor with a centre-line that rises or falls between entry and exit portals. The RRT* planner is constrained to remain inside the corridor tube at all times while inside the tunnel zone.
3. **Mid-air handoff in 3D approach space**: Drone B must arrive at the rendezvous point along an approach vector that is tangent to Drone A's velocity in full 3D — not just matched in position and speed, but also in heading and flight-path angle — so that the baton transfer is physically realistic even when the handoff occurs mid-climb or mid-dive.

---

## Problem Definition

**Setup**: Two drones transport a virtual baton through a 5-gate obstacle course in a bounded 3D arena of $[0, 15] \times [-4, 4] \times [0, 6]$ m. The course contains three obstacle types at varying heights:

- **Walls** (2 panels): vertical rectangular slabs that must be flown around or over; defined by corner vertices in 3D.
- **Hoops** (2 rings): circular gates of radius $r_\text{hoop} = 0.4$ m, tilted at angles $\psi_k \in [20°, 70°]$ from the horizontal; the drone must pass through the hoop disc.
- **Tunnels** (1 segment): a cylindrical free-space tube of radius $r_\text{tunnel} = 0.5$ m whose centre-line varies in altitude from entry to exit, requiring the drone to follow a curved 3D path through the tube interior.

Drone A carries the baton from start through gates 1–3 (including one hoop and the tunnel). At gate 3 exit it performs a 3D velocity-matched handoff to Drone B, which then threads gates 4–5 (one hoop, one wall clearance) to the finish.

**Roles**:
- **Drone A (carrier)**: Launches from $(0, 0, 1.5)$ m, threads gates 1–3 including tunnel interior, executes a 3D tangent-matched handoff.
- **Drone B (receiver)**: Holds a staging position at $(8, 3, 2)$ m, intercepts Drone A's full 3D state (position, velocity vector, flight-path angle), then threads gates 4–5 to the finish at $(15, 0, 1.5)$ m.

**Objective**: Minimise total mission time $T_\text{total}$ subject to:
- All gates passed within disc radius $r_\text{gate}$ along the gate normal direction.
- Trajectory remains inside tunnel tube throughout the tunnel zone.
- Handoff 3D position error $\leq \epsilon_p$ and 3D velocity error $\leq \epsilon_v$.
- No collision with wall panels or hoop frames.

---

## Mathematical Model

### 3D Obstacle Representation

**Wall panel** $w_j$ is a convex polygon in 3D space. Collision check uses the signed distance from a point $\mathbf{q}$ to the wall plane, combined with a point-in-polygon test on the panel face:

$$d_\text{wall}(\mathbf{q}, w_j) = (\mathbf{q} - \mathbf{c}_{w_j}) \cdot \hat{\mathbf{n}}_{w_j}$$

The trajectory is collision-free with respect to wall $w_j$ if $|d_\text{wall}| > d_\text{safe} = 0.15$ m or the projection falls outside the panel boundary.

**Hoop gate** $h_k$ is a disc of radius $r_\text{hoop}$ with centre $\mathbf{g}_k$ and unit normal $\hat{\mathbf{n}}_k$ at tilt angle $\psi_k$ from horizontal:

$$\hat{\mathbf{n}}_k = \begin{bmatrix} \sin\psi_k \cos\phi_k \\ \sin\psi_k \sin\phi_k \\ \cos\psi_k \end{bmatrix}$$

Passage condition at crossing time $t_k$: the trajectory must pierce the hoop disc plane inside the disc boundary:

$$\left| (\mathbf{p}(t_k) - \mathbf{g}_k) \cdot \hat{\mathbf{n}}_k \right| \leq \delta_\text{tol}$$

$$\left\| \mathbf{p}(t_k) - \mathbf{g}_k - \left[(\mathbf{p}(t_k) - \mathbf{g}_k) \cdot \hat{\mathbf{n}}_k\right] \hat{\mathbf{n}}_k \right\| \leq r_\text{hoop}$$

The approach velocity at crossing must have a positive component along $\hat{\mathbf{n}}_k$ to enforce correct pass direction:

$$\dot{\mathbf{p}}(t_k) \cdot \hat{\mathbf{n}}_k > 0$$

### Altitude-Varying Tunnel Constraint

The tunnel centre-line is a parametric curve $\mathbf{c}(s)$, $s \in [0, L_\text{tun}]$, defined as a cubic Hermite spline between entry portal $\mathbf{c}_0$ and exit portal $\mathbf{c}_1$ with prescribed tangent vectors. A drone position $\mathbf{p}$ inside the tunnel zone must satisfy:

$$d_\perp(\mathbf{p}) = \left\| \mathbf{p} - \mathbf{c}(s^*(\mathbf{p})) \right\| \leq r_\text{tunnel} - d_\text{safe}$$

where $s^*(\mathbf{p}) = \arg\min_s \|\mathbf{p} - \mathbf{c}(s)\|$ is the closest point on the centre-line. This constraint is incorporated into the RRT* collision checker so that no node outside the tube interior is accepted while the planner is inside the tunnel zone.

The centre-line altitude profile from entry $z_0$ to exit $z_1$ introduces a flight-path angle:

$$\gamma_\text{tun}(s) = \arctan\!\left(\frac{dz/ds}{\sqrt{(dx/ds)^2 + (dy/ds)^2}}\right)$$

The minimum-snap trajectory inside the tunnel must respect $|\gamma| \leq \gamma_\text{max} = 30°$ to avoid tube-wall contact during the altitude transition.

### RRT* with Heterogeneous Obstacle Types

The free configuration space $\mathcal{C}_\text{free}$ is defined by the intersection of the arena bounds, the complement of all wall panels (inflated by $d_\text{safe}$), the complement of hoop frame volumes (toroidal exclusion zones around hoop rings), and the tunnel interior constraint when inside the tunnel zone.

The steering function and rewiring radius follow the original S099 model. Gate centres act as mandatory waypoints. The added 3D complexity requires the obstacle set to be passed through a k-d tree for efficient nearest-obstacle queries during the $O(n \log n)$ rewiring step.

### 3D Tangent-Matched Handoff

At handoff time $t_h$, the rendezvous condition is extended to match the full 3D velocity vector including flight-path angle $\gamma$ and track angle $\chi$:

$$\|\mathbf{p}_A(t_h) - \mathbf{p}_B(t_h)\| \leq \epsilon_p$$

$$\|\dot{\mathbf{p}}_A(t_h) - \dot{\mathbf{p}}_B(t_h)\| \leq \epsilon_v$$

$$\left| \gamma_A(t_h) - \gamma_B(t_h) \right| \leq \epsilon_\gamma$$

where $\gamma = \arcsin(\dot{z} / \|\dot{\mathbf{p}}\|)$ is the flight-path angle. The track angle constraint ensures that both drones are flying in the same horizontal direction at handoff, preventing a lateral baton-snatch scenario that would introduce angular momentum on the payload:

$$\left| \chi_A(t_h) - \chi_B(t_h) \right| \leq \epsilon_\chi$$

Drone B solves a boundary-value problem (BVP) from its staging position to the handoff state $(\mathbf{p}_A(t_h),\, \dot{\mathbf{p}}_A(t_h))$ using a degree-7 minimum-snap polynomial that also satisfies the flight-path angle boundary condition:

$$\ddot{z}_B(0) = 0, \quad \frac{\dot{z}_B(T_\text{int})}{\|\dot{\mathbf{p}}_B(T_\text{int})\|} = \sin\gamma_A(t_h)$$

### 3D Minimum-Snap with Corridor Constraints

Between consecutive waypoints that lie inside the tunnel zone, the minimum-snap polynomial is augmented with a quadratic corridor penalty:

$$J = \int_0^{T_i} \left\|\frac{d^4\mathbf{p}_i}{dt^4}\right\|^2 dt + \mu \int_0^{T_i} \max\!\left(0,\; d_\perp(\mathbf{p}_i(t)) - (r_\text{tunnel} - d_\text{safe})\right)^2 dt$$

where $\mu = 100$ is the corridor-violation penalty weight. The integral is evaluated by quadrature at 50 collocation points per segment.

### Combined 3D Cost Function

$$J_\text{total} = T_\text{total} + \lambda_g \sum_{k=1}^{5} C_\text{gate,k} + \lambda_t \int_\text{tunnel} \max\!\left(0,\; d_\perp - (r_\text{tunnel} - d_\text{safe})\right)^2 dt$$

where $\lambda_g = 50$ (gate-miss penalty, identical to original S099), $\lambda_t = 100$ (tunnel wall penalty), and

$$C_\text{gate,k} = \max\!\left(0,\; d_{\text{in-disc},k}\right)^2 + \max\!\left(0,\; |\delta_\text{axial,k}| - \delta_\text{tol}\right)^2$$

encodes both radial miss distance within the hoop disc and axial approach tolerance.

---

## Key 3D Additions

- **Tilted hoop geometry**: gate normals at arbitrary tilt angles $\psi_k \in [20°, 70°]$ require approach-vector alignment, not just proximity.
- **Tunnel altitude profile**: curved centre-line with height variation of up to 2 m forces genuine 3D path planning through the tube interior; flight-path angle constraint $|\gamma| \leq 30°$ prevents vertical wall contact.
- **3D approach-direction constraint at hoops**: velocity must have positive projection onto $\hat{\mathbf{n}}_k$ to prevent backtracking or side-pass.
- **Full 3D state matching at handoff**: position, speed, flight-path angle $\gamma$, and track angle $\chi$ are all matched within tolerance; degree-7 BVP includes altitude second derivative boundary conditions.
- **Heterogeneous obstacle collision checker**: k-d tree over wall panels, hoop frame tori, and tunnel exterior; context-dependent constraint (tunnel interior required vs. obstacle exterior required) switched by zone membership.
- **3D visualisation**: isometric 3D view with hoop discs rendered as filled semi-transparent circles at correct tilt, tunnel rendered as a transparent cylindrical tube, altitude time series for both drones with tunnel zone shaded.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Arena bounds | — | $[0,15] \times [-4,4] \times [0,6]$ m |
| Number of gates | $K$ | 5 (2 hoops + 2 wall clearances + 1 tunnel exit portal) |
| Hoop disc radius | $r_\text{hoop}$ | 0.4 m |
| Hoop tilt angle range | $\psi_k$ | 20° – 70° from horizontal |
| Gate axial tolerance | $\delta_\text{tol}$ | 0.05 m |
| Tunnel centre-line radius | $r_\text{tunnel}$ | 0.5 m |
| Tunnel altitude change | $\Delta z_\text{tun}$ | up to 2.0 m |
| Max tunnel flight-path angle | $\gamma_\text{max}$ | 30° |
| Wall panel safety margin | $d_\text{safe}$ | 0.15 m |
| RRT* iterations | $n_\text{iter}$ | 4000 |
| RRT* bias probability | $p_\text{bias}$ | 0.2 |
| RRT* step size | step | 0.20 m |
| Rewiring constant | $\gamma_\text{rrt}$ | 2.5 |
| Polynomial degree | — | 7 (minimum-snap) |
| Gate-miss penalty weight | $\lambda_g$ | 50.0 |
| Tunnel wall penalty weight | $\lambda_t$ | 100.0 |
| Corridor quadrature points | — | 50 per segment |
| Handoff position tolerance | $\epsilon_p$ | 0.10 m |
| Handoff velocity tolerance | $\epsilon_v$ | 0.15 m/s |
| Handoff flight-path angle tol. | $\epsilon_\gamma$ | 5° |
| Handoff track angle tolerance | $\epsilon_\chi$ | 8° |
| Drone max speed | $v_\text{max}$ | 3.0 m/s |
| Drone max acceleration | $a_\text{max}$ | 5.0 m/s² |
| Intercept lead time | $T_\text{int}$ | 2.5 s |
| Altitude bounds | $z$ | 0.3 – 5.5 m |
| Staging position (Drone B) | — | $(8, 3, 2)$ m |

---

## Expected Output

- **3D gate course overview plot**: isometric view showing hoop discs as tilted semi-transparent rings, tunnel as transparent cylinder with curved centre-line, wall panels as grey rectangles, Drone A trajectory (red) through gates 1–3, Drone B trajectory (blue) through gates 4–5, handoff point (gold star), RRT* tree nodes (light grey scatter).
- **Altitude time series**: $z(t)$ for both drones with tunnel zone highlighted in light blue; shows altitude variation through the tunnel and any approach climb/dive to tilted hoops.
- **Gate pass quality bar chart**: for each of the 5 gates, plot the radial disc miss $d_\text{in-disc,k}$ and axial deviation $\delta_\text{axial,k}$; threshold lines at $r_\text{hoop}$ and $\delta_\text{tol}$ respectively; bars coloured green (pass) or red (fail).
- **Handoff state error panel**: time-series of $\|\mathbf{p}_A - \mathbf{p}_B\|$, $\|\dot{\mathbf{p}}_A - \dot{\mathbf{p}}_B\|$, $|\gamma_A - \gamma_B|$, and $|\chi_A - \chi_B|$ around $t_h$, with tolerance bands drawn in dashed lines.
- **Animation** (`gate_relay_3d_animation.gif`): 30 fps 3D animated playback; baton marker switches from red to blue at handoff frame; hoop rings flash green when cleanly passed; tunnel tube becomes translucent green when Drone A exits cleanly.

---

## Extensions

1. **Dynamic hoop rotation**: hoop tilt angles oscillate sinusoidally at 0.2 Hz, requiring the approach-direction constraint to be evaluated against the instantaneous normal at predicted crossing time.
2. **Three-drone chain relay with altitude segments**: add Drone C for a two-handoff sequence; assign each drone a different altitude band (low/mid/high) so all three drones are simultaneously active without vertical conflict.
3. **Wind disturbance inside tunnel**: model a crosswind jet inside the tunnel with $v_\text{wind} \sim \mathcal{N}(0, 0.3)$ m/s perpendicular to the centre-line; add a feedback correction term to keep $d_\perp$ within bounds.
4. **Physical baton swing model**: replace the virtual baton with a pendulum payload (mass $m_b = 0.05$ kg, cable length $l = 0.2$ m); add swing-damping constraints to the minimum-snap BVP so that angular momentum at handoff is near zero.
5. **Adversarial blocker drone**: a third drone attempts to hover in the tunnel entrance, coupling this scenario with pursuit-evasion (S094); add a reactive re-routing layer triggered when the blocker is detected within 1 m of the planned path.

---

## Related Scenarios

- Original 2D/3D version: [S099](../S099_obstacle_relay.md)
- Racing gate threading reference: [S082](../S082_fpv_racing.md), [S090](../S090_racing_optimal.md)
- Relay handoff reference: [S096](../S096_relay_race.md)
- Pursuit-evasion coupling: [S094](../S094_counter_drone.md)
- Grand challenge integration: [S100](../S100_grand_challenge.md)
