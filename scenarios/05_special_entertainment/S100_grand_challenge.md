# S100 Ultimate Grand Challenge — Multi-Domain Integration

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Multi-Phase Mission Sequencer (PNG + TSP + Bayesian SAR + ORCA Formation) | **Dimension**: 3D

---

## Problem Definition

**Setup**: This is the final scenario of the 100-scenario project. A team of $N = 5$ drones is
deployed into a shared $100 \times 100 \times 30$ m arena and must complete four sequential
mission phases that collectively draw on every algorithmic domain covered in the project. No phase
may begin before the previous one is declared complete by the mission sequencer, and the total
wall-clock budget is $T_{total} = 300$ s. All five drones share situational awareness through a
distributed broadcast channel: each drone publishes its position, battery level, and phase status
at 10 Hz; every other drone ingests these updates to maintain a local world model.

**Roles**:
- **Drone A — Interceptor**: participates in Phase 1 (Pursuit). Carries a narrow-field seeker
  sensor (range $r_{seek} = 20$ m) and executes Proportional Navigation Guidance (PNG) to
  intercept a rogue target drone.
- **Drones B and C — Couriers**: participate in Phase 2 (Logistics). Each carries a single
  package payload. They compute a joint TSP route over 3 delivery waypoints and manage their
  battery reserves to guarantee a safe return to base.
- **Drone D — Scout**: participates in Phase 3 (SAR). Carries a thermal sensor (footprint radius
  $r_s = 3$ m at $h = 8$ m AGL) and maintains a Bayesian belief map over a
  $50 \times 50 \times 1$ m ground-plane search zone within the arena.
- **Drone E — Formation Lead**: remains in a holding orbit at $z = 15$ m during Phases 1–3 and
  coordinates Phase 4 (Formation). Broadcasts target star-pattern waypoints to all five drones
  once Phase 3 completes.

**Rogue target drone**: a pre-programmed evasive agent that flies in the arena at $v_{evader} = 6$
m/s using a constant-bearing evasion strategy. It is neutralised (intercepted) when Drone A closes
to within $r_{capture} = 1.5$ m.

**Survivors**: two stationary targets placed at unknown positions on the ground. Their prior
probability distributions are Gaussian, centred on two seeded last-known positions with spread
$\sigma_0 = 8$ m each.

**Four-phase mission overview**:

| Phase | Name | Actor(s) | Algorithm | Time Budget |
|-------|------|----------|-----------|-------------|
| 1 | Pursuit & Intercept | Drone A | PNG (Proportional Navigation Guidance) | 0–60 s |
| 2 | Logistics Delivery | Drones B, C | Nearest-neighbour TSP + battery management | 60–160 s |
| 3 | Search & Rescue | Drone D | Bayesian belief map + greedy frontier | 160–240 s |
| 4 | Grand Formation | All 5 drones | Hungarian assignment + ORCA repulsion | 240–300 s |

**Objective**: maximise the mission score

$$S_{mission} = \sum_{p=1}^{4} w_p \cdot \mathbf{1}[\text{phase } p \text{ completed within budget}]
  - \lambda_{OT} \cdot \max(0,\; T_{actual} - T_{total})$$

where the phase weights are $w_1 = 2, w_2 = 3, w_3 = 3, w_4 = 2$ (total possible $= 10$) and
the overtime penalty coefficient is $\lambda_{OT} = 0.05$ points per second.

**Perfect score** (10 points) requires all four phases completed within 300 s with zero
collision events and zero budget overruns.

---

## Mathematical Model

### Phase 1 — Proportional Navigation Guidance (PNG)

The interceptor (Drone A) applies **pure PNG** against the rogue target. Let
$\mathbf{r}(t) = \mathbf{p}_{target}(t) - \mathbf{p}_A(t)$ be the line-of-sight (LOS) vector
from pursuer to evader, $R = \|\mathbf{r}\|$ the range, and
$\hat{\boldsymbol{\lambda}} = \mathbf{r} / R$ the LOS unit vector.

The **LOS rotation rate** (angular rate of the LOS vector):

$$\dot{\boldsymbol{\lambda}} = \frac{d}{dt}\hat{\boldsymbol{\lambda}}
  = \frac{\dot{\mathbf{r}} - (\dot{\mathbf{r}} \cdot \hat{\boldsymbol{\lambda}})\hat{\boldsymbol{\lambda}}}{R}$$

The PNG acceleration command applied to Drone A is:

$$\ddot{\mathbf{p}}_A^{cmd} = N \cdot V_{close} \cdot \dot{\boldsymbol{\lambda}}$$

where $N = 3$ is the navigation constant (dimensionless, typical value 3–5) and
$V_{close} = -\dot{R} = -\dot{\mathbf{r}} \cdot \hat{\boldsymbol{\lambda}}$ is the closing
speed (positive when the range is decreasing). The guidance law null-steers when
$\dot{\boldsymbol{\lambda}} = \mathbf{0}$, i.e., when the LOS is non-rotating — the condition
for a collision-course intercept.

**Intercept condition**: mission Phase 1 is declared complete at the first timestep $t^*$ such
that:

$$R(t^*) = \|\mathbf{p}_{target}(t^*) - \mathbf{p}_A(t^*)\| \leq r_{capture} = 1.5 \text{ m}$$

**Time-to-go estimate** (used by the mission sequencer for budget projection):

$$T_{go} = \frac{R(t)}{V_{close}(t)} \qquad \text{(first-order approximation)}$$

**Evasion model**: the rogue target applies a constant-bearing evasion manoeuvre — it sets its
velocity perpendicular to the LOS vector at each step, maintaining $v_{evader} = 6$ m/s:

$$\dot{\mathbf{p}}_{target} = v_{evader} \cdot \hat{\mathbf{n}}_\perp(t)$$

where $\hat{\mathbf{n}}_\perp$ is the unit vector perpendicular to $\hat{\boldsymbol{\lambda}}$
in the horizontal plane, updated at each timestep.

### Phase 2 — Logistics: TSP + Battery Management

Drones B and C must jointly deliver packages to $M = 3$ waypoints
$\{\mathbf{w}_1, \mathbf{w}_2, \mathbf{w}_3\} \subset \mathbb{R}^3$. Each courier carries one
package and can deliver to exactly one waypoint per flight leg.

**Joint TSP via nearest-neighbour heuristic**: construct the full pairwise distance matrix over
the set $\mathcal{W} = \{\mathbf{b}_{base}\} \cup \{\mathbf{w}_1, \mathbf{w}_2, \mathbf{w}_3\}$
(base plus 3 waypoints). Start from the base, greedily assign the nearest unvisited waypoint to
whichever of {B, C} is geographically closest to it, alternating assignments. The resulting route
partition $(\mathcal{R}_B, \mathcal{R}_C)$ minimises an upper bound on the total travel distance:

$$D_{total} = \sum_{k \in \mathcal{R}_B} d_k^B + \sum_{k \in \mathcal{R}_C} d_k^C$$

where $d_k^i$ is the Euclidean leg distance for drone $i$ to reach waypoint $k$ from the previous
stop.

**Battery model**: each drone starts Phase 2 with a normalised state-of-charge $E_0 = 1.0$.
Energy consumption per unit distance at cruise speed $v_{cruise} = 5$ m/s:

$$\dot{E} = -c_{fly} \cdot v_{cruise}$$

with $c_{fly} = 0.004$ s$^{-1}$ m$^{-1}$ (energy fraction per metre). At each waypoint a delivery
dwell of $\tau_{deliver} = 3$ s consumes $c_{hover} = 0.003$ s$^{-1}$. The **safety reserve** is
$E_{reserve} = 0.15$ (15 % of full charge must remain at return to base). The **battery trigger**:
before committing to the next leg of length $d_{next}$, drone $i$ checks

$$E_i - c_{fly} \cdot d_{next} - c_{fly} \cdot d_{home} \geq E_{reserve}$$

where $d_{home}$ is its straight-line distance back to base. If the condition fails, the drone
aborts remaining deliveries and returns to base immediately. Phase 2 completes when all three
packages are delivered (or all drones have returned) and the couriers have landed.

**Phase 2 score contribution** is prorated: one delivery point earned per package delivered,
regardless of which drone delivers it.

### Phase 3 — Search and Rescue: Bayesian Belief Map

Drone D searches for two survivors at unknown positions $\mathbf{x}_1^*, \mathbf{x}_2^*$ within
a $50 \times 50$ m ground-plane zone (the southern half of the arena, $y \in [0, 50]$ m). The
zone is discretised into a $100 \times 100$ grid of $0.5$ m cells.

**Dual-target prior**: two independent Gaussian priors are maintained, one per survivor:

$$P_0^{(s)}(\mathbf{x}) = \frac{1}{Z_s} \exp\!\left(-\frac{\|\mathbf{x} - \mathbf{x}_{LKP}^{(s)}\|^2}{2\,\sigma_0^2}\right), \qquad s \in \{1, 2\}$$

with $\sigma_0 = 8$ m. The **combined belief map** is a superposition:

$$B_t(\mathbf{x}) \propto \alpha_1(t) \cdot P_t^{(1)}(\mathbf{x}) + \alpha_2(t) \cdot P_t^{(2)}(\mathbf{x})$$

where $\alpha_s(t) = 1$ if survivor $s$ has not yet been located, $0$ otherwise.

**Sensor (likelihood) model**: Drone D flies at $h = 8$ m AGL; at each observation event the
sensor footprint is a disc of radius $r_s = 3$ m on the ground. Detection probability within
footprint: $P_d = 0.85$. False-alarm rate outside footprint: $P_{fa} = 0.01$.

**Bayesian update** (applied per survivor belief independently):

$$B_{t+1}^{(s)}(\mathbf{x}) = \frac{P(z \mid H_\mathbf{x}^{(s)}) \cdot B_t^{(s)}(\mathbf{x})}{\displaystyle\sum_{\mathbf{x}'} P(z \mid H_{\mathbf{x}'}^{(s)}) \cdot B_t^{(s)}(\mathbf{x}')}$$

where $z \in \{0, 1\}$ is the binary observation (miss/hit) and $H_\mathbf{x}^{(s)}$ is the
hypothesis that survivor $s$ is at cell $\mathbf{x}$.

**Greedy frontier policy**: Drone D selects its next waypoint as the cell maximising the
combined belief value, breaking ties by proximity:

$$\mathbf{w}_{D}^* = \arg\max_{\mathbf{w}} B_t(\mathbf{w})$$

**Survivor located** when the belief in any survivor's map exceeds the detection threshold
$\tau_{det} = 0.6$ in a $3 \times 3$ patch, or when a physical hit observation is recorded.
Phase 3 completes when both survivors have been located, or when the Phase 3 budget
($\leq 240$ s) is exhausted.

**Entropy tracking**: the joint search entropy

$$H_{joint}(t) = -\sum_{\mathbf{x}} \tilde{B}_t(\mathbf{x}) \log \tilde{B}_t(\mathbf{x})$$

where $\tilde{B}_t$ is the renormalised combined belief, is logged at each step to quantify
information gain rate.

### Phase 4 — Grand Formation: Hungarian Assignment + ORCA

All five drones must arrange themselves into a **5-pointed star** pattern at show altitude
$z_{show} = 50$ m within the arena. The star vertices are computed analytically: for a star
inscribed in a circle of radius $R_{star} = 8$ m, the $k$-th point ($k = 0, \ldots, 4$) is at:

$$\mathbf{g}_k = \begin{bmatrix}
  R_{star} \cos\!\bigl(\tfrac{2\pi k}{5} - \tfrac{\pi}{2}\bigr) \\
  R_{star} \sin\!\bigl(\tfrac{2\pi k}{5} - \tfrac{\pi}{2}\bigr) \\
  z_{show}
\end{bmatrix}$$

**Hungarian assignment**: construct the $5 \times 5$ cost matrix

$$C_{k,m} = \|\mathbf{p}_k(t_{phase4}) - \mathbf{g}_m\|, \qquad k, m \in \{0,\ldots,4\}$$

and solve the linear sum assignment problem (Kuhn–Munkres algorithm,
`scipy.optimize.linear_sum_assignment`) to obtain the bijection $\sigma^*$ minimising total
travel distance:

$$\min_\sigma \sum_{k=0}^{4} C_{k,\sigma(k)}$$

**PID position controller** (per drone, same as S085):

$$\mathbf{u}_k(t) = K_p\,\mathbf{e}_k(t) + K_i \sum_{\tau \leq t} \mathbf{e}_k(\tau)\,\Delta t + K_d\,\frac{\mathbf{e}_k(t) - \mathbf{e}_k(t-\Delta t)}{\Delta t}$$

with $\mathbf{e}_k(t) = \mathbf{g}_{\sigma^*(k)} - \mathbf{p}_k(t)$ and gains
$(K_p, K_i, K_d) = (1.2, 0.05, 0.4)$.

**ORCA-lite collision avoidance**: whenever two drones $i, j$ satisfy
$d_{ij} = \|\mathbf{p}_i - \mathbf{p}_j\| < r_{safe} = 1.2$ m, a mutual repulsive acceleration
is applied:

$$\mathbf{F}_{rep,i}^{(j)} = k_{rep}\!\left(\frac{1}{d_{ij}} - \frac{1}{r_{safe}}\right)\frac{\hat{\mathbf{n}}_{ij}}{d_{ij}^2}$$

where $\hat{\mathbf{n}}_{ij} = (\mathbf{p}_i - \mathbf{p}_j)/d_{ij}$ and $k_{rep} = 2.0$.

**Formation convergence criterion**: Phase 4 (and the entire mission) is complete at the first
instant when all five drones satisfy $\|\mathbf{p}_k - \mathbf{g}_{\sigma^*(k)}\| \leq
\varepsilon_{form} = 0.15$ m simultaneously.

**Mean formation error** (evaluated at convergence):

$$\varepsilon_f = \frac{1}{5} \sum_{k=0}^{4} \|\mathbf{p}_k - \mathbf{g}_{\sigma^*(k)}\|$$

### Mission Sequencer

The top-level sequencer is a finite state machine with states
$\mathcal{S} = \{\textsc{Phase1}, \textsc{Phase2}, \textsc{Phase3}, \textsc{Phase4}, \textsc{Done}\}$.

**Transition logic**:

$$\text{state} \leftarrow \begin{cases}
\textsc{Phase2} & \text{if state} = \textsc{Phase1} \land (R \leq r_{capture} \lor t \geq 60) \\
\textsc{Phase3} & \text{if state} = \textsc{Phase2} \land (\text{all deliveries done} \lor t \geq 160) \\
\textsc{Phase4} & \text{if state} = \textsc{Phase3} \land (\text{both survivors found} \lor t \geq 240) \\
\textsc{Done}   & \text{if state} = \textsc{Phase4} \land (\varepsilon_f \leq 0.15 \lor t \geq 300)
\end{cases}$$

At each transition the sequencer logs the completion timestamp $t_p^{end}$ and computes the
phase success flag $s_p = \mathbf{1}[t_p^{end} \leq T_p^{budget}]$.

**Overall mission score**:

$$S_{mission} = \sum_{p=1}^{4} w_p \cdot s_p - \lambda_{OT} \cdot \max(0,\; t_4^{end} - T_{total})$$

with weights $(w_1, w_2, w_3, w_4) = (2, 3, 3, 2)$, $\lambda_{OT} = 0.05$, $T_{total} = 300$ s.

**Distributed communication**: at each simulation step, each drone broadcasts a state packet
$\langle k, \mathbf{p}_k, E_k, \text{phase}, \text{status} \rangle$. All drones receive all
packets within communication range $r_{comm} = 80$ m (effectively the entire arena). This shared
situational awareness allows:
- Phase 2 couriers to broadcast delivery completion events so the sequencer can tally progress.
- Phase 3 scout to broadcast survivor-located events so Phase 4 can begin immediately.
- Phase 4 formation lead (Drone E) to trigger the Hungarian assignment for all five agents
  simultaneously.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import linear_sum_assignment
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple
from enum import Enum, auto

# ── Global arena and mission constants ────────────────────────────────────────
ARENA_X         = 100.0         # m
ARENA_Y         = 100.0         # m
ARENA_Z         = 30.0          # m (operational ceiling)
T_TOTAL         = 300.0         # s — total mission budget
T_PHASE         = [0, 60, 160, 240, 300]  # phase start/deadline times (s)
PHASE_WEIGHTS   = [2, 3, 3, 2]  # w_1 … w_4
OT_PENALTY      = 0.05          # points per second of overtime
DT              = 0.1           # s — simulation timestep
COMM_RANGE      = 80.0          # m — inter-drone broadcast range

# Phase 1 — PNG constants
N_PNG           = 3             # navigation constant
V_INTERCEPTOR   = 8.0           # m/s — Drone A cruise speed
V_EVADER        = 6.0           # m/s — rogue target speed
R_CAPTURE       = 1.5           # m — intercept distance
R_SEEK          = 20.0          # m — Drone A seeker range

# Phase 2 — Logistics constants
V_COURIER       = 5.0           # m/s
C_FLY           = 0.004         # energy fraction per metre
C_HOVER         = 0.003         # energy fraction per second (hover/delivery dwell)
TAU_DELIVER     = 3.0           # s — delivery dwell time
E_RESERVE       = 0.15          # minimum state-of-charge fraction
BASE_POS        = np.array([50.0, 95.0, 2.0])  # launch/recovery pad

DELIVERY_WPS    = np.array([    # 3 delivery waypoints (x, y, z)
    [20.0, 70.0, 5.0],
    [75.0, 60.0, 5.0],
    [40.0, 40.0, 5.0],
])

# Phase 3 — SAR constants
SAR_ZONE_LO     = np.array([0.0,  0.0])   # 2D bounding box (ground plane)
SAR_ZONE_HI     = np.array([50.0, 50.0])
GRID_RES        = 0.5           # m per cell
GRID_N          = int((SAR_ZONE_HI[0] - SAR_ZONE_LO[0]) / GRID_RES)  # 100
SCOUT_HEIGHT    = 8.0           # m AGL
SENSOR_RADIUS   = 3.0           # m footprint radius
P_DETECT        = 0.85
P_FALSE_ALARM   = 0.01
SIGMA_PRIOR_SAR = 8.0           # m — prior spread per survivor
LKP_SURVIVOR    = np.array([[15.0, 20.0],   # last known positions
                             [35.0, 35.0]])
DET_THRESH      = 0.60          # belief threshold for located declaration
V_SCOUT         = 4.0           # m/s

# Phase 4 — Formation constants
SHOW_ALT        = 50.0          # m — star formation altitude
STAR_RADIUS     = 8.0           # m — star circumradius
V_FORM          = 6.0           # m/s
KP, KI, KD     = 1.2, 0.05, 0.4
K_REP           = 2.0           # ORCA repulsion gain
R_SAFE          = 1.2           # m — repulsion activation distance
CONV_THRESH     = 0.15          # m — formation convergence threshold


# ── Enumerations ──────────────────────────────────────────────────────────────

class Phase(Enum):
    PHASE1 = auto()
    PHASE2 = auto()
    PHASE3 = auto()
    PHASE4 = auto()
    DONE   = auto()


# ── Utility helpers ───────────────────────────────────────────────────────────

def star_formation_targets(n=5, R=STAR_RADIUS, z=SHOW_ALT):
    """Return (5, 3) array of star vertex positions centred at (50, 50, z)."""
    angles = np.array([2 * np.pi * k / n - np.pi / 2 for k in range(n)])
    cx, cy = ARENA_X / 2, ARENA_Y / 2
    pts = np.stack([cx + R * np.cos(angles),
                    cy + R * np.sin(angles),
                    np.full(n, z)], axis=1)
    return pts


def hungarian_assign(positions, targets):
    """Solve 5×5 linear sum assignment. Returns assignment array."""
    C = np.linalg.norm(positions[:, None, :] - targets[None, :, :], axis=2)
    row_ind, col_ind = linear_sum_assignment(C)
    assignment = np.empty(len(positions), dtype=int)
    assignment[row_ind] = col_ind
    return assignment


def orca_repulsion(positions, i):
    """Compute ORCA-lite repulsive force on drone i from all nearby drones."""
    F = np.zeros(3)
    for j in range(len(positions)):
        if j == i:
            continue
        diff = positions[i] - positions[j]
        d = np.linalg.norm(diff)
        if 0 < d < R_SAFE:
            n_hat = diff / d
            mag = K_REP * (1.0 / d - 1.0 / R_SAFE) / (d ** 2)
            F += mag * n_hat
    return F


# ── SAR belief map helpers ────────────────────────────────────────────────────

def build_sar_grid():
    """Return (N, 2) cell centres for the SAR ground-plane grid."""
    xs = np.arange(SAR_ZONE_LO[0] + 0.5 * GRID_RES,
                   SAR_ZONE_HI[0], GRID_RES)
    ys = np.arange(SAR_ZONE_LO[1] + 0.5 * GRID_RES,
                   SAR_ZONE_HI[1], GRID_RES)
    XX, YY = np.meshgrid(xs, ys, indexing='ij')
    return np.stack([XX.ravel(), YY.ravel()], axis=1)  # (N_cells, 2)


SAR_CELLS = build_sar_grid()


def build_sar_prior(lkp):
    """Gaussian prior centred on lkp (2D)."""
    dist2 = np.sum((SAR_CELLS - lkp) ** 2, axis=1)
    log_p = -dist2 / (2.0 * SIGMA_PRIOR_SAR ** 2)
    log_p -= log_p.max()
    p = np.exp(log_p)
    return p / p.sum()


def bayes_update_sar(belief, drone_xy, hit):
    """Bayesian update for one binary observation."""
    dist = np.linalg.norm(SAR_CELLS - drone_xy, axis=1)
    in_fp = dist <= SENSOR_RADIUS
    likelihood = np.where(in_fp,
                          P_DETECT if hit else 1.0 - P_DETECT,
                          P_FALSE_ALARM if hit else 1.0 - P_FALSE_ALARM)
    posterior = likelihood * belief
    total = posterior.sum()
    if total < 1e-300:
        return belief
    return posterior / total


# ── Top-level mission loop ────────────────────────────────────────────────────

def run_mission(seed=42):
    rng = np.random.default_rng(seed)

    # ── Drone initial positions ────────────────────────────────────────────────
    pos = np.array([
        [50.0, 90.0,  2.0],   # Drone A — interceptor
        [45.0, 92.0,  2.0],   # Drone B — courier 1
        [55.0, 92.0,  2.0],   # Drone C — courier 2
        [50.0, 88.0,  2.0],   # Drone D — scout
        [50.0, 86.0, 15.0],   # Drone E — formation lead (holding orbit)
    ])
    vel = np.zeros((5, 3))

    # Rogue target starts at arena corner, flies perpendicular to LOS
    target_pos = np.array([10.0, 10.0, 10.0])

    # True survivor positions (drawn from prior)
    survivors_true = np.array([
        SAR_CELLS[rng.choice(len(SAR_CELLS),
                             p=build_sar_prior(LKP_SURVIVOR[0]))],
        SAR_CELLS[rng.choice(len(SAR_CELLS),
                             p=build_sar_prior(LKP_SURVIVOR[1]))],
    ])

    # ── SAR state ─────────────────────────────────────────────────────────────
    sar_beliefs = [build_sar_prior(LKP_SURVIVOR[0]),
                   build_sar_prior(LKP_SURVIVOR[1])]
    survivor_found = [False, False]

    # ── Phase 2 state ─────────────────────────────────────────────────────────
    energy = np.array([1.0, 1.0, 1.0, 1.0, 1.0])  # per drone
    delivered = [False, False, False]               # per waypoint
    courier_waypoints = {1: None, 2: None}          # Drone B, C current wp

    def assign_tsp():
        """Nearest-neighbour TSP split between couriers B and C."""
        remaining = list(range(3))
        routes = {1: [], 2: []}
        positions_bc = {1: pos[1, :2].copy(), 2: pos[2, :2].copy()}
        drone_turn = 1  # alternate between B and C
        while remaining:
            best_wp, best_dist, best_drone = None, np.inf, None
            for wp_idx in remaining:
                wp_xy = DELIVERY_WPS[wp_idx, :2]
                for d in [1, 2]:
                    dist = np.linalg.norm(positions_bc[d] - wp_xy)
                    if dist < best_dist:
                        best_dist, best_wp, best_drone = dist, wp_idx, d
            routes[best_drone].append(best_wp)
            positions_bc[best_drone] = DELIVERY_WPS[best_wp, :2].copy()
            remaining.remove(best_wp)
        return routes

    # ── Phase 4 state ─────────────────────────────────────────────────────────
    star_targets = star_formation_targets()
    p4_assignment = None
    pid_integral = np.zeros((5, 3))
    pid_prev_err = np.zeros((5, 3))

    # ── Logging ───────────────────────────────────────────────────────────────
    t_log           = []
    pos_log         = []
    target_log      = []
    energy_log      = []
    sar_entropy_log = []
    phase_log       = []
    phase_complete  = {1: False, 2: False, 3: False, 4: False}
    phase_end_t     = {1: None,  2: None,  3: None,  4: None}
    deliveries_made = 0

    # ── Mission FSM ───────────────────────────────────────────────────────────
    state = Phase.PHASE1
    tsp_routes = assign_tsp()
    route_ptr = {1: 0, 2: 0}   # current waypoint index in each courier's route

    t = 0.0
    while t <= T_TOTAL + 30.0 and state != Phase.DONE:

        # ── Phase 1: PNG intercept ─────────────────────────────────────────────
        if state == Phase.PHASE1:
            r_vec = target_pos - pos[0]
            R_range = np.linalg.norm(r_vec)

            if R_range <= R_CAPTURE:
                phase_complete[1] = True
                phase_end_t[1] = t
                state = Phase.PHASE2
            elif t >= T_PHASE[1]:
                phase_end_t[1] = t
                state = Phase.PHASE2
            else:
                if R_range > 1e-3:
                    lam_hat = r_vec / R_range
                    rdot = (vel[0] - np.zeros(3))  # evader vel subtracted below
                    rdot_scalar = np.dot(rdot, lam_hat)
                    v_close = -rdot_scalar
                    lam_dot = (rdot - rdot_scalar * lam_hat) / R_range
                    acc_cmd = N_PNG * max(v_close, 0.5) * lam_dot
                    vel[0] = lam_hat * V_INTERCEPTOR + acc_cmd * DT
                    spd = np.linalg.norm(vel[0])
                    if spd > V_INTERCEPTOR:
                        vel[0] = vel[0] / spd * V_INTERCEPTOR
                pos[0] += vel[0] * DT
                pos[0] = np.clip(pos[0],
                                 [0, 0, 1],
                                 [ARENA_X, ARENA_Y, ARENA_Z])
                energy[0] -= C_FLY * np.linalg.norm(vel[0]) * DT

                # Evasion: fly perpendicular to LOS
                r_vec_new = target_pos - pos[0]
                R_new = np.linalg.norm(r_vec_new)
                if R_new > 1e-3:
                    lam_hat_new = r_vec_new / R_new
                    perp = np.array([-lam_hat_new[1], lam_hat_new[0], 0.0])
                    if np.linalg.norm(perp) > 1e-3:
                        perp = perp / np.linalg.norm(perp)
                    target_pos += perp * V_EVADER * DT
                    target_pos = np.clip(target_pos,
                                         [5, 5, 5],
                                         [ARENA_X - 5, ARENA_Y - 5, 20])

        # ── Phase 2: Logistics delivery ────────────────────────────────────────
        if state == Phase.PHASE2:
            for drone_id in [1, 2]:
                route = tsp_routes[drone_id]
                ptr = route_ptr[drone_id]

                if ptr >= len(route):
                    # All assigned deliveries done — return to base
                    home_vec = BASE_POS - pos[drone_id]
                    home_dist = np.linalg.norm(home_vec)
                    if home_dist < 1.0:
                        continue  # landed
                    step = V_COURIER * DT
                    pos[drone_id] += home_vec / home_dist * min(step, home_dist)
                    energy[drone_id] -= C_FLY * min(step, home_dist)
                    continue

                wp_idx = route[ptr]
                wp = DELIVERY_WPS[wp_idx]
                direction = wp - pos[drone_id]
                dist_to_wp = np.linalg.norm(direction)

                # Battery check before committing to next leg
                d_home = np.linalg.norm(BASE_POS - pos[drone_id])
                if (energy[drone_id] - C_FLY * dist_to_wp - C_FLY * d_home
                        < E_RESERVE):
                    # Abort — return to base
                    route_ptr[drone_id] = len(route)
                    continue

                if dist_to_wp < 1.0:
                    # Arrived — deliver
                    if not delivered[wp_idx]:
                        delivered[wp_idx] = True
                        deliveries_made += 1
                    energy[drone_id] -= C_HOVER * TAU_DELIVER
                    route_ptr[drone_id] += 1
                else:
                    step = V_COURIER * DT
                    pos[drone_id] += direction / dist_to_wp * min(step, dist_to_wp)
                    energy[drone_id] -= C_FLY * min(step, dist_to_wp)

            # Phase 2 completion check
            all_couriers_done = (
                route_ptr[1] >= len(tsp_routes[1]) and
                route_ptr[2] >= len(tsp_routes[2])
            )
            if all_couriers_done or t >= T_PHASE[2]:
                if not phase_end_t[2]:
                    phase_complete[2] = (deliveries_made == 3 and
                                         t <= T_PHASE[2])
                    phase_end_t[2] = t
                    state = Phase.PHASE3

        # ── Phase 3: Bayesian SAR ──────────────────────────────────────────────
        if state == Phase.PHASE3:
            drone_xy = pos[3, :2]

            # Observe each unlocated survivor
            for s in range(2):
                if survivor_found[s]:
                    continue
                dist_to_true = np.linalg.norm(drone_xy - survivors_true[s])
                in_fp = dist_to_true <= SENSOR_RADIUS
                hit = rng.random() < (P_DETECT if in_fp else P_FALSE_ALARM)
                sar_beliefs[s] = bayes_update_sar(
                    sar_beliefs[s], drone_xy, hit)
                if hit and in_fp:
                    survivor_found[s] = True

            # Greedy waypoint: maximise combined belief
            combined = (
                sar_beliefs[0] * (0 if survivor_found[0] else 1) +
                sar_beliefs[1] * (0 if survivor_found[1] else 1)
            )
            if combined.sum() > 0:
                best_idx = np.argmax(combined)
                wp_xy = SAR_CELLS[best_idx]
                wp_3d = np.array([wp_xy[0], wp_xy[1], SCOUT_HEIGHT])
            else:
                wp_3d = pos[3].copy()

            direction = wp_3d - pos[3]
            dist_wp = np.linalg.norm(direction)
            if dist_wp > 0.1:
                step = V_SCOUT * DT
                pos[3] += direction / dist_wp * min(step, dist_wp)
            energy[3] -= C_FLY * V_SCOUT * DT

            # Phase 3 completion
            if all(survivor_found) or t >= T_PHASE[3]:
                if not phase_end_t[3]:
                    phase_complete[3] = all(survivor_found) and t <= T_PHASE[3]
                    phase_end_t[3] = t
                    state = Phase.PHASE4

        # ── Phase 4: Star formation ────────────────────────────────────────────
        if state == Phase.PHASE4:
            if p4_assignment is None:
                p4_assignment = hungarian_assign(pos, star_targets)

            assigned_targets = star_targets[p4_assignment]
            max_err = 0.0
            for k in range(5):
                err = assigned_targets[k] - pos[k]
                pid_integral[k] += err * DT
                deriv = (err - pid_prev_err[k]) / DT
                u = KP * err + KI * pid_integral[k] + KD * deriv
                F_rep = orca_repulsion(pos, k)
                u_total = u + F_rep
                spd = np.linalg.norm(u_total)
                if spd > V_FORM:
                    u_total = u_total / spd * V_FORM
                pos[k] += u_total * DT
                pid_prev_err[k] = err
                energy[k] -= C_FLY * np.linalg.norm(u_total) * DT
                max_err = max(max_err, np.linalg.norm(err))

            if max_err <= CONV_THRESH or t >= T_TOTAL + 30.0:
                phase_complete[4] = max_err <= CONV_THRESH and t <= T_TOTAL
                phase_end_t[4] = t
                state = Phase.DONE

        # ── Logging ───────────────────────────────────────────────────────────
        combined_sar = (sar_beliefs[0] * (not survivor_found[0]) +
                        sar_beliefs[1] * (not survivor_found[1]))
        norm_sar = combined_sar.sum()
        if norm_sar > 0:
            p = combined_sar / norm_sar
            mask = p > 0
            entropy_sar = -np.sum(p[mask] * np.log(p[mask]))
        else:
            entropy_sar = 0.0

        t_log.append(t)
        pos_log.append(pos.copy())
        target_log.append(target_pos.copy())
        energy_log.append(energy.copy())
        sar_entropy_log.append(entropy_sar)
        phase_log.append(state)

        t += DT

    # ── Mission score ──────────────────────────────────────────────────────────
    t_end = phase_end_t[4] if phase_end_t[4] else T_TOTAL + 30.0
    score = sum(PHASE_WEIGHTS[p - 1] * phase_complete[p] for p in range(1, 5))
    score -= OT_PENALTY * max(0.0, t_end - T_TOTAL)

    return {
        "t_log":            np.array(t_log),
        "pos_log":          np.array(pos_log),       # (steps, 5, 3)
        "target_log":       np.array(target_log),    # (steps, 3)
        "energy_log":       np.array(energy_log),    # (steps, 5)
        "sar_entropy_log":  np.array(sar_entropy_log),
        "phase_log":        phase_log,
        "phase_complete":   phase_complete,
        "phase_end_t":      phase_end_t,
        "deliveries_made":  deliveries_made,
        "survivor_found":   survivor_found,
        "sar_beliefs":      sar_beliefs,
        "SAR_CELLS":        SAR_CELLS,
        "star_targets":     star_targets,
        "p4_assignment":    p4_assignment,
        "score":            score,
    }


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_phase1(res):
    """3D trajectory of interceptor and rogue target; LOS distance vs time."""
    fig = plt.figure(figsize=(14, 5))
    ax1 = fig.add_subplot(121, projection='3d')
    pos_log    = res["pos_log"]
    tgt_log    = res["target_log"]
    t_log      = res["t_log"]
    t_end_idx  = next((i for i, ph in enumerate(res["phase_log"])
                       if ph != Phase.PHASE1), len(t_log))

    ax1.plot(pos_log[:t_end_idx, 0, 0], pos_log[:t_end_idx, 0, 1],
             pos_log[:t_end_idx, 0, 2], 'r-', lw=1.5, label='Interceptor (A)')
    ax1.plot(tgt_log[:t_end_idx, 0], tgt_log[:t_end_idx, 1],
             tgt_log[:t_end_idx, 2], 'b--', lw=1.5, label='Rogue target')
    ax1.scatter(*pos_log[0, 0], s=60, c='red', marker='o')
    ax1.scatter(*tgt_log[0], s=60, c='blue', marker='s')
    ax1.scatter(*pos_log[t_end_idx - 1, 0], s=80, c='red', marker='^')
    ax1.set_xlabel('x (m)'); ax1.set_ylabel('y (m)'); ax1.set_zlabel('z (m)')
    ax1.set_title('Phase 1 — PNG Intercept Trajectory')
    ax1.legend(fontsize=8)

    ax2 = fig.add_subplot(122)
    ranges = np.linalg.norm(
        pos_log[:t_end_idx, 0] - tgt_log[:t_end_idx], axis=1)
    ax2.plot(t_log[:t_end_idx], ranges, 'k-', lw=1.5)
    ax2.axhline(R_CAPTURE, color='red', ls='--', label=f'Capture radius {R_CAPTURE} m')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('LOS range (m)')
    ax2.set_title('Phase 1 — LOS Range vs Time')
    ax2.legend(); ax2.grid(True, ls='--', alpha=0.4)

    plt.suptitle('S100 Phase 1: Proportional Navigation Guidance Intercept',
                 fontweight='bold')
    plt.tight_layout()
    return fig


def plot_phase2(res):
    """Courier trajectories + energy profiles."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    pos_log = res["pos_log"]
    t_log   = res["t_log"]
    colors  = ['tab:orange', 'tab:purple']
    labels  = ['Drone B (Courier 1)', 'Drone C (Courier 2)']
    p2_mask = [ph in (Phase.PHASE2,) for ph in res["phase_log"]]

    ax = axes[0]
    for k, (c, lbl) in enumerate(zip(colors, labels)):
        idx = k + 1
        ax.plot(pos_log[:, idx, 0], pos_log[:, idx, 1],
                color=c, lw=1.2, label=lbl)
    for i, wp in enumerate(DELIVERY_WPS):
        status = 'Delivered' if res["deliveries_made"] > i else 'Missed'
        ax.scatter(wp[0], wp[1], s=120, marker='D', zorder=5,
                   c='green' if 'Delivered' in status else 'gray')
        ax.annotate(f'WP{i+1}', wp[:2] + np.array([1, 1]), fontsize=8)
    ax.scatter(*BASE_POS[:2], s=120, marker='H', c='black', zorder=6,
               label='Base')
    ax.set_xlim(0, ARENA_X); ax.set_ylim(0, ARENA_Y)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title('Phase 2 — Courier Routes (top-down)')
    ax.legend(fontsize=8); ax.grid(True, ls='--', alpha=0.3)

    ax2 = axes[1]
    t_arr = t_log
    for k, (c, lbl) in enumerate(zip(colors, labels)):
        ax2.plot(t_arr, res["energy_log"][:, k + 1] * 100,
                 color=c, lw=1.5, label=lbl)
    ax2.axhline(E_RESERVE * 100, color='red', ls='--',
                label=f'Safety reserve {E_RESERVE*100:.0f}%')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('State of charge (%)')
    ax2.set_title('Phase 2 — Battery Profile')
    ax2.legend(fontsize=8); ax2.grid(True, ls='--', alpha=0.4)

    plt.suptitle(f'S100 Phase 2: Logistics Delivery — '
                 f'{res["deliveries_made"]}/3 packages delivered',
                 fontweight='bold')
    plt.tight_layout()
    return fig


def plot_phase3(res):
    """SAR belief map snapshots + entropy decay."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    cells = res["SAR_CELLS"]
    grid  = res["sar_beliefs"][0].reshape(GRID_N, GRID_N)
    extent = [SAR_ZONE_LO[0], SAR_ZONE_HI[0], SAR_ZONE_LO[1], SAR_ZONE_HI[1]]

    for ax, (s_idx, title) in zip(axes[:2],
        [(0, 'Survivor 1 — Final Belief'),
         (1, 'Survivor 2 — Final Belief')]):
        belief_grid = res["sar_beliefs"][s_idx].reshape(GRID_N, GRID_N)
        im = ax.imshow(belief_grid.T, origin='lower', extent=extent,
                       cmap='hot_r', interpolation='bilinear')
        plt.colorbar(im, ax=ax, label='P(survivor at cell)')
        ax.scatter(*LKP_SURVIVOR[s_idx], s=80, c='cyan', marker='x',
                   lw=2, label='LKP')
        found = 'Located' if res["survivor_found"][s_idx] else 'Not found'
        ax.set_title(f'{title}\n({found})')
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.legend(fontsize=8)

    ax3 = axes[2]
    t_log = res["t_log"]
    ax3.plot(t_log, res["sar_entropy_log"], 'k-', lw=1.5)
    ax3.set_xlabel('Mission time (s)')
    ax3.set_ylabel('Joint SAR entropy (nats)')
    ax3.set_title('Phase 3 — Bayesian Belief Entropy Decay')
    ax3.grid(True, ls='--', alpha=0.4)

    plt.suptitle('S100 Phase 3: Search and Rescue — Bayesian Belief Maps',
                 fontweight='bold')
    plt.tight_layout()
    return fig


def plot_phase4(res):
    """3D star formation: trajectories + final formation + convergence."""
    fig = plt.figure(figsize=(14, 6))
    pos_log = res["pos_log"]
    t_log   = res["t_log"]
    star_tgt = res["star_targets"]
    assignment = res["p4_assignment"]
    p4_start = next((i for i, ph in enumerate(res["phase_log"])
                     if ph == Phase.PHASE4), None)

    ax1 = fig.add_subplot(121, projection='3d')
    colours = ['red', 'orange', 'purple', 'cyan', 'gold']
    labels  = [f'Drone {c}' for c in 'ABCDE']
    if p4_start is not None:
        for k in range(5):
            ax1.plot(pos_log[p4_start:, k, 0],
                     pos_log[p4_start:, k, 1],
                     pos_log[p4_start:, k, 2],
                     color=colours[k], lw=1.2, label=labels[k])
    ax1.scatter(star_tgt[:, 0], star_tgt[:, 1], star_tgt[:, 2],
                s=120, c='lime', marker='*', zorder=6, label='Star targets')
    ax1.set_xlabel('x (m)'); ax1.set_ylabel('y (m)'); ax1.set_zlabel('z (m)')
    ax1.set_title('Phase 4 — Formation Flight to Star')
    ax1.legend(fontsize=7)

    ax2 = fig.add_subplot(122)
    if p4_start is not None:
        p4_pos = pos_log[p4_start:]
        if assignment is not None:
            assigned_tgt = star_tgt[assignment]
            errs = np.linalg.norm(p4_pos - assigned_tgt[np.newaxis], axis=2)
            for k in range(5):
                ax2.plot(t_log[p4_start: p4_start + len(p4_pos)],
                         errs[:, k], color=colours[k], lw=1.2,
                         label=labels[k])
    ax2.axhline(CONV_THRESH, color='k', ls='--',
                label=f'Threshold {CONV_THRESH} m')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Position error (m)')
    ax2.set_title('Phase 4 — Per-Drone Convergence')
    ax2.legend(fontsize=7); ax2.grid(True, ls='--', alpha=0.4)

    plt.suptitle('S100 Phase 4: Star Formation — Hungarian + ORCA',
                 fontweight='bold')
    plt.tight_layout()
    return fig


def plot_mission_dashboard(res):
    """Grand summary dashboard: score, phase timeline, energy, phase metrics."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))

    # ── Top-left: phase timeline (Gantt) ──────────────────────────────────────
    ax = axes[0, 0]
    phase_names  = ['Phase 1\nPursuit', 'Phase 2\nLogistics',
                    'Phase 3\nSAR', 'Phase 4\nFormation']
    phase_starts = [T_PHASE[0], T_PHASE[1], T_PHASE[2], T_PHASE[3]]
    phase_budgets= [T_PHASE[1] - T_PHASE[0],
                    T_PHASE[2] - T_PHASE[1],
                    T_PHASE[3] - T_PHASE[2],
                    T_PHASE[4] - T_PHASE[3]]
    bar_colours  = ['#d62728', '#ff7f0e', '#2ca02c', '#1f77b4']

    for i, (name, start, budget, col) in enumerate(
            zip(phase_names, phase_starts, phase_budgets, bar_colours)):
        ax.barh(i, budget, left=start, color=col, alpha=0.7,
                edgecolor='black', label=name)
        t_end = res["phase_end_t"].get(i + 1)
        if t_end:
            ax.plot(t_end, i, 'k|', ms=12, mew=2.5)
            success = res["phase_complete"].get(i + 1, False)
            ax.text(t_end + 1, i, ('✓' if success else '✗'), va='center',
                    fontsize=11, color='green' if success else 'red')

    ax.set_yticks(range(4)); ax.set_yticklabels(phase_names)
    ax.set_xlabel('Mission time (s)'); ax.set_title('Phase Timeline (Gantt)')
    ax.axvline(T_TOTAL, color='k', ls='--', lw=1.5, label='Budget limit')
    ax.grid(True, axis='x', ls='--', alpha=0.3)

    # ── Top-right: overall energy profiles ────────────────────────────────────
    ax2 = axes[0, 1]
    t_log = res["t_log"]
    for k in range(5):
        ax2.plot(t_log, res["energy_log"][:, k] * 100,
                 label=f'Drone {chr(65+k)}')
    ax2.axhline(E_RESERVE * 100, color='red', ls='--',
                label=f'Reserve {E_RESERVE*100:.0f}%')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('SoC (%)')
    ax2.set_title('All-Drone Battery State of Charge')
    ax2.legend(fontsize=8); ax2.grid(True, ls='--', alpha=0.3)

    # ── Bottom-left: mission score breakdown ───────────────────────────────────
    ax3 = axes[1, 0]
    scores_per_phase = [PHASE_WEIGHTS[p - 1] * res["phase_complete"].get(p, 0)
                        for p in range(1, 5)]
    phase_labels = ['P1 Pursuit\n(w=2)', 'P2 Logistics\n(w=3)',
                    'P3 SAR\n(w=3)', 'P4 Formation\n(w=2)']
    bar_c = ['green' if s > 0 else 'lightgray' for s in scores_per_phase]
    ax3.bar(phase_labels, scores_per_phase, color=bar_c, edgecolor='black')
    ax3.axhline(res["score"], color='navy', ls='--', lw=2,
                label=f'Total score: {res["score"]:.2f} / 10')
    ax3.set_ylabel('Points earned'); ax3.set_title('Mission Score Breakdown')
    ax3.legend(fontsize=9); ax3.set_ylim(0, 4)
    ax3.grid(True, axis='y', ls='--', alpha=0.3)

    # ── Bottom-right: SAR entropy decay ───────────────────────────────────────
    ax4 = axes[1, 1]
    ax4.plot(t_log, res["sar_entropy_log"], 'teal', lw=1.5)
    ax4.set_xlabel('Mission time (s)')
    ax4.set_ylabel('SAR belief entropy (nats)')
    ax4.set_title('SAR Entropy Decay Over Full Mission')
    ax4.grid(True, ls='--', alpha=0.3)

    plt.suptitle(
        f'S100 Ultimate Grand Challenge — Mission Dashboard\n'
        f'Score: {res["score"]:.2f}/10  |  '
        f'Deliveries: {res["deliveries_made"]}/3  |  '
        f'Survivors: {sum(res["survivor_found"])}/2  |  '
        f'T_end: {res["phase_end_t"].get(4, "N/A")} s',
        fontsize=12, fontweight='bold')
    plt.tight_layout()
    return fig


def make_grand_animation(res, filename, fps=15, stride=5):
    """
    Grand 3D animation showing all 5 drones across the full mission timeline.
    Rogue target shown in blue during Phase 1. Phase label annotated per frame.
    """
    pos_log  = res["pos_log"]
    tgt_log  = res["target_log"]
    t_log    = res["t_log"]
    phase_log= res["phase_log"]
    n_frames = len(range(0, len(t_log), stride))

    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')
    colours = ['red', 'orange', 'purple', 'cyan', 'gold']
    labels  = [f'Drone {c}' for c in 'ABCDE']

    drone_lines  = [ax.plot([], [], [], 'o', ms=7, color=c)[0]
                    for c in colours]
    target_dot,  = ax.plot([], [], [], 'bs', ms=8)
    star_scatter = ax.scatter([], [], [], s=100, c='lime', marker='*')

    # Star targets
    star_tgt = res["star_targets"]

    time_text = ax.text2D(0.02, 0.96, '', transform=ax.transAxes, fontsize=9)
    phase_text= ax.text2D(0.02, 0.91, '', transform=ax.transAxes, fontsize=9,
                           color='navy')

    ax.set_xlim(0, ARENA_X)
    ax.set_ylim(0, ARENA_Y)
    ax.set_zlim(0, SHOW_ALT + 10)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_title('S100 — Ultimate Grand Challenge')

    def update(frame_idx):
        f   = frame_idx * stride
        f   = min(f, len(t_log) - 1)
        pos = pos_log[f]
        tgt = tgt_log[f]
        ph  = phase_log[f]
        t   = t_log[f]

        for k, ln in enumerate(drone_lines):
            ln.set_data([pos[k, 0]], [pos[k, 1]])
            ln.set_3d_properties([pos[k, 2]])

        if ph == Phase.PHASE1:
            target_dot.set_data([tgt[0]], [tgt[1]])
            target_dot.set_3d_properties([tgt[2]])
        else:
            target_dot.set_data([], [])
            target_dot.set_3d_properties([])

        if ph == Phase.PHASE4:
            star_scatter._offsets3d = (star_tgt[:, 0],
                                       star_tgt[:, 1],
                                       star_tgt[:, 2])
        time_text.set_text(f't = {t:.1f} s')
        phase_text.set_text(str(ph.name))
        return drone_lines + [target_dot, time_text, phase_text]

    ani = animation.FuncAnimation(
        fig, update, frames=n_frames,
        interval=int(1000 / fps), blit=False)
    ani.save(filename, writer='pillow', fps=fps)
    plt.close(fig)
    print(f"Grand animation saved → {filename}")
    return ani


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import os
    out_dir = "outputs/05_special_entertainment/s100_grand_challenge"
    os.makedirs(out_dir, exist_ok=True)

    res = run_mission(seed=42)

    print("\n=== S100 Mission Report ===")
    for p in range(1, 5):
        status = "COMPLETE" if res["phase_complete"][p] else "FAILED"
        t_end  = res["phase_end_t"][p]
        t_str  = f"{t_end:.1f} s" if t_end else "N/A"
        print(f"  Phase {p}: {status}  (ended at {t_str})")
    print(f"  Deliveries : {res['deliveries_made']}/3")
    print(f"  Survivors  : {sum(res['survivor_found'])}/2")
    print(f"  Score      : {res['score']:.2f} / 10")
    print("===========================\n")

    fig1 = plot_phase1(res)
    fig1.savefig(f"{out_dir}/s100_phase1_pursuit.png", dpi=150,
                 bbox_inches="tight")
    plt.close(fig1)

    fig2 = plot_phase2(res)
    fig2.savefig(f"{out_dir}/s100_phase2_logistics.png", dpi=150,
                 bbox_inches="tight")
    plt.close(fig2)

    fig3 = plot_phase3(res)
    fig3.savefig(f"{out_dir}/s100_phase3_sar.png", dpi=150,
                 bbox_inches="tight")
    plt.close(fig3)

    fig4 = plot_phase4(res)
    fig4.savefig(f"{out_dir}/s100_phase4_formation.png", dpi=150,
                 bbox_inches="tight")
    plt.close(fig4)

    fig5 = plot_mission_dashboard(res)
    fig5.savefig(f"{out_dir}/s100_mission_dashboard.png", dpi=150,
                 bbox_inches="tight")
    plt.close(fig5)

    make_grand_animation(res, f"{out_dir}/s100_grand_challenge.gif")
    print("All outputs saved. Mission complete.")
```

---

## Key Parameters

| Parameter | Symbol | Phase | Value |
|-----------|--------|-------|-------|
| Arena dimensions | — | All | $100 \times 100 \times 30$ m |
| Total mission budget | $T_{total}$ | All | 300 s |
| Simulation timestep | $\Delta t$ | All | 0.1 s |
| Communication range | $r_{comm}$ | All | 80 m |
| Number of drones | $N$ | All | 5 |
| **Phase 1 — Pursuit** | | | |
| PNG navigation constant | $N$ | 1 | 3 |
| Interceptor cruise speed | $V_A$ | 1 | 8.0 m/s |
| Evader speed | $V_{evader}$ | 1 | 6.0 m/s |
| Capture radius | $r_{capture}$ | 1 | 1.5 m |
| Seeker sensor range | $r_{seek}$ | 1 | 20.0 m |
| Phase 1 time budget | $T_1$ | 1 | 60 s |
| **Phase 2 — Logistics** | | | |
| Courier cruise speed | $V_{B,C}$ | 2 | 5.0 m/s |
| Energy consumption rate | $c_{fly}$ | 2 | 0.004 s$^{-1}$m$^{-1}$ |
| Hover energy rate | $c_{hover}$ | 2 | 0.003 s$^{-1}$ |
| Delivery dwell time | $\tau_{deliver}$ | 2 | 3.0 s |
| Battery safety reserve | $E_{reserve}$ | 2 | 15 % |
| Number of delivery waypoints | $M$ | 2 | 3 |
| Phase 2 time budget | $T_2$ | 2 | 100 s |
| **Phase 3 — SAR** | | | |
| SAR search zone | — | 3 | $50 \times 50$ m (south half) |
| SAR grid resolution | — | 3 | 0.5 m/cell ($100 \times 100$) |
| Scout cruise speed | $V_D$ | 3 | 4.0 m/s |
| Sensor footprint radius | $r_s$ | 3 | 3.0 m |
| Scout scan altitude | $h$ | 3 | 8.0 m AGL |
| Detection probability | $P_d$ | 3 | 0.85 |
| False-alarm probability | $P_{fa}$ | 3 | 0.01 |
| Prior spread (each survivor) | $\sigma_0$ | 3 | 8.0 m |
| Detection belief threshold | $\tau_{det}$ | 3 | 0.60 |
| Number of survivors | — | 3 | 2 |
| Phase 3 time budget | $T_3$ | 3 | 80 s |
| **Phase 4 — Formation** | | | |
| Star formation altitude | $z_{show}$ | 4 | 50 m |
| Star circumradius | $R_{star}$ | 4 | 8 m |
| Formation cruise speed | $V_{form}$ | 4 | 6.0 m/s |
| PID gains $(K_p, K_i, K_d)$ | — | 4 | (1.2, 0.05, 0.4) |
| ORCA repulsion gain | $k_{rep}$ | 4 | 2.0 |
| Repulsion activation radius | $r_{safe}$ | 4 | 1.2 m |
| Convergence threshold | $\varepsilon_{conv}$ | 4 | 0.15 m |
| Phase 4 time budget | $T_4$ | 4 | 60 s |
| **Scoring** | | | |
| Phase weights | $(w_1,w_2,w_3,w_4)$ | All | (2, 3, 3, 2) |
| Overtime penalty | $\lambda_{OT}$ | All | 0.05 pts/s |
| Maximum score | $S_{max}$ | All | 10 pts |

---

## Expected Output

- **Phase 1 figure** (`s100_phase1_pursuit.png`): two-panel figure. Left: 3D trajectory plot
  of Drone A (red) and the rogue target (blue dashed) from mission start until intercept or Phase 1
  timeout; start positions marked as spheres, capture event marked as a red star; arena boundary
  box drawn in grey. Right: LOS range $R(t)$ vs time in seconds; capture radius $r_{capture} =
  1.5$ m shown as a red dashed horizontal line; the moment of intercept (if achieved) annotated
  with a vertical marker and timestamp.

- **Phase 2 figure** (`s100_phase2_logistics.png`): two-panel figure. Left: top-down ($x$–$y$)
  view of courier routes for Drones B (orange) and C (purple); three delivery waypoints shown as
  green diamonds (delivered) or grey diamonds (missed); base marked as a black hexagon; route
  segments annotated with delivery order. Right: battery state-of-charge vs mission time for both
  couriers; safety reserve at 15 % marked as a red dashed line; delivery completion events marked
  as vertical tick marks.

- **Phase 3 figure** (`s100_phase3_sar.png`): three-panel figure. Left and centre: final Bayesian
  belief map heatmaps (one per survivor) over the $50 \times 50$ m SAR zone; colour scale hot_r
  from white (low) to dark red (high belief); last-known-position marked with cyan cross; survivor
  located/not-found status annotated in title. Right: joint SAR belief entropy $H_{joint}(t)$ vs
  mission time over the full 300 s timeline; Phase 3 active window shaded in blue; detection events
  marked with vertical dashed lines.

- **Phase 4 figure** (`s100_phase4_formation.png`): two-panel figure. Left: 3D trajectory plot of
  all five drones converging from their Phase 3 end positions to the 5-pointed star at $z = 50$ m;
  each drone coloured distinctly (red/orange/purple/cyan/gold); star target vertices shown as lime
  green stars; Hungarian assignment lines drawn as dashed connectors from initial positions to
  assigned targets. Right: per-drone position error vs time from Phase 4 start; convergence
  threshold $\varepsilon_{conv} = 0.15$ m shown as black dashed line; $T_{conv}$ (if achieved)
  marked with a green vertical marker.

- **Mission dashboard** (`s100_mission_dashboard.png`): four-panel summary figure.
  Top-left: Gantt chart of all four phases on a 0–300 s timeline; budget windows shown as coloured
  horizontal bars; actual phase completion timestamps marked as black tick marks; success/failure
  icons annotated per phase. Top-right: all-drone battery state-of-charge vs time on one axes;
  safety reserve line; annotated phase transitions. Bottom-left: bar chart of points scored per
  phase against maximum possible (coloured green if achieved, grey if missed); total score displayed
  as a navy dashed horizontal line. Bottom-right: joint SAR belief entropy over the full mission
  timeline.

- **Grand animation** (`s100_grand_challenge.gif`): full-mission 3D animation at 15 fps, showing
  all five drones as coloured spheres moving through the arena. During Phase 1 the rogue target is
  shown as a blue square; during Phase 4 the star target vertices are shown as lime green stars.
  Current mission time and active phase state label are annotated per frame. Duration covers 0–300 s
  (or until mission completion), with stride 5 (every 0.5 s rendered).

- **Console mission report**:
  - Per-phase status: COMPLETE or FAILED, with actual completion timestamp
  - Deliveries made: $n/3$ packages
  - Survivors located: $n/2$
  - Final mission score: $S_{mission}$ out of 10 points

---

## Extensions

1. **Adversarial evader with learned policy**: replace the constant-bearing evasion strategy in
   Phase 1 with a reinforcement-learning agent trained via self-play (MADDPG or PPO); measure the
   intercept success rate of the PNG pursuer against increasingly capable evaders as a function of
   the speed ratio $V_A / V_{evader}$, and derive an empirical capture boundary in speed-ratio vs
   arena-size space.

2. **Online re-planning under drone failure**: introduce a random hardware failure event (battery
   depletion or motor fault) for one drone between Phase 1 and Phase 3; the mission sequencer must
   detect the failure via heartbeat loss, re-allocate its tasks to surviving drones (reassign
   deliveries via a new TSP solve; drop a survivor search leg; reduce the formation to a 4-drone
   square), and re-score the mission with a reduced maximum — testing the resilience of the
   multi-phase architecture to single-point failures.

3. **Full 3D physics with wind and turbulence**: replace the point-mass kinematics in all four
   phases with a 6-DOF quadrotor model (inertia tensor, rotor thrust limits, aerodynamic drag)
   driven by a Dryden turbulence model at each altitude band; tune phase-specific controllers (PNG
   in Phase 1, PID in Phase 4) to reject wind disturbances, and measure score degradation as a
   function of wind speed $\|w\| \in \{0, 3, 6, 10\}$ m/s — establishing the operational wind
   envelope for a perfect-score ($S = 10$) mission.

---

## Related Scenarios

- **Domain 1 — Pursuit**: [S001 Basic Intercept](../01_pursuit_evasion/S001_basic_intercept.md)
  (PNG foundations), [S002 Evasive Manoeuvre](../01_pursuit_evasion/S002_evasive_maneuver.md)
  (constant-bearing evasion model)
- **Domain 2 — Logistics**: [S021 Point Delivery](../02_logistics_delivery/S021_point_delivery.md)
  (waypoint-following fundamentals), [S068 Large-Field Spray](../04_industrial_agriculture/S068_large_field_spray.md)
  (multi-agent coverage with battery management)
- **Domain 3 — SAR**: [S042 Missing Person Localisation](../03_environmental_sar/S042_missing_person.md)
  (Bayesian belief map and greedy frontier policy)
- **Domain 4 — Industrial**: [S066 Cooperative Crane](../04_industrial_agriculture/S066_cooperative_crane.md)
  (multi-agent coordination with shared task state)
- **Domain 5 — Formation**: [S085 Light Matrix](S085_light_matrix.md)
  (Hungarian assignment + ORCA collision avoidance, domain constants)
