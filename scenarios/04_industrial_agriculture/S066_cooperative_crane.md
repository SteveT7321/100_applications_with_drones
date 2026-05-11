# S066 Cooperative Crane Simulation

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Rigid Load Dynamics + Distributed Tension Allocation | **Dimension**: 3D

---

## Problem Definition

**Setup**: A rigid steel beam (mass $m_L = 2$ kg, length $L = 2$ m) must be lifted and transported
horizontally from point $A = (0, 0, 2)$ m to point $B = (15, 0, 2)$ m at a constant cruise speed
$v = 0.5$ m/s. Three quadrotor drones cooperate to carry the beam: each drone $i \in \{1, 2, 3\}$
is connected to a fixed anchor point on the beam via a taut cable of length $l_c = 1$ m. The
anchor points are equally spaced along the beam at offsets $(-0.75, 0, 0)$, $(0, 0, 0)$, and
$(+0.75, 0, 0)$ m from the beam's centre of mass. At all times the cables are assumed taut (tension
$T_i \geq T_{min} = 0.5$ N). Each drone is a point-mass of $m_D = 0.5$ kg actuated by a
scalar thrust $F_i$ aligned with its body $z$-axis; attitude dynamics are abstracted to a
first-order low-pass with time constant $\tau = 0.05$ s. The simulation runs at $\Delta t = 0.01$ s
for a total mission duration determined by transport completion.

**Roles**:
- **Drones** ($N = 3$): identical quadrotors positioned above the beam, each commanding a 3D
  thrust vector via a formation PID controller that tracks a desired hover offset above its
  anchor point.
- **Rigid beam**: a uniform rod treated as a 6-DOF rigid body; its translational and rotational
  state is integrated from the net wrench applied by the three cable tensions plus gravity.
- **Cable forces**: the cable from drone $i$ transmits tension only along the unit vector
  $\hat{n}_i$ from the anchor point on the beam to the drone's position; compression is forbidden
  ($T_i \geq T_{min}$).

**Objective**: Transport the beam from $A$ to $B$ while satisfying all three constraints
simultaneously:

1. **Attitude constraint**: beam pitch and roll remain within $\pm 5°$ throughout the flight
   ($|\theta_{pitch}|, |\theta_{roll}| \leq 5°$).
2. **Collision avoidance**: minimum inter-drone separation $\geq d_{min} = 0.8$ m at all times
   ($\|p_i - p_j\| \geq 0.8$ m $\forall\, i \neq j$).
3. **Trajectory tracking**: beam centre of mass follows a straight-line path at $v = 0.5$ m/s
   with position error $< 0.1$ m.

**Comparison conditions**:

1. **Centralised PID** — single controller computes all three thrust commands jointly from the
   beam error; no explicit tension allocation.
2. **Distributed tension allocation (proposed)** — each drone runs a local PID loop; a wrench
   decomposition QP distributes the desired wrench among the three cables while respecting $T_{min}$
   and minimising $\sum \|T_i\|^2$.
3. **Uncoordinated (baseline)** — each drone independently tries to hold its anchor point at a
   fixed hover offset, with no load dynamics awareness.

---

## Mathematical Model

### Beam Rigid Body State

The beam state is described by position $\mathbf{p}_L \in \mathbb{R}^3$ (centre of mass),
velocity $\dot{\mathbf{p}}_L$, orientation quaternion $\mathbf{q}_L \in \mathbb{H}$ (unit), and
angular velocity $\boldsymbol{\omega}_L \in \mathbb{R}^3$ expressed in the body frame. The inertia
tensor of a uniform rod about its centre of mass is:

$$\mathbf{I}_L = \begin{pmatrix}
\frac{1}{12} m_L L^2 & 0 & 0 \\
0 & \epsilon & 0 \\
0 & 0 & \frac{1}{12} m_L L^2
\end{pmatrix}$$

where $\epsilon \approx 0$ is a small regularisation for the axial (roll-about-beam-axis) direction.
With $m_L = 2$ kg and $L = 2$ m: $I_{yy} = I_{zz} = \tfrac{1}{12}(2)(4) = \tfrac{2}{3}$ kg m².

### Cable Geometry and Tension Vector

Let $\mathbf{a}_i$ be the world-frame position of anchor $i$ on the beam and $\mathbf{p}_i$ be the
position of drone $i$. The cable unit vector pointing from anchor to drone is:

$$\hat{n}_i = \frac{\mathbf{p}_i - \mathbf{a}_i}{\|\mathbf{p}_i - \mathbf{a}_i\|}$$

The force exerted on the beam at anchor $i$ by the cable tension $T_i \geq 0$ is:

$$\mathbf{f}_i = T_i \, \hat{n}_i$$

### Load Translational Dynamics

Newton's second law for the beam centre of mass:

$$m_L \, \ddot{\mathbf{p}}_L = \sum_{i=1}^{3} T_i \, \hat{n}_i + m_L \mathbf{g}$$

where $\mathbf{g} = (0, 0, -9.81)$ m/s². In matrix form, defining the cable direction matrix
$\mathbf{N} \in \mathbb{R}^{3 \times 3}$ with columns $\hat{n}_i$ and tension vector
$\mathbf{T} = (T_1, T_2, T_3)^\top$:

$$m_L \, \ddot{\mathbf{p}}_L = \mathbf{N} \mathbf{T} + m_L \mathbf{g}$$

### Load Rotational Dynamics

Let $\mathbf{r}_i = \mathbf{a}_i - \mathbf{p}_L$ be the moment arm from beam CoM to anchor $i$
(in world frame). The net torque about the beam CoM is:

$$\boldsymbol{\tau}_L = \sum_{i=1}^{3} \mathbf{r}_i \times (T_i \, \hat{n}_i)$$

The rotational dynamics in body frame (Euler equations):

$$\mathbf{I}_L \, \dot{\boldsymbol{\omega}}_L = \boldsymbol{\tau}_L^{body} - \boldsymbol{\omega}_L \times (\mathbf{I}_L \boldsymbol{\omega}_L)$$

The quaternion kinematics relate body angular velocity to quaternion rate:

$$\dot{\mathbf{q}}_L = \frac{1}{2} \mathbf{q}_L \otimes \begin{pmatrix} 0 \\ \boldsymbol{\omega}_L \end{pmatrix}$$

### Desired Wrench

The desired translational acceleration of the beam is commanded by a PD controller on position error:

$$\ddot{\mathbf{p}}_L^{des} = K_p (\mathbf{p}_L^{ref} - \mathbf{p}_L) + K_d (\dot{\mathbf{p}}_L^{ref} - \dot{\mathbf{p}}_L) + \ddot{\mathbf{p}}_L^{ref}$$

where $\mathbf{p}_L^{ref}(t)$ is the straight-line reference trajectory at $v = 0.5$ m/s. The
attitude stabilisation PD controller acts on roll/pitch error:

$$\dot{\boldsymbol{\omega}}_L^{des} = K_{p,\omega} \, \boldsymbol{\theta}_{err} + K_{d,\omega} \, \boldsymbol{\omega}_{err}$$

The desired wrench vector $\mathbf{W}_{des} \in \mathbb{R}^6$ concatenates the required force and
torque on the beam:

$$\mathbf{W}_{des} = \begin{pmatrix} m_L \ddot{\mathbf{p}}_L^{des} - m_L \mathbf{g} \\ \mathbf{I}_L \dot{\boldsymbol{\omega}}_L^{des} + \boldsymbol{\omega}_L \times (\mathbf{I}_L \boldsymbol{\omega}_L) \end{pmatrix}$$

### Tension Allocation via Quadratic Programme

Given the desired wrench $\mathbf{W}_{des}$, find tension magnitudes $\mathbf{T} \in \mathbb{R}^3$
that realise it. The wrench produced by the three cables is:

$$\mathbf{W}(\mathbf{T}) = \underbrace{\begin{pmatrix} \mathbf{N} \\ \mathbf{M} \end{pmatrix}}_{\mathbf{G} \,\in\, \mathbb{R}^{6\times 3}} \mathbf{T}$$

where the moment matrix $\mathbf{M} \in \mathbb{R}^{3 \times 3}$ has columns
$\mathbf{m}_i = \mathbf{r}_i \times \hat{n}_i$. The grasp matrix $\mathbf{G}$ maps tensions to
wrench. The tension allocation QP minimises tension effort subject to wrench equality and
non-negativity:

$$\min_{\mathbf{T}} \quad \mathbf{T}^\top \mathbf{T}$$

$$\text{subject to} \quad \mathbf{G} \mathbf{T} = \mathbf{W}_{des}, \qquad T_i \geq T_{min} \;\; \forall\, i$$

When $N = 3$ cables and the full 6-DOF wrench is under-determined (3 scalar unknowns, 6 equations),
the system is over-constrained in general. In practice only 5 independent wrench components are
relevant (the axial torque about the beam axis is uncontrollable). The minimum-norm pseudo-inverse
solution plus a null-space term to enforce $T_i \geq T_{min}$ is:

$$\mathbf{T}^* = \mathbf{G}^+ \mathbf{W}_{des} + (\mathbf{I} - \mathbf{G}^+ \mathbf{G}) \boldsymbol{\lambda}$$

where $\mathbf{G}^+ = \mathbf{G}^\top (\mathbf{G} \mathbf{G}^\top)^{-1}$ is the right pseudo-inverse
and $\boldsymbol{\lambda}$ is chosen to bring all tensions above $T_{min}$:

$$\boldsymbol{\lambda} = \frac{T_{min} - \min_i T^*_i}{(\mathbf{I} - \mathbf{G}^+ \mathbf{G})_{row}} \cdot \mathbf{1}$$

### Drone Position Controller

Each drone $i$ tracks a desired hover position $\mathbf{p}_i^{des}$ located $l_c = 1$ m vertically
above its anchor point (in the current beam frame). The formation PID generates a desired thrust
magnitude:

$$\mathbf{u}_i = K_{p,D} (\mathbf{p}_i^{des} - \mathbf{p}_i) + K_{d,D} (\dot{\mathbf{p}}_i^{des} - \dot{\mathbf{p}}_i)$$

$$F_i = (m_D \|\mathbf{u}_i + \mathbf{g}\|)$$

The cable tension $T_i$ applied to the beam equals the component of the drone's thrust that is
transmitted through the taut cable:

$$T_i = F_i \, (\hat{n}_i \cdot \hat{z}_{drone,i})$$

### Inter-Drone Collision Avoidance

A soft repulsion term is added to each drone's desired velocity when the inter-drone distance falls
below a safety margin $d_{safe} = 1.2$ m $> d_{min}$:

$$\mathbf{v}_{rep,ij} = \begin{cases}
k_{rep} \dfrac{d_{safe} - \|\mathbf{p}_i - \mathbf{p}_j\|}{d_{safe}} \cdot \dfrac{\mathbf{p}_i - \mathbf{p}_j}{\|\mathbf{p}_i - \mathbf{p}_j\|} & \text{if } \|\mathbf{p}_i - \mathbf{p}_j\| < d_{safe} \\
\mathbf{0} & \text{otherwise}
\end{cases}$$

### Performance Metrics

**Beam attitude error** (RMS over mission):

$$\epsilon_{att} = \sqrt{\frac{1}{T_{mission}} \int_0^{T_{mission}} \left(\theta_{pitch}^2(t) + \theta_{roll}^2(t)\right) dt}$$

**Tension balance index** (Gini-like coefficient over time):

$$\mathcal{B}(t) = 1 - \frac{\sum_{i<j} |T_i(t) - T_j(t)|}{\binom{N}{2} \cdot \bar{T}(t)}, \qquad \mathcal{B} \in [0, 1]$$

where $\bar{T}(t) = \frac{1}{N}\sum_i T_i(t)$. $\mathcal{B} = 1$ is perfectly balanced.

**Transport time** $t_{transport}$: time from departure at $A$ until beam CoM reaches within
0.05 m of $B$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation

# ── Constants ────────────────────────────────────────────────────────────────
N_DRONES    = 3
M_LOAD      = 2.0          # kg  — beam mass
L_BEAM      = 2.0          # m   — beam length
M_DRONE     = 0.5          # kg  — per-drone mass
L_CABLE     = 1.0          # m   — cable length
V_CRUISE    = 0.5          # m/s — transport speed
DT          = 0.01         # s   — timestep
T_MIN       = 0.5          # N   — minimum cable tension
D_MIN       = 0.8          # m   — hard collision avoidance threshold
D_SAFE      = 1.2          # m   — soft repulsion onset
G_VEC       = np.array([0.0, 0.0, -9.81])

# Beam inertia (uniform rod, axis along X)
I_BEAM = np.diag([1e-4, M_LOAD * L_BEAM**2 / 12, M_LOAD * L_BEAM**2 / 12])
I_BEAM_INV = np.diag([1.0 / I_BEAM[i, i] for i in range(3)])

# Anchor offsets in beam body frame (along beam axis)
ANCHOR_OFFSETS_BODY = np.array([
    [-0.75, 0.0, 0.0],
    [ 0.00, 0.0, 0.0],
    [ 0.75, 0.0, 0.0],
])

# PD gains — beam trajectory control
KP_TRANS = 4.0
KD_TRANS = 3.0
KP_ROT   = 8.0
KD_ROT   = 4.0

# PD gains — drone formation control
KP_DRONE = 12.0
KD_DRONE = 5.0

# Repulsion gain
K_REP = 0.8

# Mission points
P_START = np.array([0.0,  0.0, 2.0])
P_END   = np.array([15.0, 0.0, 2.0])


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by unit quaternion q = [w, x, y, z]."""
    r = Rotation.from_quat([q[1], q[2], q[3], q[0]])  # scipy uses [x,y,z,w]
    return r.apply(v)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Quaternion product q1 ⊗ q2, both in [w, x, y, z] convention."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_to_euler_pitch_roll(q: np.ndarray):
    """Extract pitch and roll from quaternion [w, x, y, z]."""
    r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    roll, pitch, _ = r.as_euler('xyz', degrees=True)
    return pitch, roll


def grasp_matrix(anchor_positions_world: np.ndarray,
                 cable_dirs: np.ndarray) -> np.ndarray:
    """
    Build the 6×3 grasp matrix G mapping tension vector T → wrench W.
    anchor_positions_world: (3, 3) row = world-frame anchor position
    cable_dirs: (3, 3) row = unit vector from anchor to drone
    """
    G = np.zeros((6, N_DRONES))
    for i in range(N_DRONES):
        G[:3, i] = cable_dirs[i]                       # force rows
        G[3:, i] = np.cross(anchor_positions_world[i], cable_dirs[i])  # torque rows
    return G


def allocate_tensions(G: np.ndarray, W_des: np.ndarray,
                      t_min: float = T_MIN) -> np.ndarray:
    """
    Solve min ||T||^2  s.t.  G T ≈ W_des,  T_i ≥ t_min.
    Uses pseudo-inverse + null-space offset to enforce lower bound.
    Returns T (3,).
    """
    # Reduce to force-only rows if torque rows are degenerate (3 cables, 3 unknowns)
    G_force = G[:3, :]   # 3×3 — use only translational wrench
    W_force = W_des[:3]

    try:
        T = np.linalg.solve(G_force, W_force)
    except np.linalg.LinAlgError:
        T = np.linalg.lstsq(G_force, W_force, rcond=None)[0]

    # Enforce T_i ≥ t_min via null-space offset
    null_vec = np.ones(N_DRONES)
    null_comp = null_vec - G_force @ np.linalg.lstsq(G_force, null_vec, rcond=None)[0]
    deficit = t_min - T.min()
    if deficit > 0 and np.linalg.norm(null_comp) > 1e-6:
        T += (deficit / (np.linalg.norm(null_comp)**2 + 1e-9)) * null_comp
    T = np.maximum(T, t_min)
    return T


def simulate_cooperative_crane(strategy: str = 'distributed', seed: int = 42):
    """
    Run the cooperative crane simulation.
    strategy: 'distributed' | 'centralised' | 'uncoordinated'
    Returns history dict with logged states.
    """
    rng = np.random.default_rng(seed)

    # ── Initial state ──────────────────────────────────────────────────────
    p_L   = P_START.copy()
    v_L   = np.zeros(3)
    q_L   = np.array([1.0, 0.0, 0.0, 0.0])   # [w, x, y, z]
    omega_L = np.zeros(3)

    # Drone initial positions: each 1 m above its anchor
    anchors_body = ANCHOR_OFFSETS_BODY.copy()
    anchors_world = p_L + anchors_body            # beam initially un-rotated
    drone_pos  = anchors_world + np.array([0.0, 0.0, L_CABLE])
    drone_vel  = np.zeros((N_DRONES, 3))

    # Reference trajectory (straight line at V_CRUISE)
    mission_length = np.linalg.norm(P_END - P_START)
    direction = (P_END - P_START) / mission_length
    T_mission  = mission_length / V_CRUISE

    # Logging
    log = {
        't': [], 'p_L': [], 'v_L': [], 'pitch': [], 'roll': [],
        'tensions': [], 'drone_pos': [], 'min_sep': [], 'pos_err': [],
        'balance': [],
    }

    t = 0.0
    while t <= T_mission + 1.0:
        s = min(t * V_CRUISE, mission_length)
        p_ref = P_START + s * direction
        v_ref = V_CRUISE * direction if t < T_mission else np.zeros(3)
        a_ref = np.zeros(3)

        # ── Compute anchor positions in world frame ──────────────────────
        R_beam = Rotation.from_quat([q_L[1], q_L[2], q_L[3], q_L[0]]).as_matrix()
        anchors_world = p_L + (R_beam @ anchors_body.T).T   # (3, 3)

        # ── Cable geometry ────────────────────────────────────────────────
        cable_vecs = drone_pos - anchors_world               # (3, 3)
        cable_lens = np.linalg.norm(cable_vecs, axis=1, keepdims=True)
        n_hat      = cable_vecs / (cable_lens + 1e-9)        # unit vectors

        # ── Desired beam wrench ───────────────────────────────────────────
        acc_des = (KP_TRANS * (p_ref - p_L)
                   + KD_TRANS * (v_ref - v_L)
                   + a_ref)
        pitch, roll = quat_to_euler_pitch_roll(q_L)
        theta_err = np.deg2rad(np.array([-roll, -pitch, 0.0]))
        alpha_des = KP_ROT * theta_err - KD_ROT * omega_L

        W_des = np.zeros(6)
        W_des[:3] = M_LOAD * acc_des - M_LOAD * G_VEC
        W_des[3:]  = I_BEAM @ alpha_des + np.cross(omega_L, I_BEAM @ omega_L)

        # ── Tension allocation ────────────────────────────────────────────
        if strategy == 'distributed':
            G_mat = grasp_matrix(anchors_world - p_L, n_hat)
            tensions = allocate_tensions(G_mat, W_des)
        elif strategy == 'centralised':
            G_mat = grasp_matrix(anchors_world - p_L, n_hat)
            tensions = allocate_tensions(G_mat, W_des)
        else:  # uncoordinated
            # Each drone tries to provide 1/3 of load support independently
            tensions = np.full(N_DRONES, M_LOAD * 9.81 / N_DRONES)

        # ── Load dynamics ─────────────────────────────────────────────────
        F_net = np.zeros(3)
        tau_net = np.zeros(3)
        for i in range(N_DRONES):
            f_i = tensions[i] * n_hat[i]
            F_net  += f_i
            tau_net += np.cross(anchors_world[i] - p_L, f_i)

        acc_L   = (F_net + M_LOAD * G_VEC) / M_LOAD
        v_L    += acc_L * DT
        p_L    += v_L  * DT

        # Rotational update (world-frame torque → body frame)
        tau_body = R_beam.T @ tau_net
        alpha_L  = I_BEAM_INV @ (tau_body - np.cross(omega_L, I_BEAM @ omega_L))
        omega_L += alpha_L * DT

        # Quaternion integration
        omega_quat = np.array([0.0, omega_L[0], omega_L[1], omega_L[2]])
        dq = 0.5 * quat_multiply(q_L, omega_quat)
        q_L = q_L + dq * DT
        q_L /= np.linalg.norm(q_L)

        # ── Drone formation control ───────────────────────────────────────
        # Desired drone position: L_CABLE above anchor, in world frame
        R_beam_new = Rotation.from_quat([q_L[1], q_L[2], q_L[3], q_L[0]]).as_matrix()
        anchors_new = p_L + (R_beam_new @ anchors_body.T).T
        p_des = anchors_new + np.array([[0.0, 0.0, L_CABLE]] * N_DRONES)
        v_des = np.tile(v_L, (N_DRONES, 1))

        for i in range(N_DRONES):
            ctrl_acc = (KP_DRONE * (p_des[i] - drone_pos[i])
                        + KD_DRONE * (v_des[i] - drone_vel[i]))
            # Soft repulsion
            for j in range(N_DRONES):
                if j == i:
                    continue
                diff_ij = drone_pos[i] - drone_pos[j]
                dist_ij = np.linalg.norm(diff_ij)
                if dist_ij < D_SAFE:
                    ctrl_acc += K_REP * (D_SAFE - dist_ij) / D_SAFE \
                                 * diff_ij / (dist_ij + 1e-9)
            drone_vel[i] += ctrl_acc * DT
            drone_pos[i] += drone_vel[i] * DT

        # ── Metrics ───────────────────────────────────────────────────────
        pitch, roll = quat_to_euler_pitch_roll(q_L)
        pos_err = np.linalg.norm(p_L - p_ref)
        seps = [np.linalg.norm(drone_pos[i] - drone_pos[j])
                for i in range(N_DRONES) for j in range(i+1, N_DRONES)]
        T_mean = tensions.mean()
        balance = 1.0 - (sum(abs(tensions[i] - tensions[j])
                             for i in range(N_DRONES)
                             for j in range(i+1, N_DRONES))
                         / (3 * T_mean + 1e-9))

        log['t'].append(t)
        log['p_L'].append(p_L.copy())
        log['v_L'].append(v_L.copy())
        log['pitch'].append(pitch)
        log['roll'].append(roll)
        log['tensions'].append(tensions.copy())
        log['drone_pos'].append(drone_pos.copy())
        log['min_sep'].append(min(seps))
        log['pos_err'].append(pos_err)
        log['balance'].append(balance)

        t += DT

        # Early termination once beam reaches B
        if np.linalg.norm(p_L - P_END) < 0.05:
            break

    return log


def plot_results(logs: dict):
    """
    Generate all output figures.
    logs: dict mapping strategy name -> log dict
    """
    strategies = list(logs.keys())
    colors = {'distributed': 'tab:blue', 'centralised': 'tab:orange',
              'uncoordinated': 'tab:red'}

    # ── Figure 1: 3D trajectory ───────────────────────────────────────────
    fig1 = plt.figure(figsize=(10, 7))
    ax3d = fig1.add_subplot(111, projection='3d')
    log = logs.get('distributed', list(logs.values())[0])
    p_arr = np.array(log['p_L'])
    d_arr = np.array(log['drone_pos'])   # (T, 3, 3)

    ax3d.plot(p_arr[:, 0], p_arr[:, 1], p_arr[:, 2],
              'k-', lw=2, label='Beam CoM')
    drone_colors = ['tab:red', 'tab:green', 'tab:purple']
    for i in range(N_DRONES):
        ax3d.plot(d_arr[:, i, 0], d_arr[:, i, 1], d_arr[:, i, 2],
                  color=drone_colors[i], lw=1.2, alpha=0.7,
                  label=f'Drone {i+1}')
    ax3d.scatter(*P_START, color='green', s=80, marker='^', zorder=5, label='Start A')
    ax3d.scatter(*P_END,   color='blue',  s=80, marker='s', zorder=5, label='Goal B')
    ax3d.set_xlabel('X (m)'); ax3d.set_ylabel('Y (m)'); ax3d.set_zlabel('Z (m)')
    ax3d.set_title('S066 — 3D Transport Trajectories (Distributed)')
    ax3d.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s066_cooperative_crane/'
                's066_3d_trajectory.png', dpi=150)
    plt.close()

    # ── Figure 2: Metrics panel ───────────────────────────────────────────
    fig2, axes = plt.subplots(2, 3, figsize=(15, 8))
    for strat, log in logs.items():
        t   = np.array(log['t'])
        col = colors.get(strat, 'gray')

        # Beam attitude
        axes[0, 0].plot(t, log['pitch'], color=col, label=f'{strat} pitch')
        axes[0, 0].plot(t, log['roll'],  color=col, ls='--', alpha=0.6,
                        label=f'{strat} roll')
        # Position error
        axes[0, 1].plot(t, log['pos_err'], color=col, label=strat)
        # Min drone separation
        axes[0, 2].plot(t, log['min_sep'], color=col, label=strat)
        # Per-drone tensions
        tensions = np.array(log['tensions'])
        for i in range(N_DRONES):
            axes[1, 0].plot(t, tensions[:, i], color=drone_colors[i],
                            ls=['-','--','-.'][i], alpha=0.8, label=f'T{i+1}' if strat=='distributed' else '')
        # Tension balance
        axes[1, 1].plot(t, log['balance'], color=col, label=strat)
        # Beam speed
        v_arr = np.array(log['v_L'])
        axes[1, 2].plot(t, np.linalg.norm(v_arr, axis=1), color=col, label=strat)

    axes[0, 0].axhline(5,  color='gray', ls=':', lw=1)
    axes[0, 0].axhline(-5, color='gray', ls=':', lw=1, label='±5° limit')
    axes[0, 0].set_title('Beam Pitch / Roll (°)')
    axes[0, 0].set_xlabel('Time (s)'); axes[0, 0].legend(fontsize=7)

    axes[0, 1].axhline(0.1, color='gray', ls=':', lw=1, label='0.1 m limit')
    axes[0, 1].set_title('Beam CoM Position Error (m)')
    axes[0, 1].set_xlabel('Time (s)'); axes[0, 1].legend(fontsize=8)

    axes[0, 2].axhline(D_MIN, color='red', ls=':', lw=1, label=f'd_min={D_MIN} m')
    axes[0, 2].set_title('Min Inter-Drone Separation (m)')
    axes[0, 2].set_xlabel('Time (s)'); axes[0, 2].legend(fontsize=8)

    axes[1, 0].set_title('Cable Tensions (N) — distributed')
    axes[1, 0].set_xlabel('Time (s)'); axes[1, 0].legend(fontsize=8)

    axes[1, 1].set_ylim([0, 1.05])
    axes[1, 1].set_title('Tension Balance Index B(t)')
    axes[1, 1].set_xlabel('Time (s)'); axes[1, 1].legend(fontsize=8)

    axes[1, 2].axhline(V_CRUISE, color='gray', ls=':', lw=1, label=f'v_ref={V_CRUISE} m/s')
    axes[1, 2].set_title('Beam Speed (m/s)')
    axes[1, 2].set_xlabel('Time (s)'); axes[1, 2].legend(fontsize=8)

    plt.suptitle('S066 — Cooperative Crane: Performance Metrics', fontsize=13,
                 fontweight='bold')
    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s066_cooperative_crane/'
                's066_metrics_panel.png', dpi=150)
    plt.close()


def animate_crane(log: dict, out_path: str = 'outputs/04_industrial_agriculture/'
                  's066_cooperative_crane/s066_animation.gif'):
    """Create 3D animation of the cooperative crane transport (GIF)."""
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    p_arr = np.array(log['p_L'])        # (T, 3)
    d_arr = np.array(log['drone_pos'])  # (T, 3, 3)
    n_frames = len(p_arr)
    step = max(1, n_frames // 150)      # cap at ~150 frames

    fig = plt.figure(figsize=(8, 6))
    ax  = fig.add_subplot(111, projection='3d')

    ax.set_xlim(-1, 17); ax.set_ylim(-3, 3); ax.set_zlim(0, 5)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('S066 Cooperative Crane — Transport Animation')

    beam_line,  = ax.plot([], [], [], 'k-', lw=4, label='Beam')
    com_dot,    = ax.plot([], [], [], 'ko', ms=6)
    d_dots      = [ax.plot([], [], [], 'o', color=c, ms=8)[0]
                   for c in ['tab:red', 'tab:green', 'tab:purple']]
    cable_lines = [ax.plot([], [], [], '--', color=c, lw=1, alpha=0.7)[0]
                   for c in ['tab:red', 'tab:green', 'tab:purple']]
    trail,      = ax.plot([], [], [], 'k:', lw=1, alpha=0.4)
    ax.scatter(*P_START, color='green', s=60, marker='^')
    ax.scatter(*P_END,   color='blue',  s=60, marker='s')

    # Pre-compute beam end positions (assume un-rotated for simplicity in anim)
    beam_half = np.array([L_BEAM / 2, 0.0, 0.0])

    def update(frame):
        k  = frame * step
        pL = p_arr[k]
        dp = d_arr[k]

        # Beam rod (approximate as horizontal for visualisation)
        bx = [pL[0] - beam_half[0], pL[0] + beam_half[0]]
        by = [pL[1], pL[1]]
        bz = [pL[2], pL[2]]
        beam_line.set_data(bx, by); beam_line.set_3d_properties(bz)
        com_dot.set_data([pL[0]], [pL[1]]); com_dot.set_3d_properties([pL[2]])

        # Anchor X offsets along beam
        a_offsets = [-0.75, 0.0, 0.75]
        for i in range(N_DRONES):
            ax_i = pL[0] + a_offsets[i]
            ay_i, az_i = pL[1], pL[2]
            d_dots[i].set_data([dp[i, 0]], [dp[i, 1]])
            d_dots[i].set_3d_properties([dp[i, 2]])
            cable_lines[i].set_data([ax_i, dp[i, 0]], [ay_i, dp[i, 1]])
            cable_lines[i].set_3d_properties([az_i, dp[i, 2]])

        trail.set_data(p_arr[:k, 0], p_arr[:k, 1])
        trail.set_3d_properties(p_arr[:k, 2])
        return [beam_line, com_dot, trail] + d_dots + cable_lines

    n_anim_frames = n_frames // step
    ani = animation.FuncAnimation(fig, update, frames=n_anim_frames,
                                  interval=50, blit=False)
    ani.save(out_path, writer='pillow', fps=20)
    plt.close()
    print(f"Animation saved to {out_path}")


def run_simulation_suite():
    import os
    out_dir = ('outputs/04_industrial_agriculture/s066_cooperative_crane')
    os.makedirs(out_dir, exist_ok=True)

    strategies = ['distributed', 'uncoordinated']
    logs = {}
    for strat in strategies:
        print(f"Running strategy: {strat} ...")
        log = simulate_cooperative_crane(strategy=strat)
        logs[strat] = log
        t_arr = np.array(log['t'])
        pitch = np.array(log['pitch'])
        roll  = np.array(log['roll'])
        tens  = np.array(log['tensions'])
        bal   = np.array(log['balance'])
        rms_att = np.sqrt(np.mean(pitch**2 + roll**2))
        mean_bal = bal.mean()
        print(f"  Transport time   : {t_arr[-1]:.1f} s")
        print(f"  RMS attitude err : {rms_att:.3f} °")
        print(f"  Mean balance B   : {mean_bal:.3f}")
        print(f"  Min separation   : {min(log['min_sep']):.3f} m")

    plot_results(logs)
    animate_crane(logs['distributed'])
    print("All outputs saved.")
    return logs


if __name__ == '__main__':
    run_simulation_suite()
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones | $N$ | 3 |
| Beam mass | $m_L$ | 2 kg |
| Beam length | $L$ | 2 m |
| Beam inertia (pitch/yaw) | $I_{yy} = I_{zz}$ | 2/3 kg m² |
| Drone mass | $m_D$ | 0.5 kg |
| Cable length | $l_c$ | 1 m |
| Transport speed | $v$ | 0.5 m/s |
| Transport distance ($A \to B$) | — | 15 m |
| Simulation timestep | $\Delta t$ | 0.01 s |
| Minimum cable tension | $T_{min}$ | 0.5 N |
| Beam attitude constraint | $|\theta_{pitch}|, |\theta_{roll}|$ | ≤ 5° |
| Hard collision threshold | $d_{min}$ | 0.8 m |
| Soft repulsion onset | $d_{safe}$ | 1.2 m |
| Beam trajectory PD gains | $K_p, K_d$ | 4.0, 3.0 |
| Attitude PD gains | $K_{p,\omega}, K_{d,\omega}$ | 8.0, 4.0 |
| Drone formation PD gains | $K_{p,D}, K_{d,D}$ | 12.0, 5.0 |
| Anchor offsets (body frame) | — | −0.75, 0, +0.75 m along beam |

---

## Expected Output

- **3D transport trajectory plot**: world-frame 3D axes showing the beam CoM path (black solid
  line) from $A$ to $B$; three coloured drone trajectories (red, green, purple) flying above the
  beam; start and goal markers; cable lines drawn at 5 representative instants as dashed segments
  connecting each anchor to its drone. Start $A$ marked as a green triangle, goal $B$ as a blue
  square.
- **Metrics panel (2 × 3 subplots)**:
  - Top-left: beam pitch and roll over time for each strategy; horizontal dashed lines at $\pm 5°$
    constraint.
  - Top-centre: beam CoM position error $\|p_L - p_{ref}\|$ vs time; dashed line at 0.1 m
    threshold.
  - Top-right: minimum inter-drone separation vs time; red dotted line at $d_{min} = 0.8$ m.
  - Bottom-left: individual cable tensions $T_1(t), T_2(t), T_3(t)$ for the distributed strategy.
  - Bottom-centre: tension balance index $\mathcal{B}(t)$ for all strategies; $\mathcal{B} = 1$ is
    perfect balance.
  - Bottom-right: beam speed $\|\dot{p}_L\|$ vs time; dotted reference line at $v = 0.5$ m/s.
- **Transport animation (GIF)**: 3D view updating at 20 fps; beam drawn as a thick black rod;
  drones as coloured spheres above the beam; dashed cables connecting each drone to its anchor;
  beam CoM trail shown as a dotted black line; start and goal markers fixed; frame counter and
  elapsed time displayed in the title.
- **Terminal summary**: per-strategy printed metrics — transport time, RMS attitude error,
  mean tension balance index $\bar{\mathcal{B}}$, minimum observed inter-drone separation.

---

## Extensions

1. **Variable payload geometry**: replace the symmetric three-anchor layout with an asymmetric
   one (e.g., an L-shaped frame with shifted CoM); the grasp matrix $\mathbf{G}$ becomes
   rank-deficient in some directions — add a regularisation term $\epsilon \mathbf{I}$ to the
   pseudo-inverse and compare attitude error against the symmetric layout.
2. **Cable slack detection**: model cable elasticity with spring constant $k_c$; when a drone
   descends below $l_c$ above its anchor the cable goes slack ($T_i = 0$) and must be re-tensioned
   by climbing; implement a slack-recovery state machine that temporarily over-extends the offending
   drone upward.
3. **Wind disturbance rejection**: add a stochastic horizontal wind $\mathbf{w}(t)$ with Dryden
   turbulence spectrum; compare a feedforward disturbance-observer controller (estimates $\mathbf{w}$
   from observed acceleration residuals) against pure feedback PD in terms of beam attitude RMS.
4. **$N = 4$ or $N = 6$ drone scalability**: with four or six drones the system becomes over-
   actuated (more tensions than wrench DOFs); reformulate the QP with inequality constraints on
   $T_{max}$ per motor and study how redundancy reduces the required peak tension per drone.
5. **Obstacle-laden path**: replace the straight-line $A \to B$ trajectory with a 3D spline that
   routes around building obstacles; the formation controller must handle simultaneous translation,
   yaw rotation, and altitude changes while keeping all constraints satisfied.
6. **On-line load identification**: treat $m_L$ and $I_L$ as unknown and estimate them from IMU
   measurements on the beam using a recursive least-squares identifier; compare controller
   performance with ground-truth vs estimated inertia.

---

## Related Scenarios

- Prerequisites: [S061 Precision Spraying Path](S061_precision_spraying.md), [S063 Perching and Grasping](S063_perching_grasping.md)
- Follow-ups: [S068 Aerial Assembly Line](S068_aerial_assembly.md), [S070 Swarm Construction](S070_swarm_construction.md)
- Algorithmic cross-reference: [S079 Tethered Drone Operations](S079_tethered_operations.md) (cable constraint dynamics), [S005 Formation Keeping](../01_pursuit_evasion/S005_formation_keeping.md) (multi-drone PID formation), [S020 Cooperative Interception](../01_pursuit_evasion/S020_cooperative_interception.md) (distributed multi-agent control)
