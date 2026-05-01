# S050 Swarm Cooperative Mapping (Distributed EKF-SLAM)

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A $200 \times 200$ m environment is completely unknown at mission start. No prior map
exists and no external positioning system (GPS) is available. The terrain contains $M_{true} = 30$
static landmarks (e.g., tree stumps, boulders) distributed across the area. A swarm of $N = 3$
drones is deployed from a common origin. Each drone carries a range-bearing sensor that can detect
any landmark within range $r_{sense} = 25$ m. From a detection the sensor returns a noisy range
$\rho$ and bearing $\phi$ measurement relative to the drone's current body frame. Each drone
maintains its own **local EKF-SLAM state** that encodes its own pose and the estimated positions of
all landmarks it has observed so far. When two drones come within communication range
$r_{comm} = 40$ m of each other, they exchange their local maps. **Map merging** aligns the two
local coordinate frames using common landmarks (landmarks observed by both drones) and fuses the
overlapping landmark estimates via a weighted least-squares transformation. Drones navigate
autonomously using an information-gain frontier policy: at each step, each drone moves toward the
unexplored region with the highest expected number of new landmark detections.

**Roles**:
- **Drones** ($N = 3$): homogeneous agents, each running an independent EKF-SLAM filter; state
  vector grows as new landmarks are initialised; communicate pairwise when within $r_{comm}$.
- **Landmarks** ($M_{true} = 30$): fixed, passive features in the environment; positions unknown at
  mission start; detected by range-bearing sensor when within $r_{sense}$.

**Objective**: Within the mission time $T_{max} = 600$ s, maximise the fraction of true landmarks
successfully mapped (localised with position error $< 1.0$ m) while minimising the root-mean-square
(RMS) error of the final merged map. The collaborative approach is compared against three baselines:

1. **Isolated EKF-SLAM** — each drone builds its own map independently, no communication.
2. **Centralised EKF-SLAM** — all observations fed into a single shared filter (performance upper
   bound, not realisable without perfect communication).
3. **Swarm distributed EKF-SLAM** (proposed) — local filters with pairwise map merging on contact.

---

## Mathematical Model

### Drone State and EKF-SLAM State Vector

Each drone $i$ maintains a local state vector $\mathbf{x}_i \in \mathbb{R}^{3 + 2 n_i}$:

$$\mathbf{x}_i = \begin{bmatrix} x_i \\ y_i \\ \theta_i \\ \mathbf{l}_{i,1} \\ \vdots \\ \mathbf{l}_{i,n_i} \end{bmatrix}$$

where $(x_i, y_i, \theta_i)$ is the drone's 2D pose (position and heading) and
$\mathbf{l}_{i,k} = (l_x^{(k)}, l_y^{(k)})^\top$ is the estimated global position of the $k$-th
landmark observed by drone $i$. The associated covariance matrix is
$\mathbf{P}_i \in \mathbb{R}^{(3+2n_i) \times (3+2n_i)}$.

### Motion Model (Predict Step)

Drone $i$ moves with constant forward speed $v = 3.0$ m/s in direction $\theta_i$. The discrete-time
kinematic model with timestep $\Delta t$ is:

$$f(\mathbf{x}_i) = \begin{bmatrix} x_i + v \Delta t \cos\theta_i \\ y_i + v \Delta t \sin\theta_i \\ \theta_i + \omega_i \Delta t \\ \mathbf{l}_{i,1} \\ \vdots \\ \mathbf{l}_{i,n_i} \end{bmatrix}$$

where $\omega_i$ is the commanded turn rate. The Jacobian of $f$ with respect to $\mathbf{x}_i$:

$$\mathbf{F}_i = \frac{\partial f}{\partial \mathbf{x}_i} = \begin{bmatrix}
1 & 0 & -v \Delta t \sin\theta_i & \mathbf{0} \\
0 & 1 &  v \Delta t \cos\theta_i & \mathbf{0} \\
0 & 0 & 1 & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_{2n_i}
\end{bmatrix}$$

The predicted mean and covariance are:

$$\mathbf{x}_{i,k|k-1} = f(\mathbf{x}_{i,k-1})$$

$$\mathbf{P}_{i,k|k-1} = \mathbf{F}_i \, \mathbf{P}_{i,k-1} \, \mathbf{F}_i^\top + \mathbf{Q}$$

where the process noise covariance is $\mathbf{Q} = \mathrm{blkdiag}(\mathbf{Q}_{pose}, \mathbf{0}_{2n_i})$
with $\mathbf{Q}_{pose} = \mathrm{diag}(\sigma_{ax}^2 \Delta t^2,\; \sigma_{ay}^2 \Delta t^2,\; \sigma_{\omega}^2 \Delta t^2)$.

### Observation Model

For landmark $k$ at estimated global position $\mathbf{l}_{i,k} = (l_x, l_y)^\top$, observed from
drone pose $(x_i, y_i, \theta_i)$, the predicted observation is:

$$h(\mathbf{x}_i, k) = \begin{bmatrix} \rho_k \\ \phi_k \end{bmatrix}
= \begin{bmatrix}
\sqrt{(l_x - x_i)^2 + (l_y - y_i)^2} \\
\mathrm{atan2}(l_y - y_i,\; l_x - x_i) - \theta_i
\end{bmatrix}$$

The Jacobian of $h$ with respect to the full state $\mathbf{x}_i$ (rows for $\rho, \phi$;
columns for the drone pose block and the landmark $k$ block only; all other columns are zero):

$$\mathbf{H}_{i,k} = \begin{bmatrix}
-\frac{\delta_x}{\rho_k} & -\frac{\delta_y}{\rho_k} & 0 & \cdots &
 \frac{\delta_x}{\rho_k} & \frac{\delta_y}{\rho_k} & \cdots \\
 \frac{\delta_y}{\rho_k^2} & -\frac{\delta_x}{\rho_k^2} & -1 & \cdots &
-\frac{\delta_y}{\rho_k^2} & \frac{\delta_x}{\rho_k^2} & \cdots
\end{bmatrix}$$

where $\delta_x = l_x - x_i$, $\delta_y = l_y - y_i$, $\rho_k = \sqrt{\delta_x^2 + \delta_y^2}$.

The measurement noise covariance is $\mathbf{R} = \mathrm{diag}(\sigma_\rho^2, \sigma_\phi^2)$.

### EKF Update Step

For each new range-bearing observation $\mathbf{z}_{i,k} = (\rho_k^{obs}, \phi_k^{obs})^\top$
associated with landmark $k$:

$$\mathbf{S}_{i,k} = \mathbf{H}_{i,k} \, \mathbf{P}_{i,k|k-1} \, \mathbf{H}_{i,k}^\top + \mathbf{R}$$

$$\mathbf{K}_{i,k} = \mathbf{P}_{i,k|k-1} \, \mathbf{H}_{i,k}^\top \, \mathbf{S}_{i,k}^{-1}$$

$$\mathbf{x}_{i,k} = \mathbf{x}_{i,k|k-1} + \mathbf{K}_{i,k}
  \bigl(\mathbf{z}_{i,k} - h(\mathbf{x}_{i,k|k-1}, k)\bigr)$$

$$\mathbf{P}_{i,k} = (\mathbf{I} - \mathbf{K}_{i,k} \, \mathbf{H}_{i,k}) \, \mathbf{P}_{i,k|k-1}$$

The innovation $\boldsymbol{\nu} = \mathbf{z}_{i,k} - h(\mathbf{x}_{i,k|k-1}, k)$ requires
angle-wrapping on its second component: $\nu_\phi = \mathrm{atan2}(\sin\nu_\phi, \cos\nu_\phi)$.

### Landmark Initialisation

When drone $i$ obtains a range-bearing observation $(\rho^{obs}, \phi^{obs})$ and the data
association step (below) determines this is a **new** landmark, the landmark is initialised at:

$$\mathbf{l}_{new} = \begin{bmatrix} x_i + \rho^{obs} \cos(\theta_i + \phi^{obs}) \\
y_i + \rho^{obs} \sin(\theta_i + \phi^{obs}) \end{bmatrix}$$

The initial covariance block for the new landmark is propagated from the drone's pose uncertainty
and the sensor noise via the Jacobian of the inverse observation function:

$$\mathbf{P}_{ll}^{new} = \mathbf{J}_{inv} \, \mathrm{blkdiag}(\mathbf{P}_{pose},\, \mathbf{R}) \, \mathbf{J}_{inv}^\top$$

where $\mathbf{J}_{inv} \in \mathbb{R}^{2 \times 5}$ is the Jacobian of the initialisation function
with respect to $(x_i, y_i, \theta_i, \rho^{obs}, \phi^{obs})$.

### Data Association: Nearest-Neighbour with Mahalanobis Gate

For each new observation $\mathbf{z}$ in drone $i$'s sensor sweep, compute the Mahalanobis
distance to each already-tracked landmark $j$:

$$d_{M}(\mathbf{z}, j) = \boldsymbol{\nu}_j^\top \, \mathbf{S}_j^{-1} \, \boldsymbol{\nu}_j$$

where $\boldsymbol{\nu}_j = \mathbf{z} - h(\mathbf{x}_i, j)$ and
$\mathbf{S}_j = \mathbf{H}_j \mathbf{P}_i \mathbf{H}_j^\top + \mathbf{R}$.

The observation is associated with the nearest landmark if the minimum distance falls inside the
validation gate:

$$j^* = \arg\min_j \; d_M(\mathbf{z}, j)$$

$$\text{Associate if} \quad d_M(\mathbf{z}, j^*) \leq \chi^2_{2, 0.95} = 5.991$$

If no existing landmark satisfies the gate, the observation triggers a **new landmark
initialisation**. This $\chi^2$ gate with 2 degrees of freedom (range + bearing) at the 95 %
confidence level provides a principled threshold that automatically scales with the current
uncertainty in the filter.

### Map Merging via Common Landmark Alignment

When drones $i$ and $j$ come within $r_{comm}$, they identify **common landmarks**: landmarks
appearing in both local maps whose local-frame covariance ellipses overlap at the 95 % level. Let
$\mathcal{C} = \{(a_1, b_1), \ldots, (a_m, b_m)\}$ be the set of $m \geq 3$ matched landmark pairs
(drone $i$ landmark $a_k$ identified as the same physical feature as drone $j$ landmark $b_k$).

The coordinate frames of the two maps are related by a rigid transformation
$(t_x, t_y, \alpha) \in \mathbb{R}^3$ (translation + rotation). Given matched landmark positions
$\mathbf{l}_i^{(a_k)}$ and $\mathbf{l}_j^{(b_k)}$, the least-squares alignment solves:

$$\min_{t_x, t_y, \alpha} \sum_{k=1}^{m}
  \bigl\|\mathbf{l}_i^{(a_k)} - \mathbf{R}(\alpha)\,\mathbf{l}_j^{(b_k)} - \mathbf{t}\bigr\|^2$$

where $\mathbf{R}(\alpha) = \begin{pmatrix} \cos\alpha & -\sin\alpha \\ \sin\alpha & \cos\alpha \end{pmatrix}$
and $\mathbf{t} = (t_x, t_y)^\top$. This has a closed-form solution via the SVD of the cross-covariance
matrix of the matched landmark sets:

$$\mathbf{W} = \sum_{k=1}^{m}
  \bigl(\mathbf{l}_i^{(a_k)} - \bar{\mathbf{l}}_i\bigr)\bigl(\mathbf{l}_j^{(b_k)} - \bar{\mathbf{l}}_j\bigr)^\top, \quad
\mathbf{W} = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^\top$$

$$\mathbf{R}^* = \mathbf{U} \mathbf{V}^\top, \qquad
\mathbf{t}^* = \bar{\mathbf{l}}_i - \mathbf{R}^* \bar{\mathbf{l}}_j$$

After alignment, each drone $j$ landmark not yet tracked by drone $i$ is transformed into drone $i$'s
frame and **fused** using the EKF cross-covariance update:

$$\mathbf{l}_i^{new} = \mathbf{l}_i^{existing} + \mathbf{K}_{merge}
  \bigl(\mathbf{l}_j^{transformed} - \mathbf{l}_i^{existing}\bigr)$$

$$\mathbf{K}_{merge} = \mathbf{P}_i^{(ll)} \bigl(\mathbf{P}_i^{(ll)} + \mathbf{P}_j^{(ll,transformed)}\bigr)^{-1}$$

This is a covariance-intersection update that guarantees the merged covariance is never larger than
either individual estimate.

### Exploration Policy: Information-Gain Frontier

Each drone selects its next waypoint from a set of frontier candidates — grid cells at the boundary
of already-explored territory (cells within sensor range of at least one past drone position). The
expected information gain for moving drone $i$ to candidate waypoint $\mathbf{w}$ is:

$$\mathrm{IG}(\mathbf{w}) = \sum_{\mathbf{c} \in \mathcal{F}(\mathbf{w})} \bigl(1 - \exp(-\beta \cdot B_i(\mathbf{c}))\bigr)$$

where $\mathcal{F}(\mathbf{w})$ is the set of unexplored cells within $r_{sense}$ of $\mathbf{w}$,
$B_i(\mathbf{c})$ is the prior probability that cell $\mathbf{c}$ contains an unmapped landmark
(uniform $= M_{true} / |\mathcal{G}|$ initially), and $\beta$ is an information saturation
parameter. Drone $i$ selects:

$$\mathbf{w}_i^* = \arg\max_{\mathbf{w} \in \mathcal{W}_{frontier}}
  \frac{\mathrm{IG}(\mathbf{w})}{\|\mathbf{w} - \mathbf{p}_i\|}$$

(gain-to-distance ratio, penalising distant frontiers). Drones avoid redundancy by suppressing
frontier candidates within $r_{sense}$ of any waypoint already assigned to another drone.

### Map Quality Metrics

After mission completion, the merged global map is compared to ground truth. For each true landmark
$\mathbf{l}^*_k$ and its best-matching estimated landmark $\hat{\mathbf{l}}_k$:

$$\text{RMS error} = \sqrt{\frac{1}{M_{found}} \sum_{k=1}^{M_{found}} \|\hat{\mathbf{l}}_k - \mathbf{l}^*_k\|^2}$$

$$\text{Coverage} = \frac{M_{found}}{M_{true}} \times 100\%$$

where $M_{found}$ is the number of true landmarks localised within a 1.0 m matching radius.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict

# Key constants
N_DRONES        = 3
AREA_SIZE       = 200.0       # m, square side
M_LANDMARKS     = 30          # true landmarks in environment
DRONE_SPEED     = 3.0         # m/s
DT              = 0.2         # s — simulation timestep
T_MAX           = 600.0       # s — mission horizon

SENSOR_RANGE    = 25.0        # m — landmark detection range
COMM_RANGE      = 40.0        # m — inter-drone communication range
SIGMA_RHO       = 0.3         # m — range noise std
SIGMA_PHI       = 0.03        # rad — bearing noise std (~1.7 deg)

# Process noise
SIGMA_AX        = 0.05        # m/s^2 acceleration noise
SIGMA_AY        = 0.05
SIGMA_OMEGA     = 0.02        # rad/s turn rate noise

# Data association gate (chi2, 2 dof, 95%)
MAHA_GATE       = 5.991

R_OBS = np.diag([SIGMA_RHO**2, SIGMA_PHI**2])


@dataclass
class EKFSLAMState:
    """Local EKF-SLAM state for one drone."""
    drone_id: int
    # pose: [x, y, theta]
    pose: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # landmark positions stacked: [lx1, ly1, lx2, ly2, ...]
    landmarks: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    n_landmarks: int = 0
    # full covariance matrix (3 + 2*n) x (3 + 2*n)
    P: np.ndarray = field(default_factory=lambda: np.eye(3) * 0.01)

    def state_dim(self):
        return 3 + 2 * self.n_landmarks

    def full_state(self):
        return np.concatenate([self.pose, self.landmarks.ravel()])


def predict_step(ekf: EKFSLAMState, v: float, omega: float, dt: float) -> EKFSLAMState:
    """EKF predict: propagate pose forward, landmarks unchanged."""
    x, y, th = ekf.pose
    n = ekf.n_landmarks
    dim = 3 + 2 * n

    # Predicted pose
    ekf.pose = np.array([
        x + v * dt * np.cos(th),
        y + v * dt * np.sin(th),
        th + omega * dt
    ])

    # Jacobian F (dim x dim)
    F = np.eye(dim)
    F[0, 2] = -v * dt * np.sin(th)
    F[1, 2] =  v * dt * np.cos(th)

    # Process noise Q
    Q = np.zeros((dim, dim))
    Q[0, 0] = (SIGMA_AX * dt)**2
    Q[1, 1] = (SIGMA_AY * dt)**2
    Q[2, 2] = (SIGMA_OMEGA * dt)**2

    ekf.P = F @ ekf.P @ F.T + Q
    return ekf


def observation_jacobian(ekf: EKFSLAMState, lm_idx: int):
    """Compute H (2 x dim) and innovation vector components for landmark lm_idx."""
    x, y, th = ekf.pose
    lx, ly = ekf.landmarks[lm_idx]
    dx, dy = lx - x, ly - y
    rho = np.sqrt(dx**2 + dy**2) + 1e-9
    dim = ekf.state_dim()

    H = np.zeros((2, dim))
    # Partial derivatives w.r.t. pose
    H[0, 0] = -dx / rho;  H[0, 1] = -dy / rho;  H[0, 2] = 0.0
    H[1, 0] =  dy / rho**2; H[1, 1] = -dx / rho**2; H[1, 2] = -1.0
    # Partial derivatives w.r.t. landmark position
    li = 3 + 2 * lm_idx
    H[0, li]   =  dx / rho;   H[0, li+1] =  dy / rho
    H[1, li]   = -dy / rho**2; H[1, li+1] =  dx / rho**2

    pred_rho = rho
    pred_phi = np.arctan2(dy, dx) - th
    return H, pred_rho, pred_phi


def ekf_update(ekf: EKFSLAMState, lm_idx: int, z_rho: float, z_phi: float):
    """EKF update for one range-bearing observation associated with lm_idx."""
    H, pred_rho, pred_phi = observation_jacobian(ekf, lm_idx)
    nu = np.array([z_rho - pred_rho, z_phi - pred_phi])
    # Angle wrap
    nu[1] = np.arctan2(np.sin(nu[1]), np.cos(nu[1]))

    S = H @ ekf.P @ H.T + R_OBS
    K = ekf.P @ H.T @ np.linalg.inv(S)

    full = ekf.full_state()
    full = full + K @ nu
    ekf.pose = full[:3]
    ekf.landmarks = full[3:].reshape(-1, 2)
    ekf.P = (np.eye(ekf.state_dim()) - K @ H) @ ekf.P
    return ekf


def data_association(ekf: EKFSLAMState, z_rho: float, z_phi: float) -> int:
    """
    Nearest-neighbour with Mahalanobis gate.
    Returns landmark index (>=0) if associated, or -1 if new landmark.
    """
    best_dist = np.inf
    best_idx = -1
    for j in range(ekf.n_landmarks):
        H, pred_rho, pred_phi = observation_jacobian(ekf, j)
        nu = np.array([z_rho - pred_rho, z_phi - pred_phi])
        nu[1] = np.arctan2(np.sin(nu[1]), np.cos(nu[1]))
        S = H @ ekf.P @ H.T + R_OBS
        d_maha = float(nu @ np.linalg.inv(S) @ nu)
        if d_maha < best_dist:
            best_dist = d_maha
            best_idx = j
    if best_dist <= MAHA_GATE:
        return best_idx
    return -1   # new landmark


def initialise_landmark(ekf: EKFSLAMState, z_rho: float, z_phi: float):
    """Add a new landmark to the EKF state based on current observation."""
    x, y, th = ekf.pose
    global_angle = th + z_phi
    lx = x + z_rho * np.cos(global_angle)
    ly = y + z_rho * np.sin(global_angle)

    # Jacobian of initialisation w.r.t. (x, y, th, rho, phi)
    Jinv = np.zeros((2, 5))
    Jinv[0, 0] = 1.0;  Jinv[0, 2] = -z_rho * np.sin(global_angle)
    Jinv[0, 3] = np.cos(global_angle)
    Jinv[0, 4] = -z_rho * np.sin(global_angle)
    Jinv[1, 1] = 1.0;  Jinv[1, 2] =  z_rho * np.cos(global_angle)
    Jinv[1, 3] = np.sin(global_angle)
    Jinv[1, 4] =  z_rho * np.cos(global_angle)

    src_cov = np.zeros((5, 5))
    src_cov[:3, :3] = ekf.P[:3, :3]
    src_cov[3:, 3:] = R_OBS
    P_ll = Jinv @ src_cov @ Jinv.T

    # Expand state and covariance
    old_dim = ekf.state_dim()
    new_dim = old_dim + 2
    new_P = np.zeros((new_dim, new_dim))
    new_P[:old_dim, :old_dim] = ekf.P
    new_P[old_dim:, old_dim:] = P_ll

    ekf.landmarks = np.vstack([ekf.landmarks, [lx, ly]]) if ekf.n_landmarks > 0 \
                    else np.array([[lx, ly]])
    ekf.n_landmarks += 1
    ekf.P = new_P
    return ekf


def merge_maps(host: EKFSLAMState, visitor: EKFSLAMState):
    """
    Merge visitor's map into host using common-landmark SVD alignment +
    covariance-intersection fusion for non-common landmarks.
    Modifies host in-place. Returns number of common landmarks found.
    """
    if visitor.n_landmarks == 0:
        return 0

    # Find common landmarks: closest pair within 2.0 m
    common_host, common_vis = [], []
    for vi in range(visitor.n_landmarks):
        vl = visitor.landmarks[vi]
        dists = np.linalg.norm(host.landmarks - vl, axis=1) \
                if host.n_landmarks > 0 else np.array([])
        if len(dists) > 0 and dists.min() < 2.0:
            common_host.append(np.argmin(dists))
            common_vis.append(vi)

    if len(common_host) < 3:
        return len(common_host)  # insufficient overlap to align

    # SVD-based rigid alignment (visitor -> host frame)
    Lh = host.landmarks[common_host]      # (m, 2)
    Lv = visitor.landmarks[common_vis]    # (m, 2)
    mu_h, mu_v = Lh.mean(axis=0), Lv.mean(axis=0)
    W = (Lh - mu_h).T @ (Lv - mu_v)
    U, _, Vt = np.linalg.svd(W)
    R_align = U @ Vt
    if np.linalg.det(R_align) < 0:   # reflection guard
        Vt[-1, :] *= -1
        R_align = U @ Vt
    t_align = mu_h - R_align @ mu_v

    # Fuse non-common visitor landmarks into host via covariance intersection
    for vi in range(visitor.n_landmarks):
        if vi in common_vis:
            continue
        lv_transformed = R_align @ visitor.landmarks[vi] + t_align
        P_v = visitor.P[3+2*vi : 3+2*vi+2, 3+2*vi : 3+2*vi+2]
        # Match transformed landmark against host map
        if host.n_landmarks > 0:
            dists = np.linalg.norm(host.landmarks - lv_transformed, axis=1)
            if dists.min() < 2.0:
                hi = np.argmin(dists)
                P_h = host.P[3+2*hi : 3+2*hi+2, 3+2*hi : 3+2*hi+2]
                K_m = P_h @ np.linalg.inv(P_h + P_v)
                host.landmarks[hi] = host.landmarks[hi] + K_m @ (lv_transformed - host.landmarks[hi])
                host.P[3+2*hi : 3+2*hi+2, 3+2*hi : 3+2*hi+2] = \
                    (np.eye(2) - K_m) @ P_h
                continue
        # New landmark for host
        old_dim = host.state_dim()
        new_dim = old_dim + 2
        new_P = np.zeros((new_dim, new_dim))
        new_P[:old_dim, :old_dim] = host.P
        new_P[old_dim:, old_dim:] = P_v
        host.landmarks = np.vstack([host.landmarks, lv_transformed]) \
                         if host.n_landmarks > 0 else lv_transformed[np.newaxis]
        host.n_landmarks += 1
        host.P = new_P

    return len(common_host)


def simulate_swarm_slam(true_landmarks, strategy="distributed", seed=0):
    """
    Run one complete swarm SLAM mission.
    strategy: "distributed" | "isolated" | "centralised"
    Returns per-step log of drone poses and merged map quality over time.
    """
    rng = np.random.default_rng(seed)

    # Initialise drones at spread-out start positions
    starts = [np.array([20.0, 20.0, 0.0]),
              np.array([100.0, 20.0, 0.0]),
              np.array([180.0, 20.0, 0.0])]
    ekfs = [EKFSLAMState(drone_id=k, pose=starts[k].copy()) for k in range(N_DRONES)]

    sim_time = 0.0
    history = []   # list of (t, rms, coverage) snapshots

    while sim_time < T_MAX:
        sim_time += DT

        for i, ekf in enumerate(ekfs):
            # --- Navigation: frontier-based (simplified: random walk toward unexplored edge) ---
            # In full implementation replace with information-gain frontier selector
            th = ekf.pose[2]
            omega_cmd = rng.uniform(-0.3, 0.3)   # placeholder; replace with frontier policy
            ekf = predict_step(ekf, DRONE_SPEED, omega_cmd, DT)

            # Boundary reflection
            ekf.pose[0] = np.clip(ekf.pose[0], 0.0, AREA_SIZE)
            ekf.pose[1] = np.clip(ekf.pose[1], 0.0, AREA_SIZE)

            # --- Sensor sweep: observe landmarks within range ---
            dists = np.linalg.norm(true_landmarks - ekf.pose[:2], axis=1)
            visible = np.where(dists <= SENSOR_RANGE)[0]

            for lm_true_idx in visible:
                true_rho = dists[lm_true_idx]
                true_phi = (np.arctan2(
                    true_landmarks[lm_true_idx, 1] - ekf.pose[1],
                    true_landmarks[lm_true_idx, 0] - ekf.pose[0]
                ) - ekf.pose[2])
                # Add sensor noise
                z_rho = true_rho + rng.normal(0, SIGMA_RHO)
                z_phi = true_phi + rng.normal(0, SIGMA_PHI)

                assoc = data_association(ekf, z_rho, z_phi)
                if assoc >= 0:
                    ekf = ekf_update(ekf, assoc, z_rho, z_phi)
                else:
                    ekf = initialise_landmark(ekf, z_rho, z_phi)

            ekfs[i] = ekf

        # --- Communication: pairwise map merge when within r_comm ---
        if strategy == "distributed":
            for i in range(N_DRONES):
                for j in range(i + 1, N_DRONES):
                    d_ij = np.linalg.norm(ekfs[i].pose[:2] - ekfs[j].pose[:2])
                    if d_ij <= COMM_RANGE:
                        merge_maps(ekfs[i], ekfs[j])
                        merge_maps(ekfs[j], ekfs[i])

        # --- Snapshot: map quality (every 10 s) ---
        if int(sim_time / 10.0) != int((sim_time - DT) / 10.0):
            merged = ekfs[0]   # drone 0 holds the most-merged map in distributed mode
            if merged.n_landmarks > 0:
                dists_to_true = np.array([
                    np.min(np.linalg.norm(true_landmarks - lm, axis=1))
                    for lm in merged.landmarks
                ])
                found = np.sum(dists_to_true < 1.0)
                rms = np.sqrt(np.mean(dists_to_true[dists_to_true < 1.0]**2)) \
                      if found > 0 else np.nan
                coverage = found / M_LANDMARKS
            else:
                rms, coverage = np.nan, 0.0
            history.append((sim_time, rms, coverage))

    return ekfs, history


def run_simulation():
    rng = np.random.default_rng(42)
    true_landmarks = rng.uniform(10.0, 190.0, size=(M_LANDMARKS, 2))

    results = {}
    for strategy in ("distributed", "isolated"):
        ekfs, history = simulate_swarm_slam(true_landmarks, strategy=strategy, seed=0)
        results[strategy] = (ekfs, history)
        final = history[-1] if history else (T_MAX, np.nan, 0.0)
        print(f"[{strategy:>12s}] coverage={final[2]*100:.1f}%  RMS={final[1]:.3f} m")

    return results, true_landmarks
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Environment size | 200 × 200 m |
| Number of drones $N$ | 3 |
| True landmarks $M_{true}$ | 30 |
| Drone cruise speed $v$ | 3.0 m/s |
| Simulation timestep $\Delta t$ | 0.2 s |
| Mission horizon $T_{max}$ | 600 s |
| Sensor range $r_{sense}$ | 25.0 m |
| Communication range $r_{comm}$ | 40.0 m |
| Range noise std $\sigma_\rho$ | 0.3 m |
| Bearing noise std $\sigma_\phi$ | 0.03 rad ($\approx$ 1.7 deg) |
| Pose process noise $\sigma_{ax}, \sigma_{ay}$ | 0.05 m/s² |
| Heading process noise $\sigma_\omega$ | 0.02 rad/s |
| Data association gate $\chi^2_{2, 0.95}$ | 5.991 |
| Map merging minimum common landmarks | 3 |
| Common landmark proximity threshold | 2.0 m |
| Landmark localisation success radius | 1.0 m |
| Initial pose covariance $\mathbf{P}_0$ | $0.01 \cdot \mathbf{I}_3$ |

---

## Expected Output

- **Ground truth map**: 2D scatter of 30 true landmark positions on the 200 × 200 m canvas;
  drone starting positions marked; initial sensor range circles drawn.
- **Trajectory and map comparison panel**: three-column figure (one per drone for isolated mode;
  one merged view for distributed mode) showing drone paths as coloured trails, estimated landmark
  positions as crosses with uncertainty ellipses ($1\sigma$ covariance), true positions as filled
  circles; matching lines between estimated and true pairs.
- **Coverage vs time curve**: $M_{found}(t) / M_{true}$ plotted against simulation time for
  distributed swarm, isolated drone 0, and centralised baseline; shaded bands showing min/max
  across 20 Monte Carlo trials.
- **RMS error vs time**: landmark position RMS error over time for the same three strategies;
  distributed swarm expected to converge toward the centralised upper bound as map merges
  accumulate.
- **Communication event log**: vertical tick marks on the time axis indicating each pairwise
  map-merge event; annotated with the number of common landmarks found per merge.
- **Covariance ellipse animation (GIF)**: top-down view showing drone positions (coloured
  triangles pointing in heading direction), estimated landmark ellipses growing and then shrinking
  as observations accumulate, communication links flashing when drones come within $r_{comm}$;
  frame rate 5 fps.
- **Final merged map heatmap**: 2D plot of landmark uncertainty magnitude $\|\mathbf{P}_{ll}\|_F$
  at mission end; colour-coded from blue (well-localised) to red (high uncertainty or not yet seen).

---

## Extensions

1. **Loop closure detection**: when a drone re-visits a previously mapped area after a long
   excursion, accumulated odometry drift may have shifted its coordinate frame; implement a
   graph-SLAM back-end (pose graph with loop closure constraints solved via $g^2o$-style
   Gauss-Newton) to globally correct the trajectory and landmark estimates.
2. **3D extension — altitude-varying landmarks**: extend the sensor to range-bearing-elevation
   ($\rho, \phi, \epsilon$) and the landmark state to 3D $(l_x, l_y, l_z)$; drone altitude
   variation introduces parallax that improves depth estimation of tall structures.
3. **Dynamic obstacles as false landmarks**: introduce moving objects that generate spurious
   observations; implement a landmark persistence model (track age and observation frequency) to
   prune transient features and prevent contamination of the static map.
4. **Heterogeneous sensor suite**: equip one drone with a wide-angle low-accuracy sensor
   ($\sigma_\rho = 1.0$ m, $r_{sense} = 50$ m) and two with narrow high-accuracy sensors
   ($\sigma_\rho = 0.1$ m, $r_{sense} = 15$ m); design an adaptive communication policy that
   prioritises merging the wide-angle scout's rough map first to seed the high-accuracy drones'
   search frontiers.
5. **Adversarial spoofing**: a jammer injects false range-bearing measurements targeting one drone;
   implement an outlier-robust association scheme using the Mahalanobis gate combined with a
   RANSAC-style consensus check across multiple timesteps before accepting a new landmark.

---

## Related Scenarios

- Prerequisites: [S041 Area Coverage Sweep](S041_wildfire_boundary.md), [S046 Trilateration Localisation](S046_trilateration.md), [S049 Dynamic Zone Search](S049_dynamic_zone.md)
- Follow-ups: [S051 Post-Disaster Communication Restoration](S051_post_disaster_comm.md)
- Algorithmic cross-reference: [S013 Particle Filter Intercept](../01_pursuit_evasion/S013_particle_filter_intercept.md) (probabilistic state estimation), [S042 Missing Person Localisation](S042_missing_person.md) (Bayesian belief maps), [S047 Signal Relay](S047_signal_relay.md) (inter-drone communication topology)
