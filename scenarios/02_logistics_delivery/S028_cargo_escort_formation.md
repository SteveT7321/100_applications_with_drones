# S028 Cargo Escort Formation

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: A single cargo drone (Carrier) follows a pre-planned straight-line route from a depot to a delivery waypoint. Three escort drones (Escort-1, Escort-2, Escort-3) fly in a protective diamond formation around the Carrier to shield it from lateral threats and turbulence. The Carrier flies at constant speed; each escort must maintain a designated offset position relative to the Carrier while the entire formation navigates through the arena.

**Roles**:
- **Carrier**: slow, heavy drone following a fixed waypoint-to-waypoint route; must arrive within a positional tolerance of the goal
- **Escort-1 (Lead)**: flies directly ahead of the Carrier along the travel direction
- **Escort-2 (Left)**: flies to the port side of the Carrier
- **Escort-3 (Right)**: flies to the starboard side of the Carrier
- **Escort-4 (Rear)**: flies directly behind the Carrier (optional; included when N = 4)

The default configuration uses N = 3 escorts in a triangle; the Key Parameters section documents both N = 3 and N = 4 variants.

**Objective**:
1. Minimize formation error — the RMS deviation of each escort from its designated slot position over the entire flight
2. Ensure the Carrier reaches the goal waypoint within the arrival tolerance
3. Avoid inter-drone collisions (minimum separation > $d_{safe}$)

**Constraints**:
- All drones are point-mass kinematic models with velocity-clamped commands
- Carrier max speed: $v_C = 3.0$ m/s; escort max speed: $v_E = 6.0$ m/s
- Arena: $[-10, 10]^2 \times [0, 5]$ m
- Formation offsets are defined in the Carrier body frame and rotate with the Carrier heading

---

## Mathematical Model

### Formation Slot Positions

Let $\mathbf{p}_C \in \mathbb{R}^3$ be the Carrier position and $\hat{\mathbf{u}} = \mathbf{v}_C / \|\mathbf{v}_C\|$ be the unit heading vector. Define the lateral unit vector $\hat{\mathbf{l}} = \hat{\mathbf{z}} \times \hat{\mathbf{u}}$ (horizontal cross-product of world-up and heading).

The desired slot position for escort $i$ is:

$$\mathbf{s}_i(t) = \mathbf{p}_C(t) + f_{long,i} \cdot \hat{\mathbf{u}}(t) + f_{lat,i} \cdot \hat{\mathbf{l}}(t) + f_{alt,i} \cdot \hat{\mathbf{z}}$$

Default slot offsets for the N = 3 triangle (Lead, Left, Right):

| Escort | $f_{long}$ (m) | $f_{lat}$ (m) | $f_{alt}$ (m) |
|--------|---------------|--------------|--------------|
| Lead (E1) | +2.0 | 0.0 | 0.0 |
| Left (E2) | -1.5 | +2.0 | 0.0 |
| Right (E3) | -1.5 | -2.0 | 0.0 |

### Carrier Guidance

The Carrier follows a straight-line PD position controller to the goal $\mathbf{g}$:

$$\mathbf{v}_{C,cmd}(t) = K_{pC} \cdot (\mathbf{g} - \mathbf{p}_C) - K_{dC} \cdot \mathbf{v}_C$$

$$\mathbf{v}_{C,cmd} \leftarrow v_C \cdot \frac{\mathbf{v}_{C,cmd}}{\max\!\left(\|\mathbf{v}_{C,cmd}\|,\; \varepsilon\right)}$$

where $K_{pC} = 1.5$, $K_{dC} = 0.4$, and the clamped speed is $v_C = 3.0$ m/s.

### Escort Guidance — Virtual Structure Control

Each escort uses a virtual-structure (VS) law that tracks its moving slot $\mathbf{s}_i(t)$:

$$\mathbf{v}_{E_i,cmd}(t) = K_{pE} \cdot \underbrace{\left(\mathbf{s}_i(t) - \mathbf{p}_{E_i}(t)\right)}_{\text{slot error } \boldsymbol{\delta}_i} + \mathbf{v}_{C}(t)$$

The feed-forward term $\mathbf{v}_C(t)$ prevents lag when the formation translates. The command is clamped to $v_E = 6.0$ m/s.

### Formation Error Metric

Instantaneous formation error for escort $i$:

$$e_i(t) = \|\mathbf{p}_{E_i}(t) - \mathbf{s}_i(t)\|$$

RMS formation error over the mission of duration $T$:

$$\bar{e}_{rms} = \sqrt{\frac{1}{N \cdot T} \sum_{i=1}^{N} \int_0^T e_i^2(t)\, dt}$$

Discretised:

$$\bar{e}_{rms} = \sqrt{\frac{1}{N \cdot M} \sum_{i=1}^{N} \sum_{k=1}^{M} e_i(k \Delta t)^2}$$

### Collision Avoidance Between Escorts

A pairwise repulsion force is added to each escort's velocity command when two drones come within a safety distance:

$$\mathbf{v}_{rep,ij} = \begin{cases}
k_{rep} \cdot \dfrac{d_{safe} - d_{ij}}{d_{ij}} \cdot \hat{\mathbf{r}}_{ij} & \text{if } d_{ij} < d_{safe} \\
\mathbf{0} & \text{otherwise}
\end{cases}$$

where $d_{ij} = \|\mathbf{p}_{E_i} - \mathbf{p}_{E_j}\|$, $\hat{\mathbf{r}}_{ij} = (\mathbf{p}_{E_i} - \mathbf{p}_{E_j})/d_{ij}$, $k_{rep} = 3.0$, and $d_{safe} = 1.0$ m.

### Heading Angle

When $\|\mathbf{v}_C\| > \varepsilon$, the Carrier heading angle used to rotate the formation frame is:

$$\psi_C(t) = \arctan2\!\left(v_{C,y}(t),\; v_{C,x}(t)\right)$$

The rotation matrix from body to world frame (z-axis only):

$$\mathbf{R}(\psi_C) = \begin{bmatrix} \cos\psi_C & -\sin\psi_C & 0 \\ \sin\psi_C & \cos\psi_C & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

so $\mathbf{s}_i = \mathbf{p}_C + \mathbf{R}(\psi_C) \cdot \mathbf{f}_i$ where $\mathbf{f}_i = [f_{long,i},\; f_{lat,i},\; f_{alt,i}]^\top$.

---

## Implementation

```python
# Key constants
N_ESCORTS   = 3
V_CARRIER   = 3.0    # m/s — carrier cruising speed
V_ESCORT    = 6.0    # m/s — escort max speed
K_PC        = 1.5    # carrier position gain
K_DC        = 0.4    # carrier damping gain
K_PE        = 3.0    # escort slot-tracking gain
K_REP       = 3.0    # inter-drone repulsion gain
D_SAFE      = 1.0    # m — minimum separation
DT          = 1/48   # s — simulation timestep
T_MAX       = 30.0   # s

# Formation offsets in body frame [long, lat, alt]
SLOT_OFFSETS = np.array([
    [ 2.0,  0.0,  0.0],   # Lead
    [-1.5,  2.0,  0.0],   # Left
    [-1.5, -2.0,  0.0],   # Right
])

def rotation_z(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def slot_positions(pos_c, vel_c):
    psi = np.arctan2(vel_c[1], vel_c[0])
    R = rotation_z(psi)
    return np.array([pos_c + R @ f for f in SLOT_OFFSETS])

def escort_velocity(pos_e, slot, vel_c):
    delta = slot - pos_e
    v_cmd = K_PE * delta + vel_c
    speed = np.linalg.norm(v_cmd)
    if speed > V_ESCORT:
        v_cmd = v_cmd / speed * V_ESCORT
    return v_cmd

def repulsion(pos_ei, pos_ej):
    r_vec = pos_ei - pos_ej
    d = np.linalg.norm(r_vec) + 1e-8
    if d < D_SAFE:
        return K_REP * (D_SAFE - d) / d * (r_vec / d)
    return np.zeros(3)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Carrier start | (-8, 0, 2) m |
| Carrier goal | (8, 0, 2) m |
| Escort starts | slots at $t=0$ |
| Carrier speed | 3.0 m/s |
| Escort max speed | 6.0 m/s |
| Slot offsets (Lead) | (+2.0, 0, 0) m body |
| Slot offsets (Left) | (-1.5, +2.0, 0) m body |
| Slot offsets (Right) | (-1.5, -2.0, 0) m body |
| Escort position gain $K_{pE}$ | 3.0 |
| Repulsion gain $k_{rep}$ | 3.0 |
| Safety distance $d_{safe}$ | 1.0 m |
| Arrival tolerance | 0.2 m |
| Simulation timestep $\Delta t$ | 1/48 s |
| Max simulation time | 30 s |

---

## Expected Output

- 3D trajectory plot: Carrier (green), Escort-1 (red), Escort-2 (blue), Escort-3 (orange), goal marker (green star)
- Top-down XY view showing diamond/triangle formation maintaining shape during transit
- Formation error $e_i(t)$ vs time for each escort (should converge to near zero within ~2 s and remain stable)
- RMS formation error $\bar{e}_{rms}$ printed to console (target: < 0.15 m)
- Inter-drone separation vs time (minimum should remain above $d_{safe}$)
- Animated GIF showing the full escort transit

---

## Extensions

1. Add a lateral wind disturbance $\mathbf{w}(t) = [0,\; w_y \sin(\omega t),\; 0]$ and compare RMS error with/without wind compensation
2. Carrier follows a curved S-route (3 waypoints) requiring dynamic formation rotation; test heading-lag performance
3. Replace the virtual-structure law with a consensus-based controller: each escort only communicates with its two neighbours
4. Introduce a threat drone that approaches the Carrier; one escort breaks formation to intercept (combine with S001 PNG guidance)
5. Scale to N = 6 escorts in a hexagonal ring formation and compare RMS error vs N = 3 triangle
6. 3D formation: stagger escort altitudes ($\pm 1$ m vertical offset) and add vertical slot tracking

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md), [S026](S026_cooperative_heavy_lift.md)
- Follow-ups: [S029](S029_urban_logistics_scheduling.md), [S031](S031_path_deconfliction.md)
- Domain 1 analogue: [S011](../01_pursuit_evasion/S011_swarm_encirclement.md) (virtual-structure coordination pattern)
