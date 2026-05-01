# S047 Base Station Signal Relay

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A ground base station (BS) at a known position and a rescue team deep in a mountain
valley are out of direct radio range. The intervening terrain — ridge lines, dense vegetation,
and valley walls — creates a shadowed corridor where direct line-of-sight is blocked. Three relay
drones must be positioned in the airspace above this corridor to form a communication chain that
bridges the BS and the rescue team. Each drone acts as a store-and-forward repeater; the quality
of the end-to-end link is governed by the **weakest** link in the chain (the bottleneck). The
drones are free to hover at any (x, y) position within their allowed operating area; the
optimisation task is to find the three relay positions that maximise the minimum single-hop SNR
across all four links in the chain.

**Roles**:
- **Base Station (BS)**: fixed transmitter/receiver at $\mathbf{p}_0$; transmit power $P_{tx,0}$.
- **Relay Drone 1 (R1)**: free to hover; repeats signal between BS and R2.
- **Relay Drone 2 (R2)**: free to hover; repeats signal between R1 and R3.
- **Relay Drone 3 (R3)**: free to hover; repeats signal between R2 and the rescue team.
- **Rescue Team (T)**: fixed endpoint at $\mathbf{p}_4$; receive-only for optimisation purposes.

**Objective**: Find relay positions $\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3$ that maximise the
minimum link SNR across all four hops in the chain:

$$\max_{\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3} \; \min_{k \in \{0,1,2,3\}} \text{SNR}_k$$

subject to the constraint that each consecutive pair of nodes is within the maximum communication
range $d_{max}$, and that each relay drone remains within its permitted operating bounding box.

**Comparison strategies**:
1. **Uniform spacing** — place relays at equal fractional distances along the straight line from
   BS to rescue team.
2. **Greedy hop-by-hop** — place each relay to maximise the SNR of the current hop, ignoring
   downstream links.
3. **Gradient ascent on min-SNR** — iterative coordinate ascent that directly optimises the
   bottleneck objective.

---

## Mathematical Model

### Log-Distance Path Loss with Terrain Shadowing

The received power on a single link of length $d$ (metres) follows the log-distance model with
additive shadowing:

$$L(d) = L_0 + 10 n \log_{10}\!\left(\frac{d}{d_0}\right) + X_\sigma \quad \text{(dB)}$$

where:
- $L_0$ — path loss at reference distance $d_0 = 1$ m (free-space, computed from carrier
  frequency $f_c$):

$$L_0 = 20 \log_{10}\!\left(\frac{4\pi d_0 f_c}{c}\right)$$

- $n$ — path loss exponent ($n = 2.0$ in free space; $n = 2.8$ in mixed terrain/vegetation);
- $X_\sigma \sim \mathcal{N}(0, \sigma_{sh}^2)$ — zero-mean Gaussian shadowing in dB with
  standard deviation $\sigma_{sh}$. For the purposes of this deterministic optimisation the
  expected value $X_\sigma = 0$ is used, except inside predefined **shadow zones** (terrain
  obstructions) where an additional fixed shadowing penalty $\Delta_{shadow}$ is applied.

### Per-Link SNR

For link $k$ connecting node $k$ at position $\mathbf{p}_k$ to node $k+1$ at $\mathbf{p}_{k+1}$
with hop distance $d_k = \|\mathbf{p}_{k+1} - \mathbf{p}_k\|$:

$$\text{SNR}_k \;[\text{dB}] = P_{tx} - L(d_k) - N_{floor}$$

where $P_{tx}$ is the transmit power (dBm) at node $k$ (equal for all nodes) and $N_{floor}$ is
the receiver noise floor (dBm). A link is considered viable if $\text{SNR}_k \geq \text{SNR}_{min}$.

### Terrain Shadowing Penalty

The terrain map is represented as a set of $N_s$ convex shadow polygons
$\mathcal{Z} = \{Z_1, \ldots, Z_{N_s}\}$. The effective path loss for link $k$ is:

$$L_{eff}(d_k, \mathbf{p}_k, \mathbf{p}_{k+1}) = L(d_k) + \Delta_{shadow} \cdot
  \mathbf{1}\!\left[\text{midpoint}(\mathbf{p}_k, \mathbf{p}_{k+1}) \in \bigcup_j Z_j\right]$$

where $\mathbf{1}[\cdot]$ is the indicator function and $\Delta_{shadow} = 15$ dB is the
shadowing penalty for a link whose midpoint lies inside an obstruction zone.

### Bottleneck SNR Objective

Let the chain nodes be $\mathbf{p}_0$ (BS), $\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3$
(relays), $\mathbf{p}_4$ (rescue team). The bottleneck SNR is:

$$J(\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3)
  = \min_{k=0}^{3} \text{SNR}_k(\mathbf{p}_k, \mathbf{p}_{k+1})$$

The optimisation maximises this non-smooth, non-convex scalar:

$$(\mathbf{p}_1^*, \mathbf{p}_2^*, \mathbf{p}_3^*) = \arg\max_{\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3} J$$

### Max-Comm-Range Constraint

Each hop must remain within the maximum communication range:

$$d_k = \|\mathbf{p}_{k+1} - \mathbf{p}_k\| \leq d_{max}, \quad k = 0, 1, 2, 3$$

This constraint is enforced during gradient ascent by projecting any step that would violate it
back to the feasible boundary (clipping the hop distance to $d_{max}$).

### Coordinate Ascent (Gradient Ascent on Min-SNR)

Because $J$ is the minimum of smooth functions and is therefore non-differentiable at tie points,
gradient ascent uses a smooth approximation — the soft minimum:

$$J_{soft}(\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3)
  = -\frac{1}{\alpha} \ln \!\left(\sum_{k=0}^{3} e^{-\alpha \cdot \text{SNR}_k}\right)$$

with temperature $\alpha = 0.5$. As $\alpha \to \infty$, $J_{soft} \to J$.

The gradient with respect to relay position $\mathbf{p}_i$ ($i \in \{1,2,3\}$) is:

$$\frac{\partial J_{soft}}{\partial \mathbf{p}_i}
  = \frac{\sum_{k \in \{i-1,\, i\}} w_k \cdot \frac{\partial \text{SNR}_k}{\partial \mathbf{p}_i}}
         {\sum_{k=0}^{3} e^{-\alpha \cdot \text{SNR}_k}}$$

where the soft-min weights are $w_k = e^{-\alpha \cdot \text{SNR}_k}$ and relay $i$ appears in
links $k = i-1$ and $k = i$. The gradient of a single SNR with respect to relay position:

$$\frac{\partial \text{SNR}_k}{\partial \mathbf{p}_i}
  = -\frac{10 n}{\ln 10} \cdot \frac{1}{d_k^2} \cdot (\mathbf{p}_i - \mathbf{p}_{k'})$$

where $\mathbf{p}_{k'}$ is the other endpoint of link $k$ from $\mathbf{p}_i$ (sign depends on
which end relay $i$ occupies).

Update rule with step size $\eta$:

$$\mathbf{p}_i^{(t+1)} = \mathbf{p}_i^{(t)} + \eta \cdot
  \frac{\partial J_{soft}}{\partial \mathbf{p}_i}\bigg|_{\mathbf{p}^{(t)}}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

# Key constants
N_RELAYS      = 3           # number of relay drones
F_C           = 2.4e9       # Hz — carrier frequency (2.4 GHz ISM band)
C_LIGHT       = 3e8         # m/s
D_0           = 1.0         # m — reference distance
N_LOSS        = 2.8         # path loss exponent (mixed terrain)
P_TX          = 30.0        # dBm — transmit power (all nodes)
N_FLOOR       = -95.0       # dBm — receiver noise floor
SNR_MIN       = 10.0        # dB — minimum viable SNR
D_MAX         = 600.0       # m — maximum comm range per hop
DELTA_SHADOW  = 15.0        # dB — terrain shadowing penalty
ALPHA_SOFT    = 0.5         # soft-min temperature
ETA           = 5.0         # m — gradient ascent step size
N_ITER        = 2000        # gradient ascent iterations

# Fixed endpoints
BS    = np.array([100.0,  500.0])    # base station
TEAM  = np.array([900.0,  500.0])    # rescue team in valley

# Shadow zones: list of (centre_x, centre_y, half_width, half_height)
SHADOW_ZONES = [
    (350.0, 480.0, 80.0, 120.0),    # ridge 1
    (500.0, 520.0, 70.0, 100.0),    # valley wall centre
    (650.0, 460.0, 90.0, 130.0),    # ridge 2
]

def L0_db(d0=D_0, fc=F_C):
    """Free-space path loss at reference distance d0."""
    return 20 * np.log10(4 * np.pi * d0 * fc / C_LIGHT)

L0 = L0_db()

def in_shadow(midpoint, zones):
    """Check if a midpoint falls inside any shadow zone (axis-aligned rectangles)."""
    mx, my = midpoint
    for (cx, cy, hw, hh) in zones:
        if abs(mx - cx) <= hw and abs(my - cy) <= hh:
            return True
    return False

def path_loss_db(d, midpoint, zones=SHADOW_ZONES):
    """Log-distance path loss with terrain shadowing penalty."""
    d = max(d, 1e-3)
    lfs = L0 + 10 * N_LOSS * np.log10(d / D_0)
    shadow = DELTA_SHADOW if in_shadow(midpoint, zones) else 0.0
    return lfs + shadow

def snr_link(p_a, p_b):
    """SNR (dB) for a single hop from p_a to p_b."""
    d = np.linalg.norm(p_b - p_a)
    mid = 0.5 * (p_a + p_b)
    return P_TX - path_loss_db(d, mid) - N_FLOOR

def chain_snrs(positions):
    """Compute SNR for each of the 4 links given all 5 node positions."""
    snrs = []
    for k in range(len(positions) - 1):
        snrs.append(snr_link(positions[k], positions[k + 1]))
    return np.array(snrs)

def soft_min(snrs, alpha=ALPHA_SOFT):
    """Differentiable approximation of min(snrs)."""
    return -(1.0 / alpha) * np.log(np.sum(np.exp(-alpha * snrs)))

def bottleneck_snr(relay_positions):
    """Min SNR across the chain given relay positions (shape: 3×2)."""
    positions = np.vstack([BS, relay_positions, TEAM])
    return np.min(chain_snrs(positions))

def gradient_ascent(init_relays, eta=ETA, n_iter=N_ITER, alpha=ALPHA_SOFT):
    """Coordinate ascent on soft-min SNR to find optimal relay positions."""
    relays = init_relays.copy()
    history = []

    for _ in range(n_iter):
        positions = np.vstack([BS, relays, TEAM])
        snrs = chain_snrs(positions)
        history.append(np.min(snrs))

        # Numerical gradient for each relay
        grad = np.zeros_like(relays)
        eps = 1.0
        for i in range(N_RELAYS):
            for j in range(2):   # x, y
                relays_fwd = relays.copy(); relays_fwd[i, j] += eps
                relays_bwd = relays.copy(); relays_bwd[i, j] -= eps
                J_fwd = soft_min(chain_snrs(np.vstack([BS, relays_fwd, TEAM])), alpha)
                J_bwd = soft_min(chain_snrs(np.vstack([BS, relays_bwd, TEAM])), alpha)
                grad[i, j] = (J_fwd - J_bwd) / (2 * eps)

        relays += eta * grad

        # Enforce max-comm-range constraints along the chain
        all_nodes = np.vstack([BS, relays, TEAM])
        for i in range(1, 4):    # relay indices in full chain
            prev_node = all_nodes[i - 1]
            hop_vec = all_nodes[i] - prev_node
            hop_dist = np.linalg.norm(hop_vec)
            if hop_dist > D_MAX:
                all_nodes[i] = prev_node + hop_vec / hop_dist * D_MAX
        relays = all_nodes[1:4].copy()

        # Clip to operating area
        relays = np.clip(relays, 0.0, 1000.0)

    return relays, np.array(history)

def uniform_spacing():
    """Baseline: equally spaced relays along the BS-to-team straight line."""
    return np.array([
        BS + (i / (N_RELAYS + 1)) * (TEAM - BS)
        for i in range(1, N_RELAYS + 1)
    ])

def run_simulation():
    init = uniform_spacing()

    # Strategy 1: uniform spacing (no optimisation)
    snrs_uniform = chain_snrs(np.vstack([BS, init, TEAM]))

    # Strategy 2: greedy hop-by-hop (place each relay to maximise its own link SNR)
    # Simplified: move each relay to midpoint of its assigned span
    greedy = init.copy()
    snrs_greedy = chain_snrs(np.vstack([BS, greedy, TEAM]))

    # Strategy 3: gradient ascent
    opt_relays, history = gradient_ascent(init.copy())
    snrs_opt = chain_snrs(np.vstack([BS, opt_relays, TEAM]))

    results = {
        "uniform":   {"positions": init,        "snrs": snrs_uniform},
        "greedy":    {"positions": greedy,       "snrs": snrs_greedy},
        "optimised": {"positions": opt_relays,   "snrs": snrs_opt},
    }
    return results, history
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Carrier frequency $f_c$ | 2.4 GHz |
| Reference distance $d_0$ | 1 m |
| Path loss exponent $n$ | 2.8 (mixed terrain) |
| Transmit power $P_{tx}$ | 30 dBm |
| Noise floor $N_{floor}$ | -95 dBm |
| Minimum viable SNR | 10 dB |
| Max comm range $d_{max}$ | 600 m |
| Terrain shadow penalty $\Delta_{shadow}$ | 15 dB |
| Number of relay drones | 3 |
| Shadow zone count | 3 obstructions |
| Soft-min temperature $\alpha$ | 0.5 |
| Gradient ascent step $\eta$ | 5 m / iteration |
| Gradient ascent iterations | 2000 |
| Operating area | 1000 x 1000 m |
| Base station position | (100, 500) m |
| Rescue team position | (900, 500) m |

---

## Expected Output

- **Terrain map**: 2D top-down plot (1000 x 1000 m) showing shadow zones as grey-shaded
  rectangles, BS (black square), rescue team (green triangle), and final relay positions for each
  strategy (markers colour-coded by strategy); link lines drawn between consecutive chain nodes,
  annotated with per-link SNR values in dB.
- **SNR chain bar chart**: grouped bar chart with four link bars per strategy (uniform / greedy /
  optimised), with a horizontal dashed line at $\text{SNR}_{min} = 10$ dB; bottleneck link
  highlighted with a red outline.
- **Optimisation convergence curve**: bottleneck SNR (dB) vs gradient ascent iteration number;
  shows rapid initial improvement followed by convergence to the plateau.
- **Bottleneck SNR comparison table**: single-number summary of $\min_k \text{SNR}_k$ for each
  strategy, displayed as a horizontal bar chart (uniform / greedy / optimised).
- **Relay position trajectories**: scatter animation or step-plot showing how relay positions
  $\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3$ migrate across the terrain map during gradient
  ascent, converging away from shadow zones and balancing link distances.

---

## Extensions

1. **3D altitude optimisation**: extend relay positions to $(x, y, z)$, allowing drones to climb
   above ridge lines to avoid shadow zones entirely; add an altitude penalty term (energy cost)
   to balance SNR gain against flight endurance.
2. **Stochastic shadowing**: replace fixed $X_\sigma = 0$ with Monte Carlo realisation of
   $X_\sigma \sim \mathcal{N}(0, \sigma_{sh}^2)$ per link per evaluation; optimise the
   expected bottleneck SNR or the 10th-percentile (robust design).
3. **Dynamic relay — moving rescue team**: the rescue team moves at 1 m/s; implement a
   receding-horizon controller that re-solves the relay placement every 30 s to track the
   team while maintaining chain integrity.
4. **Heterogeneous transmit power**: assign individual $P_{tx,i}$ budgets per relay drone
   subject to $\sum_i P_{tx,i} \leq P_{total}$; co-optimise positions and power allocation
   (convex subproblem at fixed positions).
5. **Relay count selection**: vary $N_{relays} \in \{1, 2, 3, 4, 5\}$ and plot the trade-off
   between bottleneck SNR improvement and drone resource cost; identify the minimum relay count
   that achieves $\text{SNR}_{min}$ on all links.
6. **Communication network expansion**: connect to [S051](S051_post_disaster_comm_network.md)
   by treating the optimal relay chain as a backbone link within a wider mesh network formed
   after a disaster event.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary_scan.md), [S046 3D Trilateration](S046_3d_trilateration.md)
- Follow-ups: [S048 Lawnmower Coverage](S048_lawnmower_coverage.md), [S051 Post-Disaster Comm Network](S051_post_disaster_comm_network.md)
- Algorithmic cross-reference: [S059 Sonar Buoy Relay](S059_sonar_buoy_relay.md) (relay chain in underwater acoustics), [S049 Dynamic Zone Assignment](S049_dynamic_zone_assignment.md) (multi-agent positioning under constraints)
