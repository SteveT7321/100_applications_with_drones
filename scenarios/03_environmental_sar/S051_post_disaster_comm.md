# S051 Post-Disaster Communication Network Restoration

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Max-coverage greedy set-cover + connectivity repair (MST / relay chain) | **Dimension**: 2D

---

## Problem Definition

**Setup**: A natural disaster has destroyed all ground communication infrastructure across a
$200 \times 200$ m zone. $K = 8$ survivor camps are scattered throughout the area; none of them
can communicate with each other or with the rescue base station located at the zone boundary. A
fleet of $N = 5$ drones is deployed to act as airborne relay nodes hovering at a fixed altitude.
Each drone has a communication radius $r_{comm} = 30$ m within which it can relay packets between
two endpoints. A camp is considered **covered** if it falls within $r_{comm}$ of at least one drone.
A drone placement configuration is **connected** if the induced graph — nodes are drones plus the
base station, edges connect pairs within $r_{comm}$ of each other — is a single connected component.
Battery-life limits impose a maximum relay distance from the base: no drone may be placed farther
than $D_{max} = 120$ m from the base station (Manhattan distance along the relay chain is not
directly constrained, but total chain length is bounded by the reachable hover grid).

**Roles**:
- **Base station**: fixed anchor at position $(0, 100)$ m (left edge midpoint); always included as
  a node in the connectivity graph.
- **Drones** ($N = 5$): homogeneous relay nodes; each hovers at one position selected from a
  discrete candidate grid; communication range $r_{comm}$ is symmetric and omnidirectional.
- **Survivor camps** ($K = 8$): fixed demand points at known positions; a camp is served if any
  drone covers it.

**Objective**: Choose $N$ hover positions from a candidate grid that **maximise the fraction of
covered camps** subject to the constraint that the relay graph (drones + base station) is
**connected**. If full coverage is not achievable, the algorithm reports the maximum attainable
coverage rate and the minimum number of drones needed for full coverage (sensitivity analysis).

**Comparison strategies**:
1. **Greedy max-coverage (no connectivity constraint)** — pure set-cover greedy; ignores whether
   the resulting relay graph is connected.
2. **Greedy + connectivity repair (2-opt swaps)** — start from the greedy solution, then apply
   position swaps to repair disconnected components while minimising coverage loss.
3. **MST-anchored relay chain** — build a minimum spanning tree over camps and base station, place
   drones along MST edges to guarantee connectivity, then maximise residual coverage.

---

## Mathematical Model

### Candidate Grid and Coverage Sets

Discretise the $200 \times 200$ m area into a grid with resolution $\delta = 5$ m, yielding
$G = 40 \times 40 = 1600$ candidate hover positions. Let $\mathcal{P} = \{g_1, \ldots, g_G\}$
be the candidate set. For each candidate position $g \in \mathcal{P}$ define its **coverage set**:

$$C(g) = \bigl\{c \in \mathcal{C} : \|g - c\| \leq r_{comm}\bigr\}$$

where $\mathcal{C} = \{c_1, \ldots, c_K\}$ is the set of camp positions. A camp $c$ is covered by
a placement $S \subseteq \mathcal{P}$ of $N$ drones if $\exists\, g \in S : c \in C(g)$.

### Max-$N$-Coverage Objective

$$\max_{S \subseteq \mathcal{P},\; |S| = N} \; f(S) := \left|\bigcup_{g \in S} C(g)\right|$$

This is the **maximum $k$-coverage problem**, an NP-hard generalisation of set cover. The greedy
$(1 - 1/e) \approx 63.2\%$ approximation guarantee applies: at each step, add the candidate with
maximum **marginal gain**:

$$\Delta(g \mid S) = \bigl|C(g) \setminus \bigcup_{q \in S} C(q)\bigr|$$

$$g^* = \arg\max_{g \in \mathcal{P} \setminus S} \Delta(g \mid S), \qquad S \leftarrow S \cup \{g^*\}$$

### Connectivity Constraint

Define the relay graph $\mathcal{G}(S) = (V, E)$ where $V = S \cup \{b\}$ ($b$ = base station)
and:

$$E = \bigl\{(u, v) : \|u - v\| \leq r_{comm},\; u, v \in V\bigr\}$$

The placement $S$ is **connected** iff $\mathcal{G}(S)$ is a single connected component,
verified via BFS/DFS from $b$:

$$\text{connected}(S) = \bigl(|\text{BFS}(b, \mathcal{G}(S))| = N + 1\bigr)$$

### Connectivity Repair via 2-opt Swaps

After the greedy phase, if $\mathcal{G}(S)$ is disconnected, let $\mathcal{K}$ be the set of
connected components not containing $b$. For each isolated component $K_i \in \mathcal{K}$,
find the swap that adds a bridge position with minimum coverage loss:

$$(\hat{g}_{out}, \hat{g}_{in}) = \arg\min_{\substack{g_{out} \in S \cap K_i \\ g_{in} \in \mathcal{P} \setminus S}} \;\bigl[f(S) - f\bigl((S \setminus \{g_{out}\}) \cup \{g_{in}\}\bigr)\bigr]$$

subject to $g_{in}$ reducing the number of disconnected components (i.e., $g_{in}$ is within
$r_{comm}$ of a node in a component already connected to $b$). Swaps are applied iteratively until
$\text{connected}(S) = \text{True}$.

### MST-Anchored Relay Chain (Baseline)

Construct a minimum spanning tree $T^*$ over the $K$ camps and base station $b$ using Euclidean
edge weights. For each MST edge $(u, v)$ with length $\ell_{uv} > r_{comm}$, place
$\lceil \ell_{uv} / r_{comm} \rceil - 1$ relay drones uniformly along the segment $[u, v]$,
snapping each to the nearest grid point in $\mathcal{P}$. If the total number of relays exceeds
$N$, prioritise edges incident to $b$ (backbone first). Residual drones (if any) are placed
greedily for coverage.

### Coverage and Network Metrics

| Metric | Formula |
|--------|---------|
| Camp coverage rate | $\displaystyle\eta = \frac{1}{K}\left|\bigcup_{g \in S} C(g)\right|$ |
| Relay hop count | $\displaystyle H = \max_{c \in \mathcal{C}_{cov}} \text{dist}_{\mathcal{G}}(b, g_c)$ where $g_c$ covers $c$ |
| Network diameter | $\displaystyle D_{net} = \max_{u,v \in V} \text{dist}_{\mathcal{G}}(u, v)$ (in hops) |
| Coverage loss from repair | $\displaystyle \Delta\eta_{repair} = f(S_{greedy}) - f(S_{repaired})$ (in camps) |
| Min drones for full coverage | smallest $N'$ such that $\max_{S, |S|=N'} f(S) = K$ |

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from collections import deque

# -------------------------------------------------------------------
# Key constants
# -------------------------------------------------------------------
AREA_SIZE    = 200.0      # m — square zone side length
GRID_RES     = 5.0        # m — candidate grid resolution
N_DRONES     = 5          # number of relay drones
K_CAMPS      = 8          # number of survivor camps
R_COMM       = 30.0       # m — communication / coverage radius
D_MAX        = 120.0      # m — maximum distance from base station
BASE_POS     = np.array([0.0, 100.0])   # base station position
SEED         = 42

# -------------------------------------------------------------------
# Build candidate hover grid
# -------------------------------------------------------------------
def build_candidate_grid(area_size, res):
    """Return (G, 2) array of candidate hover positions."""
    xs = np.arange(res / 2, area_size, res)
    ys = np.arange(res / 2, area_size, res)
    xv, yv = np.meshgrid(xs, ys, indexing='ij')
    return np.stack([xv.ravel(), yv.ravel()], axis=1)

# -------------------------------------------------------------------
# Connectivity check via BFS
# -------------------------------------------------------------------
def is_connected(selected_pos, base, r_comm):
    """
    Return True if the relay graph (selected_pos + base) is connected.
    selected_pos: (N, 2) array of drone positions.
    """
    nodes = np.vstack([base[np.newaxis], selected_pos])  # (N+1, 2)
    n = len(nodes)
    dist_mat = cdist(nodes, nodes)
    adj = dist_mat <= r_comm
    np.fill_diagonal(adj, False)

    visited = np.zeros(n, dtype=bool)
    queue = deque([0])   # start BFS from base (index 0)
    visited[0] = True
    while queue:
        u = queue.popleft()
        for v in np.where(adj[u])[0]:
            if not visited[v]:
                visited[v] = True
                queue.append(v)
    return visited.all()

def connected_components(selected_pos, base, r_comm):
    """
    Return list of component index sets.
    Node 0 = base; nodes 1..N = drones.
    """
    nodes = np.vstack([base[np.newaxis], selected_pos])
    n = len(nodes)
    dist_mat = cdist(nodes, nodes)
    adj = dist_mat <= r_comm
    np.fill_diagonal(adj, False)

    visited = np.zeros(n, dtype=bool)
    components = []
    for start in range(n):
        if visited[start]:
            continue
        comp = []
        queue = deque([start])
        visited[start] = True
        while queue:
            u = queue.popleft()
            comp.append(u)
            for v in np.where(adj[u])[0]:
                if not visited[v]:
                    visited[v] = True
                    queue.append(v)
        components.append(set(comp))
    return components

# -------------------------------------------------------------------
# Coverage helpers
# -------------------------------------------------------------------
def compute_coverage_sets(candidates, camps, r_comm):
    """
    Return list of frozensets: coverage_sets[i] = camps covered by candidates[i].
    """
    dist = cdist(candidates, camps)     # (G, K)
    covered = dist <= r_comm            # (G, K) bool
    return [frozenset(np.where(row)[0]) for row in covered]

def total_covered(selected_indices, coverage_sets):
    """Number of unique camps covered by the selected candidate positions."""
    union = set()
    for i in selected_indices:
        union |= coverage_sets[i]
    return len(union)

# -------------------------------------------------------------------
# Greedy max-coverage (no connectivity constraint)
# -------------------------------------------------------------------
def greedy_max_coverage(coverage_sets, n_drones, n_camps):
    """
    Standard greedy set-cover for max-k-coverage.
    Returns list of selected candidate indices.
    """
    selected = []
    covered  = set()
    for _ in range(n_drones):
        best_idx, best_gain = -1, -1
        for i, cs in enumerate(coverage_sets):
            if i in selected:
                continue
            gain = len(cs - covered)
            if gain > best_gain:
                best_gain = gain
                best_idx  = i
        if best_idx == -1 or best_gain == 0:
            # No further coverage gain; pick any unused position (placeholder)
            remaining = [i for i in range(len(coverage_sets)) if i not in selected]
            best_idx = remaining[0] if remaining else 0
        selected.append(best_idx)
        covered |= coverage_sets[best_idx]
    return selected

# -------------------------------------------------------------------
# Connectivity repair via 2-opt swaps
# -------------------------------------------------------------------
def repair_connectivity(selected, candidates, coverage_sets, base, r_comm):
    """
    Iteratively swap out isolated-component drones for bridge positions
    until the relay graph is connected. Returns repaired selection and
    the number of coverage-loss camps.
    """
    selected = list(selected)
    max_iters = 50

    for _ in range(max_iters):
        if is_connected(candidates[selected], base, r_comm):
            break

        comps = connected_components(candidates[selected], base, r_comm)
        # Find first component that does NOT contain the base (node 0)
        base_comp = next(c for c in comps if 0 in c)
        isolated_comp = next(c for c in comps if 0 not in c)

        # Drone indices within the isolated component (subtract 1 for base offset)
        isolated_drone_ids = [node - 1 for node in isolated_comp]
        # Candidate indices of drones in base_comp (for edge proximity check)
        base_nodes = np.array([candidates[selected[node - 1]]
                                for node in base_comp if node != 0])
        base_nodes = np.vstack([base[np.newaxis], base_nodes])

        best_swap = None
        best_loss = np.inf

        for out_id in isolated_drone_ids:
            out_sel_idx = selected[out_id]
            # Current coverage without this drone
            test_sel = [s for s in selected if s != out_sel_idx]
            cov_without = total_covered(test_sel, coverage_sets)

            # Find bridge candidates: within r_comm of base component
            for in_idx in range(len(candidates)):
                if in_idx in selected:
                    continue
                p_in = candidates[in_idx]
                # Bridge condition: close to base_comp node AND close to isolated node
                dist_to_base_comp = np.min(cdist([p_in], base_nodes))
                dist_to_isolated  = np.min(cdist(
                    [p_in],
                    candidates[[selected[j] for j in isolated_drone_ids]]
                ))
                if dist_to_base_comp > r_comm or dist_to_isolated > r_comm:
                    continue

                cov_with = total_covered(test_sel + [in_idx], coverage_sets)
                loss = cov_without - cov_with  # negative = gain
                if loss < best_loss:
                    best_loss = loss
                    best_swap = (out_id, in_idx)

        if best_swap is None:
            # No valid bridge found; fall back to connecting nearest pair
            break
        out_id, in_idx = best_swap
        selected[out_id] = in_idx

    coverage_loss = 0
    if best_swap is not None:
        coverage_loss = max(0, best_loss)
    return selected, coverage_loss

# -------------------------------------------------------------------
# MST-anchored relay chain baseline
# -------------------------------------------------------------------
def mst_relay_chain(camps, base, candidates, coverage_sets, n_drones, r_comm):
    """
    Build MST over camps + base, place relay drones along MST edges,
    then use residual drones for greedy coverage.
    Returns selected candidate indices.
    """
    from scipy.sparse.csgraph import minimum_spanning_tree
    from scipy.sparse import csr_matrix

    all_pts = np.vstack([base[np.newaxis], camps])   # (K+1, 2)
    dist_all = cdist(all_pts, all_pts)
    mst = minimum_spanning_tree(csr_matrix(dist_all)).toarray()

    relay_positions = []
    edges = np.argwhere(mst > 0)
    for u, v in edges:
        length = dist_all[u, v]
        n_relays = max(0, int(np.ceil(length / r_comm)) - 1)
        if n_relays == 0:
            continue
        for k in range(1, n_relays + 1):
            t = k / (n_relays + 1)
            p = (1 - t) * all_pts[u] + t * all_pts[v]
            # Snap to nearest grid candidate
            dists_to_grid = np.linalg.norm(candidates - p, axis=1)
            relay_positions.append(int(np.argmin(dists_to_grid)))
        if len(relay_positions) >= n_drones:
            break

    relay_positions = list(dict.fromkeys(relay_positions))[:n_drones]  # dedup + cap

    # Fill remaining slots with greedy coverage
    if len(relay_positions) < n_drones:
        remaining = n_drones - len(relay_positions)
        covered = set()
        for i in relay_positions:
            covered |= coverage_sets[i]
        for _ in range(remaining):
            best_idx, best_gain = -1, -1
            for i, cs in enumerate(coverage_sets):
                if i in relay_positions:
                    continue
                gain = len(cs - covered)
                if gain > best_gain:
                    best_gain = gain
                    best_idx  = i
            if best_idx == -1:
                break
            relay_positions.append(best_idx)
            covered |= coverage_sets[best_idx]

    return relay_positions

# -------------------------------------------------------------------
# Main simulation entry point
# -------------------------------------------------------------------
def run_simulation():
    rng = np.random.default_rng(SEED)

    # Generate K random camp positions inside the area
    camps = rng.uniform(10.0, AREA_SIZE - 10.0, size=(K_CAMPS, 2))

    candidates = build_candidate_grid(AREA_SIZE, GRID_RES)
    coverage_sets = compute_coverage_sets(candidates, camps, R_COMM)

    results = {}

    # Strategy 1: Greedy (no connectivity constraint)
    sel_greedy = greedy_max_coverage(coverage_sets, N_DRONES, K_CAMPS)
    cov_greedy  = total_covered(sel_greedy, coverage_sets)
    conn_greedy = is_connected(candidates[sel_greedy], BASE_POS, R_COMM)
    results['greedy'] = dict(selected=sel_greedy, coverage=cov_greedy,
                              connected=conn_greedy)
    print(f"[greedy]   coverage={cov_greedy}/{K_CAMPS}  connected={conn_greedy}")

    # Strategy 2: Greedy + connectivity repair
    sel_repaired, loss = repair_connectivity(
        sel_greedy, candidates, coverage_sets, BASE_POS, R_COMM
    )
    cov_repaired  = total_covered(sel_repaired, coverage_sets)
    conn_repaired = is_connected(candidates[sel_repaired], BASE_POS, R_COMM)
    results['repaired'] = dict(selected=sel_repaired, coverage=cov_repaired,
                                connected=conn_repaired, coverage_loss=loss)
    print(f"[repaired] coverage={cov_repaired}/{K_CAMPS}  connected={conn_repaired}"
          f"  loss={loss} camps")

    # Strategy 3: MST-anchored relay chain
    sel_mst = mst_relay_chain(camps, BASE_POS, candidates, coverage_sets,
                               N_DRONES, R_COMM)
    cov_mst  = total_covered(sel_mst, coverage_sets)
    conn_mst = is_connected(candidates[sel_mst], BASE_POS, R_COMM)
    results['mst'] = dict(selected=sel_mst, coverage=cov_mst, connected=conn_mst)
    print(f"[mst]      coverage={cov_mst}/{K_CAMPS}  connected={conn_mst}")

    # Sensitivity: minimum drones needed for full coverage (ignoring connectivity)
    for n_test in range(1, 20):
        sel_test = greedy_max_coverage(coverage_sets, n_test, K_CAMPS)
        if total_covered(sel_test, coverage_sets) == K_CAMPS:
            print(f"Min drones for full camp coverage (unconstrained): {n_test}")
            break

    return results, candidates, camps

if __name__ == "__main__":
    run_simulation()
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Zone area | 200 × 200 m |
| Candidate grid resolution $\delta$ | 5 m (40 × 40 = 1600 positions) |
| Number of relay drones $N$ | 5 |
| Number of survivor camps $K$ | 8 |
| Communication / coverage radius $r_{comm}$ | 30 m |
| Maximum distance from base $D_{max}$ | 120 m |
| Base station position | $(0,\; 100)$ m |
| Greedy approximation ratio | $1 - 1/e \approx 63.2\%$ |
| Connectivity repair max iterations | 50 |
| Bridge candidate criterion | within $r_{comm}$ of both components |
| MST edge weight | Euclidean distance |
| Random seed | 42 |

---

## Expected Output

- **Network topology plot**: 2D top-down map of the $200 \times 200$ m zone showing: survivor camp
  positions as yellow stars; base station as a black square; candidate grid as faint grey dots;
  selected drone hover positions for each of the three strategies as coloured circles (red = greedy,
  blue = repaired, green = MST); communication range circles ($r_{comm}$) drawn around each drone;
  relay edges (lines between nodes within $r_{comm}$) for each strategy; covered camps highlighted
  with a filled circle, uncovered camps as open markers.
- **Coverage comparison bar chart**: grouped bar chart showing the number of covered camps (out of
  $K = 8$) for the three strategies side by side, with a horizontal dashed line at $K = 8$ (full
  coverage); annotated with whether the relay graph is connected.
- **Connectivity graph panels**: three separate network diagrams (one per strategy) drawn as
  force-directed or positional graphs; nodes are base station + drones; edges are active relay
  links; disconnected components shown in different colours; BFS tree from base highlighted.
- **Sensitivity curve**: line plot of camp coverage fraction $\eta$ vs number of drones $N'$
  (sweeping $N' = 1$ to $15$) for the greedy-only strategy; mark the minimum $N'$ achieving
  $\eta = 100\%$; overlay the same curve with the connectivity constraint enforced.
- **Printed console metrics**:

  ```
  [greedy]   coverage=X/8  connected=True/False
  [repaired] coverage=X/8  connected=True   loss=Y camps
  [mst]      coverage=X/8  connected=True
  Min drones for full camp coverage (unconstrained): Z
  ```

- **Coverage area heatmap animation (GIF)**: top-down view sweeping through $N' = 1$ to $10$
  drones; at each frame, the greedy placement is shown with coverage discs filled in; camps colour
  from red (uncovered) to green (covered) as $N'$ increases; frame rate 2 fps.

---

## Extensions

1. **Probabilistic link model**: replace the hard $r_{comm}$ threshold with a distance-dependent
   packet delivery rate $p_{link}(d) = \exp(-\lambda d^2)$; redefine coverage as expected delivery
   probability $\geq 0.9$ and connectivity as the probability that a path from base to any drone
   exists above a threshold; solve the resulting chance-constrained placement problem via
   sample-average approximation.
2. **Mobile relay drones**: allow drones to move continuously rather than hover; formulate as a
   vehicle routing problem where drones periodically reposition to maintain coverage as camps are
   evacuated and new camps emerge; use a rolling-horizon replanning loop.
3. **Heterogeneous radios**: equip some drones with long-range (low-bandwidth) backbone radios
   ($r_{backbone} = 80$ m) and others with short-range (high-bandwidth) access-point radios
   ($r_{access} = 20$ m); design a two-tier placement: backbone drones first ensure connectivity,
   then access-point drones maximise per-camp throughput.
4. **Energy-aware hover time**: each hover position has a wind-dependent power draw; model battery
   depletion as $\dot{E} = P_{hover}(\mathbf{p})$; jointly optimise placement and drone rotation
   schedule (drones swap out for charging) to maximise total uptime of the relay network.
5. **Adversarial jamming**: a jammer at an unknown location disrupts links within its jamming
   radius $r_{jam}$; implement an online detection scheme (link quality monitoring) and trigger a
   repositioning response using the remaining available drones to route around the jammed region.

---

## Related Scenarios

- Prerequisites: [S047 Signal Relay Placement](S047_signal_relay.md) (single-relay coverage geometry), [S049 Dynamic Zone Search](S049_dynamic_zone.md) (Voronoi partitioning, multi-drone coordination), [S050 Swarm SLAM](S050_slam.md) (inter-drone communication topology during mapping)
- Follow-ups: [S052 Flood Extent Mapping](S052_flood_extent.md) (coverage maximisation in post-disaster terrain)
- Algorithmic cross-reference: [S011 Network Flow Interception](../01_pursuit_evasion/S011_network_flow.md) (graph connectivity, flow constraints), [S036 Multi-Depot Routing](../02_logistics_delivery/S036_multi_depot_routing.md) (MST-based path planning)
