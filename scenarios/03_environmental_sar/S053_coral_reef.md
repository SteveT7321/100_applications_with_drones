# S053 Coral Reef 3D Reconstruction

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: SfM Viewpoint Coverage + TSP Tour | **Dimension**: 3D

---

## Problem Definition

**Setup**: A survey drone is deployed over a $30 \times 30$ m coral reef patch located 3–8 m below
the water surface. The reef surface is irregular: depth varies continuously across the patch,
forming a rough 3D topology that must be captured at sub-centimetre resolution for offline
Structure-from-Motion (SfM) reconstruction. No pre-existing bathymetric chart is available; the
mission plan is computed from a coarse prior mesh of the reef (modelled here as a sinusoidal height
field). The drone flies above the water surface at one of two flight altitudes: $h_{low} = 5$ m and
$h_{high} = 12$ m above the surface (8–15 m above the deepest reef faces), carrying a nadir-tilting
RGB camera with focal length $f = 24$ mm (full-frame equivalent), sensor width $s_w = 36$ mm,
and image width $W_{px} = 6000$ pixels.

Candidate camera viewpoints are distributed on a hemisphere dome at each altitude: a $5 \times 9$
latitude–longitude grid at $h_{low}$ (45 positions) and a $3 \times 9$ grid at $h_{high}$
(27 positions), giving $N_v = 72$ candidate viewpoints in total. The reef surface is discretised
into $M = 400$ triangular face-lets (a $20 \times 20$ quad mesh of $1.5 \times 1.5$ m cells,
each split into two triangles).

**Roles**:
- **Drone**: single survey platform; moves between selected viewpoints at cruise speed $v = 4.0$ m/s;
  hovers for $t_{cap} = 2.0$ s at each viewpoint to capture a full-resolution image.
- **Reef surface**: static 3D structure; modelled as a sinusoidal height field
  $z_{reef}(x, y) = -5.5 + 2.5 \sin(2\pi x / 15) \cos(2\pi y / 20)$ m (relative to water surface);
  surface normal $\hat{\mathbf{n}}(x,y)$ computed analytically for visibility tests.

**Objective**: Select the minimum subset $S \subseteq \{1, \ldots, N_v\}$ of camera viewpoints such
that every reef face-let is observed from at least $k_{min} = 3$ distinct viewpoints (triangulation
condition), with adjacent captured images sharing $\geq 60\%$ overlap (feature matching condition),
and then find a shortest TSP tour through $S$ to minimise total flight time. Three strategies are
compared:

1. **Nadir-only grid** — uniform lawnmower at $h_{low}$; all camera axes point straight down.
2. **Single-altitude greedy cover** — greedy set-cover using only $h_{low}$ viewpoints.
3. **Dual-altitude greedy cover + TSP** (proposed) — set-cover on all 72 viewpoints, followed by
   nearest-neighbour TSP tour optimisation.

---

## Mathematical Model

### Ground Sampling Distance and Resolution Condition

For a camera at altitude $h$ above a reef face at depth $d_{reef}$ below the surface, the effective
camera-to-face distance is $H = h + |d_{reef}|$. The Ground Sampling Distance (GSD) — the real-world
size of one image pixel on the face — is:

$$\mathrm{GSD}(H) = \frac{s_w \cdot H}{f \cdot W_{px}}$$

The resolution condition requires $\mathrm{GSD} < \mathrm{GSD}_{max} = 2$ cm so that feature
keypoints can be extracted at sub-centimetre scale. For a viewpoint at altitude $h$ observing a
face at depth $d_{reef}$:

$$\mathrm{GSD}(h + |d_{reef}|) < 0.02 \;\text{m}$$

This gives a maximum admissible camera-to-face distance:

$$H_{max} = \frac{f \cdot W_{px} \cdot 0.02}{s_w} = \frac{0.024 \times 6000 \times 0.02}{0.036} = 80 \;\text{m}$$

All candidate viewpoints at $h_{low}$ and $h_{high}$ satisfy this condition for the reef depth range
considered.

### Visibility Matrix

A viewpoint $i$ at position $\mathbf{c}_i \in \mathbb{R}^3$ covers face-let $j$ with centroid
$\mathbf{f}_j$ and outward normal $\hat{\mathbf{n}}_j$ if and only if three conditions are jointly
satisfied:

1. **Incidence angle** (camera not grazing the face):

$$\cos\theta_{ij} = -\frac{(\mathbf{c}_i - \mathbf{f}_j)}{\|\mathbf{c}_i - \mathbf{f}_j\|} \cdot \hat{\mathbf{n}}_j \geq \cos\theta_{max}$$

with $\theta_{max} = 60°$ (i.e., $\cos\theta_{max} = 0.5$).

2. **Resolution condition**: $\mathrm{GSD}(\|\mathbf{c}_i - \mathbf{f}_j\|) < \mathrm{GSD}_{max}$.

3. **Field-of-view**: face centroid $\mathbf{f}_j$ projects inside the image sensor footprint
   (half-angle $\psi = \arctan(s_w / (2f)) \approx 36.9°$):

$$\left|\arccos\!\left(\frac{(\mathbf{f}_j - \mathbf{c}_i) \cdot \hat{\mathbf{a}}_i}{\|\mathbf{f}_j - \mathbf{c}_i\|}\right)\right| \leq \psi$$

where $\hat{\mathbf{a}}_i$ is the camera optical axis direction at viewpoint $i$ (pointing toward the
reef centroid for non-nadir viewpoints). The binary visibility matrix is:

$$V[i,j] = \mathbf{1}\!\left[\text{conditions 1–3 all hold}\right] \in \{0,1\}^{N_v \times M}$$

### Coverage Condition

For a selected viewpoint set $S \subseteq \{1,\ldots,N_v\}$, define the coverage count of face $j$:

$$c_j(S) = \sum_{i \in S} V[i,j]$$

The $k_{min}$-fold coverage constraint requires:

$$\forall\, j \in \{1,\ldots,M\} : \quad c_j(S) \geq k_{min} = 3$$

The optimisation objective is:

$$\min_{S} \;|S| \quad \text{s.t.} \quad c_j(S) \geq k_{min} \;\forall j$$

### Greedy Set-Cover Algorithm

Finding the minimum $|S|$ is NP-hard in general (weighted set cover). The greedy algorithm with
$k_{min}$-fold coverage selects viewpoints iteratively:

$$i^* = \arg\max_{i \notin S} \;\left|\left\{j : c_j(S \cup \{i\}) > c_j(S) \;\text{ and }\; c_j(S) < k_{min}\right\}\right|$$

At each step, the viewpoint that covers the most currently under-covered face-lets (those with
$c_j < k_{min}$) is added to $S$. The algorithm terminates when $c_j(S) \geq k_{min}$ for all $j$,
or when no further improvement is possible (infeasible faces flagged). The greedy approximation
guarantee is within a $\ln(M)$ factor of the true optimum.

### Image Overlap Condition

Two adjacent captured images (from viewpoints $i$ and $i'$ consecutive in the TSP tour) must share
at least $p_{ov} = 60\%$ footprint overlap for reliable feature matching in SfM. The footprint of
viewpoint $i$ projected onto the reef plane at mean depth $\bar{d} = 5.5$ m is approximately a
rectangle of side:

$$L_i = 2 (h_i + \bar{d}) \tan\psi$$

The linear overlap fraction between footprints of adjacent viewpoints $i, i'$ separated by
horizontal distance $\delta_{ii'} = \|\mathbf{c}_i^{xy} - \mathbf{c}_{i'}^{xy}\|$ is:

$$p_{ov}(i,i') = \max\!\left(0,\; 1 - \frac{\delta_{ii'}}{\min(L_i, L_{i'})}\right) \geq 0.60$$

This constraint is enforced as a post-check after TSP tour construction; viewpoints with
insufficient overlap to their tour neighbours are flagged for insertion of an intermediate capture
position.

### Baseline Angle and Reprojection Error

For each face $j$, let $I_j = \{i \in S : V[i,j] = 1\}$ be the set of viewpoints covering it.
For any pair $i, i' \in I_j$, the **baseline angle** is the angle between the two viewing directions
toward face $j$:

$$\alpha_{ii'} = \arccos\!\left(\hat{\mathbf{v}}_{ij} \cdot \hat{\mathbf{v}}_{i'j}\right)$$

where $\hat{\mathbf{v}}_{ij} = (\mathbf{f}_j - \mathbf{c}_i) / \|\mathbf{f}_j - \mathbf{c}_i\|$.

The approximate **reprojection error** for SfM triangulation from a pair of views at mean distance
$\bar{H}_j = \frac{1}{2}(\|\mathbf{f}_j - \mathbf{c}_i\| + \|\mathbf{f}_j - \mathbf{c}_{i'}\|)$ is:

$$\varepsilon_j \approx \frac{\sigma_{px} \cdot \bar{H}_j}{f_{px} \cdot \sin\alpha_{ii'}}$$

where $\sigma_{px} = 0.5$ px is the keypoint localisation error and
$f_{px} = f \cdot W_{px} / s_w = 4000$ px is the focal length in pixels. Larger baseline angles
reduce $\varepsilon_j$; the dual-altitude plan mixes $h_{low}$ and $h_{high}$ viewpoints to
increase effective $\alpha_{ii'}$ beyond what a single altitude can achieve.

The mean reconstruction error over all faces:

$$\bar{\varepsilon} = \frac{1}{M} \sum_{j=1}^{M} \min_{(i,i') \in \binom{I_j}{2}} \varepsilon_j(i,i')$$

### TSP Nearest-Neighbour Tour

Given the selected viewpoint set $S$ of size $n_s = |S|$, the flight path is computed by the
nearest-neighbour heuristic starting from the viewpoint closest to the launch position
$\mathbf{p}_{launch} = (15, 15, h_{low})$ m:

$$\pi_1 = \arg\min_{i \in S} \|\mathbf{c}_i - \mathbf{p}_{launch}\|$$

$$\pi_{k+1} = \arg\min_{i \in S \setminus \{\pi_1,\ldots,\pi_k\}} \|\mathbf{c}_i - \mathbf{c}_{\pi_k}\|$$

Total tour length $L_{tour} = \sum_{k=1}^{n_s - 1} \|\mathbf{c}_{\pi_{k+1}} - \mathbf{c}_{\pi_k}\|$
and estimated flight time (excluding hover time):

$$T_{flight} = \frac{L_{tour}}{v} + n_s \cdot t_{cap}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from itertools import combinations

# ── Constants ──────────────────────────────────────────────────────────────────
REEF_SIZE      = 30.0          # m, square patch side
GRID_N         = 20            # quad mesh resolution (20×20 quads → 800 triangles)
H_LOW          = 5.0           # m, low-altitude flight height above surface
H_HIGH         = 12.0          # m, high-altitude flight height above surface
F_MM           = 24e-3         # m, focal length
SW_M           = 36e-3         # m, sensor width
W_PX           = 6000          # pixels, image width
F_PX           = F_MM * W_PX / SW_M   # 4000 px
GSD_MAX        = 0.02          # m, max ground sampling distance (2 cm)
H_MAX          = F_MM * W_PX * GSD_MAX / SW_M  # 80 m
THETA_MAX      = np.deg2rad(60.0)     # max incidence angle from face normal
PSI            = np.arctan(SW_M / (2 * F_MM))  # camera half-FOV ≈ 36.9 deg
K_MIN          = 3             # minimum viewpoints per face (triangulation)
P_OV_MIN       = 0.60          # minimum image overlap fraction
DRONE_SPEED    = 4.0           # m/s
T_CAP          = 2.0           # s, hover time per capture
SIGMA_PX       = 0.5           # px, keypoint localisation noise
LAUNCH_POS     = np.array([15.0, 15.0, H_LOW])


# ── Reef surface model ─────────────────────────────────────────────────────────
def reef_depth(x, y):
    """Reef depth below water surface (negative = below surface)."""
    return -5.5 + 2.5 * np.sin(2 * np.pi * x / 15.0) * np.cos(2 * np.pi * y / 20.0)

def reef_normal(x, y):
    """Outward (upward) unit normal of the reef surface."""
    dz_dx = 2.5 * (2 * np.pi / 15.0) * np.cos(2 * np.pi * x / 15.0) * np.cos(2 * np.pi * y / 20.0)
    dz_dy = -2.5 * (2 * np.pi / 20.0) * np.sin(2 * np.pi * x / 15.0) * np.sin(2 * np.pi * y / 20.0)
    n = np.array([-dz_dx, -dz_dy, 1.0])
    return n / np.linalg.norm(n)


def build_reef_mesh():
    """
    Build triangular face-lets over the reef patch.
    Returns:
        centroids : (M, 3) array of face centroid positions
        normals   : (M, 3) array of outward unit normals
        verts     : list of M triangle vertex arrays, each (3, 3)
    """
    xs = np.linspace(0.0, REEF_SIZE, GRID_N + 1)
    ys = np.linspace(0.0, REEF_SIZE, GRID_N + 1)
    centroids, normals, verts = [], [], []

    for ix in range(GRID_N):
        for iy in range(GRID_N):
            # Four corners of the quad
            corners = [
                (xs[ix],   ys[iy]),
                (xs[ix+1], ys[iy]),
                (xs[ix+1], ys[iy+1]),
                (xs[ix],   ys[iy+1]),
            ]
            corners_3d = [np.array([cx, cy, reef_depth(cx, cy)]) for cx, cy in corners]

            # Two triangles per quad
            for tri_idx in [(0, 1, 2), (0, 2, 3)]:
                tri = [corners_3d[k] for k in tri_idx]
                c = np.mean(tri, axis=0)
                cx, cy = c[0], c[1]
                n = reef_normal(cx, cy)
                centroids.append(c)
                normals.append(n)
                verts.append(np.array(tri))

    return np.array(centroids), np.array(normals), verts


# ── Viewpoint generation ───────────────────────────────────────────────────────
def build_viewpoints():
    """
    Generate candidate viewpoints on a hemisphere dome at h_low and h_high.
    Camera optical axis points toward reef centroid (15, 15, reef_depth(15,15)).
    Returns: (N_v, 3) array of viewpoint positions.
    """
    reef_center = np.array([15.0, 15.0, reef_depth(15.0, 15.0)])
    viewpoints = []

    for altitude, n_lat, n_lon in [(H_LOW, 5, 9), (H_HIGH, 3, 9)]:
        # lat in [0, 60 deg], lon in [0, 360 deg)
        for lat_deg in np.linspace(0.0, 60.0, n_lat):
            for lon_deg in np.linspace(0.0, 360.0, n_lon, endpoint=False):
                lat = np.deg2rad(lat_deg)
                lon = np.deg2rad(lon_deg)
                # Offset from reef centre on hemisphere
                r_horiz = (altitude + abs(reef_center[2])) * np.tan(lat)
                x = reef_center[0] + r_horiz * np.cos(lon)
                y = reef_center[1] + r_horiz * np.sin(lon)
                z = altitude
                # Clamp to a generous boundary
                x = np.clip(x, -10.0, REEF_SIZE + 10.0)
                y = np.clip(y, -10.0, REEF_SIZE + 10.0)
                viewpoints.append(np.array([x, y, z]))

    return np.array(viewpoints)


# ── Visibility matrix ──────────────────────────────────────────────────────────
def build_visibility_matrix(viewpoints, centroids, normals):
    """
    Compute V[i, j] = 1 if viewpoint i covers face j.
    Checks: incidence angle, GSD, field-of-view.
    Returns: (N_v, M) bool array.
    """
    N_v = len(viewpoints)
    M   = len(centroids)
    V   = np.zeros((N_v, M), dtype=bool)

    reef_center = np.array([15.0, 15.0, reef_depth(15.0, 15.0)])

    for i, cam in enumerate(viewpoints):
        # Camera optical axis: nadir for lat=0, tilted toward reef_center otherwise
        axis = reef_center - cam
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-6:
            continue
        a_hat = axis / axis_norm

        for j, (face, nhat) in enumerate(zip(centroids, normals)):
            vec = face - cam
            dist = np.linalg.norm(vec)
            if dist < 1e-6:
                continue
            v_hat = vec / dist

            # 1. Incidence angle check
            cos_theta = -v_hat @ nhat     # nhat points upward, v_hat points down
            if cos_theta < np.cos(THETA_MAX):
                continue

            # 2. GSD check
            gsd = SW_M * dist / (F_MM * W_PX)
            if gsd >= GSD_MAX:
                continue

            # 3. Field-of-view check
            cos_fov = v_hat @ a_hat
            if cos_fov < np.cos(PSI):
                continue

            V[i, j] = True

    return V


# ── Greedy set-cover ───────────────────────────────────────────────────────────
def greedy_set_cover(V, k_min=K_MIN):
    """
    Greedy k-fold set cover.
    Args:
        V     : (N_v, M) bool visibility matrix
        k_min : required coverage count per face
    Returns:
        selected : list of selected viewpoint indices
        coverage : (M,) int array of final coverage counts
    """
    N_v, M = V.shape
    coverage = np.zeros(M, dtype=int)
    selected = []
    remaining = set(range(N_v))

    while True:
        under = coverage < k_min
        if not np.any(under):
            break
        if not remaining:
            print(f"WARNING: {np.sum(under)} faces could not reach {k_min}-fold coverage.")
            break

        # Choose viewpoint that covers the most under-covered faces
        best_i, best_gain = -1, -1
        for i in remaining:
            gain = np.sum(V[i] & under)
            if gain > best_gain:
                best_gain, best_i = gain, i

        if best_gain == 0:
            print(f"WARNING: No further coverage gain possible. "
                  f"{np.sum(under)} faces remain under-covered.")
            break

        selected.append(best_i)
        coverage += V[best_i].astype(int)
        remaining.remove(best_i)

    return selected, coverage


# ── TSP nearest-neighbour tour ─────────────────────────────────────────────────
def tsp_nearest_neighbour(viewpoints, selected_idx, launch=LAUNCH_POS):
    """
    Nearest-neighbour TSP heuristic starting from viewpoint closest to launch.
    Args:
        viewpoints   : (N_v, 3) all candidate positions
        selected_idx : list of selected viewpoint indices
    Returns:
        tour  : ordered list of viewpoint indices
        dists : list of leg distances (m)
    """
    pts = viewpoints[selected_idx]
    n = len(selected_idx)

    # Start from viewpoint nearest to launch
    start = int(np.argmin(np.linalg.norm(pts - launch, axis=1)))
    tour_local = [start]
    visited = {start}

    while len(tour_local) < n:
        current = tour_local[-1]
        best_j, best_d = -1, np.inf
        for j in range(n):
            if j in visited:
                continue
            d = np.linalg.norm(pts[j] - pts[current])
            if d < best_d:
                best_d, best_j = d, j
        tour_local.append(best_j)
        visited.add(best_j)

    tour = [selected_idx[k] for k in tour_local]
    dists = [np.linalg.norm(viewpoints[tour[k+1]] - viewpoints[tour[k]])
             for k in range(n - 1)]
    return tour, dists


# ── Reprojection error ─────────────────────────────────────────────────────────
def compute_reprojection_errors(viewpoints, centroids, V, selected_idx):
    """
    For each face, find the pair in selected viewpoints with the largest baseline
    angle and compute the approximate reprojection error.
    Returns: (M,) array of per-face reprojection errors (m).
    """
    S_set = set(selected_idx)
    errors = np.full(len(centroids), np.nan)

    for j, face in enumerate(centroids):
        covering = [i for i in selected_idx if V[i, j]]
        if len(covering) < 2:
            continue
        best_alpha, best_eps = 0.0, np.inf
        for i, ip in combinations(covering, 2):
            vi = (face - viewpoints[i])
            vip = (face - viewpoints[ip])
            vi_hat  = vi  / (np.linalg.norm(vi)  + 1e-9)
            vip_hat = vip / (np.linalg.norm(vip) + 1e-9)
            cos_a = np.clip(vi_hat @ vip_hat, -1.0, 1.0)
            alpha = np.arccos(cos_a)
            if alpha > best_alpha:
                best_alpha = alpha
                H_bar = 0.5 * (np.linalg.norm(vi) + np.linalg.norm(vip))
                eps = SIGMA_PX * H_bar / (F_PX * np.sin(alpha + 1e-9))
                best_eps = eps
        errors[j] = best_eps

    return errors


# ── Main simulation ────────────────────────────────────────────────────────────
def run_simulation():
    rng = np.random.default_rng(42)

    print("Building reef mesh …")
    centroids, normals, verts = build_reef_mesh()
    M = len(centroids)
    print(f"  {M} face-lets generated.")

    print("Building viewpoints …")
    viewpoints = build_viewpoints()
    N_v = len(viewpoints)
    print(f"  {N_v} candidate viewpoints.")

    print("Building visibility matrix …")
    V = build_visibility_matrix(viewpoints, centroids, normals)
    print(f"  Visibility matrix: {V.shape}, density={V.mean():.3f}")

    print("Running greedy set-cover …")
    selected, coverage = greedy_set_cover(V, k_min=K_MIN)
    n_s = len(selected)
    print(f"  Selected {n_s} viewpoints; "
          f"min coverage={coverage.min()}, mean={coverage.mean():.2f}")

    print("Computing TSP tour …")
    tour, leg_dists = tsp_nearest_neighbour(viewpoints, selected)
    L_tour = sum(leg_dists)
    T_flight = L_tour / DRONE_SPEED + n_s * T_CAP
    print(f"  Tour length: {L_tour:.1f} m | Estimated flight time: {T_flight:.1f} s")

    print("Computing reprojection errors …")
    errors = compute_reprojection_errors(viewpoints, centroids, V, selected)
    valid_errors = errors[~np.isnan(errors)]
    print(f"  Mean reprojection error: {valid_errors.mean()*100:.2f} cm  "
          f"| Max: {valid_errors.max()*100:.2f} cm")

    return viewpoints, centroids, normals, verts, V, selected, coverage, tour, leg_dists, errors
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Reef patch size | — | 30 × 30 m |
| Reef depth range | $d_{reef}$ | −3 m to −8 m |
| Mesh resolution | $N_{grid}$ | 20 × 20 quads → 800 triangles |
| Low flight altitude | $h_{low}$ | 5 m |
| High flight altitude | $h_{high}$ | 12 m |
| Candidate viewpoints | $N_v$ | 72 |
| Focal length | $f$ | 24 mm |
| Sensor width | $s_w$ | 36 mm |
| Image width | $W_{px}$ | 6000 px |
| Focal length (pixels) | $f_{px}$ | 4000 px |
| Max GSD | $\mathrm{GSD}_{max}$ | 2 cm |
| Max incidence angle | $\theta_{max}$ | 60° |
| Camera half-FOV | $\psi$ | ≈ 36.9° |
| Min coverage fold | $k_{min}$ | 3 |
| Min image overlap | $p_{ov}$ | 60% |
| Cruise speed | $v$ | 4.0 m/s |
| Hover time per capture | $t_{cap}$ | 2.0 s |
| Keypoint noise | $\sigma_{px}$ | 0.5 px |

---

## Expected Output

- **Reef surface and viewpoint map (3D)**: `mpl_toolkits.mplot3d` scene showing the sinusoidal reef
  surface colour-coded by depth (blue → green); all 72 candidate viewpoints as grey spheres at their
  respective altitudes; selected viewpoints highlighted in red; camera frustum cones drawn from each
  selected viewpoint toward the reef centroid.
- **Visibility coverage heatmap (3D)**: reef face-lets colour-coded by coverage count $c_j(S)$
  after greedy set-cover; faces with $c_j < k_{min}$ flagged in magenta (should be zero for a
  feasible plan); colour bar from 0 to $\max c_j$.
- **TSP flight path animation**: 3D animated GIF (`FuncAnimation`) showing the drone icon
  (red sphere) traversing the selected viewpoints in tour order; completed legs drawn as a solid
  red polyline; at each capture viewpoint a brief camera-frustum flash indicates image acquisition;
  reef surface rendered as a semi-transparent mesh.
- **Reprojection error map (3D)**: face-lets colour-coded by per-face minimum reprojection error
  $\varepsilon_j$ (blue = sub-mm, yellow/red = > 5 mm); annotated with mean and max values;
  dual-altitude plan expected to show substantially lower $\varepsilon_j$ than single-altitude
  baseline.
- **Strategy comparison bar chart**: three grouped bars (Nadir grid / Single-altitude greedy /
  Dual-altitude greedy + TSP) for: number of selected viewpoints, total flight time $T_{flight}$,
  mean reprojection error $\bar{\varepsilon}$, fraction of faces achieving $\geq 3$-fold coverage.
- **Metrics printed to stdout**:

```
Selected viewpoints : 28
Tour length         : 312.4 m
Flight time         : 134.1 s
Min coverage        : 3
Mean coverage       : 4.7
Mean reproj. error  : 0.81 cm
Max reproj. error   : 2.34 cm
```

---

## Extensions

1. **Adaptive GSD-driven densification**: after a first SfM pass produces a sparse point cloud,
   identify faces with point density below a threshold and automatically insert additional close-up
   viewpoints at $h_{extra} = 3$ m to fill coverage gaps; re-run TSP on the augmented set.
2. **Water refraction correction**: account for the refractive index of water ($n_w = 1.33$) when
   computing the apparent position of underwater features; the GSD formula becomes
   $\mathrm{GSD}_{refr} = \mathrm{GSD} \cdot (h + |d_{reef}|/n_w) / (h + |d_{reef}|)$;
   update the visibility and reprojection error models accordingly.
3. **Photogrammetry-aware viewpoint selection**: replace the greedy set-cover with a
   differentiable coverage objective and solve via gradient-free optimisation (CMA-ES) that jointly
   maximises baseline diversity $\bar{\alpha}$ and minimises $n_s$, trading off the two objectives
   on a Pareto front.
4. **Wind-disturbed hover model**: at each capture viewpoint, the drone hovers under a random wind
   force $\mathbf{w} \sim \mathcal{N}(\mathbf{0}, \sigma_w^2 \mathbf{I})$; image blur is modelled
   as a function of the lateral drift during $t_{cap}$; re-plan the tour to avoid capture near the
   upwind reef edge where turbulence is highest.
5. **Online re-planning from live SfM**: stream partial reconstruction results from an onboard
   SfM pipeline; when a face's point cloud variance exceeds a threshold mid-mission, insert an
   additional repair viewpoint immediately after the current position in the tour and recompute the
   remaining path.

---

## Related Scenarios

- Prerequisites: [S048 Lawnmower Coverage](S048_lawnmower_coverage.md) (systematic 2D coverage baseline), [S050 Swarm Cooperative Mapping](S050_slam.md) (3D map building), [S044 Wall Crack Inspection](S044_wall_crack_inspection.md) (close-range surface imaging)
- Follow-ups: [S054 Post-Disaster Photogrammetry](S054_post_disaster_photogrammetry.md) (urban SfM reconstruction)
- Algorithmic cross-reference: [S048 Lawnmower Coverage](S048_lawnmower_coverage.md) (TSP path planning), [S050 Swarm Cooperative Mapping](S050_slam.md) (visibility and map quality metrics), [S041 Wildfire Boundary Scan](S041_wildfire_boundary.md) (coverage optimisation)
