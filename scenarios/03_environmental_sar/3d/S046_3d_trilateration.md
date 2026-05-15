# S046 3D Upgrade — Trilateration with Altitude

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S046 original](../S046_trilateration.md)

---

## What Changes in 3D

The original S046 already operates with four drones placed at deliberate altitude tiers, so
range measurements are nominally 3D. However, the key limitation addressed here is that the
baseline scenario treats anchor positions as fixed by a designer-chosen heuristic (three drones
at 120° azimuth, one overhead) and only measures slant ranges (RSSI/acoustic). This 3D upgrade
adds three orthogonal challenges:

1. **Fourth anchor constraint**: with $N = 3$ anchors in 3D, the range equations define a
   unique point only when the three sphere surfaces intersect at exactly one point — a condition
   that is geometrically fragile. With $N \geq 4$ non-coplanar anchors the system is
   over-determined and the solution is unique and robust to noise. This variant enforces a
   minimum of $N = 4$ drones that span full 3D volume (not just different altitudes on a single
   circle).
2. **Altitude diversity optimisation**: drone altitudes are treated as free variables and
   jointly optimised with azimuthal positions to minimise 3D GDOP; vertical GDOP and horizontal
   GDOP are tracked separately.
3. **3D TDOA hyperbolic localisation**: in addition to one-way range measurements, time-
   difference-of-arrival (TDOA) between drone pairs is modelled, converting range differences
   into hyperboloids whose intersection gives the beacon location; this is strictly a 3D
   construction because TDOA hyperboloids are degenerate in a 2D plane.

---

## Problem Definition

**Setup**: A survivor's emergency beacon is at an unknown position
$\mathbf{x}^* = (x^*, y^*, z^*) \in \mathbb{R}^3$ inside a post-disaster area of
$300 \times 300 \times 50$ m. A fleet of $N = 5$ drones hovers at known, optimally placed
positions $\{\mathbf{p}_i\}_{i=1}^5$ that span distinct altitudes from 5 m to 45 m. Each drone
independently measures either the one-way slant range (RSSI ranging) or records its signal
arrival time (TDOA) relative to a synchronised clock shared over the mesh network. Both
measurement modalities are corrupted by additive Gaussian noise.

**Roles**:
- **Drones** ($N = 5$): hovering anchors at known 3D positions; each contributes one range
  or one TDOA measurement per epoch. Positions are computed by a pre-mission GDOP minimisation
  procedure that respects altitude bounds and inter-drone collision margins.
- **Beacon / Survivor**: passive transmitter at unknown $\mathbf{x}^*$ at an altitude that
  may range from ground level ($z = 0$ m, e.g. buried under rubble) up to 10 m (e.g. trapped
  in a partially collapsed structure).
- **Ground station**: fuses all measurements and computes $\hat{\mathbf{x}}$ via LLS,
  Gauss-Newton, or TDOA hyperbolic least squares; reports 3D GDOP and the separate vertical
  GDOP $\text{VDOP}$ and horizontal GDOP $\text{HDOP}$.

**Objective**: Extend the S046 baseline to a scenario where

1. Drone positions are chosen by minimising 3D GDOP subject to altitude diversity constraints.
2. TDOA-based hyperbolic localisation is derived and compared to direct ranging.
3. VDOP and HDOP are tracked separately to show how altitude spread reduces vertical
   uncertainty — the dominant weakness of coplanar configurations.

---

## Mathematical Model

### Range and TDOA Measurement Models

One-way slant range measurement from drone $i$:

$$r_i = \|\mathbf{p}_i - \mathbf{x}^*\| + \epsilon_i^r, \qquad \epsilon_i^r \sim \mathcal{N}(0,\, \sigma_r^2)$$

TDOA between drones $i$ and reference drone $j = 1$, expressed in metres:

$$d_{i1} = \|\mathbf{p}_i - \mathbf{x}^*\| - \|\mathbf{p}_1 - \mathbf{x}^*\| + \epsilon_i^d, \qquad \epsilon_i^d \sim \mathcal{N}(0,\, 2\sigma_r^2)$$

Note that $\text{Var}(\epsilon_i^d) = 2\sigma_r^2$ because the TDOA difference noise is the sum
of two independent range noise terms.

### TDOA Hyperbolic Least Squares

Each measured TDOA $d_{i1}$ constrains the beacon to lie on a hyperboloid whose two foci are
$\mathbf{p}_i$ and $\mathbf{p}_1$. Squaring the TDOA equation and rearranging:

$$\|\mathbf{p}_i - \mathbf{x}\|^2 - \|\mathbf{p}_1 - \mathbf{x}\|^2 = d_{i1}^2 + 2 d_{i1} \|\mathbf{p}_1 - \mathbf{x}\|$$

This is nonlinear in $\|\mathbf{p}_1 - \mathbf{x}\|$. A two-stage linearisation is applied:

**Stage 1** — treat $\rho_1 = \|\mathbf{p}_1 - \mathbf{x}\|$ as an auxiliary unknown and
augment the state to $\tilde{\mathbf{x}} = [x, y, z, \rho_1]^T$. Subtracting the squared
distance identity:

$$\|\mathbf{p}_i\|^2 - \|\mathbf{p}_1\|^2 - d_{i1}^2 - 2(\mathbf{p}_i - \mathbf{p}_1)^T \mathbf{x} - 2 d_{i1} \rho_1 = 0$$

Collecting $N - 1$ such equations into the linear system:

$$\mathbf{C} \tilde{\mathbf{x}} = \mathbf{g}$$

where the rows of $\mathbf{C} \in \mathbb{R}^{(N-1) \times 4}$ and $\mathbf{g} \in \mathbb{R}^{N-1}$ are:

$$\mathbf{C}_i = \bigl[\,2(\mathbf{p}_i - \mathbf{p}_1)^T,\; 2 d_{i1}\,\bigr]$$

$$g_i = \|\mathbf{p}_i\|^2 - \|\mathbf{p}_1\|^2 - d_{i1}^2$$

**Stage 2** — solve via weighted least squares:

$$\hat{\tilde{\mathbf{x}}} = (\mathbf{C}^T \mathbf{W}_d \mathbf{C})^{-1} \mathbf{C}^T \mathbf{W}_d \mathbf{g}$$

where $\mathbf{W}_d = \text{diag}(1/(2\sigma_r^2))$ and $\hat{\mathbf{x}}$ is the first three
components of $\hat{\tilde{\mathbf{x}}}$.

### 3D Gauss-Newton Refinement

Given any initial estimate $\mathbf{x}_0$ (from LLS or TDOA Stage 1), iterate:

$$\mathbf{H}_k = \left[\frac{\mathbf{p}_i - \mathbf{x}_k}{\|\mathbf{p}_i - \mathbf{x}_k\|}\right]_{i=1}^N \in \mathbb{R}^{N \times 3}$$

$$\mathbf{f}(\mathbf{x}_k) = \bigl[r_i - \|\mathbf{p}_i - \mathbf{x}_k\|\bigr]_{i=1}^N$$

$$\mathbf{x}_{k+1} = \mathbf{x}_k + (\mathbf{H}_k^T \mathbf{H}_k)^{-1} \mathbf{H}_k^T \mathbf{f}(\mathbf{x}_k)$$

until $\|\mathbf{x}_{k+1} - \mathbf{x}_k\| < \epsilon_{tol}$.

### 3D GDOP, VDOP, and HDOP

The full 3D position covariance is:

$$\boldsymbol{\Sigma} = \sigma_r^2 \, (\mathbf{H}^T \mathbf{H})^{-1} \in \mathbb{R}^{3 \times 3}$$

Decompose into components aligned with the local vertical and horizontal:

$$\text{GDOP} = \sqrt{\text{tr}(\boldsymbol{\Sigma})} \;/\; \sigma_r$$

$$\text{VDOP} = \sqrt{\boldsymbol{\Sigma}_{zz}} \;/\; \sigma_r$$

$$\text{HDOP} = \sqrt{\boldsymbol{\Sigma}_{xx} + \boldsymbol{\Sigma}_{yy}} \;/\; \sigma_r$$

with the identity $\text{GDOP}^2 = \text{HDOP}^2 + \text{VDOP}^2$.

### Altitude Diversity Optimisation

Parameterise the five drone positions as:

$$\mathbf{p}_i = \bigl(R_i \cos\phi_i,\; R_i \sin\phi_i,\; z_i\bigr)$$

where $\phi_i$ is azimuth, $R_i$ is horizontal radius, and $z_i$ is altitude. Minimise:

$$\min_{\{\phi_i, R_i, z_i\}} \; \text{GDOP}(\{\mathbf{p}_i\}, \mathbf{x}^*)$$

subject to:

$$z_i \in [z_{min},\, z_{max}] \quad \forall i$$

$$\|\mathbf{p}_i - \mathbf{p}_j\|_2 \geq d_{sep} \quad \forall i \neq j \quad \text{(collision avoidance)}$$

$$R_i \in [R_{min},\, R_{max}] \quad \forall i$$

Solved with `scipy.optimize.minimize` (L-BFGS-B) using the analytic gradient of GDOP with
respect to anchor positions. The altitude spread constraint is additionally enforced as:

$$\max_i(z_i) - \min_i(z_i) \geq \Delta z_{min} = 20 \text{ m}$$

to prevent the optimiser from collapsing all drones to a single altitude.

### Cramer-Rao Lower Bound (3D TDOA)

For the TDOA estimator, the Fisher information matrix takes the form:

$$\mathbf{I}_{TDOA}(\mathbf{x}^*) = \frac{1}{2\sigma_r^2} \tilde{\mathbf{H}}^T \tilde{\mathbf{H}}$$

where $\tilde{\mathbf{H}} \in \mathbb{R}^{(N-1) \times 3}$ has rows:

$$\tilde{\mathbf{H}}_i = \frac{\mathbf{p}_i - \mathbf{x}^*}{\|\mathbf{p}_i - \mathbf{x}^*\|} - \frac{\mathbf{p}_1 - \mathbf{x}^*}{\|\mathbf{p}_1 - \mathbf{x}^*\|}$$

The CRLB on 3D position variance satisfies:

$$\text{Var}(\hat{x}_j) \geq \bigl[\mathbf{I}_{TDOA}^{-1}\bigr]_{jj}$$

---

## Key 3D Additions

- **Minimum 4-drone uniqueness proof**: with $N < 4$ non-coplanar anchors the range system has
  a 1D family of solutions (a circle intersection); $N = 4$ guarantees a unique solution and
  $N = 5$ provides one degree of freedom for GDOP optimisation.
- **TDOA hyperbolic localisation**: each TDOA measurement constrains the beacon to a
  hyperboloid in 3D space; the two-stage linearised solver yields a closed-form first estimate
  without iterating, suitable for real-time embedded deployment.
- **Altitude diversity constraint**: the GDOP optimisation enforces a minimum vertical spread
  of 20 m so that VDOP remains bounded regardless of beacon altitude; without this, L-BFGS-B
  collapses all drones to a horizontal ring, reproducing the coplanar worst case.
- **Separate VDOP/HDOP tracking**: simulation reports VDOP and HDOP as functions of the
  altitude spread parameter $\Delta z$, demonstrating the quadratic degradation in VDOP as all
  drones approach coplanarity ($\Delta z \to 0$).
- **Beacon altitude sweep**: true beacon altitude $z^*$ is swept from 0 to 10 m and
  localisation error is reported for both ranging and TDOA, showing how beacon height relative
  to the drone floor affects VDOP.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of drones $N$ | 5 (minimum 4 for unique 3D solution) |
| Search volume | $300 \times 300 \times 50$ m |
| Beacon altitude range $z^*$ | 0 – 10 m |
| Range noise std dev $\sigma_r$ | 0.5 m (baseline) |
| TDOA noise std dev | $\sqrt{2} \cdot \sigma_r \approx 0.71$ m |
| Drone altitude bounds $[z_{min}, z_{max}]$ | [5, 45] m |
| Minimum altitude spread $\Delta z_{min}$ | 20 m |
| Horizontal radius bounds $[R_{min}, R_{max}]$ | [30, 120] m |
| Inter-drone separation $d_{sep}$ | 15 m |
| GDOP optimiser | L-BFGS-B via `scipy.optimize.minimize` |
| Gauss-Newton tolerance $\epsilon_{tol}$ | $10^{-4}$ m |
| Max GN iterations $K_{max}$ | 100 |
| Monte Carlo trials per configuration | 500 |
| Noise sweep range $\sigma_r$ | 0.1 – 3.0 m |
| GDOP target (optimised layout, $N=5$) | $\leq 1.2$ |
| VDOP coplanar config | $\approx 8.0$ (severely degraded) |
| VDOP optimised 3D config | $\approx 1.4$ |

---

## Expected Output

- **3D anchor geometry plot**: 3D scatter of the five optimised drone positions with lines
  drawn to the beacon; GDOP ellipsoid (3$\sigma$ error ellipsoid from $\boldsymbol{\Sigma}$)
  rendered as a translucent surface centred at the beacon; compared side-by-side with the
  coplanar baseline.
- **VDOP/HDOP vs altitude spread**: two curves showing how VDOP and HDOP change as
  $\Delta z = z_{max} - z_{min}$ increases from 0 (coplanar) to 40 m; VDOP collapses
  rapidly once $\Delta z > 10$ m while HDOP remains nearly constant.
- **TDOA hyperboloid visualisation**: 3D rendering of three TDOA hyperboloids (for $N = 4$
  drones with one reference) and their intersection point at $\mathbf{x}^*$; coloured
  translucent surfaces illustrate the geometry of the 3D uniqueness condition.
- **RMSE vs noise level**: four curves (Range LLS, Range GN, TDOA LLS, TDOA GN) for the
  optimised 5-drone layout plotted against $\sigma_r \in [0.1, 3.0]$ m; CRLB envelopes for
  both estimators overlaid as shaded bands.
- **Beacon altitude sweep**: 3D position RMSE, VDOP, and HDOP as functions of true beacon
  altitude $z^* \in [0, 10]$ m, for both the optimised layout and the coplanar baseline.
- **GDOP optimisation convergence**: objective value (GDOP) per L-BFGS-B iteration; final
  optimised anchor positions printed in a table with their VDOP, HDOP, and GDOP values.
- **Error histogram**: 3D position error distribution from 500 Monte Carlo trials at
  $\sigma_r = 0.5$ m; separate bars for horizontal and vertical components showing that TDOA
  GN reaches the CRLB while TDOA LLS carries a small bias at high noise.

---

## Extensions

1. **Asynchronous TDOA**: relax the synchronised-clock assumption; model the clock offset
   $\delta_i$ at each drone as an additional unknown, augmenting the state to
   $[\mathbf{x}, \delta_2, \ldots, \delta_N]^T$, and derive the modified TDOA Fisher matrix
   to quantify the localisation penalty from clock drift.
2. **Online drone repositioning**: after a first localisation epoch, re-solve the GDOP
   minimisation with the current $\hat{\mathbf{x}}$ as the target and command the drones to
   fly to the new optimal positions; simulate convergence of GDOP over successive epochs as
   the beacon estimate is refined.
3. **Multi-beacon scenario**: extend to $M = 3$ simultaneous survivors; formulate a joint
   GDOP metric averaged over all beacon positions and solve the multi-target anchor placement
   problem as a multi-objective optimisation (Pareto frontier of VDOP vs HDOP vs coverage).
4. **Robust TDOA under NLOS**: model occasional non-line-of-sight (NLOS) bias as a positive-
   only contamination $b_i \sim \text{Exp}(\lambda)$ added to the range; apply an $M$-estimator
   (Huber or Tukey bisquare) in place of ordinary least squares and evaluate breakdown point.
5. **EKF beacon tracking**: the beacon (a moving survivor) follows a random-walk process;
   run an iterated EKF that fuses TDOA and inertial prediction to track position at 1 Hz
   update rate; compare steady-state RMSE against the static CRLB.

---

## Related Scenarios

- Original 2D version: [S046](../S046_trilateration.md)
- Prerequisite: [S042 Missing Person Search](../S042_missing_person.md) — motivation for
  accurate 3D localisation of survivors in complex terrain
- Follow-up: [S050 Swarm SLAM](../S050_slam.md) — extends anchor-based localisation to
  simultaneous mapping when anchor positions are themselves uncertain
- Algorithmic cross-reference: [S008 Stochastic Pursuit](../../01_pursuit_evasion/S008_stochastic_pursuit.md)
  (Kalman filtering under Gaussian noise), [S047 Signal Relay Enhancement](../S047_signal_relay.md)
  (mesh network timing synchronisation needed for TDOA)
