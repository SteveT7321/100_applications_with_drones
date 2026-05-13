# S098 Swarm Synchronized Dance

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Distributed PTP Clock Sync + Choreography Spline Playback | **Dimension**: 3D

---

## Problem Definition

**Setup**: Ten drones perform a 60-second pre-scripted aerial dance at a show altitude of 50 m.
Each drone independently reads its waypoints from a shared choreography script using its own local
clock. Without any synchronisation mechanism, local clocks drift independently — after 60 seconds
the swarm desynchronises by up to 50 ms, producing visible position mismatches that ruin the visual
effect. A distributed Precision Time Protocol (PTP) consensus corrects each drone's clock offset
continuously throughout the performance so that the maximum synchronisation error stays below 10 ms
at all times.

**Roles**:
- **Drones** ($N = 10$): identical quadrotors performing individual choreography tracks from a
  pre-authored waypoint script. Each drone has a slightly drifting local clock whose offset evolves
  as a random walk with standard deviation $\sigma_{drift} = 5$ ms per correction interval.
- **Choreography script**: a fixed sequence of 3D waypoints per drone, parameterised by the global
  time $t \in [0, 60]$ s. Positions between waypoints are interpolated with a cubic spline.
- **PTP consensus network**: a fully connected broadcast graph — every drone broadcasts its current
  local timestamp at the start of each correction interval, and every peer applies an exponential
  moving-average correction to its local clock offset estimate.

**Objective**: maintain the per-drone synchronisation error
$\varepsilon_k(t) = |t_{local,k}(t) - t_{global}(t)|$ below **10 ms** throughout the 60 s
performance. The primary metric is $\varepsilon_{sync}(t) = \max_k \varepsilon_k(t)$. A secondary
metric, visual fidelity $V(t) \in [0, 1]$, measures how close the actual drone positions (computed
with drifted clocks) are to the ideal choreography positions (computed with the ground-truth global
clock).

**Comparison strategies**:
1. **No sync** — clocks drift freely; no PTP correction applied.
2. **PTP sync** ($\beta = 0.5$) — standard PTP exponential averaging at every correction interval.
3. **Aggressive PTP** ($\beta = 0.9$) — faster correction weight; convergence speed vs. noise
   sensitivity trade-off examined.

---

## Mathematical Model

### Local Clock Model

Each drone $k$ maintains a local time $t_{local,k}(t)$ that deviates from the true global time
$t_{global}$ by a clock offset $\delta_k(t)$:

$$t_{local,k}(t) = t_{global}(t) + \delta_k(t)$$

The offset evolves as a discrete-time random walk at each PTP correction interval
$\Delta t_{ptp}$:

$$\delta_k(t + \Delta t_{ptp}) = \delta_k(t) + \eta_k, \qquad \eta_k \sim \mathcal{N}(0,\, \sigma_{drift}^2)$$

Initial offsets are drawn uniformly: $\delta_k(0) \sim \mathcal{U}(-5, +5)$ ms.

### PTP Clock Correction

At each correction interval every drone $k$ broadcasts its local timestamp. Drone $k$ receives
timestamps $t_j^{recv}$ from all peers $j \in \mathcal{N}_k = \{1,\ldots,N\} \setminus \{k\}$.
Ignoring propagation delay (drones are co-located within a few hundred metres), the offset estimate
that peer $j$ provides to drone $k$ is:

$$\hat{\delta}_{k \leftarrow j} = t_j^{recv} - t_k^{sent}$$

Drone $k$ computes the consensus offset correction as the mean over all peers:

$$\Delta_k = \frac{1}{|\mathcal{N}_k|} \sum_{j \in \mathcal{N}_k} \hat{\delta}_{k \leftarrow j}$$

It then applies an exponential moving-average update to its internal offset estimate
$\hat{\delta}_k$:

$$\hat{\delta}_k \leftarrow (1 - \beta)\,\hat{\delta}_k + \beta\,\Delta_k$$

and advances its local clock by:

$$\delta_k \leftarrow \delta_k - \hat{\delta}_k$$

The correction gain $\beta \in (0, 1)$ controls the speed-versus-noise trade-off: large $\beta$
corrects quickly but amplifies measurement noise; small $\beta$ is smoother but slower to converge.

### Choreography Playback via Cubic Spline

Each drone $k$ has a pre-authored waypoint table
$\{(t_n^{wp},\, \mathbf{w}_{k,n})\}_{n=0}^{N_{wp}}$ where $\mathbf{w}_{k,n} \in \mathbb{R}^3$.
The commanded position at any time query $\tau$ is the natural cubic spline:

$$\mathbf{p}_k^{cmd}(\tau) = \mathrm{CubicSpline}\!\left(\{t_n^{wp}\},\, \{\mathbf{w}_{k,n}\}\right)\!(\tau)$$

During a real flight drone $k$ queries the spline at its local time:

$$\mathbf{p}_k^{actual}(t) = \mathbf{p}_k^{cmd}\!\left(t_{local,k}(t)\right)$$

whereas the ideal position uses the global clock:

$$\mathbf{p}_k^{ideal}(t) = \mathbf{p}_k^{cmd}\!\left(t_{global}(t)\right)$$

### Synchronisation Error

The per-drone synchronisation error (in ms) is:

$$\varepsilon_k(t) = \bigl|t_{local,k}(t) - t_{global}(t)\bigr| \times 10^3$$

The swarm-level synchronisation error used as the primary KPI is:

$$\varepsilon_{sync}(t) = \max_{k \in \{1,\ldots,N\}} \varepsilon_k(t) \quad [\text{ms}]$$

The **target** throughout is $\varepsilon_{sync}(t) < 10$ ms $\;\forall\, t \in [0, 60]$ s.

### Visual Fidelity

For each drone the position error due to clock offset is
$\|\mathbf{p}_k^{actual}(t) - \mathbf{p}_k^{ideal}(t)\|$. The mean-speed of the choreography at
time $t$ is $d_{move}(t) = \max_k \|\dot{\mathbf{p}}_k^{ideal}(t)\| \cdot \Delta t_{ptp}$
(clamped to a minimum of 0.01 m to avoid division by zero). The per-drone visual fidelity is:

$$v_k(t) = 1 - \frac{\|\mathbf{p}_k^{actual}(t) - \mathbf{p}_k^{ideal}(t)\|}{d_{move}(t)}$$

clamped to $[0, 1]$. The swarm-averaged visual fidelity is:

$$V(t) = \frac{1}{N} \sum_{k=1}^{N} v_k(t)$$

A value of $V = 1$ means all drones are at their ideal scripted positions; $V < 0.9$ is considered a
visible artefact.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
import matplotlib.animation as animation

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
N_DRONES        = 10
T_SHOW          = 60.0          # s  — total performance duration
LED_HEIGHT      = 50.0          # m  — show altitude (z base)
DT_SIM          = 0.05          # s  — simulation timestep (20 Hz)
DT_PTP          = 0.5           # s  — PTP correction interval
SIGMA_DRIFT     = 5e-3          # s  — clock drift std dev per PTP interval (5 ms)
SYNC_TARGET_MS  = 10.0          # ms — synchronisation error target
BETA_STANDARD   = 0.5           # PTP correction gain (standard)
BETA_AGGRESSIVE = 0.9           # PTP correction gain (aggressive)
N_WP            = 13            # waypoints per drone (every ~5 s)
RNG_SEED        = 42

# ---------------------------------------------------------------------------
# Choreography generation
# ---------------------------------------------------------------------------

def build_choreography(rng):
    """
    Generate pre-scripted waypoints for N_DRONES drones over T_SHOW seconds.
    Drones are arranged on two concentric rings at LED_HEIGHT.
    Returns: t_wp (N_WP,), waypoints (N_DRONES, N_WP, 3).
    """
    t_wp = np.linspace(0.0, T_SHOW, N_WP)
    waypoints = np.zeros((N_DRONES, N_WP, 3))

    for k in range(N_DRONES):
        ring  = 0 if k < 5 else 1
        r_base = 8.0 if ring == 0 else 16.0
        phi0   = (2 * np.pi * k / 5) + (ring * np.pi / 5)

        for n, t in enumerate(t_wp):
            # Phase-modulated spiral with sinusoidal altitude variation
            phase = phi0 + 0.5 * np.pi * np.sin(2 * np.pi * t / T_SHOW)
            r     = r_base + 3.0 * np.sin(4 * np.pi * t / T_SHOW + phi0)
            z     = LED_HEIGHT + 5.0 * np.sin(2 * np.pi * t / T_SHOW + k * np.pi / N_DRONES)
            waypoints[k, n] = [r * np.cos(phase), r * np.sin(phase), z]

    return t_wp, waypoints


def build_splines(t_wp, waypoints):
    """Return list of CubicSpline objects, one per drone (queries return (3,) position)."""
    splines = []
    for k in range(N_DRONES):
        cs = CubicSpline(t_wp, waypoints[k], axis=0, bc_type='not-a-knot')
        splines.append(cs)
    return splines

# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def run_simulation(splines, t_wp, strategy='ptp_standard'):
    """
    Simulate the 60-second performance under a given clock-sync strategy.

    Parameters
    ----------
    strategy : str
        'no_sync'        — clocks drift freely, no correction
        'ptp_standard'   — PTP with BETA_STANDARD
        'ptp_aggressive' — PTP with BETA_AGGRESSIVE

    Returns
    -------
    t_log        : (T_steps,) array of global timestamps
    delta_log    : (T_steps, N_DRONES) clock offsets (s)
    pos_actual   : (T_steps, N_DRONES, 3) positions from drifted clocks
    pos_ideal    : (T_steps, N_DRONES, 3) positions from global clock
    sync_err_ms  : (T_steps,) max |delta_k| across drones, in ms
    vis_fidelity : (T_steps,) swarm-averaged visual fidelity V(t)
    ptp_events   : list of t values at which PTP corrections were applied
    """
    rng = np.random.default_rng(RNG_SEED)

    # Initial clock offsets in seconds (uniform ±5 ms)
    delta = rng.uniform(-5e-3, 5e-3, size=N_DRONES)
    delta_hat = np.zeros(N_DRONES)      # drone's own estimate of its offset

    beta = {'no_sync': 0.0, 'ptp_standard': BETA_STANDARD,
            'ptp_aggressive': BETA_AGGRESSIVE}[strategy]

    t_sim_steps = np.arange(0.0, T_SHOW + DT_SIM / 2, DT_SIM)
    T_steps = len(t_sim_steps)

    delta_log    = np.zeros((T_steps, N_DRONES))
    pos_actual   = np.zeros((T_steps, N_DRONES, 3))
    pos_ideal    = np.zeros((T_steps, N_DRONES, 3))
    sync_err_ms  = np.zeros(T_steps)
    vis_fidelity = np.zeros(T_steps)
    ptp_events   = []

    t_next_ptp = DT_PTP

    for i, t_global in enumerate(t_sim_steps):
        # PTP correction at each correction interval
        if t_global >= t_next_ptp and strategy != 'no_sync':
            # Each drone broadcasts its local timestamp
            t_local_now = t_global + delta          # shape (N_DRONES,)
            for k in range(N_DRONES):
                # Peers' timestamps as received by drone k
                peer_offsets = t_local_now - t_local_now[k]  # what peers look like to k
                delta_consensus = peer_offsets.mean()         # mean offset correction
                # Add clock drift noise for this interval
                delta[k] += rng.normal(0.0, SIGMA_DRIFT)
                # EMA update of offset estimate
                delta_hat[k] = (1 - beta) * delta_hat[k] + beta * delta_consensus
                # Correct local clock
                delta[k] -= delta_hat[k]

            ptp_events.append(t_global)
            t_next_ptp += DT_PTP
        else:
            # Between corrections: clocks drift passively
            # Drift is applied at PTP intervals; between them offsets are constant
            pass

        # Query splines at local times (clamp to valid script range)
        t_local = np.clip(t_global + delta, 0.0, T_SHOW)
        t_ideal_clamped = np.clip(t_global, 0.0, T_SHOW)

        for k in range(N_DRONES):
            pos_actual[i, k] = splines[k](t_local[k])
            pos_ideal[i, k]  = splines[k](t_ideal_clamped)

        delta_log[i]  = delta.copy()

        # Sync error: max |delta_k| in ms
        sync_err_ms[i] = np.max(np.abs(delta)) * 1e3

        # Visual fidelity
        pos_err = np.linalg.norm(pos_actual[i] - pos_ideal[i], axis=1)  # (N_DRONES,)
        # Characteristic motion scale: spline speed at t_global
        if t_global < T_SHOW - DT_PTP:
            speeds = np.array([np.linalg.norm(splines[k](t_ideal_clamped, 1))
                               for k in range(N_DRONES)])
        else:
            speeds = np.ones(N_DRONES)
        d_move = np.maximum(speeds * DT_PTP, 0.01)
        v_k = np.clip(1.0 - pos_err / d_move, 0.0, 1.0)
        vis_fidelity[i] = v_k.mean()

    return t_sim_steps, delta_log, pos_actual, pos_ideal, sync_err_ms, vis_fidelity, ptp_events


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_results(results_dict, splines, t_wp):
    """
    Generate all required figures.
    results_dict: {'no_sync': (...), 'ptp_standard': (...), 'ptp_aggressive': (...)}
    """
    colours  = {'no_sync': 'crimson', 'ptp_standard': 'steelblue',
                'ptp_aggressive': 'darkorange'}
    labels   = {'no_sync': 'No Sync', 'ptp_standard': f'PTP β={BETA_STANDARD}',
                'ptp_aggressive': f'PTP β={BETA_AGGRESSIVE}'}

    # ------------------------------------------------------------------
    # Figure 1: Sync error over time for all three strategies
    # ------------------------------------------------------------------
    fig1, ax1 = plt.subplots(figsize=(10, 4))
    for strat, res in results_dict.items():
        t_log, _, _, _, sync_err_ms, _, _ = res
        ax1.plot(t_log, sync_err_ms, color=colours[strat], label=labels[strat], linewidth=1.8)
    ax1.axhline(SYNC_TARGET_MS, color='green', linestyle='--', linewidth=1.5,
                label='Target (<10 ms)')
    ax1.set_xlabel('Global time  $t$ (s)')
    ax1.set_ylabel('$\\varepsilon_{sync}(t)$  (ms)')
    ax1.set_title('Swarm Synchronisation Error — Three Strategies')
    ax1.legend()
    ax1.set_xlim(0, T_SHOW)
    ax1.set_ylim(bottom=0)
    fig1.tight_layout()
    fig1.savefig('outputs/05_special_entertainment/s098_synchronized_dance/sync_error.png', dpi=150)

    # ------------------------------------------------------------------
    # Figure 2: Visual fidelity over time
    # ------------------------------------------------------------------
    fig2, ax2 = plt.subplots(figsize=(10, 4))
    for strat, res in results_dict.items():
        t_log, _, _, _, _, vis_fidelity, _ = res
        ax2.plot(t_log, vis_fidelity * 100, color=colours[strat],
                 label=labels[strat], linewidth=1.8)
    ax2.axhline(90, color='green', linestyle='--', linewidth=1.5, label='Acceptable (90%)')
    ax2.set_xlabel('Global time  $t$ (s)')
    ax2.set_ylabel('$V(t)$  (%)')
    ax2.set_title('Swarm Visual Fidelity Over Performance')
    ax2.legend()
    ax2.set_xlim(0, T_SHOW)
    ax2.set_ylim(0, 102)
    fig2.tight_layout()
    fig2.savefig('outputs/05_special_entertainment/s098_synchronized_dance/visual_fidelity.png',
                 dpi=150)

    # ------------------------------------------------------------------
    # Figure 3: Per-drone clock offset evolution (PTP standard)
    # ------------------------------------------------------------------
    t_log, delta_log, _, _, _, _, ptp_events = results_dict['ptp_standard']
    fig3, ax3 = plt.subplots(figsize=(10, 4))
    cmap = plt.get_cmap('tab10')
    for k in range(N_DRONES):
        ax3.plot(t_log, delta_log[:, k] * 1e3, color=cmap(k),
                 label=f'Drone {k+1}', linewidth=1.2, alpha=0.85)
    for ev in ptp_events[::4]:   # mark every 4th PTP event to avoid clutter
        ax3.axvline(ev, color='grey', linewidth=0.4, linestyle=':')
    ax3.axhline(0, color='black', linewidth=0.8)
    ax3.set_xlabel('Global time  $t$ (s)')
    ax3.set_ylabel('Clock offset  $\\delta_k(t)$  (ms)')
    ax3.set_title(f'Per-Drone Clock Offsets — PTP β={BETA_STANDARD}')
    ax3.legend(fontsize=7, ncol=5, loc='upper right')
    ax3.set_xlim(0, T_SHOW)
    fig3.tight_layout()
    fig3.savefig('outputs/05_special_entertainment/s098_synchronized_dance/clock_offsets.png',
                 dpi=150)

    # ------------------------------------------------------------------
    # Figure 4: 3D choreography trajectories at t=0..T_SHOW (ideal)
    # ------------------------------------------------------------------
    fig4 = plt.figure(figsize=(10, 8))
    ax4 = fig4.add_subplot(111, projection='3d')
    cmap = plt.get_cmap('tab10')
    t_dense = np.linspace(0, T_SHOW, 300)
    for k in range(N_DRONES):
        traj = np.array([splines[k](t) for t in t_dense])
        ax4.plot(traj[:, 0], traj[:, 1], traj[:, 2],
                 color=cmap(k), linewidth=1.4, alpha=0.85)
        ax4.scatter(*traj[0], color=cmap(k), s=40, zorder=5)    # start
        ax4.scatter(*traj[-1], color=cmap(k), s=40, marker='^', zorder=5)  # end
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.set_zlabel('Z (m)')
    ax4.set_title('Ideal Choreography — All 10 Drone Tracks (3D)')
    fig4.tight_layout()
    fig4.savefig('outputs/05_special_entertainment/s098_synchronized_dance/choreography_3d.png',
                 dpi=150)

    # ------------------------------------------------------------------
    # Animation: top-down XY view of actual vs ideal positions (PTP vs no sync)
    # ------------------------------------------------------------------
    t_log_ns, _, pos_actual_ns, pos_ideal_ns, _, _, _ = results_dict['no_sync']
    t_log_pt, _, pos_actual_pt, pos_ideal_pt, _, _, _ = results_dict['ptp_standard']

    # Subsample to 10 fps for animation
    step = max(1, int(0.1 / DT_SIM))
    frames = range(0, len(t_log_ns), step)

    fig_a, axes = plt.subplots(1, 2, figsize=(14, 6))
    titles = ['No Synchronisation', f'PTP β={BETA_STANDARD}']
    cmap = plt.get_cmap('tab10')

    def init_anim():
        for ax, title in zip(axes, titles):
            ax.set_xlim(-22, 22)
            ax.set_ylim(-22, 22)
            ax.set_aspect('equal')
            ax.set_title(title)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
        return []

    scat_ideal_ns = axes[0].scatter([], [], c='grey', s=30, alpha=0.5, label='Ideal')
    scat_actual_ns = axes[0].scatter([], [], c=[cmap(k) for k in range(N_DRONES)],
                                     s=60, zorder=5, label='Actual')
    scat_ideal_pt = axes[1].scatter([], [], c='grey', s=30, alpha=0.5, label='Ideal')
    scat_actual_pt = axes[1].scatter([], [], c=[cmap(k) for k in range(N_DRONES)],
                                     s=60, zorder=5, label='Actual')
    time_text_ns = axes[0].text(0.02, 0.95, '', transform=axes[0].transAxes, fontsize=9)
    time_text_pt = axes[1].text(0.02, 0.95, '', transform=axes[1].transAxes, fontsize=9)

    for ax in axes:
        ax.legend(loc='upper right', fontsize=8)

    def update(frame):
        i = frame
        t = t_log_ns[i]
        scat_ideal_ns.set_offsets(pos_ideal_ns[i, :, :2])
        scat_actual_ns.set_offsets(pos_actual_ns[i, :, :2])
        scat_ideal_pt.set_offsets(pos_ideal_pt[i, :, :2])
        scat_actual_pt.set_offsets(pos_actual_pt[i, :, :2])
        time_text_ns.set_text(f't = {t:.1f} s')
        time_text_pt.set_text(f't = {t:.1f} s')
        return [scat_ideal_ns, scat_actual_ns, scat_ideal_pt, scat_actual_pt,
                time_text_ns, time_text_pt]

    init_anim()
    anim = animation.FuncAnimation(fig_a, update, frames=list(frames),
                                   interval=100, blit=True)
    anim.save('outputs/05_special_entertainment/s098_synchronized_dance/sync_dance_animation.gif',
              writer='pillow', fps=10)
    plt.close('all')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    import os
    os.makedirs('outputs/05_special_entertainment/s098_synchronized_dance', exist_ok=True)

    rng = np.random.default_rng(RNG_SEED)
    t_wp, waypoints = build_choreography(rng)
    splines = build_splines(t_wp, waypoints)

    strategies = ['no_sync', 'ptp_standard', 'ptp_aggressive']
    results = {}
    for strat in strategies:
        print(f'Running strategy: {strat} ...')
        results[strat] = run_simulation(splines, t_wp, strategy=strat)

    # Print summary metrics
    print('\n=== Summary ===')
    for strat, res in results.items():
        t_log, _, _, _, sync_err_ms, vis_fidelity, _ = res
        print(f'{strat:20s}  max_sync_err={sync_err_ms.max():.1f} ms'
              f'  mean_vis_fidelity={vis_fidelity.mean()*100:.1f}%'
              f'  target_met={sync_err_ms.max() < SYNC_TARGET_MS}')

    plot_results(results, splines, t_wp)
    print('\nAll figures saved to outputs/05_special_entertainment/s098_synchronized_dance/')
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones | $N$ | 10 |
| Performance duration | $T_{show}$ | 60 s |
| Show altitude | $z_{base}$ | 50 m |
| Simulation timestep | $\Delta t_{sim}$ | 0.05 s (20 Hz) |
| PTP correction interval | $\Delta t_{ptp}$ | 0.5 s |
| Clock drift std dev | $\sigma_{drift}$ | 5 ms per correction interval |
| Initial offset range | $\delta_k(0)$ | $\mathcal{U}(-5, +5)$ ms |
| PTP gain (standard) | $\beta$ | 0.5 |
| PTP gain (aggressive) | $\beta$ | 0.9 |
| Synchronisation target | $\varepsilon_{sync}$ | < 10 ms |
| Visual fidelity threshold | $V$ | > 90% |
| Inner ring radius | $r_1$ | 8 m |
| Outer ring radius | $r_2$ | 16 m |
| Altitude modulation amplitude | — | ±5 m |
| Waypoints per drone | $N_{wp}$ | 13 (every ~5 s) |
| Choreography interpolation | — | Natural cubic spline |

---

## Expected Output

- **Sync error plot** (`sync_error.png`): $\varepsilon_{sync}(t)$ over the 60-second performance
  for all three strategies on the same axes; green dashed line at the 10 ms target; the no-sync
  curve should drift beyond 50 ms while both PTP curves remain below 10 ms after initial convergence.

- **Visual fidelity plot** (`visual_fidelity.png`): $V(t)$ (%) over time for all three strategies;
  90% acceptability threshold marked; no-sync performance degrades visibly while PTP strategies
  maintain near-100% fidelity.

- **Clock offset traces** (`clock_offsets.png`): per-drone clock offsets $\delta_k(t)$ in ms for
  the PTP-standard strategy; 10 coloured lines converging toward zero after initial PTP kicks in;
  vertical dotted lines at every fourth PTP event.

- **3D choreography map** (`choreography_3d.png`): 3D trajectories of all 10 drones across the
  full performance using the ideal (global-clock) playback; two concentric spiralling rings with
  sinusoidal altitude modulation at 50 m; start positions marked with circles, end positions with
  triangles.

- **Synchronised dance animation** (`sync_dance_animation.gif`): side-by-side top-down XY view
  comparing no-sync (left) vs PTP-standard (right) at 10 fps; grey circles show ideal positions,
  coloured markers show actual drone positions; visual separation grows on the left and stays
  negligible on the right.

**Typical numerical results**:

| Strategy | Max sync error | Mean visual fidelity | Target met |
|----------|---------------|----------------------|------------|
| No sync | ~50 ms | ~75% | No |
| PTP $\beta=0.5$ | <10 ms | >97% | Yes |
| PTP $\beta=0.9$ | <8 ms | >98% | Yes |

---

## Extensions

1. **Propagation-delay compensation**: model finite radio propagation delay
   $\tau_{prop} = d_{kj} / c_{radio}$ between each drone pair; incorporate the classic PTP
   round-trip delay estimate to remove systematic bias in the offset measurement.
2. **Packet loss and Byzantine peers**: randomly drop 20% of PTP broadcasts; one drone becomes a
   faulty clock broadcasting a large constant offset; examine the robustness of the median-based
   consensus vs. the mean-based correction used here.
3. **Adaptive $\beta$ scheduling**: use a large correction gain ($\beta = 0.9$) during the
   first 5 seconds of the show to achieve fast initial convergence, then reduce to $\beta = 0.3$
   for steady-state noise suppression.
4. **Waypoint density optimisation**: given a maximum tolerable visual artefact at 10 ms offset,
   compute the maximum allowable inter-waypoint interval as a function of choreography speed; this
   sets a principled $N_{wp}$ rather than the fixed 13 used here.
5. **Hardware-in-the-loop**: replace simulated clock drift with actual NTP/PTP measurements from
   real drone flight computers; benchmark against the IEEE 1588v2 standard used in broadcast
   production.
6. **Music beat synchronisation**: align PTP correction events with the beat grid of a backing
   audio track so that any residual clock jitter is masked by the music rhythm; evaluate perceptual
   quality with a just-noticeable-difference model.

---

## Related Scenarios

- Prerequisites: [S083 Formation Flight](S083_formation_flight.md), [S085 LED Light Matrix](S085_light_matrix.md)
- Follow-ups: [S089 Crowd Safety Monitoring](S089_crowd_safety.md) (real-time coordination under safety constraints)
- Algorithmic cross-reference: [S095 Swarm Communication](S095_swarm_communication.md) (broadcast graph topology), [S049 Dynamic Zone Search](../03_environmental_sar/S049_dynamic_zone.md) (distributed consensus)
- Domain overview: [domains/05_special_entertainment.md](../../domains/05_special_entertainment.md)
