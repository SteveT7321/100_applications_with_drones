# S097 Aerial Puzzle Assembly

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Consensus Positioning + Impedance Fitting Control | **Dimension**: 3D

---

## Problem Definition

**Setup**: Six identical quadrotor drones ($N = 6$) each carry one equilateral-triangular puzzle
piece that, when assembled, forms a regular hexagon. The target assembly location is centred at
$\mathbf{p}_{centre} = (0,\, 0,\, 10)$ m in an open airspace. Each drone $k \in \{0, 1, \ldots, 5\}$
starts at a dispersed staging position on a circle of radius $r_{stage} = 5$ m at $z = 8$ m and
must navigate to its designated slot in the hexagon at $z = 10$ m. The simulation timestep is
$\Delta t = 0.02$ s; each drone has mass $m_D = 0.5$ kg. Drone-to-drone communication follows a
ring topology: drone $k$ exchanges position with its two immediate neighbours
$\mathcal{N}(k) = \{k-1 \bmod 6,\; k+1 \bmod 6\}$.

**Roles**:
- **Drones** ($N = 6$): identical quadrotors that communicate over a ring graph, run local
  consensus updates to converge to global hexagonal slot positions, and apply impedance control
  during the final fitting phase.
- **Puzzle pieces**: rigid triangular panels; each panel's pose is tied to its carrier drone's
  position and heading angle $\psi_k$.
- **Hexagonal assembly frame**: a virtual fixed reference frame centred at $\mathbf{p}_{centre}$;
  the six slot centroids are the vertices of the target hexagon.

**Mission phases**:

1. **Ascent** ($t = 0$ to $t_{asc}$): all drones climb from $z = 8$ m to $z = 10$ m while
   spreading out on the staging circle.
2. **Consensus convergence** ($t_{asc}$ to $t_{conv}$): drones run the consensus update at every
   timestep, iteratively contracting toward their target slots $\mathbf{p}_k^*$.
3. **Impedance fitting** ($t_{conv}$ to $t_{fit}$): once every drone satisfies
   $\|\mathbf{p}_k - \mathbf{p}_k^*\| \leq \delta_{switch} = 0.05$ m, the controller switches to
   impedance mode; drones apply a compliant inward force to slide piece edges into contact with a
   gap tolerance of 5 mm.
4. **Hold**: drones maintain final positions; assembly error $\varepsilon_{assembly}$ is logged.

**Objective**: achieve $\varepsilon_{assembly} = \max_k \|\mathbf{p}_k - \mathbf{p}_k^*\| \leq 5$ mm
with all six drones simultaneously, while maintaining minimum inter-drone separation
$d_{min} = 0.3$ m throughout.

---

## Mathematical Model

### Assembly Target Positions

The six slot centroids form a regular hexagon of circumradius $r_{hex} = 1.0$ m in the horizontal
plane at height $z_{asm} = 10$ m:

$$\mathbf{p}_k^* = \mathbf{p}_{centre} + r_{hex} \begin{pmatrix} \cos\!\left(\dfrac{2\pi k}{6}\right) \\[4pt] \sin\!\left(\dfrac{2\pi k}{6}\right) \\[4pt] 0 \end{pmatrix}, \qquad k = 0, 1, \ldots, 5$$

Each slot also requires a target heading angle $\psi_k^* = \dfrac{2\pi k}{6}$ so that each
triangular piece is radially oriented.

### Consensus Position Update

During the consensus phase every drone updates its commanded position at each timestep using a
discrete-time consensus law with an attraction term toward the individual target:

$$\mathbf{p}_k[t+1] = \mathbf{p}_k[t]
  + \varepsilon \sum_{j \in \mathcal{N}(k)} \!\bigl(\mathbf{p}_j[t] - \mathbf{p}_k[t]\bigr)
  + \alpha \bigl(\mathbf{p}_k^* - \mathbf{p}_k[t]\bigr)$$

where $\varepsilon = 0.05$ is the consensus coupling gain and $\alpha = 0.1$ is the target
attraction gain. The ring-graph Laplacian $\mathbf{L} \in \mathbb{R}^{6 \times 6}$ has entries
$L_{kk} = 2$, $L_{k,k\pm 1} = -1$, and zeros elsewhere. The stacked update for all drones is:

$$\mathbf{P}[t+1] = \mathbf{P}[t] - \varepsilon \mathbf{L} \mathbf{P}[t] + \alpha \bigl(\mathbf{P}^* - \mathbf{P}[t]\bigr)$$

$$= \bigl(\mathbf{I} - \varepsilon \mathbf{L} - \alpha \mathbf{I}\bigr)\mathbf{P}[t] + \alpha \mathbf{P}^*$$

where $\mathbf{P}[t] \in \mathbb{R}^{6 \times 3}$ stacks all drone positions row-wise and
$\mathbf{P}^* \in \mathbb{R}^{6 \times 3}$ stacks all target positions.

### Convergence Analysis (Lyapunov)

Define the aggregate position error:

$$V[t] = \sum_{k=0}^{5} \bigl\|\mathbf{p}_k[t] - \mathbf{p}_k^*\bigr\|^2$$

With the combined update matrix $\mathbf{A} = \mathbf{I} - \varepsilon \mathbf{L} - \alpha \mathbf{I}$,
the spectral radius $\rho(\mathbf{A}) < 1$ when $\varepsilon$ and $\alpha$ satisfy:

$$\varepsilon \lambda_{max}(\mathbf{L}) + \alpha < 2, \qquad \alpha > 0$$

For a 6-node ring graph $\lambda_{max}(\mathbf{L}) = 4$; with $\varepsilon = 0.05$ and
$\alpha = 0.1$ the spectral radius is $\rho = \max_i |1 - \varepsilon \lambda_i - \alpha|$, which
gives $\rho \approx 0.70$, guaranteeing geometric convergence:

$$V[t] \leq \rho^{2t} \, V[0] \quad \Longrightarrow \quad \bigl\|\mathbf{p}_k[t] - \mathbf{p}_k^*\bigr\| \to 0 \text{ as } t \to \infty$$

### Impedance Fitting Control

Once the consensus phase ends (all drones within $\delta_{switch}$ of their target), the
controller switches to impedance mode to account for physical piece-edge contact forces during
final fitting. The impedance law produces a compliant force command:

$$\mathbf{F}_k^{imp} = K_{imp}\bigl(\mathbf{p}_k^* - \mathbf{p}_k\bigr) - B_{imp}\,\dot{\mathbf{p}}_k$$

with stiffness $K_{imp} = 8.0$ N/m and damping $B_{imp} = 4.0$ N s/m. This renders the
closed-loop dynamics equivalent to a spring-damper:

$$m_D \ddot{\mathbf{e}}_k + B_{imp}\,\dot{\mathbf{e}}_k + K_{imp}\,\mathbf{e}_k = \mathbf{F}_{contact,k}$$

where $\mathbf{e}_k = \mathbf{p}_k^* - \mathbf{p}_k$ is the slot error and
$\mathbf{F}_{contact,k}$ is the reaction force from adjacent pieces (approximated as zero for
drones in free flight, non-zero during edge-contact). The natural frequency and damping ratio are:

$$\omega_n = \sqrt{\frac{K_{imp}}{m_D}} = 4.0 \text{ rad/s}, \qquad \zeta = \frac{B_{imp}}{2\sqrt{K_{imp}\,m_D}} = 1.0 \text{ (critically damped)}$$

Critical damping ensures drones approach the slot smoothly without oscillation.

### Assembly Error Metric

$$\varepsilon_{assembly}(t) = \max_{k \in \{0,\ldots,5\}} \bigl\|\mathbf{p}_k(t) - \mathbf{p}_k^*\bigr\|$$

Consensus convergence time $t_{conv}$ is defined as the first $t$ for which
$\varepsilon_{assembly}(t) \leq \delta_{switch} = 0.05$ m. Final fit gap is reported after the
impedance phase completes:

$$\varepsilon_{fit} = \varepsilon_{assembly}(t_{fit}) \quad (\text{target:} \leq 5 \text{ mm})$$

### Piece Orientation Control

Each drone yaws at a constant angular rate toward its target heading during the consensus phase:

$$\dot{\psi}_k = K_\psi \bigl(\psi_k^* - \psi_k\bigr), \qquad K_\psi = 1.0 \text{ rad/s per rad}$$

Integrated as $\psi_k[t+1] = \psi_k[t] + K_\psi(\psi_k^* - \psi_k[t]) \cdot \Delta t$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import os

# ── Constants ────────────────────────────────────────────────────────────────
N_DRONES    = 6
M_DRONE     = 0.5          # kg
DT          = 0.02         # s
G           = 9.81         # m/s²

# Assembly geometry
P_CENTRE    = np.array([0.0, 0.0, 10.0])   # m
R_HEX       = 1.0                           # m  — hexagon circumradius
Z_ASM       = 10.0                          # m

# Staging
R_STAGE     = 5.0                           # m  — staging circle radius
Z_STAGE     = 8.0                           # m

# Consensus gains
EPSILON     = 0.05    # consensus coupling
ALPHA       = 0.10    # target attraction

# Impedance gains
K_IMP       = 8.0     # N/m
B_IMP       = 4.0     # N·s/m

# Phase thresholds
DELTA_SWITCH = 0.05   # m — enter impedance phase
DELTA_FIT    = 0.005  # m — assembly success (5 mm)

# Safety
D_MIN       = 0.30    # m — minimum inter-drone separation
K_REP       = 1.5     # repulsion gain

# Heading control
K_PSI       = 1.0     # rad/s per rad


def make_hex_targets():
    """Return target positions (N,3) and headings (N,) for hexagonal assembly."""
    angles = np.array([2 * np.pi * k / N_DRONES for k in range(N_DRONES)])
    targets = np.column_stack([
        P_CENTRE[0] + R_HEX * np.cos(angles),
        P_CENTRE[1] + R_HEX * np.sin(angles),
        np.full(N_DRONES, Z_ASM),
    ])
    psi_targets = angles.copy()
    return targets, psi_targets


def ring_laplacian(n: int) -> np.ndarray:
    """Build the n-node ring graph Laplacian."""
    L = 2.0 * np.eye(n)
    for k in range(n):
        L[k, (k - 1) % n] = -1.0
        L[k, (k + 1) % n] = -1.0
    return L


def run_simulation(seed: int = 42):
    """
    Run the full aerial puzzle assembly simulation.
    Returns a history dict with logged states.
    """
    rng = np.random.default_rng(seed)
    targets, psi_targets = make_hex_targets()
    L_lap = ring_laplacian(N_DRONES)

    # ── Initial positions: staging circle ────────────────────────────────────
    angles_init = np.array([2 * np.pi * k / N_DRONES for k in range(N_DRONES)])
    # Add small jitter so consensus is non-trivial
    angles_init += rng.uniform(-0.3, 0.3, N_DRONES)
    pos = np.column_stack([
        R_STAGE * np.cos(angles_init),
        R_STAGE * np.sin(angles_init),
        np.full(N_DRONES, Z_STAGE),
    ])
    vel = np.zeros((N_DRONES, 3))
    psi = angles_init.copy()

    # ── Logging ───────────────────────────────────────────────────────────────
    log = {
        't': [],
        'pos': [],          # (T, N, 3)
        'vel': [],          # (T, N, 3)
        'psi': [],          # (T, N)
        'err_assembly': [], # (T,)   max slot error
        'err_each': [],     # (T, N) per-drone slot error
        'phase': [],        # (T,)   0=ascent 1=consensus 2=impedance 3=hold
        'min_sep': [],      # (T,)
    }

    # Phase state
    ascent_done  = False
    fit_done     = False
    hold_steps   = 0
    HOLD_MAX     = 100

    t = 0.0
    phase = 0  # 0: ascent, 1: consensus, 2: impedance, 3: hold

    while True:
        errs = np.linalg.norm(pos - targets, axis=1)   # (N,)
        eps_asm = errs.max()

        # ── Phase transitions ────────────────────────────────────────────────
        if phase == 0:
            # Ascent complete when all drones reach z_asm
            if np.all(pos[:, 2] >= Z_ASM - 0.05):
                phase = 1
        elif phase == 1:
            if eps_asm <= DELTA_SWITCH:
                phase = 2
        elif phase == 2:
            if eps_asm <= DELTA_FIT:
                phase = 3
        elif phase == 3:
            hold_steps += 1
            if hold_steps >= HOLD_MAX:
                break

        # ── Control ──────────────────────────────────────────────────────────
        acc = np.zeros((N_DRONES, 3))

        if phase == 0:
            # Climb straight up to z_asm, spread on staging ring
            for k in range(N_DRONES):
                p_des_k = np.array([
                    R_STAGE * np.cos(angles_init[k]),
                    R_STAGE * np.sin(angles_init[k]),
                    Z_ASM,
                ])
                acc[k] = 8.0 * (p_des_k - pos[k]) - 4.0 * vel[k]

        elif phase == 1:
            # Consensus update on commanded positions, then track with PD
            consensus_update = -EPSILON * (L_lap @ pos) + ALPHA * (targets - pos)
            p_cmd = pos + consensus_update   # one-step lookahead command
            acc = 10.0 * (p_cmd - pos) - 5.0 * vel

        elif phase in (2, 3):
            # Impedance control
            acc = (K_IMP * (targets - pos) - B_IMP * vel) / M_DRONE

        # Soft repulsion (all phases)
        for k in range(N_DRONES):
            for j in range(N_DRONES):
                if j == k:
                    continue
                diff = pos[k] - pos[j]
                dist = np.linalg.norm(diff)
                if dist < D_MIN * 2.0 and dist > 1e-6:
                    acc[k] += K_REP * (D_MIN * 2.0 - dist) / (dist) * diff / dist

        # ── Integrate ─────────────────────────────────────────────────────────
        vel += acc * DT
        pos += vel * DT

        # Yaw toward target
        psi += K_PSI * (psi_targets - psi) * DT

        # ── Metrics ───────────────────────────────────────────────────────────
        errs = np.linalg.norm(pos - targets, axis=1)
        eps_asm = errs.max()
        seps = [
            np.linalg.norm(pos[i] - pos[j])
            for i in range(N_DRONES)
            for j in range(i + 1, N_DRONES)
        ]

        log['t'].append(t)
        log['pos'].append(pos.copy())
        log['vel'].append(vel.copy())
        log['psi'].append(psi.copy())
        log['err_assembly'].append(eps_asm)
        log['err_each'].append(errs.copy())
        log['phase'].append(phase)
        log['min_sep'].append(min(seps))

        t += DT

    return log


def plot_results(log: dict, out_dir: str):
    """Generate static output figures."""
    t_arr  = np.array(log['t'])
    pos    = np.array(log['pos'])      # (T, N, 3)
    errs   = np.array(log['err_each']) # (T, N)
    eps    = np.array(log['err_assembly'])
    phases = np.array(log['phase'])
    seps   = np.array(log['min_sep'])
    targets, _ = make_hex_targets()

    drone_colors = ['tab:red', 'tab:orange', 'tab:green',
                    'tab:cyan', 'tab:blue', 'tab:purple']

    # ── Figure 1: 3D trajectory ───────────────────────────────────────────
    fig1 = plt.figure(figsize=(10, 8))
    ax3d = fig1.add_subplot(111, projection='3d')

    for k in range(N_DRONES):
        ax3d.plot(pos[:, k, 0], pos[:, k, 1], pos[:, k, 2],
                  color=drone_colors[k], lw=1.4, alpha=0.8,
                  label=f'Drone {k}')
        ax3d.scatter(*pos[0, k, :], color=drone_colors[k], marker='o',
                     s=50, zorder=5)
        ax3d.scatter(*targets[k], color=drone_colors[k], marker='*',
                     s=120, zorder=6)

    # Draw target hexagon outline
    hex_xy = np.vstack([targets, targets[0]])
    ax3d.plot(hex_xy[:, 0], hex_xy[:, 1],
              np.full(len(hex_xy), 10.0), 'k--', lw=1.5, alpha=0.5,
              label='Target hexagon')
    ax3d.scatter(*P_CENTRE, color='green', s=100, marker='^', zorder=7,
                 label='Assembly centre')

    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')
    ax3d.set_title('S097 — Aerial Puzzle Assembly: 3D Drone Trajectories')
    ax3d.legend(fontsize=7, loc='upper left')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 's097_3d_trajectory.png'), dpi=150)
    plt.close()

    # ── Figure 2: Metrics panel ───────────────────────────────────────────
    fig2, axes = plt.subplots(2, 2, figsize=(13, 8))

    # Phase background shading
    phase_colors = {0: '#fff3e0', 1: '#e8f5e9', 2: '#e3f2fd', 3: '#f3e5f5'}
    phase_labels = {0: 'Ascent', 1: 'Consensus', 2: 'Impedance', 3: 'Hold'}
    for ax in axes.flat:
        for ph in range(4):
            mask = phases == ph
            if not np.any(mask):
                continue
            t_on  = t_arr[mask][0]
            t_off = t_arr[mask][-1]
            ax.axvspan(t_on, t_off, color=phase_colors[ph], alpha=0.25,
                       label=phase_labels[ph])

    # Top-left: assembly error over time
    axes[0, 0].semilogy(t_arr, eps, 'k-', lw=2, label='$\\varepsilon_{assembly}$')
    axes[0, 0].axhline(DELTA_SWITCH, color='orange', ls='--', lw=1.2,
                       label=f'$\\delta_{{switch}}={DELTA_SWITCH}$ m')
    axes[0, 0].axhline(DELTA_FIT, color='red', ls=':', lw=1.5,
                       label=f'$\\delta_{{fit}}={DELTA_FIT*1000:.0f}$ mm')
    axes[0, 0].set_title('Assembly Error $\\varepsilon_{assembly}$ (m, log scale)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].grid(True, which='both', alpha=0.3)

    # Top-right: per-drone slot error
    for k in range(N_DRONES):
        axes[0, 1].plot(t_arr, errs[:, k], color=drone_colors[k],
                        lw=1.2, alpha=0.85, label=f'Drone {k}')
    axes[0, 1].axhline(DELTA_FIT, color='red', ls=':', lw=1.5,
                       label='5 mm target')
    axes[0, 1].set_title('Per-Drone Slot Error $\\|\\mathbf{p}_k - \\mathbf{p}_k^*\\|$ (m)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].legend(fontsize=7)
    axes[0, 1].grid(True, alpha=0.3)

    # Bottom-left: minimum inter-drone separation
    axes[1, 0].plot(t_arr, seps, 'tab:brown', lw=1.5, label='Min separation')
    axes[1, 0].axhline(D_MIN, color='red', ls=':', lw=1.2,
                       label=f'$d_{{min}}={D_MIN}$ m')
    axes[1, 0].set_title('Minimum Inter-Drone Separation (m)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.3)

    # Bottom-right: drone Z altitude over time
    for k in range(N_DRONES):
        axes[1, 1].plot(t_arr, np.array(log['pos'])[:, k, 2],
                        color=drone_colors[k], lw=1.2, alpha=0.85,
                        label=f'Drone {k}')
    axes[1, 1].axhline(Z_ASM, color='gray', ls='--', lw=1.2,
                       label=f'$z_{{asm}}={Z_ASM}$ m')
    axes[1, 1].set_title('Drone Altitude Z (m)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].legend(fontsize=7)
    axes[1, 1].grid(True, alpha=0.3)

    # Add a shared legend for phase shading (first subplot only)
    from matplotlib.patches import Patch
    phase_patches = [Patch(facecolor=phase_colors[ph], alpha=0.5, label=phase_labels[ph])
                     for ph in range(4)]
    fig2.legend(handles=phase_patches, loc='lower center', ncol=4,
                fontsize=9, title='Mission Phase', framealpha=0.8)
    plt.suptitle('S097 — Aerial Puzzle Assembly: Performance Metrics',
                 fontsize=13, fontweight='bold')
    plt.tight_layout(rect=[0, 0.07, 1, 1])
    plt.savefig(os.path.join(out_dir, 's097_metrics_panel.png'), dpi=150)
    plt.close()


def animate_assembly(log: dict, out_path: str):
    """Create 3D animation of the hexagonal puzzle assembly (GIF)."""
    pos_all  = np.array(log['pos'])    # (T, N, 3)
    psi_all  = np.array(log['psi'])    # (T, N)
    phases   = np.array(log['phase'])
    n_frames_total = len(pos_all)
    step     = max(1, n_frames_total // 200)
    targets, _ = make_hex_targets()

    drone_colors = ['tab:red', 'tab:orange', 'tab:green',
                    'tab:cyan', 'tab:blue', 'tab:purple']
    phase_names  = {0: 'Ascent', 1: 'Consensus', 2: 'Impedance Fitting', 3: 'Hold'}

    fig = plt.figure(figsize=(9, 7))
    ax  = fig.add_subplot(111, projection='3d')

    ax.set_xlim(-6, 6); ax.set_ylim(-6, 6); ax.set_zlim(7, 12)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

    # Target hexagon (static)
    hex_xy = np.vstack([targets, targets[0]])
    ax.plot(hex_xy[:, 0], hex_xy[:, 1],
            np.full(len(hex_xy), 10.0), 'k--', lw=1.2, alpha=0.5)
    ax.scatter(*P_CENTRE, color='green', s=80, marker='^', zorder=8)

    # Drone markers and trails
    drone_dots  = [ax.plot([], [], [], 'o', color=c, ms=10, zorder=7)[0]
                   for c in drone_colors]
    drone_trails = [ax.plot([], [], [], '-', color=c, lw=0.8, alpha=0.5)[0]
                    for c in drone_colors]
    title_text = ax.set_title('')

    def update(frame_idx):
        k = frame_idx * step
        k = min(k, n_frames_total - 1)
        pos = pos_all[k]   # (N, 3)
        ph  = phases[k]

        for i in range(N_DRONES):
            drone_dots[i].set_data([pos[i, 0]], [pos[i, 1]])
            drone_dots[i].set_3d_properties([pos[i, 2]])
            trail_start = max(0, k - 60)
            drone_trails[i].set_data(pos_all[trail_start:k, i, 0],
                                     pos_all[trail_start:k, i, 1])
            drone_trails[i].set_3d_properties(pos_all[trail_start:k, i, 2])

        t_val = k * DT
        title_text.set_text(
            f'S097 Aerial Puzzle Assembly — t = {t_val:.2f} s  '
            f'[{phase_names.get(ph, "")}]'
        )
        return drone_dots + drone_trails + [title_text]

    n_anim_frames = (n_frames_total - 1) // step + 1
    ani = animation.FuncAnimation(fig, update, frames=n_anim_frames,
                                  interval=50, blit=False)
    ani.save(out_path, writer='pillow', fps=20)
    plt.close()
    print(f"Animation saved to {out_path}")


def run_simulation_suite():
    out_dir = 'outputs/05_special_entertainment/s097_aerial_puzzle'
    os.makedirs(out_dir, exist_ok=True)

    print("Running S097 Aerial Puzzle Assembly simulation ...")
    log = run_simulation(seed=42)

    t_arr  = np.array(log['t'])
    eps    = np.array(log['err_assembly'])
    phases = np.array(log['phase'])

    # Compute summary statistics
    t_conv_idx = next((i for i, ph in enumerate(phases) if ph >= 2), len(phases) - 1)
    t_fit_idx  = next((i for i, ph in enumerate(phases) if ph >= 3), len(phases) - 1)
    t_conv = t_arr[t_conv_idx]
    t_fit  = t_arr[t_fit_idx]
    final_err_mm = eps[-1] * 1000.0

    print(f"  Consensus convergence time : {t_conv:.2f} s")
    print(f"  Impedance fitting time     : {t_fit:.2f} s")
    print(f"  Final assembly error       : {final_err_mm:.2f} mm")
    print(f"  Min inter-drone separation : {min(log['min_sep']):.3f} m")

    plot_results(log, out_dir)
    animate_assembly(log, os.path.join(out_dir, 's097_animation.gif'))
    print("All outputs saved.")
    return log


if __name__ == '__main__':
    run_simulation_suite()
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones / pieces | $N$ | 6 |
| Drone mass | $m_D$ | 0.5 kg |
| Assembly centre | $\mathbf{p}_{centre}$ | $(0,\, 0,\, 10)$ m |
| Hexagon circumradius | $r_{hex}$ | 1.0 m |
| Staging circle radius | $r_{stage}$ | 5.0 m |
| Staging altitude | $z_{stage}$ | 8.0 m |
| Assembly altitude | $z_{asm}$ | 10.0 m |
| Simulation timestep | $\Delta t$ | 0.02 s |
| Consensus coupling gain | $\varepsilon$ | 0.05 |
| Target attraction gain | $\alpha$ | 0.10 |
| Ring-Laplacian max eigenvalue | $\lambda_{max}$ | 4.0 |
| Spectral radius of update matrix | $\rho(\mathbf{A})$ | ≈ 0.70 |
| Impedance stiffness | $K_{imp}$ | 8.0 N/m |
| Impedance damping | $B_{imp}$ | 4.0 N s/m |
| Natural frequency (impedance) | $\omega_n$ | 4.0 rad/s |
| Damping ratio (impedance) | $\zeta$ | 1.0 (critical) |
| Phase-switch threshold | $\delta_{switch}$ | 50 mm |
| Assembly success threshold | $\delta_{fit}$ | 5 mm |
| Minimum inter-drone separation | $d_{min}$ | 0.30 m |
| Yaw control gain | $K_\psi$ | 1.0 rad/s per rad |
| Communication topology | — | Ring graph ($k \leftrightarrow k\pm1$) |

---

## Expected Output

- **3D trajectory plot** (`s097_3d_trajectory.png`): world-frame 3D axes showing all six drone
  paths from staging circle ($z = 8$ m) to hexagonal slots ($z = 10$ m); each drone rendered in a
  distinct colour; open circles mark start positions; star markers indicate target slots; dashed
  hexagon outline drawn at $z = 10$ m; green triangle at assembly centre.
- **Metrics panel** (`s097_metrics_panel.png`, 2 × 2 subplots): background shading distinguishes
  the four mission phases (ascent / consensus / impedance / hold) in each subplot.
  - Top-left: assembly error $\varepsilon_{assembly}(t)$ on a log scale; orange dashed line at
    $\delta_{switch} = 50$ mm; red dotted line at $\delta_{fit} = 5$ mm.
  - Top-right: per-drone slot error $\|\mathbf{p}_k - \mathbf{p}_k^*\|$ for all six drones on
    linear scale.
  - Bottom-left: minimum inter-drone separation vs time; red dotted line at $d_{min} = 0.30$ m.
  - Bottom-right: drone altitude $z_k(t)$ for all six drones; dashed line at $z_{asm} = 10$ m.
- **Assembly animation** (`s097_animation.gif`): 3D view at 20 fps; drones shown as coloured
  spheres with 60-step trailing paths; dashed hexagon target outline fixed at $z = 10$ m; mission
  phase and elapsed time displayed in the title.
- **Terminal summary**: consensus convergence time, impedance fitting time, final assembly error
  (mm), minimum observed inter-drone separation.

---

## Extensions

1. **Alternative communication topologies**: replace the ring graph with an all-to-all or star
   topology and compare consensus convergence time; for the star topology drone 0 acts as the
   hub and the others only communicate through it — analyse how hub failure degrades assembly
   accuracy.
2. **Piece shape mismatch**: introduce a ±2° manufacturing error in each triangular piece's angle;
   augment the impedance controller with a torque channel ($K_{imp,\psi}$) to correct in-plane
   orientation, and measure the maximum gap after fitting relative to the ideal case.
3. **Wind disturbance during fitting**: apply a horizontal gust $\mathbf{w} = [w_x, 0, 0]$ with
   $w_x \sim \mathcal{N}(0, 0.5)$ m/s² during the impedance phase; add a disturbance observer
   that estimates and cancels $\mathbf{w}$ from position residuals; compare fitting error with
   and without the observer.
4. **Scalability to $N = 12$ (dodecagon)**: extend the hexagon to a 12-piece puzzle; study how
   the spectral gap of the Laplacian scales with $N$ and quantify the increase in consensus
   convergence time; compare ring vs grid communication graphs.
5. **Online reassignment**: if one drone fails mid-consensus, redistribute the remaining five
   drones to a pentagon configuration using the Hungarian algorithm on position costs; measure
   reassignment overhead and final assembly quality.
6. **Physical contact modelling**: replace the zero-contact approximation with a spring-damper
   contact force between adjacent piece edges ($k_c = 50$ N/m, $b_c = 5$ N s/m); simulate the
   edge-sliding behaviour and verify that impedance gains keep contact forces below a structural
   limit of 2 N.

---

## Related Scenarios

- Prerequisites: [S085 Swarm Light Painting](S085_swarm_light_painting.md), [S088 Formation Morphing](S088_formation_morphing.md)
- Follow-ups: [S098 Aerial Calligraphy](S098_aerial_calligraphy.md)
- Algorithmic cross-reference: [S005 Formation Keeping](../01_pursuit_evasion/S005_formation_keeping.md) (multi-drone formation PID), [S020 Cooperative Interception](../01_pursuit_evasion/S020_cooperative_interception.md) (distributed multi-agent consensus), [S066 Cooperative Crane](../04_industrial_agriculture/S066_cooperative_crane.md) (compliant load manipulation)
