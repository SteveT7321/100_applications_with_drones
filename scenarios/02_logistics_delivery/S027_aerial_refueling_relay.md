# S027 Aerial Refueling Relay

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: A delivery drone (Receiver) departs from a base station carrying a package destined for a distant target. Its battery is insufficient to complete the round trip. A tanker drone with a full charge launches from the same base, flies ahead, and holds a loiter orbit at the refueling rendezvous point. When the Receiver arrives and achieves soft-docking (zero relative velocity, relative position < tolerance), a power transfer is simulated. After transfer the Receiver resumes its delivery flight; the Tanker returns to base.

**Roles**:
- **Receiver (R)**: delivery drone, battery drains during flight, requests refueling when predicted remaining range falls below threshold.
- **Tanker (T)**: dedicated support drone, full battery, flies to rendezvous orbit, holds station until R arrives, executes docking approach.

**Objective**: Receiver completes the full outbound delivery and inbound return without a forced landing. Minimise total mission time subject to safe docking (relative position error $< \epsilon_p$, relative velocity error $< \epsilon_v$) and minimum transferred energy $\Delta E_{req}$.

---

## Mathematical Model

### Battery and Range Model

Battery state-of-charge (SoC) depletes as a function of speed and hover/flight regime:

$$\dot{E}_i = -k_f \, \|\mathbf{v}_i\|^2 - k_h$$

where $k_f$ is the flight power coefficient and $k_h$ is the baseline hover drain. Predicted remaining range at time $t$:

$$d_{rem}(t) = \frac{E_R(t)}{k_f \, v_R^2 + k_h} \cdot v_R$$

Refueling is triggered when:

$$d_{rem}(t) < d_{target}(t) + d_{base} + d_{margin}$$

where $d_{target}$ is distance to delivery target and $d_{base}$ is distance from target to home base.

### Rendezvous Point Selection

The rendezvous waypoint $\mathbf{w}_{rvz}$ is placed on the straight-line route from base $\mathbf{p}_0$ to target $\mathbf{p}_{T}$ at fraction $\alpha \in (0,1)$:

$$\mathbf{w}_{rvz} = (1 - \alpha)\,\mathbf{p}_0 + \alpha\,\mathbf{p}_{T}$$

Optimal $\alpha$ balances the Tanker's travel cost against the Receiver's waiting time. With equal cruise speeds $V$, the earliest simultaneous arrival occurs when:

$$\alpha^* = \frac{1}{2}\left(1 + \frac{t_T^{dep} - t_R^{dep}}{t_{total}}\right)$$

In the standard configuration both depart at $t=0$, so $\alpha^* = 0.5$.

### Tanker Loiter Orbit

While waiting for the Receiver, the Tanker holds a horizontal circular orbit of radius $R_{orb}$ centred on $\mathbf{w}_{rvz}$:

$$\mathbf{p}_T(t) = \mathbf{w}_{rvz} + R_{orb}\begin{bmatrix}\cos(\omega_{orb}\,t + \phi_0)\\\sin(\omega_{orb}\,t + \phi_0)\\0\end{bmatrix}$$

$$\omega_{orb} = \frac{V_{T,orb}}{R_{orb}}$$

Battery consumption during loiter uses the hover + low-speed flight model with $\|\mathbf{v}_T\| = V_{T,orb}$.

### Relative Motion and Docking Control

Define the relative state between Receiver and Tanker:

$$\Delta\mathbf{r} = \mathbf{p}_R - \mathbf{p}_T, \qquad \Delta\mathbf{v} = \mathbf{v}_R - \mathbf{v}_T$$

The Receiver executes a two-phase docking approach:

**Phase 1 — Approach** ($\|\Delta\mathbf{r}\| > d_{dock}$): proportional-derivative intercept law

$$\mathbf{u}_R = -K_p\,\Delta\mathbf{r} - K_d\,\Delta\mathbf{v} + \mathbf{a}_T$$

where $\mathbf{a}_T$ is the Tanker's known acceleration (feedforward).

**Phase 2 — Soft Dock** ($\|\Delta\mathbf{r}\| \leq d_{dock}$): velocity matching with gentle final closure

$$\mathbf{u}_R = -K_{d2}\,\Delta\mathbf{v} - K_{p2}\,\Delta\mathbf{r}$$

Docking declared successful when:

$$\|\Delta\mathbf{r}\| < \epsilon_p \quad \text{and} \quad \|\Delta\mathbf{v}\| < \epsilon_v$$

### Energy Transfer

Instantaneous power transfer at dock (modelled as step):

$$E_R \leftarrow E_R + \Delta E_{xfer}, \qquad E_T \leftarrow E_T - \Delta E_{xfer} - \Delta E_{loss}$$

where $\Delta E_{loss} = \eta_{loss}\,\Delta E_{xfer}$ accounts for inductive transfer inefficiency $\eta_{loss}$.

### Post-Dock Return

After transfer the Receiver resumes direct flight to the delivery target:

$$\mathbf{v}_R^{cmd} = V_R \cdot \frac{\mathbf{p}_{target} - \mathbf{p}_R}{\|\mathbf{p}_{target} - \mathbf{p}_R\|}$$

The Tanker turns toward base with proportional heading control.

---

## Implementation

```python
# Key constants
V_CRUISE    = 5.0    # m/s cruise speed (both drones)
V_T_ORB     = 2.0    # m/s tanker loiter speed
R_ORB       = 3.0    # m  loiter radius
K_FLIGHT    = 0.3    # W·s²/m²  flight power coeff
K_HOVER     = 0.5    # W        baseline drain
E0_R        = 40.0   # J  Receiver initial battery
E0_T        = 80.0   # J  Tanker initial battery
E_XFER      = 30.0   # J  energy transferred at dock
ETA_LOSS    = 0.05   # transfer loss fraction
D_DOCK      = 0.4    # m  docking engagement radius
EPS_P       = 0.15   # m  position tolerance (success)
EPS_V       = 0.05   # m/s velocity tolerance (success)
KP, KD      = 2.0, 1.5   # approach gains
KP2, KD2    = 4.0, 2.5   # soft-dock gains
MARGIN      = 5.0    # m  range safety margin

def battery_drain(speed, dt):
    return (K_FLIGHT * speed**2 + K_HOVER) * dt

def range_remaining(energy, speed):
    power = K_FLIGHT * speed**2 + K_HOVER
    return (energy / power) * speed

def tanker_loiter(t, w_rvz, omega, phi0, R):
    angle = omega * t + phi0
    return w_rvz + R * np.array([np.cos(angle), np.sin(angle), 0.0])

def docking_control(dr, dv, a_tanker, phase):
    if phase == "approach":
        return -KP * dr - KD * dv + a_tanker
    else:  # soft_dock
        return -KP2 * dr - KD2 * dv

# Main simulation loop
phase = "fly_to_rvz"   # Receiver FSM state
docked = False

for step in range(MAX_STEPS):
    # --- Receiver FSM ---
    if phase == "fly_to_rvz":
        dir_r = (w_rvz - pos_R) / np.linalg.norm(w_rvz - pos_R)
        vel_R = V_CRUISE * dir_r
        if np.linalg.norm(pos_R - w_rvz) < 2.0 * R_ORB:
            phase = "docking"

    elif phase == "docking":
        dr = pos_R - pos_T
        dv = vel_R - vel_T
        dock_phase = "soft_dock" if np.linalg.norm(dr) < D_DOCK else "approach"
        acc_R = docking_control(dr, dv, acc_T, dock_phase)
        vel_R += acc_R * DT
        if np.linalg.norm(dr) < EPS_P and np.linalg.norm(dv) < EPS_V:
            # Energy transfer event
            E_R += E_XFER
            E_T -= E_XFER * (1 + ETA_LOSS)
            docked = True
            phase = "deliver"

    elif phase == "deliver":
        dir_r = (pos_target - pos_R) / np.linalg.norm(pos_target - pos_R)
        vel_R = V_CRUISE * dir_r

    # --- Tanker FSM ---
    if not docked and np.linalg.norm(pos_T - w_rvz) > 1.0:
        dir_t = (w_rvz - pos_T) / np.linalg.norm(w_rvz - pos_T)
        vel_T = V_CRUISE * dir_t
    elif not docked:
        # Hold loiter orbit
        pos_T = tanker_loiter(t, w_rvz, V_T_ORB / R_ORB, phi0, R_ORB)
    else:
        # Return to base
        dir_t = (pos_base - pos_T) / np.linalg.norm(pos_base - pos_T)
        vel_T = V_CRUISE * dir_t

    # Battery drain
    E_R -= battery_drain(np.linalg.norm(vel_R), DT)
    E_T -= battery_drain(np.linalg.norm(vel_T), DT)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Cruise speed $V$ | 5.0 m/s |
| Tanker loiter speed $V_{T,orb}$ | 2.0 m/s |
| Loiter radius $R_{orb}$ | 3.0 m |
| Flight power coefficient $k_f$ | 0.3 W·s²/m² |
| Hover baseline drain $k_h$ | 0.5 W |
| Receiver initial battery $E_{0,R}$ | 40 J |
| Tanker initial battery $E_{0,T}$ | 80 J |
| Energy transferred $\Delta E_{xfer}$ | 30 J |
| Transfer loss fraction $\eta_{loss}$ | 5 % |
| Docking engagement radius $d_{dock}$ | 0.4 m |
| Position dock tolerance $\epsilon_p$ | 0.15 m |
| Velocity dock tolerance $\epsilon_v$ | 0.05 m/s |
| Approach PD gains $(K_p, K_d)$ | (2.0, 1.5) |
| Soft-dock PD gains $(K_{p2}, K_{d2})$ | (4.0, 2.5) |
| Rendezvous fraction $\alpha$ | 0.5 (midpoint) |
| Base-to-target distance | 60.0 m |

---

## Expected Output

- 3D trajectory plot: Receiver (blue) and Tanker (orange) full paths, rendezvous point marked, docking segment highlighted in green
- Battery SoC vs time for both drones with vertical lines at: Tanker arrival at orbit, docking start, docking success, energy transfer event
- Relative distance $\|\Delta\mathbf{r}\|$ vs time: exponential convergence to $\epsilon_p$ during docking phase
- Relative velocity $\|\Delta\mathbf{v}\|$ vs time: convergence during soft-dock phase
- Tanker loiter orbit detail (XY top-down inset): spiral approach of Receiver into the orbit
- Comparison panel: mission success with refueling vs mission failure (forced landing) without refueling — battery trace and range deficit annotated

---

## Extensions

1. Optimal rendezvous fraction $\alpha^*$: sweep $\alpha \in [0.3, 0.7]$ and minimise total mission time; plot Pareto curve of dock wait time vs post-refuel detour cost
2. Wind disturbance during docking: add Dryden turbulence model (see S024) and test docking robustness; re-tune $K_p$, $K_d$ gains
3. Multi-hop refueling chain: add a second Tanker for ultra-long-range delivery (three-leg relay); generalise the FSM to $N$ tankers along the route
4. Partial transfer strategy: instead of full $\Delta E_{xfer}$, transfer only the minimum required energy and send Tanker on a secondary delivery mission
5. Moving refueling platform: Tanker does not loiter but flies a straight segment at constant speed; Receiver must intercept a moving target (couples with S023 moving landing pad logic)

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md), [S024](S024_wind_compensation.md)
- Follow-ups: [S028](S028_cargo_escort_formation.md), [S036](S036_last_mile_relay.md)
- Cross-domain reference: [S012](../01_pursuit_evasion/S012_relay_pursuit.md) — relay handoff state machine pattern
