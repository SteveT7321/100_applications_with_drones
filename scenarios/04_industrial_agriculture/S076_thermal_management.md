# S076 Extreme Heat Cooling Strategy

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: First-Order Motor Thermal Model + Predictive Throttle Scheduling | **Dimension**: 2D

---

## Problem Definition

**Setup**: A single inspection drone must survey a $100 \times 100$ m agricultural field using a
boustrophedon (lawnmower) pattern in an extreme-heat environment where the ambient temperature is
$T_{amb} = 45°C$. Motor temperature evolves according to a first-order thermal model driven by
speed-dependent electrical power dissipation. If motor temperature exceeds $T_{max} = 85°C$, the
drone risks thermal shutdown. It may respond by reducing cruise speed (lowering motor power) or
pausing in a shaded hover for $T_{pause} = 30$ s to allow forced cooling. The nominal inspection
speed is $v_0 = 1.0$ m/s.

**Roles**:
- **Drone**: single UAV executing a pre-planned lawnmower path across a $100 \times 100$ m field;
  strip width $d = 5$ m; subject to first-order motor thermal dynamics at every timestep.
- **Thermal environment**: constant ambient temperature $T_{amb} = 45°C$; no shade during cruise
  segments; cooling pauses modelled as full shaded hover with an accelerated cooling time constant.

**Objective**: Complete the full lawnmower inspection while keeping $T_{motor}(t) \leq T_{max}$ at
all times and minimising total mission time $T_{mission}$.

**Comparison strategies**:
1. **No thermal management** — fly at constant $v = v_0 = 1.0$ m/s with no speed adjustment;
   temperature allowed to exceed $T_{max}$ (baseline showing the need for management).
2. **Reactive throttle** — monitor $T_{motor}$ each timestep; reduce speed to $v_{safe}(T_m)$ when
   $T_m > T_{warn} = 80°C$; trigger a cooling pause when $T_m \geq T_{max}$.
3. **Predictive scheduled pauses** — pre-compute, before flight, the maximum continuous cruise
   distance possible before $T_m$ reaches $T_{warn}$; insert mandatory cooling pauses at fixed
   waypoints to prevent the temperature from ever entering the warning zone.

---

## Mathematical Model

### First-Order Motor Thermal Model

Motor temperature $T_m(t)$ obeys a first-order linear ODE driven by the balance between power
dissipated in the motor windings and heat lost to the ambient through the motor casing:

$$\tau \frac{dT_m}{dt} = R_{th} \cdot P_{motor}(v) - \bigl(T_m - T_{amb}\bigr)$$

where:

- $\tau = C_{th} \cdot R_{th}$ — thermal time constant (s)
- $R_{th}$ — thermal resistance (°C/W)
- $C_{th}$ — thermal capacitance (J/°C)
- $T_{amb}$ — ambient temperature (°C)

### Speed-Dependent Motor Power

Total electrical power consumed by the motors as a function of horizontal cruise speed $v$:

$$P_{motor}(v) = P_{hover} + k_v \cdot v^2$$

where $P_{hover}$ is the hover power (minimum load to maintain altitude) and $k_v$ is the
aerodynamic drag power coefficient. Higher speed increases both thrust and induced losses.

### Steady-State Temperature Rise

As $t \to \infty$ at constant speed $v$, the motor temperature converges to:

$$T_{m,\infty}(v) = T_{amb} + R_{th} \cdot P_{motor}(v) = T_{amb} + \Delta T_{ss}(v)$$

The steady-state temperature rise is therefore:

$$\Delta T_{ss}(v) = R_{th} \bigl(P_{hover} + k_v v^2\bigr)$$

### Safe Speed Limit

The maximum cruise speed at which the motor can operate indefinitely without exceeding $T_{max}$
is found by setting $T_{m,\infty}(v_{safe}) = T_{max}$:

$$v_{safe} = \sqrt{\frac{T_{max} - T_{amb}}{R_{th} \cdot k_v} - \frac{P_{hover}}{k_v}}$$

For the reactive controller, the instantaneous safe speed given the current motor temperature
$T_m$ is:

$$v_{safe}(T_m) = \sqrt{\max\!\left(0,\; \frac{T_{max} - T_m}{R_{th} \cdot k_v} - \frac{P_{hover}}{k_v}\right)}$$

### Cooling Pause Temperature Decay

During a shaded hover pause the drone operates at reduced power; the motor cools exponentially:

$$T_m(t) = T_{amb} + \bigl(T_{m0} - T_{amb}\bigr) \exp\!\left(-\frac{t}{\tau_{cool}}\right)$$

where $T_{m0}$ is the motor temperature at the start of the pause and $\tau_{cool} < \tau$ is the
(shorter) cooling time constant reflecting enhanced convective cooling at rest.

### Transient Temperature During Cruise

Starting from initial temperature $T_{m0}$ and flying at constant speed $v$, the closed-form
solution to the first-order ODE is:

$$T_m(t) = T_{m,\infty}(v) + \bigl(T_{m0} - T_{m,\infty}(v)\bigr) \exp\!\left(-\frac{t}{\tau}\right)$$

This expression is used by the predictive scheduler to compute exactly when $T_m$ will reach
$T_{warn}$ and to insert a cooling pause before that instant.

### Mission Time Penalty

Total mission time decomposes into cruise time, turn time, and pause time:

$$T_{mission} = \frac{L_{total}}{v_{eff}} + N_{turn} \cdot t_{turn} + N_{pause} \cdot T_{pause}$$

where $L_{total}$ is the total path length, $v_{eff}$ is the effective (possibly reduced) cruise
speed averaged over all segments, $N_{turn}$ is the number of 180° heading reversals, and
$N_{pause}$ is the number of cooling pauses. The mission time penalty relative to the unconstrained
baseline is:

$$\Delta T_{mission} = N_{pause} \cdot T_{pause} + \sum_{k} \frac{\delta L_k}{v_{safe}^{(k)}} - \frac{\delta L_k}{v_0}$$

where the sum runs over all throttled segments of length $\delta L_k$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation

# ── Field and path parameters ─────────────────────────────────────────────────
FIELD_W         = 100.0     # m  field width (cross-track)
FIELD_H         = 100.0     # m  field length (along-track)
STRIP_D         = 5.0       # m  strip spacing (centre-to-centre)
INSPECTION_SPEED = 1.0      # m/s  nominal cruise speed (domain constant)

# ── Thermal model parameters ──────────────────────────────────────────────────
T_AMB           = 45.0      # °C  ambient temperature
T_MAX           = 85.0      # °C  motor temperature hard limit
T_WARN          = 80.0      # °C  warning threshold for reactive throttle
T_PAUSE         = 30.0      # s   forced cooling pause duration
T_INIT          = T_AMB     # °C  motor temperature at mission start
R_TH            = 0.8       # °C/W  thermal resistance
C_TH            = 50.0      # J/°C  thermal capacitance
TAU             = C_TH * R_TH  # s  thermal time constant (40 s)
TAU_COOL        = 20.0      # s  cooling time constant during shade pause

# ── Motor power model ─────────────────────────────────────────────────────────
P_HOVER         = 30.0      # W  hover power (v = 0)
K_V             = 15.0      # W·s²/m²  speed-dependent drag power coefficient
# P_motor(v) = P_HOVER + K_V * v**2

# ── Turn parameters ────────────────────────────────────────────────────────────
T_TURN          = 3.0       # s  time to execute one 180° heading reversal
P_TURN          = P_HOVER   # W  motor power during turn (hovering speed)

# ── Simulation timestep ────────────────────────────────────────────────────────
DT              = 0.1       # s  integration timestep


# ─────────────────────────────────────────────────────────────────────────────
# Helper functions
# ─────────────────────────────────────────────────────────────────────────────

def p_motor(v):
    """Motor power as a function of cruise speed (W)."""
    return P_HOVER + K_V * v**2


def t_ss(v):
    """Steady-state motor temperature at cruise speed v (°C)."""
    return T_AMB + R_TH * p_motor(v)


def v_safe_from_temp(t_m):
    """Maximum safe cruise speed given current motor temperature (m/s)."""
    val = (T_MAX - t_m) / (R_TH * K_V) - P_HOVER / K_V
    return float(np.sqrt(max(0.0, val)))


def thermal_step_cruise(t_m, v, dt):
    """Euler-integrate motor temperature during one cruise timestep."""
    t_inf = t_ss(v)
    dt_dt = (t_inf - t_m) / TAU
    return t_m + dt_dt * dt


def thermal_step_cool(t_m, dt):
    """Euler-integrate motor temperature during one shade-pause timestep."""
    dt_dt = (T_AMB - t_m) / TAU_COOL
    return t_m + dt_dt * dt


def time_to_warn(t_m0, v):
    """
    Closed-form time until T_m reaches T_WARN flying at constant speed v.
    Returns np.inf if T_ss(v) <= T_WARN (safe to cruise indefinitely).
    """
    t_inf = t_ss(v)
    if t_inf <= T_WARN:
        return np.inf
    if t_m0 >= T_WARN:
        return 0.0
    return -TAU * np.log((T_WARN - t_inf) / (t_m0 - t_inf))


# ─────────────────────────────────────────────────────────────────────────────
# Lawnmower waypoint generation
# ─────────────────────────────────────────────────────────────────────────────

def generate_lawnmower(field_w, field_h, strip_d, x0=0.0, y0=0.0):
    """
    Return list of (x, y) waypoints for boustrophedon scan and strip count.
    Strips run along x (along-track); cross-track steps along y.
    """
    n_strips = int(np.ceil(field_w / strip_d))
    waypoints = []
    for i in range(n_strips):
        y_c = y0 + (i + 0.5) * strip_d
        if i % 2 == 0:
            waypoints.append((x0,          y_c))
            waypoints.append((x0 + field_h, y_c))
        else:
            waypoints.append((x0 + field_h, y_c))
            waypoints.append((x0,           y_c))
    return waypoints, n_strips


# ─────────────────────────────────────────────────────────────────────────────
# Predictive pause scheduler
# ─────────────────────────────────────────────────────────────────────────────

def build_predictive_schedule(waypoints, v):
    """
    Pre-compute pause locations along the lawnmower path.
    Returns a list of distances-from-start at which cooling pauses are inserted.
    """
    pause_distances = []
    t_m = T_INIT
    dist_acc = 0.0  # accumulated path distance

    for seg_idx in range(len(waypoints) - 1):
        p0 = np.array(waypoints[seg_idx])
        p1 = np.array(waypoints[seg_idx + 1])
        seg_len = np.linalg.norm(p1 - p0)
        seg_dist_done = 0.0

        while seg_dist_done < seg_len:
            # How far can we cruise before hitting T_WARN?
            t_warn = time_to_warn(t_m, v)
            dist_before_warn = v * t_warn if not np.isinf(t_warn) else np.inf
            remaining = seg_len - seg_dist_done

            if dist_before_warn > remaining:
                # Safe to finish this segment without pausing
                t_cruise = remaining / v
                for _ in range(int(t_cruise / DT)):
                    t_m = thermal_step_cruise(t_m, v, DT)
                seg_dist_done = seg_len
                dist_acc += remaining
            else:
                # Must pause at dist_before_warn into this segment
                t_cruise = dist_before_warn / v
                for _ in range(int(t_cruise / DT)):
                    t_m = thermal_step_cruise(t_m, v, DT)
                seg_dist_done += dist_before_warn
                dist_acc += dist_before_warn
                pause_distances.append(dist_acc)
                # Cool down for T_PAUSE seconds
                for _ in range(int(T_PAUSE / DT)):
                    t_m = thermal_step_cool(t_m, DT)

        # Turn between strips (modelled as hover at P_TURN for T_TURN seconds)
        if seg_idx < len(waypoints) - 2 and (seg_idx % 2 == 1):
            for _ in range(int(T_TURN / DT)):
                t_m = thermal_step_cruise(t_m, v, DT)

    return pause_distances


# ─────────────────────────────────────────────────────────────────────────────
# Simulation engine
# ─────────────────────────────────────────────────────────────────────────────

def run_simulation(strategy='none'):
    """
    Simulate the lawnmower inspection mission under one of three strategies:
      'none'       – constant speed, no thermal management
      'reactive'   – reduce speed at T_warn, pause at T_max
      'predictive' – pre-scheduled pauses, constant nominal speed

    Returns a trajectory log dict and a mission outcome dict.
    """
    waypoints, n_strips = generate_lawnmower(FIELD_W, FIELD_H, STRIP_D)
    v_nominal = INSPECTION_SPEED

    if strategy == 'predictive':
        pause_dists = build_predictive_schedule(waypoints, v_nominal)
        pause_dist_set = set(round(d, 2) for d in pause_dists)
    else:
        pause_dist_set = set()

    # ── Logging ───────────────────────────────────────────────────────────────
    log = dict(t=[], x=[], y=[], t_motor=[], speed=[], event=[])

    t_m   = T_INIT
    clock = 0.0
    dist_total = 0.0
    n_pauses   = 0
    t_exceeded = 0.0  # cumulative time above T_MAX (for 'none' strategy)
    pos = np.array(waypoints[0], dtype=float)

    def record(x, y, tm, v, ev=''):
        log['t'].append(clock)
        log['x'].append(x)
        log['y'].append(y)
        log['t_motor'].append(tm)
        log['speed'].append(v)
        log['event'].append(ev)

    record(*pos, t_m, 0.0, 'start')

    for seg_idx in range(len(waypoints) - 1):
        p_start = np.array(waypoints[seg_idx])
        p_end   = np.array(waypoints[seg_idx + 1])
        seg_len = np.linalg.norm(p_end - p_start)
        seg_unit = (p_end - p_start) / seg_len
        seg_done = 0.0

        while seg_done < seg_len - 1e-9:
            # ── Determine current speed ───────────────────────────────────────
            if strategy == 'none':
                v = v_nominal
            elif strategy == 'reactive':
                if t_m >= T_MAX:
                    # Trigger reactive pause
                    n_pauses += 1
                    record(*pos, t_m, 0.0, 'pause_start')
                    for _ in range(int(T_PAUSE / DT)):
                        t_m = thermal_step_cool(t_m, DT)
                        clock += DT
                    record(*pos, t_m, 0.0, 'pause_end')
                    v = v_nominal
                elif t_m >= T_WARN:
                    v = max(0.05, v_safe_from_temp(t_m))
                else:
                    v = v_nominal
            else:  # predictive
                # Check if a pre-scheduled pause is due at this distance
                dist_key = round(dist_total, 2)
                if any(abs(dist_total - pd) < v_nominal * DT * 1.5
                       for pd in pause_dists):
                    # Only trigger once per pause location
                    closest = min(pause_dists, key=lambda d: abs(d - dist_total))
                    if abs(dist_total - closest) < v_nominal * DT * 1.5:
                        n_pauses += 1
                        pause_dists_copy = [pd for pd in pause_dists
                                            if pd != closest]
                        pause_dists[:] = pause_dists_copy
                        record(*pos, t_m, 0.0, 'pause_start')
                        for _ in range(int(T_PAUSE / DT)):
                            t_m = thermal_step_cool(t_m, DT)
                            clock += DT
                        record(*pos, t_m, 0.0, 'pause_end')
                v = v_nominal

            # ── Advance one timestep along the segment ────────────────────────
            step = min(v * DT, seg_len - seg_done)
            pos = p_start + seg_unit * (seg_done + step)
            seg_done   += step
            dist_total += step
            clock      += step / v if v > 1e-6 else DT

            t_m = thermal_step_cruise(t_m, v, DT)

            if t_m > T_MAX:
                t_exceeded += DT

            record(*pos, t_m, v)

        # ── Turn between strips ───────────────────────────────────────────────
        is_turn = (seg_idx % 2 == 1) and (seg_idx < len(waypoints) - 2)
        if is_turn:
            record(*pos, t_m, 0.0, 'turn_start')
            for _ in range(int(T_TURN / DT)):
                t_m = thermal_step_cruise(t_m, v_nominal, DT)
                clock += DT
            record(*pos, t_m, 0.0, 'turn_end')

    record(*pos, t_m, 0.0, 'mission_end')

    # ── Convert to arrays ─────────────────────────────────────────────────────
    for k in log:
        log[k] = np.array(log[k]) if k != 'event' else log[k]

    outcome = {
        'strategy':      strategy,
        'mission_time_s': float(clock),
        'max_t_motor':   float(np.max(log['t_motor'])),
        'n_pauses':      n_pauses,
        't_exceeded_s':  float(t_exceeded),
        'n_strips':      n_strips,
        'path_length_m': float(dist_total),
    }
    return log, outcome


# ─────────────────────────────────────────────────────────────────────────────
# Plotting
# ─────────────────────────────────────────────────────────────────────────────

def plot_results(logs, outcomes):
    """Generate comparison figures for all three strategies."""
    colours = {'none': 'tomato', 'reactive': 'steelblue', 'predictive': 'seagreen'}
    labels  = {'none': 'No management', 'reactive': 'Reactive throttle',
               'predictive': 'Predictive pauses'}
    strategies = ['none', 'reactive', 'predictive']

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('S076 Extreme Heat Cooling Strategy — Motor Thermal Management', fontsize=13)

    # ── Plot 1: Motor temperature traces ─────────────────────────────────────
    ax = axes[0, 0]
    for s in strategies:
        log = logs[s]
        ax.plot(log['t'], log['t_motor'], color=colours[s], linewidth=1.5,
                label=labels[s])
    ax.axhline(T_MAX,  color='black', linestyle='--', linewidth=1.2,
               label=f'$T_{{max}}$ = {T_MAX}°C')
    ax.axhline(T_WARN, color='orange', linestyle=':', linewidth=1.2,
               label=f'$T_{{warn}}$ = {T_WARN}°C')
    ax.axhline(T_AMB,  color='gray',  linestyle=':', linewidth=1.0,
               label=f'$T_{{amb}}$ = {T_AMB}°C')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Motor temperature (°C)')
    ax.set_title('Motor Temperature vs Time')
    ax.legend(fontsize=8)

    # ── Plot 2: Speed profiles ────────────────────────────────────────────────
    ax = axes[0, 1]
    for s in ['reactive', 'predictive']:
        log = logs[s]
        ax.plot(log['t'], log['speed'], color=colours[s], linewidth=1.2,
                label=labels[s], alpha=0.85)
    ax.axhline(INSPECTION_SPEED, color='gray', linestyle='--', linewidth=1.0,
               label=f'Nominal $v_0$ = {INSPECTION_SPEED} m/s')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cruise speed (m/s)')
    ax.set_title('Speed Profile (managed strategies)')
    ax.legend(fontsize=8)

    # ── Plot 3: 2D flight path with pause markers ─────────────────────────────
    ax = axes[0, 2]
    for s in strategies:
        log = logs[s]
        ax.plot(log['x'], log['y'], color=colours[s], linewidth=0.9,
                alpha=0.7, label=labels[s])
        # Mark pause locations
        ev = np.array(logs[s]['event'])
        pause_idx = [i for i, e in enumerate(ev) if e == 'pause_start']
        if pause_idx:
            ax.scatter(log['x'][pause_idx], log['y'][pause_idx],
                       color=colours[s], marker='x', s=60, zorder=5)
    # Field boundary
    rect = mpatches.Rectangle((0, 0), FIELD_H, FIELD_W,
                               linewidth=1.5, edgecolor='black', facecolor='wheat',
                               alpha=0.3, label='Field boundary')
    ax.add_patch(rect)
    ax.set_xlim(-5, FIELD_H + 5)
    ax.set_ylim(-5, FIELD_W + 5)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Flight Path (× = cooling pause)')
    ax.legend(fontsize=7)

    # ── Plot 4: Mission timeline (Gantt-style) ────────────────────────────────
    ax = axes[1, 0]
    y_pos = {'none': 2, 'reactive': 1, 'predictive': 0}
    colours_event = {'cruise': 0.7, 'pause': 0.1, 'turn': 0.4}
    for i, s in enumerate(strategies):
        log = logs[s]
        ev  = logs[s]['event']
        t_arr = log['t']
        in_pause = False
        t_p_start = None
        for j in range(len(ev)):
            if ev[j] == 'pause_start':
                in_pause = True
                t_p_start = t_arr[j]
            elif ev[j] == 'pause_end' and in_pause:
                ax.barh(y_pos[s], t_arr[j] - t_p_start, left=t_p_start,
                        height=0.6, color='steelblue', alpha=0.8)
                in_pause = False
        t_end = outcomes[s]['mission_time_s']
        ax.barh(y_pos[s], t_end, left=0, height=0.6,
                color=colours[s], alpha=0.35, label=labels[s])
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(['Predictive', 'Reactive', 'No mgmt'], fontsize=9)
    ax.set_xlabel('Mission time (s)')
    ax.set_title('Mission Timeline (blue bars = cooling pauses)')

    # ── Plot 5: Steady-state temperature vs speed curve ───────────────────────
    ax = axes[1, 1]
    v_range = np.linspace(0.0, 2.0, 200)
    t_ss_vals = T_AMB + R_TH * (P_HOVER + K_V * v_range**2)
    ax.plot(v_range, t_ss_vals, 'k-', linewidth=1.8, label='$T_{m,\\infty}(v)$')
    ax.axhline(T_MAX,  color='black',  linestyle='--', linewidth=1.2,
               label=f'$T_{{max}}$ = {T_MAX}°C')
    ax.axhline(T_WARN, color='orange', linestyle=':', linewidth=1.2,
               label=f'$T_{{warn}}$ = {T_WARN}°C')
    ax.axvline(INSPECTION_SPEED, color='tomato', linestyle='--', linewidth=1.0,
               label=f'$v_0$ = {INSPECTION_SPEED} m/s')
    # Safe speed line
    v_s = np.sqrt(max(0, (T_MAX - T_AMB) / (R_TH * K_V) - P_HOVER / K_V))
    ax.axvline(v_s, color='seagreen', linestyle='-.', linewidth=1.0,
               label=f'$v_{{safe}}$ = {v_s:.2f} m/s')
    ax.fill_between(v_range, T_MAX, t_ss_vals,
                    where=(t_ss_vals >= T_MAX), color='red', alpha=0.15,
                    label='Unsafe zone')
    ax.set_xlabel('Cruise speed $v$ (m/s)')
    ax.set_ylabel('Steady-state motor temperature (°C)')
    ax.set_title('$T_{m,\\infty}$ vs Speed')
    ax.legend(fontsize=8)

    # ── Plot 6: Mission metrics bar chart ─────────────────────────────────────
    ax = axes[1, 2]
    x = np.arange(len(strategies))
    w = 0.25
    mission_times = [outcomes[s]['mission_time_s'] / 60.0 for s in strategies]
    max_temps     = [outcomes[s]['max_t_motor']         for s in strategies]
    n_pauses_list = [outcomes[s]['n_pauses']            for s in strategies]

    ax2 = ax.twinx()
    b1 = ax.bar(x - w, mission_times,  w, color='steelblue',  alpha=0.8, label='Mission time (min)')
    b2 = ax.bar(x,     n_pauses_list,  w, color='seagreen',   alpha=0.8, label='Pause count')
    b3 = ax2.bar(x + w, max_temps,     w, color='tomato',     alpha=0.8, label='Max $T_m$ (°C)')

    ax.set_xticks(x)
    ax.set_xticklabels([labels[s] for s in strategies], fontsize=8)
    ax.set_ylabel('Mission time (min) / Pause count')
    ax2.set_ylabel('Max motor temperature (°C)')
    ax.set_title('Mission Metrics Comparison')

    lines = [b1, b2, b3]
    ax.legend(handles=lines, fontsize=8, loc='upper left')

    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s076_thermal_management/thermal_comparison.png',
                dpi=150, bbox_inches='tight')
    plt.show()


def animate_mission(log, strategy_name):
    """
    Animate the drone lawnmower path, colour-coding the trail by motor
    temperature (top-down 2D view).
    """
    x   = log['x']
    y   = log['y']
    tm  = log['t_motor']
    n   = len(x)
    step = max(1, n // 300)

    fig, (ax_map, ax_temp) = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle(f'S076 Mission Animation — {strategy_name}', fontsize=11)

    # Field background
    rect = mpatches.Rectangle((0, 0), FIELD_H, FIELD_W,
                               linewidth=1.5, edgecolor='black',
                               facecolor='wheat', alpha=0.3)
    ax_map.add_patch(rect)
    ax_map.set_xlim(-5, FIELD_H + 5)
    ax_map.set_ylim(-5, FIELD_W + 5)
    ax_map.set_aspect('equal')
    ax_map.set_xlabel('x (m)')
    ax_map.set_ylabel('y (m)')
    ax_map.set_title('Flight path (colour = $T_{motor}$)')

    trail = ax_map.scatter([], [], c=[], cmap='hot_r',
                           vmin=T_AMB, vmax=T_MAX + 5,
                           s=3, zorder=3)
    drone_dot, = ax_map.plot([], [], 'ko', markersize=7, zorder=5)
    time_text  = ax_map.text(2, FIELD_W + 1, '', fontsize=9)

    ax_temp.set_xlim(0, log['t'][-1])
    ax_temp.set_ylim(T_AMB - 2, T_MAX + 8)
    ax_temp.axhline(T_MAX,  color='black',  linestyle='--', linewidth=1.2,
                    label=f'$T_{{max}}$ = {T_MAX}°C')
    ax_temp.axhline(T_WARN, color='orange', linestyle=':',  linewidth=1.2,
                    label=f'$T_{{warn}}$ = {T_WARN}°C')
    ax_temp.set_xlabel('Time (s)')
    ax_temp.set_ylabel('Motor temperature (°C)')
    ax_temp.set_title('Motor temperature trace')
    ax_temp.legend(fontsize=8)
    temp_line, = ax_temp.plot([], [], 'r-', linewidth=1.5)
    temp_dot,  = ax_temp.plot([], [], 'ro', markersize=6)

    def init():
        trail.set_offsets(np.empty((0, 2)))
        trail.set_array(np.array([]))
        drone_dot.set_data([], [])
        temp_line.set_data([], [])
        temp_dot.set_data([], [])
        return trail, drone_dot, temp_line, temp_dot, time_text

    def update(frame):
        idx = min(frame * step, n - 1)
        xy  = np.column_stack([x[:idx+1], y[:idx+1]])
        trail.set_offsets(xy)
        trail.set_array(tm[:idx+1])
        drone_dot.set_data([x[idx]], [y[idx]])
        temp_line.set_data(log['t'][:idx+1], tm[:idx+1])
        temp_dot.set_data([log['t'][idx]], [tm[idx]])
        time_text.set_text(
            f't = {log["t"][idx]:.0f} s   '
            f'$T_m$ = {tm[idx]:.1f}°C'
        )
        return trail, drone_dot, temp_line, temp_dot, time_text

    anim = FuncAnimation(fig, update, frames=n // step,
                         init_func=init, blit=True, interval=40)
    fname = (f'outputs/04_industrial_agriculture/s076_thermal_management/'
             f'animation_{strategy_name.lower().replace(" ", "_")}.gif')
    anim.save(fname, writer='pillow', fps=25)
    plt.close()
    print(f'Animation saved: {fname}')


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import os
    os.makedirs('outputs/04_industrial_agriculture/s076_thermal_management', exist_ok=True)

    logs, outcomes = {}, {}
    for strategy in ('none', 'reactive', 'predictive'):
        print(f'Running strategy: {strategy} ...')
        logs[strategy], outcomes[strategy] = run_simulation(strategy=strategy)
        oc = outcomes[strategy]
        print(f"  Mission time : {oc['mission_time_s']:.0f} s  "
              f"({oc['mission_time_s']/60:.1f} min)")
        print(f"  Max T_motor  : {oc['max_t_motor']:.1f} °C")
        print(f"  Pause count  : {oc['n_pauses']}")
        print(f"  Time > T_max : {oc['t_exceeded_s']:.1f} s")

    plot_results(logs, outcomes)
    animate_mission(logs['reactive'],   'Reactive Throttle')
    animate_mission(logs['predictive'], 'Predictive Pauses')

    print('Done. Outputs saved to outputs/04_industrial_agriculture/s076_thermal_management/')
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Field dimensions | — | 100 × 100 m |
| Strip spacing | $d$ | 5 m |
| Nominal inspection speed | $v_0$ | 1.0 m/s |
| Ambient temperature | $T_{amb}$ | 45 °C |
| Motor temperature limit | $T_{max}$ | 85 °C |
| Warning threshold | $T_{warn}$ | 80 °C |
| Cooling pause duration | $T_{pause}$ | 30 s |
| Thermal resistance | $R_{th}$ | 0.8 °C/W |
| Thermal capacitance | $C_{th}$ | 50 J/°C |
| Thermal time constant (heating) | $\tau = C_{th} R_{th}$ | 40 s |
| Thermal time constant (cooling) | $\tau_{cool}$ | 20 s |
| Hover power | $P_{hover}$ | 30 W |
| Speed-drag power coefficient | $k_v$ | 15 W·s²/m² |
| Motor power at $v_0$ | $P_{motor}(v_0)$ | 45 W |
| Steady-state $T_m$ at $v_0$ | $T_{m,\infty}(v_0)$ | 81 °C |
| Safe cruise speed (unlimited) | $v_{safe}$ | ≈ 0.91 m/s |
| Turn time per reversal | $t_{turn}$ | 3 s |
| Simulation timestep | $\Delta t$ | 0.1 s |
| Number of strips | $N$ | 20 |
| Total path length | $L$ | ≈ 2 095 m |

---

## Expected Output

- **Motor temperature trace**: time-series plot of $T_m(t)$ for all three strategies on the same
  axes; $T_{max}$ and $T_{warn}$ shown as horizontal reference lines; the no-management strategy
  visibly breaches $T_{max}$ while the two managed strategies remain below; cooling dips visible as
  exponential valleys during pauses.
- **Speed profile**: cruise speed $v(t)$ for the reactive and predictive strategies; reactive
  strategy shows brief speed reductions near $T_{warn}$; predictive strategy maintains constant
  $v_0$ between scheduled pauses.
- **2D flight path with pause markers**: top-down view of the 100 × 100 m field with the lawnmower
  path; cooling pause locations marked with × symbols; colour or line style distinguishes strategies.
- **Mission timeline (Gantt chart)**: horizontal bar chart with one row per strategy; blue
  sub-bars indicate cooling pause intervals; total bar length shows mission time; clearly
  illustrates the time cost of each approach.
- **Steady-state temperature vs speed curve**: $T_{m,\infty}(v)$ plotted against speed; safe and
  warning thresholds marked; nominal speed $v_0$ shown as a vertical line; the curve illustrates
  why $v_0 = 1.0$ m/s is marginally above the safe continuous-cruise speed.
- **Mission metrics bar chart**: grouped bars comparing mission time (min), pause count, and peak
  motor temperature for all three strategies; twin y-axis for temperature.
- **Animation GIF** (reactive and predictive): top-down drone trail colour-coded by $T_m$ (hot
  colourmap); simultaneous temperature trace panel; cooling pauses visible as stationary drone with
  falling temperature.

**Expected metric targets**:

| Metric | No management | Reactive | Predictive |
|--------|--------------|----------|------------|
| Max $T_{motor}$ | $> 85°C$ (violation) | $\leq 85°C$ | $\leq 80°C$ |
| Pause count | 0 | 3–6 | 4–7 |
| Mission time | ≈ 34 min | ≈ 37–39 min | ≈ 36–38 min |
| Time above $T_{max}$ | $> 0$ s | 0 s | 0 s |

---

## Extensions

1. **Adaptive ambient temperature**: replace the constant $T_{amb}$ with a diurnal profile
   (sinusoidal, peaking at solar noon); re-optimise the pause schedule to exploit cooler morning
   windows and minimise mid-day penalties.
2. **Multi-motor thermal heterogeneity**: model four independent motor temperatures with slightly
   different $R_{th}$ and $C_{th}$ values; trigger the speed-reduction policy on the hottest
   motor; compare with a symmetric single-motor approximation.
3. **Battery–thermal joint optimisation**: co-optimise the trade-off between flying faster (shorter
   mission but more battery energy and more heat) versus flying slower (longer mission, less heat,
   lower total energy consumed); use a Pareto front across mission time and peak temperature.
4. **MPC predictive controller**: replace the heuristic scheduled-pause policy with a
   Model Predictive Controller that minimises mission time subject to the thermal constraint over a
   receding horizon; compare computational cost versus the open-loop schedule.
5. **Shade-aware path replanning**: introduce discrete shade patches on the field map where
   $T_{amb,local} = 25°C$; replan the lawnmower path to route cooling pauses over shade regions,
   shortening the required pause duration and improving total mission time.

---

## Related Scenarios

- Prerequisites: [S063 Solar Panel Fault Detection](S063_solar_panel_fault_detection.md)
  (industrial inspection lawnmower structure), [S067 Spray Overlap Optimisation](S067_spray_overlap_optimization.md)
  (field-coverage path planning in Domain 4)
- Algorithmic cross-reference: [S079 Night Flight Navigation](S079_night_flight_navigation.md)
  (environment-constrained mission planning), [S048 Full-Area Coverage Scan](../../03_environmental_sar/S048_lawnmower.md)
  (boustrophedon path geometry reference)
- Follow-ups: extend the first-order thermal model to a two-mass RC circuit (winding + casing)
  for higher-fidelity temperature prediction; integrate with a real ESC telemetry stream for
  closed-loop hardware-in-the-loop validation.
