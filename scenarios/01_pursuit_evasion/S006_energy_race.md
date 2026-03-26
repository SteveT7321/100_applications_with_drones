# S006 Energy Race

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: Pursuer and evader both carry limited batteries. Power consumption scales as $P = k \cdot v^2$. The pursuer must capture the evader before either battery runs dry.

**Question**: what pursuer speed minimises capture time while remaining within the energy budget?

**Roles**:
- **Pursuer**: pure pursuit at one of three constant speeds (3, 4, 5 m/s)
- **Evader**: constant straight escape at 3 m/s

---

## Mathematical Model

### Energy Consumption

$$E(t) = E_0 - \int_0^t k \cdot v(\tau)^2 \, d\tau$$

For constant speed, maximum flight time before battery depletes:

$$T_{max}(v) = \frac{E_0}{k \cdot v^2}$$

### Approximate Capture Time (Pure Pursuit)

$$T_{cap} \approx \frac{r_0}{v_P - v_E}$$

### Feasibility Condition

Capture is achievable only if $T_{cap} \leq T_{max}(v_P)$:

$$\frac{r_0}{v_P - v_E} \leq \frac{E_0}{k \cdot v_P^2}$$

---

## Implementation

```python
K_ENERGY   = 0.5    # W·s²/m²
E0         = 80.0   # J

def max_flight_time(v, e0=E0):
    return e0 / (K_ENERGY * v**2)

# Analytical check before simulation
for v_p in [3.0, 4.0, 5.0]:
    t_cap = r0 / (v_p - V_EVADER)
    t_max = max_flight_time(v_p)
    print(f"v={v_p}  T_cap={t_cap:.2f}s  T_max={t_max:.2f}s  "
          f"{'OK' if t_cap <= t_max else 'BATTERY OUT'}")

# In simulation: stop when energy <= 0
energy = E0
for step in ...:
    energy -= K_ENERGY * speed**2 * DT
    if energy <= 0:
        break  # pursuer runs out of battery
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Initial distance | 6 m |
| Pursuer speed options | 3.0, 4.0, 5.0 m/s |
| Evader speed | 3.0 m/s |
| Battery capacity (both) | 80 J |
| Energy coefficient k | 0.5 W·s²/m² |
| Capture radius | 0.15 m |

---

## Expected Output

- Bar chart: capture time vs pursuer speed (infeasible cases shown separately)
- Energy remaining vs time for each speed level
- Feasibility boundary: minimum energy required vs initial distance

---

## Extensions

1. Variable-speed pursuer: adapt speed based on remaining energy
2. Evader also optimises its energy — slower when far, sprint when close
3. Wind field: directional energy cost (tailwind vs headwind)

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md)
- Next: [S010](S010_asymmetric_speed.md), [S012](S012_relay_pursuit.md)
