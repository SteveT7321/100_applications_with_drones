# S026 Cooperative Heavy Lift — Simulation Outputs

## Scenario

Four quadrotor drones cooperatively lift and transport a heavy load (0.8 kg) via cable suspension. The load mass exceeds the single-drone payload limit of 0.1 kg. Two tension-allocation strategies are compared throughout a full lift-translate-descend mission.

## Mission Profile

| Phase | Time | Description |
|-------|------|-------------|
| Liftoff | 0–5 s | Load raised from ground (0 m) to delivery altitude (4 m) |
| Cruise | 5–15 s | Horizontal translation from (0, 0, 4) to (3, 2, 4) m |
| Descent | 15–20 s | Coordinated descent to target ground position (3, 2, 0) m |

## Output Files

| File | Description |
|------|-------------|
| `trajectory_3d.png` | 3D paths of load centroid and all four drones; solid=QP, dashed=Pseudo-Inverse |
| `cable_tensions.png` | Cable tensions f1–f4 vs time for both strategies; f_min threshold shown |
| `slack_margin.png` | Slack margin δ = min_i(f_i) − f_min over time; negative = slack violation |
| `tension_comparison.png` | Bar chart of mean/peak tension per drone: QP vs Pseudo-Inverse |
| `position_error.png` | Load tracking error ‖q − q_ref‖ vs time for both strategies |
| `formation_snapshots.png` | Top-down view of drone cross formation at t=0 s and t=10 s |

## Key Results

### QP (Anti-Slack) Strategy
- Slack violations: **0 steps (0.0%)** — all cables remain taut throughout mission
- Min cable tension: 0.0500 N (exactly f_min = 0.05 N, constraint active)
- Max cable tension: 23.54 N (during acceleration phases)
- Mean tensions per drone: 10.45, 10.42, 10.28, 10.30 N (well-balanced)
- RMS load position error: 0.324 m
- Final load position: (3.004, 1.981, 0.085) m — near target (3, 2, 0)

### Pseudo-Inverse Strategy
- Slack violations: **618 steps (30.9%)** — cables go slack during maneuvers
- Min cable tension: 0.0 N (cables fully slack at times)
- Max cable tension: 23.54 N
- Min slack margin: −0.050 N (negative → slack event)
- RMS load position error: 0.294 m

### Comparison

| Metric | QP | Pseudo-Inverse |
|--------|----|----------------|
| Slack violations | 0 (0.0%) | 618 (30.9%) |
| Min tension (N) | 0.050 | 0.000 |
| CoV (tension uniformity) | 0.713 | 0.814 |
| RMS position error (m) | 0.324 | 0.294 |

The QP strategy successfully eliminates all slack events at a small cost in RMS tracking error compared to the unconstrained pseudo-inverse solution. The pseudo-inverse produces a lower-norm solution mathematically but allows physically infeasible negative tensions which manifest as slack cable violations.

## Configuration

```
N_DRONES     = 4
MASS_LOAD    = 0.8 kg
MASS_DRONE   = 0.5 kg
CABLE_LENGTH = 0.5 m
FORMATION_S  = 0.6 m  (horizontal separation)
FORMATION_H  = 0.40 m (vertical cable projection)
F_MIN        = 0.05 N (anti-slack minimum tension)
KP / KD      = 5.0 / 2.5
DT           = 0.01 s
T_TOTAL      = 20.0 s
```
