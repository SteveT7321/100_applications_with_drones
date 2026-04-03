# S028 Cargo Escort Formation — Output README

## Simulation Summary

A cargo drone (Carrier) flies from `(-8, 0, 2)` to `(8, 0, 2)` m while three
escort drones maintain a protective triangle formation using virtual-structure
(leader-follower) control with carrier feed-forward velocity and pairwise
repulsion for collision avoidance.

## Key Results

| Metric | Value | Target / Constraint |
|---|---|---|
| RMS formation error (all escorts) | 0.0000 m | < 0.15 m ✓ |
| Escort-1 (Lead) RMS error | 0.0000 m | — |
| Escort-2 (Left) RMS error | 0.0000 m | — |
| Escort-3 (Right) RMS error | 0.0000 m | — |
| Min inter-drone separation | 4.0000 m | ≥ 1.0 m (d_safe) ✓ |
| Carrier final error to goal | 0.1994 m | < 0.2 m (arrival tol.) ✓ |
| Carrier arrival time | ~6.85 s | — |
| Simulation duration | 30.0 s | 30.0 s |

All three pass/fail criteria satisfied.

## Output Files

| File | Description |
|---|---|
| `trajectory_3d.png` | 3D trajectories of Carrier and all escorts with formation snapshots |
| `topdown_xy.png` | Top-down XY view showing triangle formation shape during transit |
| `formation_metrics.png` | Formation slot error per escort + min inter-drone separation vs time |
| `combined_metrics.png` | 4-panel summary: XY view, error, separation, carrier distance-to-goal |
| `formation_animation.gif` | Animated GIF of the full escort transit (15 fps, top-down) |

## Simulation Configuration

- **Timestep**: 1/48 s (~48 Hz), **Duration**: 30 s
- **Carrier speed**: 3.0 m/s, **Escort max speed**: 6.0 m/s
- **Formation**: N=3 triangle — Lead (+2,0,0), Left (-1.5,+2,0), Right (-1.5,-2,0) m body frame
- **Escort gain** K_PE = 3.0, **Repulsion gain** K_REP = 3.0, **d_safe** = 1.0 m
- **Arena**: [-10,10] x [-10,10] x [0,5] m

## Algorithm

1. Carrier uses a PD position controller clamped to 3.0 m/s toward the goal waypoint.
2. At each timestep the carrier heading angle ψ is computed from its velocity vector.
3. Each escort's slot position is computed by rotating the body-frame offset by R(ψ) and adding it to the carrier position.
4. Escort velocity = K_PE × slot_error + v_carrier (feed-forward), clamped to 6.0 m/s.
5. Pairwise repulsion impulses are added when escorts come within d_safe = 1.0 m.
