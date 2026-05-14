# S022 3D Upgrade — Obstacle Avoidance Delivery

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Completed
**Based on**: [S022 original](../S022_obstacle_avoidance_delivery.md)

---

## What Changes in 3D

The original S022 uses RRT* in a 2D plane with infinite-height cylindrical obstacles. In this 3D upgrade, obstacles are finite-height cylinders — the drone can fly **over** short obstacles or **around** tall ones. The planner operates in full (x, y, z) space.

---

## Problem Definition

**Setup**: A delivery drone must navigate from start (0, 0, 2) m to goal (10, 10, 8) m through a field of finite-height cylindrical obstacles. Obstacle heights vary — some are shorter than the goal altitude, allowing overflight.

**Objective**: Find a collision-free, near-optimal path in 3D using RRT*, then track it with a PD controller.

---

## Mathematical Model

3D RRT* with Euclidean cost. Collision check tests both xy-distance to cylinder axis and z-height against cylinder top. Steering limited to `step_size = 0.5 m`.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Start | (0, 0, 2) m |
| Goal | (10, 10, 8) m |
| Step size | 0.5 m |
| Max iterations | 5000 |
| Goal tolerance | 0.5 m |

---

## Related Scenarios

- Original: [S022](../S022_obstacle_avoidance_delivery.md)
- See also: [S043 Confined Space](../../03_environmental_sar/S043_confined_space.md)
