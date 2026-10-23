# S022 3D Upgrade — Obstacle Avoidance Delivery

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Completed
**Based on**: [S022 original](../S022_obstacle_avoidance_delivery.md)

---

## What Changes in 3D

The original S022 already samples (x, y, z), but it restricts flight to z ∈ [1, 5] m and puts start and goal at the same 2 m altitude, so in practice it can only route around obstacles. In this 3D upgrade the goal is elevated, and the obstacles are finite-height cylinders that the drone can fly **over** (short ones) or must fly **around** (ones taller than the flight ceiling). The planner operates in full (x, y, z) space.

---

## Problem Definition

**Setup**: A delivery drone must navigate from start (0, 0, 2) m to goal (10, 10, 8) m through a field of finite-height cylindrical obstacles. Obstacle heights vary — some are shorter than the goal altitude, allowing overflight.

**Objective**: Find a collision-free, near-optimal path in 3D using RRT*, then track it with a PD controller.

---

## Mathematical Model

3D RRT* with Euclidean cost. The collision check tests both the xy-distance to the cylinder axis and the z-height against the cylinder top; both are inflated by the drone radius. Steering is limited to `step_size = 0.5 m`. After each rewire, the new cost is propagated to the whole subtree of the rewired node.

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

- Original: [S022 Obstacle Avoidance Delivery](../S022_obstacle_avoidance_delivery.md)
- See also: [S043 Confined Space Exploration](../../03_environmental_sar/S043_confined_space.md)
