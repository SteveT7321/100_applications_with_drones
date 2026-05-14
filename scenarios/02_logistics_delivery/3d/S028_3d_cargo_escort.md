# S028 3D Upgrade — Cargo Escort Formation

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Completed
**Based on**: [S028 original](../S028_cargo_escort_formation.md)

---

## What Changes in 3D

The original S028 keeps all drones at a fixed altitude. In this 3D upgrade, the cargo drone flies a waypoint mission with varying altitude. Escort drones maintain a **diamond formation** in 3D: two escorts at ±offset in xy and two at ±offset in z.

---

## Problem Definition

**Setup**: A cargo drone flies a 3D waypoint mission (climb/descend between deliveries). Four escort drones maintain a 3D diamond formation around it, dynamically adjusting as the cargo drone changes altitude.

**Objective**: Minimise formation error (RMS deviation from desired 3D offsets) while the cargo drone completes its mission.

---

## Mathematical Model

Each escort tracks: `p_escort_desired = p_cargo + R(yaw) * offset_i`

Formation offsets (diamond): `[±d, 0, 0]`, `[0, ±d, 0]`, `[0, 0, ±d]` — four escorts use the lateral and vertical pairs.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Formation offset d | 3 m |
| Cargo waypoints | 5 (varying altitude 5–20 m) |
| Controller | PD position control |

---

## Related Scenarios

- Original: [S028](../S028_cargo_escort_formation.md)
- See also: [S011 Swarm Encirclement](../../01_pursuit_evasion/S011_swarm_encirclement.md)
