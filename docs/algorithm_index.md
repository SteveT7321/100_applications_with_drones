# Algorithm Index

> Quick-reference table of algorithms used in each scene. For detailed mathematics see [MATH_FOUNDATIONS.md](../MATH_FOUNDATIONS.md)

## Control Algorithms

| Algorithm | Scenes | Python Package |
|-----------|--------|----------------|
| **PID (Cascade)** | S001~S005, S021~S025, S061~S065 | `gym_pybullet_drones.control.DSLPIDControl` |
| **LQR** | S009, S010, S082 | `scipy.linalg.solve_continuous_are` + `control` |
| **MPC** | S006, S024, S031 | `scipy.optimize.minimize` or `casadi` (optional) |
| **Proportional Navigation Guidance (PNG)** | S001~S010 | Custom implementation |
| **Optimal Control (Pontryagin)** | S009, S010 | `scipy.integrate.solve_bvp` |

## Path Planning

| Algorithm | Scenes | Description |
|-----------|--------|-------------|
| **RRT*** | S022, S043, S064 | `scipy` spatial tree or custom implementation |
| **A*** | S021, S048 | Grid-based search |
| **Dubins Path** | S082, S090 | Shortest path with turning radius constraint |
| **TSP (Nearest Neighbor / OR-Tools)** | S029, S068 | Multi-target visit ordering |
| **VRP** | S030, S040 | Vehicle routing problem, `ortools` |
| **Lawnmower** | S048, S067 | Regular strip scanning |
| **Potential Field** | S004, S089 | Real-time obstacle avoidance |

## Multi-Agent

| Algorithm | Scenes | Description |
|-----------|--------|-------------|
| **Hungarian Algorithm** | S018, S019, S029 | `scipy.optimize.linear_sum_assignment` |
| **Reynolds Flocking** | S011, S017, S097 | Custom implementation |
| **Distributed MPC** | S016, S031 | Independent MPC per drone + communication |
| **Consensus Algorithm** | S015, S051 | Distributed state estimation |
| **Auction Algorithm** | S019, S040 | Dynamic task reassignment |

## Estimation and Sensing

| Algorithm | Scenes | Package |
|-----------|--------|---------|
| **Kalman Filter (KF)** | S007, S008 | `filterpy` or custom implementation |
| **Extended Kalman Filter (EKF)** | S015, S045 | `filterpy` |
| **Particle Filter** | S045, S046 | Custom implementation |
| **SLAM** | S050, S074 | Custom implementation (simplified) |

## Game Theory

| Algorithm | Scenes | Description |
|-----------|--------|-------------|
| **Differential Game (Isaacs)** | S009, S020 | Numerical HJI solver |
| **Nash Equilibrium** | S017, S020 | `nashpy` (optional) |
| **Stackelberg Game** | S014, S016 | Leader-follower game |

## Reinforcement Learning (Advanced)

| Algorithm | Scenes | Package |
|-----------|--------|---------|
| **PPO** | S010, S020 | `stable_baselines3` |
| **SAC** | S006, S082 | `stable_baselines3` |
| **MAPPO (Multi-Agent)** | S017, S097 | Custom implementation or `epymarl` |

## Quick Lookup: Scene → Algorithm

| Scene | Primary Algorithm |
|-------|-------------------|
| S001 | PID + Proportional Navigation Guidance |
| S002 | PID + Optimal Evasion (analytical solution) |
| S006 | MPC + Energy consumption model |
| S007 | PID + EKF (target estimation) |
| S009 | Differential game numerical solution |
| S011 | Potential field + encirclement contraction |
| S018 | Hungarian Algorithm + PID |
| S020 | Differential game + PPO |
| S022 | RRT* + PID trajectory tracking |
| S026 | Distributed force control |
| S029 | VRP + A* |
| S045 | EKF + gradient ascent (chemical gradient) |
| S048 | Lawnmower + coverage calculation |
| S050 | EKF-SLAM (simplified) |
| S082 | Dubins + LQR |
| S088 | Geometric shape interpolation + collision avoidance |
| S089 | Olfati-Saber collision avoidance protocol |
