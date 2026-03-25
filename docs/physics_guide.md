# PyBullet Physics Engine Guide

> Physics modes, parameter tuning, and common configuration notes for gym-pybullet-drones

## Physics Simulation Frequency

| Parameter | Recommended Value | Description |
|-----------|-------------------|-------------|
| `pyb_freq` | 240 Hz | PyBullet physics step frequency |
| `ctrl_freq` | 48 Hz | Controller update frequency |
| Ratio | 5:1 | Each control step = 5 physics steps |

Lowering `pyb_freq` speeds up simulation but reduces accuracy. 240 Hz is recommended for research simulations.

## Physics Effect Modes

### PYB (Basic)
```python
physics=Physics.PYB
```
- Pure PyBullet rigid body dynamics
- No aerodynamic effects simulated
- Fastest, suitable for large-scale multi-drone scenarios

### DYN (Explicit Dynamics, Default)
```python
physics=Physics.DYN
```
- Uses explicit quadrotor dynamics equations
- Accurately simulates thrust and torque
- Suitable for most research scenarios

### PYB_DW (With Downwash, Required for Multi-Drone)
```python
physics=Physics.PYB_DW
```
- Simulates propeller downwash effects on other drones
- Significant at close range (< 2 arm lengths)
- Higher computational cost

### PYB_GND (Ground Effect)
```python
physics=Physics.PYB_GND
```
- Thrust increases approximately 15~30% near the ground (< 1 arm length)
- Required for low-altitude pursuit and takeoff/landing scenarios

## Environment Configuration Example

```python
from gym_pybullet_drones.envs import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# Standard single-drone simulation
env = CtrlAviary(
    drone_model=DroneModel.CF2X,
    num_drones=1,
    physics=Physics.PYB,
    pyb_freq=240,
    ctrl_freq=48,
    gui=False,           # True = open PyBullet GUI window
    record=False,
    obstacles=False,
    initial_xyzs=np.array([[0, 0, 1]]),   # Initial position
    initial_rpys=np.array([[0, 0, 0]]),   # Initial attitude (RPY)
)

# Multi-drone simulation (N drones)
N = 3
env = CtrlAviary(
    drone_model=DroneModel.CF2X,
    num_drones=N,
    physics=Physics.PYB_DW,
    pyb_freq=240,
    ctrl_freq=48,
    gui=False,
    initial_xyzs=np.array([[i*0.5, 0, 1] for i in range(N)]),
)
```

## Adding Obstacles

gym-pybullet-drones uses the PyBullet URDF format to define obstacles:

```python
import pybullet as p

# Add a spherical obstacle
obstacle_id = p.loadURDF(
    "sphere_small.urdf",
    basePosition=[2.0, 0.0, 1.0],
    physicsClientId=env.CLIENT
)

# Add a box
obstacle_id = p.loadURDF(
    "cube.urdf",
    basePosition=[1.0, 1.0, 0.5],
    globalScaling=0.5,
    physicsClientId=env.CLIENT
)
```

Custom geometry:
```python
# Create collision shape
col_id = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[0.5, 0.5, 1.0],
    physicsClientId=env.CLIENT
)
vis_id = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.5, 0.5, 1.0],
    rgbaColor=[0.5, 0.5, 0.5, 0.8],
    physicsClientId=env.CLIENT
)
body_id = p.createMultiBody(
    baseMass=0,            # Static obstacle mass=0
    baseCollisionShapeIndex=col_id,
    baseVisualShapeIndex=vis_id,
    basePosition=[1, 1, 0.5],
    physicsClientId=env.CLIENT
)
```

## Standard Simulation Loop

```python
import numpy as np
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl

# Initialize
obs, info = env.reset()
ctrl = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(N)]

# Collect trajectories
trajectories = [[] for _ in range(N)]
T = 500   # Simulation steps

for step in range(T):
    # Compute control actions
    action = np.zeros((N, 4))
    for i in range(N):
        target_pos = compute_target(i, step, obs)   # Implement yourself
        rpm, _, _ = ctrl[i].computeControlFromState(
            control_timestep=env.CTRL_TIMESTEP,
            state=obs[i],
            target_pos=target_pos,
        )
        action[i] = rpm

    # Step
    obs, reward, terminated, truncated, info = env.step(action)

    # Record positions
    for i in range(N):
        trajectories[i].append(obs[i][0:3].copy())

    if terminated or truncated:
        break

env.close()

# Convert to numpy arrays
trajs = [np.array(t) for t in trajectories]
```

## Collision Detection

```python
import pybullet as p

def check_collision(env, drone_i, drone_j, threshold=0.1):
    """Check whether two drones have collided"""
    contacts = p.getContactPoints(
        bodyA=env.DRONE_IDS[drone_i],
        bodyB=env.DRONE_IDS[drone_j],
        physicsClientId=env.CLIENT
    )
    return len(contacts) > 0

def check_capture(pos_pursuer, pos_evader, threshold=0.15):
    """Check whether the pursuer has captured the evader"""
    return np.linalg.norm(pos_pursuer - pos_evader) < threshold
```

## Performance Optimization

| Scale | Recommended Settings |
|-------|----------------------|
| 1~3 drones | `Physics.DYN`, `pyb_freq=240` |
| 4~10 drones | `Physics.PYB`, `pyb_freq=240` |
| 10+ drones | `Physics.PYB`, `pyb_freq=120`, `gui=False` |
| Light show 50+ | `Physics.PYB`, `pyb_freq=48` or pure mathematical model |

> For more than 20 drones, consider switching to a pure mathematical (ODE) simulation without PyBullet, using only matplotlib for visualization.
