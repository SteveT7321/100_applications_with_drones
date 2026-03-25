# Drone Model Reference

> Drone models and physical parameters provided by gym-pybullet-drones

## Available Drone Models

| Model ID | Name | Characteristics |
|----------|------|-----------------|
| `cf2x` | Crazyflie 2.x (X-type) | Lightweight and small, mainstream in academic research |
| `cf2p` | Crazyflie 2.x (+type) | Same physics as cf2x, different propeller orientation |
| `race` | Racing Drone | High thrust-to-weight ratio, fast maneuverability |

This project uses `cf2x` (Crazyflie 2.x) by default.

---

## CF2X Physical Parameters

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Mass | $m$ | 0.027 | kg |
| Arm length | $l$ | 0.0397 | m |
| Thrust coefficient | $k_f$ | 3.16e-10 | N/(rad/s)² |
| Torque coefficient | $k_m$ | 7.94e-12 | Nm/(rad/s)² |
| Moment of inertia Ixx | $I_{xx}$ | 1.4e-5 | kg⋅m² |
| Moment of inertia Iyy | $I_{yy}$ | 1.4e-5 | kg⋅m² |
| Moment of inertia Izz | $I_{zz}$ | 2.17e-5 | kg⋅m² |
| Max thrust (single propeller) | $F_{max}$ | 0.3 | N |
| Max rotation speed | $\Omega_{max}$ | 21702 | rad/s |
| Hover thrust ratio | — | ≈ 0.33 | — |

---

## Physics Mode (Physics Enum)

```python
from gym_pybullet_drones.utils.enums import Physics

Physics.PYB          # Pure PyBullet rigid body dynamics
Physics.DYN          # Explicit dynamics equations (default, most accurate)
Physics.PYB_GND      # PyBullet + ground effect
Physics.PYB_DRAG     # PyBullet + drag
Physics.PYB_DW       # PyBullet + downwash (multi-drone)
Physics.PYB_GND_DRAG_DW  # All effects (most complete, slowest)
```

Research recommendations:
- Single-drone basic simulation: `Physics.DYN`
- Multi-drone close-range interaction: `Physics.PYB_DW`
- Full-fidelity simulation: `Physics.PYB_GND_DRAG_DW`

---

## Control Interface

### CtrlAviary (Open-loop Control)

```python
from gym_pybullet_drones.envs import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

env = CtrlAviary(
    drone_model=DroneModel.CF2X,
    num_drones=1,
    physics=Physics.PYB,
    pyb_freq=240,          # Physics simulation frequency (Hz)
    ctrl_freq=48,          # Control frequency (Hz)
    gui=False,             # Headless mode
    record=False,
    obstacles=False,
    initial_xyzs=None,     # Initial positions shape (N, 3)
    initial_rpys=None,     # Initial attitudes (N, 3)
)
```

### DSLPIDControl (PID Controller)

```python
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl

ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

# Compute control output each step
rpm, _, _ = ctrl.computeControlFromState(
    control_timestep=1/ctrl_freq,
    state=obs[i],          # 20-dimensional state vector
    target_pos=np.array([x, y, z]),
    target_rpy=np.array([0, 0, yaw]),
    target_vel=np.zeros(3),
)
```

### 20-Dimensional State Vector Description

```
obs[0:3]   = position (x, y, z)
obs[3:7]   = quaternion (x, y, z, w)
obs[7:10]  = roll, pitch, yaw
obs[10:13] = velocity (vx, vy, vz)
obs[13:16] = angular velocity (wx, wy, wz)
obs[16:20] = last RPM (4 rotors)
```

---

## Environment Step Flow

```python
obs, reward, terminated, truncated, info = env.step(action)
```

`action` shape: `(num_drones, 4)`, the RPM values for four propellers (0 ~ max_rpm)

---

## Sensor Coordinate Frame Diagram

```
      ^ z (up)
      |
      |    ^ x (forward)
      |   /
      |  /
      | /
      +---------> y (right)
      Body center
```

---

## Reference Documents

- Model parameter source: `lib/gym-pybullet-drones/gym_pybullet_drones/assets/` (`.urdf` files)
- PID controller: `lib/gym-pybullet-drones/gym_pybullet_drones/control/DSLPIDControl.py`
- Base environment: `lib/gym-pybullet-drones/gym_pybullet_drones/envs/BaseAviary.py`
