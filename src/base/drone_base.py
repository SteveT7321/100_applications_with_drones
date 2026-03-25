import numpy as np


class DroneBase:
    """Point-mass drone base class for all simulation scenarios."""

    def __init__(self, init_pos, max_speed=5.0, dt=1/48):
        self.init_pos = np.array(init_pos, dtype=float)
        self.max_speed = max_speed
        self.dt = dt
        self.pos = self.init_pos.copy()
        self.vel = np.zeros(3)
        self.trajectory = [self.pos.copy()]

    def step(self, v_cmd):
        """Apply velocity command and update position (clamped to max_speed)."""
        speed = np.linalg.norm(v_cmd)
        if speed > self.max_speed:
            v_cmd = v_cmd / speed * self.max_speed
        self.vel = v_cmd
        self.pos = self.pos + self.vel * self.dt
        self.trajectory.append(self.pos.copy())

    def reset(self):
        self.pos = self.init_pos.copy()
        self.vel = np.zeros(3)
        self.trajectory = [self.pos.copy()]

    def get_trajectory(self):
        return np.array(self.trajectory)
