"""
6-DOF rigid-body quadrotor model with first-order motor dynamics (NumPy only).

Used by the hardware-oriented scenarios (S082, S084, S090, S091). All other scenarios use the
point-mass model in ``drone_base.py``.

State (per drone, batched over a leading dimension N so that Monte Carlo runs and strategy
comparisons can be simulated side by side with vectorised NumPy):

    pos    (N, 3)  world position, ENU / z-up                                 [m]
    vel    (N, 3)  world velocity                                             [m/s]
    quat   (N, 4)  unit quaternion body->world, scalar first (w, x, y, z)
    omega  (N, 3)  body angular rate (FLU body frame)                         [rad/s]
    rotor  (N, 4)  rotor speeds                                               [rad/s]

Equations of motion:

    p'     = v
    m v'   = R(q) [0, 0, T] - R C_d R^T (v - v_wind) + m g + F_ext
    q'     = 1/2 q (x) [0, omega]
    J w'   = tau - omega x (J omega)
    Omega' = (Omega_cmd - Omega) / tau_m           (first-order motor lag)
    F_i    = k_f Omega_i^2,  Q_i = k_m Omega_i^2   (thrust, reaction torque)

The rigid body (p, v, q, omega) is integrated with classical RK4 at a fixed step. The motor ODE
is linear under a zero-order-hold command, so the rotor speed inside the step is evaluated
exactly, Omega(s) = Omega_cmd + (Omega_0 - Omega_cmd) exp(-s / tau_m), and fed to the RK4 stages.
The quaternion is renormalised after every step.

Motor layout (X frame, identical to gym-pybullet-drones ``cf2x``): motor 0 front-right (+x, -y),
1 rear-right (-x, -y), 2 rear-left (-x, +y), 3 front-left (+x, +y). Motors 0 and 2 produce a
negative yaw reaction torque, motors 1 and 3 a positive one.

Control (cascaded, PX4-like):

    position P  ->  velocity PI  ->  thrust vector / attitude (quaternion P)  ->  body-rate PI
                ->  mixer (wrench -> rotor speeds, with airmode-style desaturation)

Entry points mirror ``DroneBase``: ``step(v_cmd)`` takes a velocity command, ``step_position``
a position setpoint (with optional velocity / acceleration / body-rate feed-forward),
``step_acc`` an acceleration command, ``step_rates(thrust, rates)`` a direct collective-thrust +
body-rate command (racing / acrobatics), ``step_wrench(thrust, torque)`` the output of an
external attitude controller and ``step_motors`` raw rotor-speed commands. ``ext_force`` (N, 3)
adds a world-frame force (e.g. quadratic aerodynamic drag) held constant over one step.

Mixer saturation handling (``desat_mode``):
  * 'airmode' (default, Betaflight-airmode-like): yaw torque has priority over collective thrust -
    when the torques do not fit, the collective is shifted first, yaw is reduced only if no shift
    works. A large yaw-rate error can therefore raise the collective a lot (on fpv_5inch a
    150 rad/s yaw-rate error under step_position climbs about 140 m in 6 s, see the self-test).
  * 'px4' (PX4-like): roll/pitch > collective thrust > yaw - yaw is reduced first without moving
    the collective; the collective is shifted only for roll/pitch.

What the model does NOT include (see the scenario READMEs, "Sim2Real Notes"): blade flapping and
rotor drag beyond a linear term, ground effect, downwash, propeller gyroscopic torque, rotor
spin-up reaction torque (J_r * d omega_i / dt, which dominates fast yaw transients on real quads;
only the steady reaction torque k_m omega^2 is modelled), battery voltage sag, ESC/motor
nonlinearity beyond saturation, sensor latency and state estimation (the outer loops use the true
or noise-corrupted state).
"""

import numpy as np

G = 9.81                      # m/s^2
RHO_AIR = 1.225               # kg/m^3
RPM_TO_RADS = 2.0 * np.pi / 60.0
E_Z = np.array([0.0, 0.0, 1.0])


# ── Presets ──────────────────────────────────────────────────────────────────
# crazyflie: values from gym-pybullet-drones 2.0.0, file
#   <drones env>/Lib/site-packages/gym_pybullet_drones/assets/cf2x.urdf
#   <properties arm="0.0397" kf="3.16e-10" km="7.94e-12" thrust2weight="2.25"
#    prop_radius="2.31348e-2" drag_coeff_xy="9.1785e-7" drag_coeff_z="10.311e-7" />
#   <mass value="0.027"/> <inertia ixx="1.4e-5" iyy="1.4e-5" izz="2.17e-5"/>
# gym-pybullet-drones applies kf, km to rotor speeds in RPM (BaseAviary._dynamics:
# forces = rpm**2 * KF), so they are converted here to rad/s: k[rad/s] = k[RPM] / (2 pi / 60)^2.
# Its drag model (BaseAviary._drag) is F_body = -drag_coeff (elementwise) * sum(rotor speed in
# rad/s) * (R^T v_world), i.e. a per-axis coefficient applied in the body frame to the world
# velocity rotated into the body frame. This model uses the same form, F_world = -R diag(c) R^T
# (v - v_wind), with c evaluated once at the hover rotor speed (constant linear drag, N per m/s);
# the rotor-speed dependence is dropped.
# Not in the URDF (estimates, labelled as such): motor time constant, minimum rotor speed,
# motor efficiency. Caveat: the URDF km is a yaw-torque coefficient; used as shaft power
# k_m w^3 it gives ~10 W at hover (figure of merit ~0.11), so Crazyflie energy numbers from this
# model are pessimistic and must be replaced by measured power for endurance studies.
#
# fpv_5inch: TYPICAL ESTIMATES for a 5-inch freestyle / racing quad on 6S (~600 g all-up,
# 2306-class motors, 5x4.3 tri-blade props). None of these numbers is a measurement; every one
# must be replaced by thrust-stand / swing-test values before any sim-to-real claim.

def _cf_params():
    kf = 3.16e-10 / RPM_TO_RADS ** 2          # N/(rad/s)^2   (URDF, converted)
    km = 7.94e-12 / RPM_TO_RADS ** 2          # N m/(rad/s)^2 (URDF, converted)
    mass = 0.027
    t2w = 2.25
    w_hover = np.sqrt(mass * G / (4 * kf))
    w_max = np.sqrt(t2w * mass * G / (4 * kf))
    drag_xy = 9.1785e-7 * 4 * w_hover         # N/(m/s) at hover rotor speed
    drag_z = 10.311e-7 * 4 * w_hover
    return dict(
        name='crazyflie', mass=mass, arm=0.0397, kf=kf, km=km,
        inertia=np.array([1.4e-5, 1.4e-5, 2.17e-5]),
        w_min=0.0, w_max=w_max,
        tau_motor=0.020,       # ESTIMATE: brushed 7x16 mm motor, ~20 ms (not in the URDF)
        prop_radius=2.31348e-2,
        drag=np.array([drag_xy, drag_xy, drag_z]),
        motor_eff=0.6,         # ESTIMATE: electrical -> shaft power efficiency
        source='gym-pybullet-drones 2.0.0 assets/cf2x.urdf (tau_motor, w_min, motor_eff estimated)',
    )


def _fpv_params():
    mass = 0.60                                # ESTIMATE: all-up mass incl. 6S 1100 mAh pack
    w_max = 30000 * RPM_TO_RADS                # ESTIMATE: ~30 krpm at full throttle on 6S
    f_max = 9.0                                # ESTIMATE: ~0.92 kgf static thrust per motor
    kf = f_max / w_max ** 2                    # -> T/W = 4*9 / (0.6*9.81) = 6.1
    # km from an assumed hover figure of merit FM = 0.6:
    # P_shaft = k_m w^3 = P_ideal / FM, P_ideal = T^1.5 / sqrt(2 rho A) per rotor.
    r_prop = 0.0635
    t_h = mass * G / 4
    w_h = np.sqrt(t_h / kf)
    p_ideal = t_h ** 1.5 / np.sqrt(2 * RHO_AIR * np.pi * r_prop ** 2)
    km = p_ideal / 0.6 / w_h ** 3
    return dict(
        name='fpv_5inch', mass=mass,
        arm=0.110,                             # ESTIMATE: 220 mm motor-to-motor diagonal
        kf=kf, km=km,
        inertia=np.array([1.4e-3, 1.4e-3, 2.4e-3]),   # ESTIMATE: lumped-mass build-up
        w_min=0.08 * w_max,                    # ESTIMATE: ~8 % idle (motors never stop in air)
        w_max=w_max,
        tau_motor=0.015,                       # ESTIMATE: 15 ms with DShot + active braking
        prop_radius=r_prop,
        drag=np.array([0.05, 0.05, 0.05]),     # linear drag 0.05 kg/s (S082 card value)
        motor_eff=0.75,                        # ESTIMATE
        source='typical-value estimates (no measured data)',
    )


PRESET_BUILDERS = {'crazyflie': _cf_params, 'fpv_5inch': _fpv_params}

# Controller gains per preset (tuned in test_rigid_body_drone.py for stable, well-damped steps).
# Rate-loop crossover is set below 1/tau_motor so the motor lag costs < 35 deg of phase.
DEFAULT_GAINS = {
    'crazyflie': dict(
        kp_pos=np.array([1.5, 1.5, 2.0]), kp_vel=np.array([3.0, 3.0, 4.0]),
        ki_vel=np.array([1.5, 1.5, 2.0]), vel_int_max=2.0, vel_int_band=0.2,
        k_att=np.array([8.0, 8.0, 3.0]), kp_rate=np.array([30.0, 30.0, 15.0]),
        ki_rate=np.array([20.0, 20.0, 5.0]), rate_int_max=2.0,
        max_rate=np.array([8.0, 8.0, 3.0]), max_tilt=np.radians(35.0), v_max=3.0,
    ),
    'fpv_5inch': dict(
        kp_pos=np.array([2.0, 2.0, 2.5]), kp_vel=np.array([4.0, 4.0, 5.0]),
        ki_vel=np.array([2.0, 2.0, 2.5]), vel_int_max=4.0, vel_int_band=0.3,
        k_att=np.array([12.0, 12.0, 4.0]), kp_rate=np.array([40.0, 40.0, 20.0]),
        ki_rate=np.array([30.0, 30.0, 8.0]), rate_int_max=3.0,
        max_rate=np.array([15.0, 15.0, 6.0]), max_tilt=np.radians(70.0), v_max=20.0,
    ),
}


class QuadParams:
    """Physical parameters of one quadrotor type (attributes from a preset dict)."""

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)
        self.inertia = np.asarray(self.inertia, dtype=float)
        self.drag = np.asarray(self.drag, dtype=float)
        d = self.arm / np.sqrt(2.0)
        c = self.km / self.kf
        # [T, tau_x, tau_y, tau_z]^T = A @ [F0, F1, F2, F3]^T
        self.alloc = np.array([
            [1.0, 1.0, 1.0, 1.0],
            [-d, -d, d, d],
            [-d, d, d, -d],
            [-c, c, -c, c],
        ])
        self.alloc_inv = np.linalg.inv(self.alloc)
        self.weight = self.mass * G
        self.w_hover = np.sqrt(self.weight / (4 * self.kf))
        self.thrust_max = 4 * self.kf * self.w_max ** 2
        self.t2w = self.thrust_max / self.weight


def make_params(preset='crazyflie', **overrides):
    """Return QuadParams for a preset, optionally overriding any field."""
    d = PRESET_BUILDERS[preset]()
    d.update(overrides)
    return QuadParams(**d)


def default_gains(preset='crazyflie', **overrides):
    g = {k: (v.copy() if isinstance(v, np.ndarray) else v)
         for k, v in DEFAULT_GAINS[preset].items()}
    g.update(overrides)
    return g


# ── Quaternion / rotation helpers (batched, scalar-first) ────────────────────
def quat_mul(a, b):
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], axis=-1)


def quat_conj(q):
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def quat_to_rot(q):
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = 1 - 2 * (y * y + z * z)
    R[..., 0, 1] = 2 * (x * y - w * z)
    R[..., 0, 2] = 2 * (x * z + w * y)
    R[..., 1, 0] = 2 * (x * y + w * z)
    R[..., 1, 1] = 1 - 2 * (x * x + z * z)
    R[..., 1, 2] = 2 * (y * z - w * x)
    R[..., 2, 0] = 2 * (x * z - w * y)
    R[..., 2, 1] = 2 * (y * z + w * x)
    R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def rot_to_quat(R):
    """Shepperd's method, batched over leading dims; returns w >= 0."""
    r00, r11, r22 = R[..., 0, 0], R[..., 1, 1], R[..., 2, 2]
    tr = r00 + r11 + r22
    k = np.argmax(np.stack([tr, r00, r11, r22], axis=-1), axis=-1)
    s0 = 2 * np.sqrt(np.maximum(1 + tr, 1e-12))
    s1 = 2 * np.sqrt(np.maximum(1 + r00 - r11 - r22, 1e-12))
    s2 = 2 * np.sqrt(np.maximum(1 + r11 - r00 - r22, 1e-12))
    s3 = 2 * np.sqrt(np.maximum(1 + r22 - r00 - r11, 1e-12))
    a21m12 = R[..., 2, 1] - R[..., 1, 2]
    a02m20 = R[..., 0, 2] - R[..., 2, 0]
    a10m01 = R[..., 1, 0] - R[..., 0, 1]
    a01p10 = R[..., 0, 1] + R[..., 1, 0]
    a02p20 = R[..., 0, 2] + R[..., 2, 0]
    a12p21 = R[..., 1, 2] + R[..., 2, 1]
    cand = np.stack([
        np.stack([s0 / 4, a21m12 / s0, a02m20 / s0, a10m01 / s0], axis=-1),
        np.stack([a21m12 / s1, s1 / 4, a01p10 / s1, a02p20 / s1], axis=-1),
        np.stack([a02m20 / s2, a01p10 / s2, s2 / 4, a12p21 / s2], axis=-1),
        np.stack([a10m01 / s3, a02p20 / s3, a12p21 / s3, s3 / 4], axis=-1),
    ], axis=-2)
    q = np.take_along_axis(cand, k[..., None, None], axis=-2)[..., 0, :]
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    return np.where(q[..., :1] < 0, -q, q)


def quat_from_euler(roll, pitch, yaw):
    """ZYX Euler angles (rad) -> quaternion; scalars or arrays of equal shape."""
    cr, sr = np.cos(np.asarray(roll) / 2), np.sin(np.asarray(roll) / 2)
    cp, sp = np.cos(np.asarray(pitch) / 2), np.sin(np.asarray(pitch) / 2)
    cy, sy = np.cos(np.asarray(yaw) / 2), np.sin(np.asarray(yaw) / 2)
    return np.stack([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy], axis=-1)


def quat_to_euler(q):
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=-1)


def tilt_angle(q):
    """Angle between body z and world z (rad)."""
    return np.arccos(np.clip(quat_to_rot(q)[..., 2, 2], -1.0, 1.0))


def momentum_theory_power(thrust_per_rotor, prop_radius, rho=RHO_AIR):
    """Ideal induced hover power of one rotor, T^1.5 / sqrt(2 rho A)  [W]."""
    return np.asarray(thrust_per_rotor) ** 1.5 / np.sqrt(2 * rho * np.pi * prop_radius ** 2)


# ── Wind ─────────────────────────────────────────────────────────────────────
class GustWind:
    """Mean wind + first-order Gauss-Markov (Ornstein-Uhlenbeck) gusts, seeded.

    w_gust[k+1] = a w_gust[k] + sigma sqrt(1 - a^2) xi,  a = exp(-dt / tau_c),  xi ~ N(0, I).
    With ``shared=True`` every drone of a batch sees the same gust sequence (common random
    numbers for strategy comparisons).
    """

    def __init__(self, mean, sigma, tau_c, seed=0, shared=True):
        self.mean = np.asarray(mean, dtype=float)
        self.sigma = np.broadcast_to(np.asarray(sigma, dtype=float), (3,)).copy()
        self.tau_c = float(tau_c)
        self.rng = np.random.default_rng(seed)
        self.shared = shared
        self.gust = None

    def __call__(self, t, pos, dt):
        n = pos.shape[0]
        rows = 1 if self.shared else n
        if self.gust is None:
            self.gust = self.sigma * self.rng.standard_normal((rows, 3))
        else:
            a = np.exp(-dt / self.tau_c)
            self.gust = a * self.gust + self.sigma * np.sqrt(1 - a * a) * \
                self.rng.standard_normal((rows, 3))
        return np.broadcast_to(self.mean + self.gust, (n, 3))


# ── Drone ────────────────────────────────────────────────────────────────────
class RigidBodyDrone:
    """Batched 6-DOF quadrotor with motor lag, mixer and cascaded PID control."""

    def __init__(self, init_pos, preset='crazyflie', dt=0.002, n_substeps=1, params=None,
                 gains=None, wind=None, imu_noise=None, seed=0, init_yaw=0.0,
                 ground_z=None, drag_ff=True, record=True, desat_mode='airmode'):
        self.init_pos = np.atleast_2d(np.asarray(init_pos, dtype=float)).copy()
        self.n = self.init_pos.shape[0]
        self.preset = preset
        self.p = params if params is not None else make_params(preset)
        self.g = gains if gains is not None else default_gains(preset)
        self.dt = float(dt)
        self.n_substeps = int(n_substeps)
        self.wind = wind
        self.imu = dict(gyro_std=0.0, gyro_bias_std=0.0, pos_std=0.0, vel_std=0.0)
        if imu_noise:
            self.imu.update(imu_noise)
        self.seed = seed
        self.init_yaw = np.broadcast_to(np.asarray(init_yaw, dtype=float), (self.n,)).copy()
        self.ground_z = ground_z
        self.drag_ff = drag_ff
        self.record = record
        if desat_mode not in ('airmode', 'px4'):
            raise ValueError(f'unknown desat_mode {desat_mode!r}')
        self.desat_mode = desat_mode
        self.max_speed = self.g['v_max']         # DroneBase compatibility
        self.reset()

    # ── state management ──
    def reset(self):
        n, p = self.n, self.p
        self.rng = np.random.default_rng(self.seed)
        self.t = 0.0
        self.pos = self.init_pos.copy()
        self.vel = np.zeros((n, 3))
        self.quat = quat_from_euler(np.zeros(n), np.zeros(n), self.init_yaw)
        self.omega = np.zeros((n, 3))
        self.rotor = np.full((n, 4), p.w_hover)
        self.w_min = np.full((n, 4), float(p.w_min))
        self.w_max = np.full((n, 4), float(p.w_max))
        self.rotor_cmd = self.rotor.copy()
        self.yaw_sp = self.init_yaw.copy()
        self.ext_force = np.zeros((n, 3))
        self.wind_now = np.zeros((n, 3))
        self.gyro_bias = self.imu['gyro_bias_std'] * self.rng.standard_normal((n, 3))
        self._vel_int = np.zeros((n, 3))
        self._rate_int = np.zeros((n, 3))
        self.energy = np.zeros(n)          # shaft energy  [J]
        self.n_sat = 0                     # control steps with any mixer desaturation
        self.last_rate_sp = np.zeros((n, 3))
        self.last_thrust_cmd = np.full(n, p.weight)
        self.log = {k: [] for k in ('t', 'pos', 'vel', 'quat', 'omega', 'rotor',
                                    'thrust', 'power', 'rate_sp', 'wind')}
        if self.record:
            self._record()

    def set_state(self, pos=None, vel=None, quat=None, omega=None, rotor=None):
        for name, val in (('pos', pos), ('vel', vel), ('quat', quat), ('omega', omega),
                          ('rotor', rotor)):
            if val is not None:
                cur = getattr(self, name)
                setattr(self, name, np.broadcast_to(np.asarray(val, dtype=float),
                                                    cur.shape).copy())

    def set_motor_limits(self, motor, w_min=None, w_max=None, drones=slice(None)):
        """Change one motor's speed limits (e.g. a damaged or weak motor) and clip its speed."""
        if w_min is not None:
            self.w_min[drones, motor] = w_min
        if w_max is not None:
            self.w_max[drones, motor] = w_max
        self.rotor = np.clip(self.rotor, self.w_min, self.w_max)
        self.rotor_cmd = np.clip(self.rotor_cmd, self.w_min, self.w_max)

    # ── derived quantities ──
    def rotation(self):
        return quat_to_rot(self.quat)

    def thrust_total(self, rotor=None):
        r = self.rotor if rotor is None else rotor
        return self.p.kf * np.sum(r ** 2, axis=-1)

    def power_shaft(self, rotor=None):
        r = self.rotor if rotor is None else rotor
        return self.p.km * np.sum(r ** 3, axis=-1)

    def power_elec(self, rotor=None):
        return self.power_shaft(rotor) / self.p.motor_eff

    def speed(self):
        return np.linalg.norm(self.vel, axis=-1)

    def yaw(self):
        return quat_to_euler(self.quat)[..., 2]

    def _meas_rates(self):
        w = self.omega + self.gyro_bias
        if self.imu['gyro_std'] > 0:
            w = w + self.imu['gyro_std'] * self.rng.standard_normal(w.shape)
        return w

    def _meas_posvel(self):
        p, v = self.pos, self.vel
        if self.imu['pos_std'] > 0:
            p = p + self.imu['pos_std'] * self.rng.standard_normal(p.shape)
        if self.imu['vel_std'] > 0:
            v = v + self.imu['vel_std'] * self.rng.standard_normal(v.shape)
        return p, v

    # ── controllers ──
    def _vel_ctrl(self, v_sp, v_meas, a_ff):
        g = self.g
        e = v_sp - v_meas
        # conditional integration: large transient errors do not wind the integrator up
        e_i = np.where(np.abs(e) < g['vel_int_band'], e, 0.0)
        self._vel_int = np.clip(self._vel_int + e_i * self.dt, -g['vel_int_max'], g['vel_int_max'])
        return a_ff + g['kp_vel'] * e + g['ki_vel'] * self._vel_int

    def _acc_to_att(self, a_cmd, v_meas, yaw_sp):
        """Desired acceleration -> (collective thrust [N], desired rotation matrix)."""
        p, g = self.p, self.g
        f = p.mass * (a_cmd + G * E_Z)
        if self.drag_ff:                         # compensate known drag of the airframe
            R = self.rotation()
            vb = np.einsum('nji,nj->ni', R, v_meas)
            f = f + np.einsum('nij,nj->ni', R, p.drag * vb)
        # keep a minimum upward component and respect the tilt limit
        fz = np.maximum(f[:, 2], 0.05 * p.weight)
        fh = f[:, :2]
        fh_max = fz * np.tan(g['max_tilt'])
        nh = np.linalg.norm(fh, axis=1)
        scale = np.where(nh > fh_max, fh_max / np.maximum(nh, 1e-12), 1.0)
        f = np.column_stack([fh * scale[:, None], fz])
        zb = f / np.linalg.norm(f, axis=1, keepdims=True)
        xc = np.column_stack([np.cos(yaw_sp), np.sin(yaw_sp), np.zeros(self.n)])
        yb = np.cross(zb, xc)
        yb /= np.linalg.norm(yb, axis=1, keepdims=True)
        xb = np.cross(yb, zb)
        R_des = np.stack([xb, yb, zb], axis=-1)
        z_now = self.rotation()[:, :, 2]
        thrust = np.clip(np.sum(f * z_now, axis=1), 0.0, None)
        return thrust, R_des

    def _att_ctrl(self, R_des, rate_ff=None):
        q_des = rot_to_quat(R_des)
        q_e = quat_mul(quat_conj(self.quat), q_des)
        q_e = np.where(q_e[:, :1] < 0, -q_e, q_e)
        rate_sp = 2.0 * self.g['k_att'] * q_e[:, 1:]
        if rate_ff is not None:
            rate_sp = rate_sp + rate_ff
        mr = self.g['max_rate']
        return np.clip(rate_sp, -mr, mr)

    def _rate_ctrl(self, rate_sp, ang_acc_ff=None):
        g, J = self.g, self.p.inertia
        w_m = self._meas_rates()
        e = rate_sp - w_m
        self._rate_int = np.clip(self._rate_int + e * self.dt,
                                 -g['rate_int_max'], g['rate_int_max'])
        alpha = g['kp_rate'] * e + g['ki_rate'] * self._rate_int
        if ang_acc_ff is not None:
            alpha = alpha + ang_acc_ff
        return J * alpha + np.cross(w_m, J * w_m)

    def mix(self, thrust, torque):
        """(N,) collective thrust + (N,3) body torque -> (N,4) rotor-speed commands.

        desat_mode 'airmode': (1) shift the collective so the torques fit, (2) if that is
        impossible, scale the yaw torque down, (3) then roll/pitch, (4) clip to the limits.
        desat_mode 'px4': (1) scale the yaw torque down with the collective unchanged, (2) if
        roll/pitch alone do not fit, drop yaw and shift the collective, (3) then scale
        roll/pitch, (4) clip.
        """
        p = self.p
        Ai = p.alloc_inv
        lo = p.kf * self.w_min ** 2
        hi = p.kf * self.w_max ** 2
        f_t = thrust[:, None] * Ai[:, 0][None, :]
        f_rp = torque[:, :2] @ Ai[:, 1:3].T
        f_y = torque[:, 2:3] * Ai[:, 3][None, :]
        F = f_t + f_rp + f_y
        bad = np.any((F < lo - 1e-12) | (F > hi + 1e-12), axis=1)
        if np.any(bad):
            self.n_sat += 1
            idx = np.where(bad)[0]
            desat = self._desaturate_px4 if self.desat_mode == 'px4' else self._desaturate
            F[idx] = desat(f_t[idx], f_rp[idx], f_y[idx], lo[idx], hi[idx])
        F = np.clip(F, lo, hi)
        return np.sqrt(F / p.kf)

    @staticmethod
    def _gap(x, lo, hi):
        """>0 when no common collective offset brings every motor of x inside [lo, hi]."""
        return np.max(lo - x, axis=1) - np.min(hi - x, axis=1)

    @staticmethod
    def _viol(x, lo, hi):
        """>0 when some motor of x is outside [lo, hi] (no collective shift allowed)."""
        return np.max(np.maximum(lo - x, x - hi), axis=1)

    def _desaturate_px4(self, f_t, f_rp, f_y, lo, hi):
        n = len(f_t)
        base = f_t + f_rp
        fits = self._viol(base, lo, hi) <= 0      # thrust + roll/pitch fit without yaw
        # largest yaw scale that still fits with the collective unchanged (violation is convex
        # in the scale and <= 0 at 0, so the feasible scales form an interval [0, s*])
        a, b = np.zeros(n), np.ones(n)
        for _ in range(10):
            m = 0.5 * (a + b)
            ok = self._viol(base + m[:, None] * f_y, lo, hi) <= 0
            a, b = np.where(ok, m, a), np.where(ok, b, m)
        out = base + np.where(fits, a, 0.0)[:, None] * f_y
        if np.any(~fits):
            # yaw dropped; roll/pitch handled as in airmode (collective shift, then scaling)
            k = np.where(~fits)[0]
            out[k] = self._desaturate(f_t[k], f_rp[k], np.zeros_like(f_y[k]), lo[k], hi[k])
        return out

    def _desaturate(self, f_t, f_rp, f_y, lo, hi):
        s_y = np.ones(len(f_t))
        s_rp = np.ones(len(f_t))
        need_y = self._gap(f_t + f_rp + f_y, lo, hi) > 0
        if np.any(need_y):
            rp_ok = self._gap(f_t + f_rp, lo, hi) <= 0
            # bisection on the yaw scale (gap is convex in the scale, gap(0) <= 0)
            a, b = np.zeros(len(f_t)), np.ones(len(f_t))
            for _ in range(10):
                m = 0.5 * (a + b)
                ok = self._gap(f_t + f_rp + m[:, None] * f_y, lo, hi) <= 0
                a, b = np.where(ok, m, a), np.where(ok, b, m)
            s_y = np.where(need_y, np.where(rp_ok, a, 0.0), 1.0)
            need_rp = need_y & ~rp_ok
            if np.any(need_rp):
                a, b = np.zeros(len(f_t)), np.ones(len(f_t))
                for _ in range(10):
                    m = 0.5 * (a + b)
                    ok = self._gap(f_t + m[:, None] * f_rp, lo, hi) <= 0
                    a, b = np.where(ok, m, a), np.where(ok, b, m)
                s_rp = np.where(need_rp, a, 1.0)
        x = f_t + s_rp[:, None] * f_rp + s_y[:, None] * f_y
        cmin = np.max(lo - x, axis=1)
        cmax = np.min(hi - x, axis=1)
        c = np.where(cmin <= cmax, np.clip(0.0, cmin, cmax), 0.0)
        return x + c[:, None]

    # ── public step functions ──
    def step(self, v_cmd, yaw=None, a_ff=None):
        """Velocity command (like DroneBase.step), clamped to v_max."""
        v_cmd = np.atleast_2d(np.asarray(v_cmd, dtype=float))
        sp = np.linalg.norm(v_cmd, axis=1, keepdims=True)
        v_cmd = np.where(sp > self.max_speed, v_cmd / np.maximum(sp, 1e-12) * self.max_speed,
                         v_cmd)
        _, v_m = self._meas_posvel()
        a_ff = np.zeros((self.n, 3)) if a_ff is None else np.atleast_2d(a_ff)
        a_cmd = self._vel_ctrl(v_cmd, v_m, a_ff)
        self.step_acc(a_cmd, yaw=yaw, _v_meas=v_m)

    def step_position(self, p_ref, v_ff=None, a_ff=None, yaw=None, rate_ff=None):
        """Position setpoint with optional velocity / acceleration feed-forward."""
        p_ref = np.atleast_2d(np.asarray(p_ref, dtype=float))
        p_m, v_m = self._meas_posvel()
        v_ff = np.zeros((self.n, 3)) if v_ff is None else np.atleast_2d(v_ff)
        a_ff = np.zeros((self.n, 3)) if a_ff is None else np.atleast_2d(a_ff)
        v_sp = v_ff + self.g['kp_pos'] * (p_ref - p_m)
        sp = np.linalg.norm(v_sp, axis=1, keepdims=True)
        v_sp = np.where(sp > self.max_speed, v_sp / np.maximum(sp, 1e-12) * self.max_speed, v_sp)
        a_cmd = self._vel_ctrl(v_sp, v_m, a_ff)
        self.step_acc(a_cmd, yaw=yaw, rate_ff=rate_ff, _v_meas=v_m)

    def step_acc(self, a_cmd, yaw=None, rate_ff=None, _v_meas=None):
        a_cmd = np.atleast_2d(np.asarray(a_cmd, dtype=float))
        if yaw is not None:
            self.yaw_sp = np.broadcast_to(np.asarray(yaw, dtype=float), (self.n,)).copy()
        v_m = self._meas_posvel()[1] if _v_meas is None else _v_meas
        thrust, R_des = self._acc_to_att(a_cmd, v_m, self.yaw_sp)
        rate_sp = self._att_ctrl(R_des, rate_ff)
        self.step_rates(thrust, rate_sp)

    def step_attitude(self, thrust, R_des, rate_ff=None):
        self.step_rates(np.broadcast_to(np.asarray(thrust, dtype=float), (self.n,)),
                        self._att_ctrl(np.asarray(R_des, dtype=float).reshape(self.n, 3, 3),
                                       rate_ff))

    def step_rates(self, thrust, rates, ang_acc_ff=None):
        """Direct collective thrust [N] + body-rate setpoint [rad/s] (acro mode)."""
        thrust = np.broadcast_to(np.asarray(thrust, dtype=float), (self.n,)).copy()
        rates = np.atleast_2d(np.asarray(rates, dtype=float))
        self.last_rate_sp = rates.copy()
        self.last_thrust_cmd = thrust
        torque = self._rate_ctrl(rates, ang_acc_ff)
        self.step_motors(self.mix(thrust, torque))

    def step_wrench(self, thrust, torque):
        """Collective thrust [N] + body torque [N m] from an external attitude controller."""
        self.last_thrust_cmd = np.broadcast_to(np.asarray(thrust, dtype=float), (self.n,)).copy()
        self.step_motors(self.mix(self.last_thrust_cmd, np.atleast_2d(torque)))

    def step_motors(self, rotor_cmd):
        """Raw rotor-speed command [rad/s], clipped to the per-motor limits."""
        self.rotor_cmd = np.clip(np.atleast_2d(rotor_cmd), self.w_min, self.w_max)
        self._integrate(self.rotor_cmd)

    # ── dynamics ──
    def _deriv(self, pos, vel, quat, omega, rotor, wind):
        p = self.p
        R = quat_to_rot(quat)
        F = p.kf * rotor ** 2
        wrench = F @ p.alloc.T                      # (N,4): T, tau_x, tau_y, tau_z
        v_rel_b = np.einsum('nji,nj->ni', R, vel - wind)
        drag_w = -np.einsum('nij,nj->ni', R, p.drag * v_rel_b)
        acc = (R[:, :, 2] * wrench[:, :1] + drag_w + self.ext_force) / p.mass
        acc[:, 2] -= G
        qdot = 0.5 * quat_mul(quat, np.column_stack([np.zeros(len(omega)), omega]))
        J = p.inertia
        wdot = (wrench[:, 1:] - np.cross(omega, J * omega)) / J
        return vel, acc, qdot, wdot

    def _rotor_at(self, r0, rc, s):
        return rc + (r0 - rc) * np.exp(-s / self.p.tau_motor)

    def _integrate(self, rotor_cmd):
        h = self.dt / self.n_substeps
        for _ in range(self.n_substeps):
            if self.wind is None:
                wind = np.zeros((self.n, 3))
            elif callable(self.wind):
                wind = np.asarray(self.wind(self.t, self.pos, h), dtype=float)
            else:
                wind = np.broadcast_to(np.asarray(self.wind, dtype=float), (self.n, 3))
            self.wind_now = np.array(wind)
            r0 = self.rotor
            rm = self._rotor_at(r0, rotor_cmd, 0.5 * h)
            r1 = self._rotor_at(r0, rotor_cmd, h)
            x0 = (self.pos, self.vel, self.quat, self.omega)
            k1 = self._deriv(*x0, r0, wind)
            x2 = tuple(a + 0.5 * h * b for a, b in zip(x0, k1))
            k2 = self._deriv(*x2, rm, wind)
            x3 = tuple(a + 0.5 * h * b for a, b in zip(x0, k2))
            k3 = self._deriv(*x3, rm, wind)
            x4 = tuple(a + h * b for a, b in zip(x0, k3))
            k4 = self._deriv(*x4, r1, wind)
            new = [a + h / 6.0 * (b1 + 2 * b2 + 2 * b3 + b4)
                   for a, b1, b2, b3, b4 in zip(x0, k1, k2, k3, k4)]
            self.pos, self.vel, q, self.omega = new
            self.quat = q / np.linalg.norm(q, axis=1, keepdims=True)
            # Simpson's rule on shaft power (exact for the analytic rotor profile to O(h^4))
            self.energy += h / 6.0 * (self.power_shaft(r0) + 4 * self.power_shaft(rm)
                                      + self.power_shaft(r1))
            self.rotor = r1
            self.t += h
            if self.ground_z is not None:
                below = self.pos[:, 2] < self.ground_z
                if np.any(below):
                    self.pos[below, 2] = self.ground_z
                    self.vel[below] = 0.0
                    self.omega[below] = 0.0
        if self.record:
            self._record()

    def _record(self):
        L = self.log
        L['t'].append(self.t)
        L['pos'].append(self.pos.copy())
        L['vel'].append(self.vel.copy())
        L['quat'].append(self.quat.copy())
        L['omega'].append(self.omega.copy())
        L['rotor'].append(self.rotor.copy())
        L['thrust'].append(self.thrust_total())
        L['power'].append(self.power_shaft())
        L['rate_sp'].append(self.last_rate_sp.copy())
        L['wind'].append(self.wind_now.copy())

    def get_log(self):
        """Dict of arrays with shape (T, N, ...)."""
        return {k: np.array(v) for k, v in self.log.items()}

    def get_trajectory(self):
        """(T, 3) for a single drone, (T, N, 3) for a batch (DroneBase compatible)."""
        tr = np.array(self.log['pos'])
        return tr[:, 0] if self.n == 1 else tr

    @property
    def trajectory(self):
        return self.log['pos']
