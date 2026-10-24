"""
Self-test for src/base/rigid_body_drone.py (plain asserts, runnable as a script).

Usage:
    python src/base/test_rigid_body_drone.py
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from rigid_body_drone import (RigidBodyDrone, GustWind, make_params, quat_to_rot,  # noqa: E402
                              tilt_angle, momentum_theory_power, G)

PRESETS = ['crazyflie', 'fpv_5inch']
DT = 0.002


def run(drone, n_steps, fn):
    for k in range(n_steps):
        fn(drone, k)
    return drone.get_log()


def test_hover_thrust():
    for pr in PRESETS:
        d = RigidBodyDrone([0, 0, 1], preset=pr, dt=DT)
        # start off-equilibrium: rotors at 80 % of hover speed, sinking at 0.5 m/s
        d.set_state(rotor=0.8 * d.p.w_hover, vel=[0.0, 0.0, -0.5])
        L = run(d, 2500, lambda dr, k: dr.step_position([0, 0, 1]))
        w = d.p.weight
        t_last = L['thrust'][-500:, 0]
        rel = np.abs(t_last.mean() - w) / w
        rel_max = np.max(np.abs(t_last - w)) / w
        assert rel < 0.01 and rel_max < 0.01, (pr, rel, rel_max)
        drift = np.linalg.norm(L['pos'][-1, 0] - [0, 0, 1])
        assert drift < 1e-3, (pr, drift)
        w_h = np.sqrt(w / (4 * d.p.kf))
        print(f'  [{pr}] hover: thrust/weight = {t_last.mean() / w:.6f} '
              f'(max dev {100 * rel_max:.4f} %), rotor {w_h:.1f} rad/s '
              f'({w_h * 60 / (2 * np.pi):.0f} RPM), T/W max {d.p.t2w:.2f}, drift {drift:.2e} m')


def test_step_response():
    tgt = np.array([1.0, 1.0, 2.0])
    for pr in PRESETS:
        d = RigidBodyDrone([0, 0, 1], preset=pr, dt=DT)
        L = run(d, 3000, lambda dr, k: dr.step_position(tgt))
        pos, t = L['pos'][:, 0], L['t']
        err = np.linalg.norm(pos - tgt, axis=1)
        assert np.all(np.isfinite(pos))
        assert err[-1] < 0.01, (pr, err[-1])
        over = max(pos[:, 0].max() - 1.0, pos[:, 1].max() - 1.0, pos[:, 2].max() - 2.0)
        assert over < 0.25, (pr, over)
        t_settle = t[np.where(err > 0.05)[0][-1] + 1]
        assert t_settle < 4.0, (pr, t_settle)
        # no divergence: error after settling never grows again
        assert np.max(err[t >= t_settle]) <= 0.05 + 1e-12
        print(f'  [{pr}] step (1,1,1) m: settle(5 cm) {t_settle:.2f} s, overshoot '
              f'{100 * over:.1f} %, final error {1000 * err[-1]:.2f} mm, max tilt '
              f'{np.degrees(tilt_angle(L["quat"][:, 0]).max()):.1f} deg')


def test_velocity_command():
    for pr in PRESETS:
        d = RigidBodyDrone([0, 0, 1], preset=pr, dt=DT)
        L = run(d, 2000, lambda dr, k: dr.step([1.0, 0.0, 0.0]))
        v = L['vel'][-1, 0]
        assert abs(v[0] - 1.0) < 0.02 and np.all(np.abs(v[1:]) < 0.02), (pr, v)
        print(f'  [{pr}] velocity command 1 m/s: v = ({v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f}) m/s')


def test_mixer_roundtrip():
    rng = np.random.default_rng(1)
    for pr in PRESETS:
        d = RigidBodyDrone(np.zeros((50, 3)), preset=pr, dt=DT)
        T = d.p.weight * rng.uniform(0.8, 1.2, 50)
        tau_scale = d.p.weight * d.p.arm * 0.02
        tau = tau_scale * rng.uniform(-1, 1, (50, 3)) * [1, 1, 0.1]
        w = d.mix(T, tau)
        wrench = (d.p.kf * w ** 2) @ d.p.alloc.T
        assert d.n_sat == 0
        assert np.allclose(wrench, np.column_stack([T, tau]), rtol=1e-9, atol=1e-12), pr
    print('  mixer: wrench -> rotor speeds -> wrench round trip exact for 50 random wrenches')


def test_single_motor_saturation():
    for pr in PRESETS:
        for frac in (1.1, 0.8):
            d = RigidBodyDrone([0, 0, 2], preset=pr, dt=DT)
            # motor 0 can only produce frac x its hover share of thrust
            d.set_motor_limits(0, w_max=d.p.w_hover * np.sqrt(frac))
            tgt = np.array([2.0, 0.0, 5.0])
            L = run(d, 2500, lambda dr, k: dr.step_position(tgt))
            for k in ('pos', 'vel', 'quat', 'omega', 'rotor'):
                assert np.all(np.isfinite(L[k])), (pr, frac, k)
            r = L['rotor'][1:, 0]          # entry 0 was logged before the limit change
            assert np.all(r <= d.w_max[0] + 1e-9) and np.all(r >= d.w_min[0] - 1e-9)
            assert np.max(np.abs(L['pos'][:, 0])) < 30.0
            assert np.max(np.abs(L['omega'][:, 0, :2])) < 20.0     # roll/pitch stay bounded
            assert d.n_sat > 0
            err = np.linalg.norm(L['pos'][-1, 0] - tgt)
            print(f'  [{pr}] motor 0 limited to {frac:.1f} x hover thrust: mixer saturated in '
                  f'{d.n_sat}/{len(L["t"]) - 1} steps, final error {err:.2f} m, max tilt '
                  f'{np.degrees(tilt_angle(L["quat"][:, 0]).max()):.1f} deg, max |yaw rate| '
                  f'{np.max(np.abs(L["omega"][:, 0, 2])):.2f} rad/s, z range '
                  f'[{L["pos"][:, 0, 2].min():.2f}, {L["pos"][:, 0, 2].max():.2f}] m')
            if frac > 1.0:
                assert err < 0.1, (pr, err)   # still flyable, only slower
            else:
                # Zero roll/pitch/yaw torque forces F0 = F1 = F2 = F3, so the mixer can deliver
                # at most 4 x the weak motor's thrust = frac x weight: the drone keeps a level,
                # non-spinning attitude and sinks instead of flipping or spinning up.
                t_end = L['thrust'][-1, 0] / d.p.weight
                assert abs(t_end - frac) < 0.02, (pr, t_end)
                assert np.degrees(tilt_angle(L['quat'][:, 0]).max()) < 30.0
                assert np.max(np.abs(L['omega'][:, 0, 2])) < 1.0


def test_quaternion_norm():
    for pr in PRESETS:
        d = RigidBodyDrone([0, 0, 10], preset=pr, dt=DT)
        rates = np.array([6.0, -4.0, 2.0])
        L = run(d, 1500, lambda dr, k: dr.step_rates(dr.p.weight, rates * np.sin(0.01 * k)))
        nrm = np.abs(np.linalg.norm(L['quat'][:, 0], axis=1) - 1.0)
        assert nrm.max() < 1e-12, (pr, nrm.max())
        # torque-free spin about a principal axis: 2 pi rad/s for 1 s returns to identity
        d = RigidBodyDrone([0, 0, 10], preset=pr, dt=DT)
        d.set_state(omega=[2 * np.pi, 0, 0])
        run(d, 500, lambda dr, k: dr.step_motors(np.full((1, 4), dr.p.w_hover)))
        qv = d.quat[0]
        ang_err = np.degrees(2 * np.arctan2(np.linalg.norm(qv[1:]), abs(qv[0])))
        assert ang_err < 1e-3, (pr, ang_err)
        print(f'  [{pr}] quaternion: max | |q| - 1 | = {nrm.max():.1e} during acro rates; '
              f'2 pi spin closes to {ang_err:.1e} deg')


def test_energy():
    for pr in PRESETS:
        # (a) free fall without drag, rotors off: kinetic + potential energy conserved
        p = make_params(pr, drag=np.zeros(3), w_min=0.0)
        d = RigidBodyDrone([0, 0, 50], preset=pr, params=p, dt=DT)
        d.set_state(vel=[1.0, 0.0, 2.0], rotor=np.zeros(4))
        e0 = 0.5 * p.mass * np.sum(d.vel ** 2) + p.mass * G * d.pos[0, 2]
        run(d, 500, lambda dr, k: dr.step_motors(np.zeros((1, 4))))
        e1 = 0.5 * p.mass * np.sum(d.vel ** 2) + p.mass * G * d.pos[0, 2]
        rel = abs(e1 - e0) / e0
        assert rel < 1e-9, (pr, rel)
        # (b) work-energy theorem in a climb without drag: dE = integral of T (z_b . v)
        d = RigidBodyDrone([0, 0, 1], preset=pr, params=p, dt=DT)
        L = run(d, 1500, lambda dr, k: dr.step_position([1.0, -1.0, 3.0]))
        zb = quat_to_rot(L['quat'][:, 0])[:, :, 2]
        p_thr = L['thrust'][:, 0] * np.sum(zb * L['vel'][:, 0], axis=1)
        work = np.sum(0.5 * (p_thr[1:] + p_thr[:-1]) * np.diff(L['t']))
        de = (0.5 * p.mass * np.sum(L['vel'][-1, 0] ** 2) + p.mass * G * L['pos'][-1, 0, 2]) - \
             (0.5 * p.mass * np.sum(L['vel'][0, 0] ** 2) + p.mass * G * L['pos'][0, 0, 2])
        rel_w = abs(work - de) / abs(de)
        assert rel_w < 0.01, (pr, work, de)
        # (c) hover shaft power above the momentum-theory ideal (figure of merit <= 1)
        q = make_params(pr)
        t_rotor = q.weight / 4
        p_ideal = 4 * momentum_theory_power(t_rotor, q.prop_radius)
        p_shaft = 4 * q.km * q.w_hover ** 3
        fm = p_ideal / p_shaft
        assert 0.05 < fm <= 1.0, (pr, fm)
        # (d) integrated shaft energy (Simpson inside the step) matches the logged power
        e_log = np.sum(0.5 * (L['power'][1:, 0] + L['power'][:-1, 0]) * np.diff(L['t']))
        rel_e = abs(d.energy[0] - e_log) / e_log
        assert rel_e < 0.01, (pr, d.energy[0], e_log)
        print(f'  [{pr}] energy: free-fall drift {rel:.1e}; climb work {work:.5f} J vs dE '
              f'{de:.5f} J ({100 * rel_w:.3f} %); hover shaft power {p_shaft:.2f} W, ideal '
              f'{p_ideal:.2f} W, FM {fm:.2f}, electrical {p_shaft / q.motor_eff:.2f} W; climb shaft '
              f'energy {d.energy[0]:.2f} J ({100 * rel_e:.3f} % vs logged power)')


def test_wind_noise_determinism():
    def sim(seed):
        wind = GustWind(mean=[3.0, 0.0, 0.0], sigma=1.0, tau_c=1.0, seed=seed)
        d = RigidBodyDrone([0, 0, 1], preset='crazyflie', dt=DT, wind=wind, seed=seed,
                           imu_noise=dict(gyro_std=0.01, gyro_bias_std=0.002))
        return run(d, 2500, lambda dr, k: dr.step_position([0, 0, 1]))
    a, b, c = sim(3), sim(3), sim(4)
    for k in a:
        assert np.array_equal(a[k], b[k]), k
    assert not np.array_equal(a['pos'], c['pos'])
    dev = np.linalg.norm(a['pos'][1000:, 0] - [0, 0, 1], axis=1).max()
    assert dev < 0.3, dev
    print(f'  wind 3 m/s + 1 m/s gusts + gyro noise: identical for equal seeds, different for '
          f'other seed; max hold error after 2 s {100 * dev:.1f} cm')


def main():
    tests = [test_hover_thrust, test_step_response, test_velocity_command, test_mixer_roundtrip,
             test_single_motor_saturation, test_quaternion_norm, test_energy,
             test_wind_noise_determinism]
    for t in tests:
        print(t.__name__)
        t()
    print(f'ALL {len(tests)} TESTS PASSED')


if __name__ == '__main__':
    main()
