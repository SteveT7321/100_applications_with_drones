# S084 Extreme Weather Wind Resistance Test

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: A 1 kg validation quadrotor flies 100 m along $x$ at 1 m/s and a height of 10 m. A crosswind blows in $+y$: a 12 m/s mean plus Dryden turbulence (σ 4 m/s lateral, 2 m/s vertical). The wind pushes the drone through quadratic drag $\mathbf{F}_w = \tfrac12\rho C_D A\,(\mathbf{V}_w - \mathbf{v})\odot|\mathbf{V}_w - \mathbf{v}|$. The mission **fails** at the first instant that $|y| > 5$ m or that roll or pitch exceeds 40°. It succeeds when $x \ge 100$ m within 120 s.

**Controllers compared** (same wind realisation per seed, common random numbers):
1. **PID only**: the cascaded controller of the base model (position P, velocity PI, attitude P, body-rate PI), tilt limited to $\theta_{lim}$ = 30°, no wind knowledge.
2. **LQR**: the card's LQR on a 10-state linearisation about hover. It commands thrust and roll/pitch torques directly; there is no wind estimate.
3. **LQR + RLS** (proposed): the same LQR, linearised around an equilibrium shifted by the RLS estimate $\hat{\mathbf{w}}$ of the wind acceleration. The shift supplies a feed-forward tilt and thrust that cancel $-m\hat{\mathbf{w}}$.

**Plant**: the 6-DOF rigid body in [`src/base/rigid_body_drone.py`](../../../src/base/rigid_body_drone.py) (quaternion attitude, motor lag, mixer). It uses the `fpv_5inch` rotors with the card's mass of 1.0 kg and roll/pitch inertia of 0.01 kg m². The quadratic wind drag is applied as an external force. The headline run is seed 42; a 20-seed Monte Carlo (seeds 42–61) gives the statistics.

---

## Mathematical Model Summary

### Dryden turbulence

Lateral gust: $H(s) = \sigma_w\sqrt{\tau}\,\dfrac{1 + \sqrt3\,\tau s}{(1 + \tau s)^2}$ with $\tau = L_w/V$ = 16.67 s. It is driven by unit-intensity white noise ($n_k = \xi_k/\sqrt{\Delta t}$) and discretised exactly (zero-order hold) at 5 ms. Each series starts from the stationary distribution, so the output standard deviation is $\sigma_w$. The vertical gust uses the same filter with σ = 2 m/s and $L$ = 100 m. Total wind: $\mathbf{V}_w = (0,\ 12 + W_y,\ W_z)$.

### Aerodynamic force

$$\mathbf{F}_w = \tfrac12\rho C_D A\,(\mathbf{V}_w - \mathbf{v}) \odot |\mathbf{V}_w - \mathbf{v}|,\qquad \tfrac12\rho C_D A = 0.0147\ \text{kg/m}$$

The mean wind gives 2.117 N, which needs a 12.2° tilt to cancel. A 20 m/s gust gives 5.88 N and needs 30.9°, already above the 30° clamp.

### LQR (card)

State $(p_x, p_y, p_z, v_x, v_y, v_z, \phi, \theta, \dot\phi, \dot\theta)$, input $(\delta T, \tau_\phi, \tau_\theta)$, with $\dot v_x = g\theta$, $\dot v_y = -g\phi$, $\dot v_z = \delta T/m$ and $\ddot\phi = \tau_\phi/I_{xx}$. The gain $K$ comes from the continuous algebraic Riccati equation (CARE) with $Q$ = diag(1, 20, 20, 0.5, 4, 4, 8, 8, 1, 1) and $R$ = diag(0.1, 1, 1). The applied thrust is $T = (T_0 + \delta T)/(\cos\phi\cos\theta)$, with $T_0 = mg$ (LQR) or $T_0 = \|\mathbf{f}_{ff}\|$ (LQR + RLS). Yaw is held by a separate PD loop (16, 8). Closed-loop eigenvalue real parts run from −1.698 to −99.96. For the linear model under the mean wind the steady-state offset is $y$ = 0.436 m at a roll of 12.36°: LQR has no integral action.

### RLS wind estimator (card, 50 Hz)

$$\hat{\mathbf{w}}_{k+1} = \hat{\mathbf{w}}_k + \Gamma\bigl(\mathbf{a}_{meas,k} - \mathbf{a}_{model,k} - \hat{\mathbf{w}}_k\bigr),\qquad \Gamma = 0.05\ \ (\text{time constant } 0.40\ \text{s})$$

Here $\mathbf{a}_{meas} = \Delta\mathbf{v}/\Delta t$ plus accelerometer noise of 0.1 m/s², and $\mathbf{a}_{model} = R\mathbf{e}_3 T/m - g\mathbf{e}_3$ uses the actual rotor thrust. The feed-forward is $\mathbf{f}_{ff} = m(g\mathbf{e}_3 - \hat{\mathbf{w}})$. It gives the reference attitude $\phi_{ff} = \arcsin(-f_y/\|\mathbf{f}\|)$ and $\theta_{ff} = \operatorname{atan2}(f_x, f_z)$, clipped to ±30°, and the thrust $\|\mathbf{f}_{ff}\|$.

### Attitude protection

PID: the commanded thrust vector is tilt-limited to 30°. LQR: if $|\phi| > 30°$ and the roll torque pushes $|\phi|$ further out, that torque is scaled by $30°/|\phi|$. Pitch is handled the same way.

### Metrics

- RMS and peak $|y|$ and peak attitude up to mission end or failure. The peaks are taken at the 200 Hz control rate, RMS from the 50 Hz log.
- RLS convergence: the first time $|\hat w_y - \bar W_{eff}| < 0.5$ m/s², where $\bar W_{eff}$ = 2.117 m/s². The RMS error of $\hat w_y$ against the true instantaneous wind acceleration after 2 s is also reported.
- Shaft energy until the end of the mission.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Mass / $I_{xx}$, $I_{yy}$ / $I_{zz}$ | 1.0 kg / 0.01 kg m² (card) / 0.018 kg m² (estimate) |
| Rotors | `fpv_5inch` preset: 9 N per motor, T/W = 3.67, motor time constant 15 ms |
| Mean wind / σ lateral / $L_w$ / $V$ | 12 m/s / 4 m/s / 200 m / 12 m/s (card) |
| Vertical gust σ / $L$ | 2 m/s / 100 m (card code) |
| $\rho$ / $C_D$ / $A_{ref}$ | 1.225 kg/m³ / 0.6 / 0.04 m² (card) |
| Reference | $x = t$ (1 m/s) to 100 m, $y$ = 0, $z$ = 10 m |
| Fail limits | $|y| > 5$ m, $\max(|\phi|, |\theta|) > 40°$ |
| Attitude clamp $\theta_{lim}$ | 30° |
| LQR $Q$ / $R$ | diag(1, 20, 20, 0.5, 4, 4, 8, 8, 1, 1) / diag(0.1, 1, 1) (card) |
| RLS gain Γ / update rate | 0.05 / 50 Hz (card Δt = 0.02 s) |
| Accelerometer noise | 0.1 m/s² (added) |
| PID gains (base model, `fpv_5inch`) | position P (2, 2, 2.5); velocity PI (4, 4, 5) / (2, 2, 2.5), integrating only while the error is below 0.3 m/s; attitude P (12, 12, 4); rate PI (40, 40, 20) / (30, 30, 8) |
| Control / physics step | 5 ms (200 Hz), RK4 |
| Mission timeout | 120 s |
| Monte Carlo | 20 seeds (42–61), same wind for all three controllers |

---

## Implementation

```
src/base/rigid_body_drone.py                 # 6-DOF rigid-body model (shared)
src/05_special_entertainment/s084_wind_endurance.py
```

```bash
conda activate drones
python src/05_special_entertainment/s084_wind_endurance.py
```

Runtime ≈ 50–180 s, depending on machine load (measured while about 10 other simulations shared the 12-thread machine).

---

## Results

**Dryden check**: over a 2000 s sample, the card's gain $\sigma\sqrt{2V/(\pi L)}$ gives a gust standard deviation of only 0.184 m/s (theory 0.191 m/s). The corrected gain $\sigma\sqrt{\tau}$ gives 3.842 m/s for σ = 4 m/s (card corrected). The 100 s missions are short compared with the 16.7 s correlation time, so individual realisations vary widely: the per-seed standard deviation of $W_y$ is 2.53–5.22 m/s. Seed 42 is the harshest one (mean 11.73, std 5.22, maximum 28.76 m/s).

**Headline seed 42:**

| Controller | Result | End | RMS $\|y\|$ | Peak $\|y\|$ | Peak attitude | Mean attitude | RMS $z$ error | Shaft energy | RLS convergence | RLS RMS error |
|------------|--------|----:|-----:|-----:|-----:|-----:|-----:|-----:|-----:|-----:|
| PID only | FAIL: $\|y\|$ = 5.01 m | 90.50 s | 0.521 m | 5.007 m | 30.00° | 10.62° | 0.005 m | 13700 J | — | — |
| LQR | FAIL: attitude 40.0° | 88.80 s | 0.423 m | 1.270 m | 40.01° | 10.30° | 0.008 m | 13403 J | — | — |
| LQR + RLS | FAIL: attitude 40.0° | 88.85 s | **0.045 m** | **0.220 m** | 40.01° | 10.34° | 0.028 m | 13423 J | 1.82 s | 0.260 m/s² |

**Monte Carlo, 20 seeds:**

| Controller | Success | RMS $\|y\|$ | Peak $\|y\|$ | Peak attitude | Energy (successful missions) |
|------------|:-------:|-----:|-----:|-----:|-----:|
| PID only | 15/20 | 0.428 ± 0.334 m | 2.338 ± 2.052 m | 28.82 ± 2.10° | 15274 J |
| LQR | 18/20 | 0.494 ± 0.089 m | 1.073 ± 0.194 m | 33.04 ± 5.63° | 15310 J |
| **LQR + RLS** | **18/20** | **0.047 ± 0.007 m** | **0.196 ± 0.065 m** | 33.37 ± 5.30° | 15315 J |

Failures:
- **PID only**: seeds 42, 51, 52, 55 and 58, all lateral ($|y|$ > 5 m at 90.5, 39.5, 74.3, 45.6 and 39.2 s).
- **LQR** and **LQR + RLS**: seeds 42 and 58, both on attitude (> 40° at 88.8/88.9 s and 38.5 s).

Head-to-head on the same seeds: LQR + RLS has the lower RMS $|y|$ on 20/20 seeds against LQR and on 20/20 against PID. LQR beats PID on only 8/20.

Peak attitude over successful missions: PID 28.42 ± 2.29° (max 30.00°), LQR 32.27 ± 5.40° (max 39.65°), LQR + RLS 32.64 ± 5.08° (max 39.36°). Only 2/15, 2/18 and 2/18 of these runs stay below 25°.

RLS: first within 0.5 m/s² of the mean-wind acceleration after 3.27 ± 6.04 s (max 25.70 s; never on 0 seeds). RMS error against the true instantaneous wind acceleration: 0.268 ± 0.031 m/s².

**Key findings**:
- **RLS feed-forward cuts the lateral error by a factor of 10.** It reduces RMS $|y|$ from 0.494 m (LQR) to 0.047 m and peak $|y|$ from 1.073 to 0.196 m, on 20/20 seeds. The estimator follows the gusts themselves, not only the mean: its RMS error against the instantaneous wind acceleration is 0.268 m/s², with a 0.40 s time constant against a 16.7 s gust correlation time. The card's targets for LQR + RLS (RMS < 0.5 m, peak < 1 m, convergence < 15 s on average) hold.
- **The PID does not drift as the card expects.** The base model's velocity loop has integral action, which removes the steady wind offset. The PID's RMS $|y|$ (0.428 m) is about the same as the LQR's (0.494 m), and it beats LQR on 8/20 seeds. It fails on 5/20 seeds, never on attitude. When a gust needs more than the 30° tilt clamp, the PID holds the clamp and is blown out past 5 m, as on seed 42: $|y|$ goes from about 0.1 m to 5 m within about 6 s at the end of the run (read from the figure). The card's "PID: RMS > 5 m, FAIL" holds only on those 5 seeds.
- **The LQR's offset is the one the linear model predicts.** The linear model predicts 0.436 m under the mean wind, and the flown RMS is 0.494 m. The card's expected 1–3 m RMS and 2–4 m peak do not appear.
- **Two seeds cannot be flown by any controller.** On seeds 42 and 58 the gusts need more tilt than the airframe may use. A 20 m/s gust already needs 30.9°, and seed 42 reaches 28.76 m/s. The PID gives up position (it is tilt-limited) and fails on $|y|$. Both LQR variants keep position and exceed 40° instead. The "attitude protection" only scales torques that push past 30°, so the position loop still drives the tilt up. This trade-off is set by the fail limits, not by the estimator.
- **Peak attitude does not go down with RLS** (33.37 ± 5.30° against 33.04 ± 5.63°). The feed-forward produces the same tilt that the wind demands; it just produces it earlier. The card's "< 25°" target holds on only 2/18 successful runs, because the gusts themselves need 25–40° (the mean wind alone needs 12.2°).
- **Energy is the same for all three** (15274–15315 J per successful mission of about 100 s, i.e. roughly 150 W). The thrust needed to hold against the wind dominates; how well the drone tracks hardly matters. The values are estimates: see the Sim2Real notes.

### 3D Trajectory

The three paths on seed 42 (reference = green dotted line, grey dashes = the 5 m fail bound, × = failure):

![3D trajectory](./trajectory_3d.png)

### Lateral Deviation and Roll

Top: $y(t)$ with the ±5 m fail limits. Bottom: roll angle with the 30° clamp (dotted) and the 40° fail limit (dashed). The final gust (from about 82 s) drives the PID out sideways and both LQR variants past 40°:

![Lateral deviation and attitude](./lateral_and_attitude.png)

### RLS Convergence

$\hat w_y$ (red) against the true lateral wind acceleration $F_{w,y}/m$ (grey), the mean-wind acceleration of 2.12 m/s² and the ±0.5 m/s² band:

![RLS convergence](./rls_convergence.png)

### Dryden Gust Realisation

$W_y^{total}$ and $W_z^{turb}$ for seed 42, with the $\bar W_y \pm 2\sigma_w$ lines:

![Dryden gusts](./dryden_gusts.png)

### Performance Metrics

Bars: mean ± std over 20 seeds; triangles: seed 42. Right: success counts and mean peak attitude (the dashed line is 40°):

![Metrics](./metrics_bar.png)

### 3D Path Coloured by Deviation

LQR + RLS on seed 42, coloured by $|y|$, with the wind $(W_y, W_z)$ drawn every 5 s at 1/6 scale:

![Path deviation 3D](./path_deviation_3d.png)

### Animation

Top-down view of all three controllers on seed 42. The side bar shows the lateral wind speed. 92 frames at 12 fps:

![Animation](./animation.gif)

### Deviations from the scenario card

| Card | Implementation | Reason |
|------|----------------|--------|
| Linearised 10-state plant integrated by Euler | 6-DOF rigid body (motors, mixer, quaternion attitude); the card's linear model is used only to design the LQR | Hardware-oriented scenario; the controller is tested on dynamics it was not designed on. |
| Dryden gain $\sigma\sqrt{2V/(\pi L)}$ | $\sigma\sqrt{L/V}$ (card corrected) | The card's gain gives a std of 0.184 m/s instead of 4 m/s (printed check). |
| Vertical gust "same filter with scale $L_w$" (text) | σ = 2 m/s, $L$ = 100 m (card code) | The card's text and code disagree; the code values are more realistic near the ground. |
| LQR thrust command $u_0 = -K e$ used as total thrust (card code) | $T = (mg + \delta T)/(\cos\phi\cos\theta)$ | The card code leaves out the hover thrust, so the drone would start by falling. |
| PID maps acceleration to "pseudo torques" (×5) | Base-model cascaded PID (position P, velocity PI, attitude, rate PI) with a 30° tilt limit | A standard, reasonable PID baseline (X2). The card's map has no physical meaning. |
| Feed-forward $u_{ff} = -m\hat{w}$ mapped to torques (×5) | LQR linearised around the equilibrium tilt and thrust that cancel $-m\hat{\mathbf{w}}$ | A torque does not cancel a force; the equilibrium shift does. |
| $a_{meas}$ = model + true force (card code) | Finite-difference velocity plus 0.1 m/s² noise, against a model using the actual rotor thrust | The card code hands the estimator the exact answer. |
| Attitude clamp scales the torque whenever $\|\phi\| > 30°$ | Scales only a torque that pushes $\|\phi\|$ further out | Scaling a restoring torque would slow the recovery. |
| One run, Δt = 0.02 s | 20-seed Monte Carlo with common wind, control at 5 ms, RLS at the card's 0.02 s | X2; the rigid-body inner loops need a faster step. |
| RLS convergence "within 0.5 m/s of the mean wind" | Within 0.5 m/s² of the mean-wind acceleration (the card's metric table) | $\hat w$ is an acceleration. |
| Peaks taken from the sampled log | Peaks at the 200 Hz control rate | The 50 Hz log can miss the failing sample. |

---

## Sim2Real Notes

**Model assumptions.** Wind acts only through the card's quadratic drag on the airframe: one $C_D A$, isotropic, independent of attitude. Rotor-induced drag (which grows with airspeed and rotor speed), blade flapping and the change of thrust with inflow in a 12–29 m/s crosswind are not modelled. Above about 15 m/s these effects change both the force and the available thrust. The motors are the `fpv_5inch` estimates (9 N each, T/W 3.67 at 1 kg). The energy figures use $k_m\omega^3$ with an assumed hover figure of merit of 0.6, so they are order-of-magnitude only. The estimator gets velocity differences plus white noise; there is no GPS/IMU fusion, latency or accelerometer vibration.

**What must be measured on the real airframe.**
- **Thrust stand**: $k_f$, maximum thrust (it sets the tilt available in gusts) and the thrust loss at high inflow speed (a wind-tunnel sweep, or a moving thrust stand).
- **Motor time constant** (DShot telemetry step response).
- **Inertia** (trifilar pendulum); the card's 0.01 kg m² is large for 1 kg.
- **Airframe drag**: $C_D A$ against airspeed and attitude, in a wind tunnel or from free-flight coast-downs. The whole wind force scales with this one number.
- **Accelerometer noise and vibration spectrum** in hover. This sets how far Γ can be raised before $\hat w$ becomes noisy.

**Path to PX4 / Betaflight SITL.**
1. **PX4** already contains the PID baseline (its multicopter position controller) and an integrator that plays the RLS role. The fair test is PX4 SITL with Gazebo's wind plugin, configured with the Dryden parameters above (σ 4 m/s, $L$ 200 m). Fly the same 100 m mission through MAVLink and compare `vehicle_local_position` against this model.
2. Add the LQR + RLS as a custom module that publishes `VehicleAttitudeSetpoint` and thrust, with the estimator fed by `vehicle_acceleration`. PX4's rate controller then replaces the torque-level LQR inner loop, so re-design the LQR on the rate-loop closed dynamics.
3. **Betaflight** is not a natural target: it has no position loop. Run the outer loops on a companion computer and send angle-mode commands through MSP.
4. Set the fail limits as PX4 failsafes (`MPC_TILTMAX_AIR` = 30°, geofence 5 m) and confirm that the gust levels at which this model fails match flight logs (seed 42 peaks at 28.76 m/s).

---

## Extensions

1. **Variable-gain RLS** (card extension 1): raise Γ when the innovation is large. The current Γ = 0.05 already tracks gusts with an RMS error of 0.268 m/s².
2. **Joint state and wind EKF** with GPS-rate position updates and IMU-rate prediction.
3. **Tilt-limit trade study**: allow up to 35–40° in gusts, and compare mission success with the fail rule kept at 40°.
4. **Rotor-drag model**: add $-k_d\,\omega\,\mathbf{v}_{rel,\perp}$ per rotor and check whether the RLS absorbs it.
5. **Larger Monte Carlo** (card extension 4: 500 seeds) for a certification-style success rate.

---

## Related Scenarios

- Prerequisites: [S058 Typhoon Eye Probing](../../../scenarios/03_environmental_sar/S058_typhoon.md)
- Algorithmic cross-reference: [S001 Basic Intercept](../../../scenarios/01_pursuit_evasion/S001_basic_intercept.md), [S024 Wind Compensation](../../../scenarios/02_logistics_delivery/S024_wind_compensation.md)
- Same rigid-body model: [S082 FPV Racing Circuit](../../../scenarios/05_special_entertainment/S082_fpv_racing.md), [S090 Racing Drone Optimal Path](../../../scenarios/05_special_entertainment/S090_racing_optimal.md), [S091 Aerial Acrobatic Maneuvers](../../../scenarios/05_special_entertainment/S091_acrobatics.md)
