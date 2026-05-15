# S095 3D Upgrade — Water Surface Takeoff/Landing

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S095 original](../S095_water_landing.md)

---

## What Changes in 3D

The original S095 model treats the boat deck as a purely heaving (vertical-only) surface: both wave components act only on $z_{deck}$, the horizontal deck position $(x_d, y_d) = (8, 0)$ m is fixed for all time, and the drone's approach trajectory is constrained to a vertical-plane descent directly above the deck. Three structural limitations emerge in practice:

1. **Surface tilt is ignored.** Real ocean swells tilt the deck, rotating it by a pitch angle $\Theta_{deck}$ and a roll angle $\Phi_{deck}$ that change the effective landing normal and impose a lateral thrust demand at touchdown.
2. **Approach geometry is unoptimised.** The timed descent always arrives from directly above; there is no 3D approach angle that minimises the time the drone spends traversing the most hazardous low-altitude zone near the heaving surface.
3. **Wave phase synchronisation is purely temporal.** With a tilting surface, the drone must also match the instantaneous normal vector at touchdown, not only the instantaneous height.

This 3D upgrade adds: a full 3D wave surface model with pitch and roll kinematics, an attitude-compensation thrust law that corrects for deck tilt at touchdown, and a 3D approach angle optimiser that selects the inbound azimuth and glide slope that minimise exposure time to the tilt-disturbance zone while maximising alignment with the wave phase at contact.

---

## Problem Definition

**Setup**: A drone must land on a $1 \times 1$ m boat deck subject to two-frequency ocean swell. The deck undergoes coupled heave, pitch, and roll:

$$z_{deck}(t) = z_0 + A_1 \sin\!\left(\frac{2\pi t}{T_1} + \varphi_1\right)
              + A_2 \sin\!\left(\frac{2\pi t}{T_2} + \varphi_2\right)$$

$$\Phi_{deck}(t) = \Phi_1 \sin\!\left(\frac{2\pi t}{T_1} + \varphi_1 + \delta_\Phi\right)
                + \Phi_2 \sin\!\left(\frac{2\pi t}{T_2} + \varphi_2 + \delta_\Phi\right)$$

$$\Theta_{deck}(t) = \Theta_1 \cos\!\left(\frac{2\pi t}{T_1} + \varphi_1\right)
                  + \Theta_2 \cos\!\left(\frac{2\pi t}{T_2} + \varphi_2\right)$$

with heave amplitudes $A_1 = 0.40$ m, $A_2 = 0.15$ m, roll amplitudes $\Phi_1 = 6°$, $\Phi_2 = 3°$, pitch amplitudes $\Theta_1 = 4°$, $\Theta_2 = 2°$, and phase offsets $\delta_\Phi = \pi/4$ (roll lags heave by 45°). Mean deck height $z_0 = 2.0$ m. The deck centre follows a slow horizontal surge $x_d(t) = 8.0 + 0.2\sin(0.1\pi t)$ m, $y_d(t) = 0.1\sin(0.07\pi t)$ m.

**Roles**:

- **Drone**: single quadrotor; full 3D linearised dynamics (10-state); controlled by a wave-synchronised approach planner and an attitude-compensation thrust law.
- **Deck**: heaving, pitching, and rolling platform with slow horizontal drift; deck normal vector $\hat{\mathbf{n}}_{deck}(t)$ varies continuously.
- **Landing window**: a brief interval where $|\dot{z}_{deck}|$ is small, $|\Phi_{deck}|$ and $|\Theta_{deck}|$ are below threshold $\theta_{tilt,max} = 4°$, and the approach azimuth aligns with the instantaneous deck normal.

**Objective**: Touch down within the deck bounds with all three conditions satisfied simultaneously:

1. Position error: $\|\mathbf{p}_{drone} - \mathbf{p}_{deck}\| \leq r_{land} = 0.30$ m.
2. Relative vertical velocity: $|\dot{z}_{drone} - \dot{z}_{deck}| \leq v_{land,max} = 0.10$ m/s.
3. Attitude misalignment: $\angle(\hat{\mathbf{z}}_{drone},\; \hat{\mathbf{n}}_{deck}) \leq \theta_{tilt,max} = 4°$.

**Comparison strategies** (two compared):

1. **2D-equivalent** — ignores deck tilt and horizontal drift; descends from directly above; baseline from S095.
2. **3D wave-synchronised** — selects 3D approach angle to maximise crest-flat coincidence; applies tilt-compensation torque at descent initiation.

---

## Mathematical Model

### 3D Wave Surface Model

The instantaneous deck surface normal is derived from the deck rotation matrix $R_{deck}$:

$$R_{deck}(t) = R_x\!\left(\Phi_{deck}(t)\right)\, R_y\!\left(\Theta_{deck}(t)\right)$$

where $R_x(\phi)$ and $R_y(\theta)$ are elementary rotation matrices about the x- and y-axes respectively. The deck normal vector is:

$$\hat{\mathbf{n}}_{deck}(t) = R_{deck}(t)\,\hat{\mathbf{z}} =
\begin{pmatrix}
  \sin\Theta_{deck}(t) \\
 -\sin\Phi_{deck}(t)\cos\Theta_{deck}(t) \\
  \cos\Phi_{deck}(t)\cos\Theta_{deck}(t)
\end{pmatrix}$$

The tilt magnitude is:

$$\psi_{tilt}(t) = \arccos\!\left(\hat{\mathbf{n}}_{deck}(t) \cdot \hat{\mathbf{z}}\right)
               = \arccos\!\left(\cos\Phi_{deck}\cos\Theta_{deck}\right)$$

### FFT Wave Prediction — Extended to Tilt

The drone observes $N_{obs} = 64$ samples of $z_{deck}$, $\Phi_{deck}$, and $\Theta_{deck}$ at $f_s = 10$ Hz. Separate FFTs are applied to each channel:

$$\hat{Z}[k] = \sum_{n=0}^{N_{obs}-1} z_{obs}[n]\,e^{-j2\pi kn/N_{obs}}$$

$$\hat{\Phi}[k] = \sum_{n=0}^{N_{obs}-1} \Phi_{obs}[n]\,e^{-j2\pi kn/N_{obs}}$$

$$\hat{\Theta}[k] = \sum_{n=0}^{N_{obs}-1} \Theta_{obs}[n]\,e^{-j2\pi kn/N_{obs}}$$

From the top-2 spectral peaks of $|\hat{Z}[k]|$, the heave, roll, and pitch prediction functions $\hat{z}_{deck}(t)$, $\hat{\Phi}_{deck}(t)$, $\hat{\Theta}_{deck}(t)$ are reconstructed as in S095.

### 3D Approach Angle Optimisation

Define the approach direction from a holding waypoint $\mathbf{p}_{hold}$ at altitude $z_{hold} = z_0 + 1.5$ m. The holding waypoint is parameterised by azimuth $\psi \in [0, 2\pi)$ and horizontal standoff $d_{hold} = 2.0$ m from the deck centre:

$$\mathbf{p}_{hold}(\psi) = \begin{pmatrix} x_d + d_{hold}\cos\psi \\ y_d + d_{hold}\sin\psi \\ z_{hold} \end{pmatrix}$$

The glide-slope angle from $\mathbf{p}_{hold}$ to the deck is:

$$\gamma = \arctan\!\left(\frac{z_{hold} - z_0}{d_{hold}}\right)$$

During descent the approach direction unit vector is:

$$\hat{\mathbf{d}}(\psi) = \frac{\mathbf{p}_{deck} - \mathbf{p}_{hold}(\psi)}{\|\mathbf{p}_{deck} - \mathbf{p}_{hold}(\psi)\|}$$

The optimal azimuth $\psi^*$ minimises the misalignment between the approach direction and the deck normal at the predicted landing time $t_{land}$:

$$\psi^* = \argmin_{\psi \in [0,2\pi)}\;\left[\underbrace{\angle\!\left(-\hat{\mathbf{d}}(\psi),\;\hat{\mathbf{n}}_{deck}(t_{land})\right)}_{\text{approach-normal misalignment}} + \lambda_t \cdot |t_{land}(\psi) - t_{crest}|\right]$$

where $\lambda_t = 0.5$ s$^{-1}$ trades off tilt alignment against crest-timing accuracy and $t_{crest}$ is the predicted heave crest from the FFT. $\psi^*$ is found by scanning over $N_\psi = 36$ candidate azimuths.

### Attitude-Compensation Thrust Law

At descent initiation, the drone's commanded roll and pitch track the predicted deck tilt so that at touchdown the drone body frame is aligned with the deck normal:

$$\phi_{cmd}(t) = \alpha_{att} \cdot \hat{\Phi}_{deck}(t_{land}) \cdot \frac{t - t_{start}}{t_{land} - t_{start}}$$

$$\theta_{cmd}(t) = \alpha_{att} \cdot \hat{\Theta}_{deck}(t_{land}) \cdot \frac{t - t_{start}}{t_{land} - t_{start}}$$

where $\alpha_{att} \in [0, 1]$ is a blending gain (0 = no compensation, 1 = full pre-tilt). The thrust vector is rotated to maintain the desired vertical acceleration component despite body tilt:

$$T_{cmd}(t) = \frac{m \left(a_{z,des} + g\right)}{\cos\phi_{cmd}(t)\cos\theta_{cmd}(t)}$$

This increases the required thrust by a factor of up to $1/\cos(4°) \approx 1.002$ at the tilt angles considered here, keeping the thrust budget within the $2mg$ saturation limit.

### 3D Landing Success Metric

Landing is **successful** if at the contact instant $t_l$:

$$\left\|\mathbf{p}_{xy,drone}(t_l) - \mathbf{p}_{xy,deck}(t_l)\right\| \leq r_{land}$$

$$\left|p_z(t_l) - z_{deck}(t_l)\right| \leq r_{land}$$

$$\left|\dot{z}_{drone}(t_l) - \dot{z}_{deck}(t_l)\right| \leq v_{land,max}$$

$$\angle\!\left(\hat{\mathbf{z}}_{drone}(t_l),\; \hat{\mathbf{n}}_{deck}(t_l)\right) \leq \theta_{tilt,max}$$

The Monte Carlo success probability over $N_{trials}$ trials with randomised phases $\varphi_1, \varphi_2 \sim \mathcal{U}[0, 2\pi)$ is:

$$P_{land} = \frac{N_{success}}{N_{trials}} \times 100\%$$

---

## Key 3D Additions

- **3D wave surface model**: coupled heave + pitch + roll kinematics; deck normal vector $\hat{\mathbf{n}}_{deck}(t)$ tracked continuously; horizontal deck drift included.
- **Tilt-channel FFT prediction**: separate FFT for $z_{deck}$, $\Phi_{deck}$, $\Theta_{deck}$; predicted tilt state at landing instant $t_{land}$ used for both approach planning and attitude pre-compensation.
- **3D approach angle optimisation**: azimuth $\psi^*$ selected to minimise combined approach-normal misalignment and crest-timing error over $N_\psi = 36$ candidate directions; adds a horizontal standoff phase not present in S095.
- **Attitude-compensation thrust law**: linearly interpolates body roll/pitch from zero to predicted deck tilt over the descent interval; thrust scaled to maintain vertical force despite tilt.
- **Extended touchdown criterion**: attitude misalignment angle $\angle(\hat{\mathbf{z}}_{drone}, \hat{\mathbf{n}}_{deck}) \leq 4°$ added to the three S095 conditions.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Primary heave amplitude $A_1$ | 0.40 m |
| Primary period $T_1$ | 6.0 s |
| Secondary heave amplitude $A_2$ | 0.15 m |
| Secondary period $T_2$ | 2.5 s |
| Roll amplitudes $\Phi_1, \Phi_2$ | 6°, 3° |
| Pitch amplitudes $\Theta_1, \Theta_2$ | 4°, 2° |
| Roll-heave phase offset $\delta_\Phi$ | $\pi/4$ rad |
| Deck horizontal surge amplitude | 0.20 m at 0.05 Hz |
| Deck horizontal sway amplitude | 0.10 m at 0.035 Hz |
| Mean deck height $z_0$ | 2.0 m |
| Deck size | 1 × 1 m |
| Horizontal standoff $d_{hold}$ | 2.0 m |
| Approach azimuth candidates $N_\psi$ | 36 |
| Tilt-timing trade-off weight $\lambda_t$ | 0.5 s$^{-1}$ |
| Attitude blend gain $\alpha_{att}$ | 1.0 |
| Max tilt tolerance $\theta_{tilt,max}$ | 4° |
| Landing position tolerance $r_{land}$ | 0.30 m |
| Landing velocity tolerance $v_{land,max}$ | 0.10 m/s |
| FFT window $N_{obs}$ | 64 samples at 10 Hz |
| Descent speed $v_{desc}$ | 0.40 m/s |
| Simulation timestep $\Delta t$ | 0.02 s |
| Mission horizon $T_{max}$ | 60 s |
| Monte Carlo trials $N_{trials}$ | 200 |
| z range | 0.5 – 6.0 m |

---

## Expected Output

- **3D trajectory plots**: full 3D approach path with azimuth-optimised final leg; deck surface rendered as a rotating rectangle whose normal vector is drawn as an arrow; 2D-equivalent baseline shown in blue dashed, 3D-optimised in red.
- **Deck kinematics panel**: four-subplot time series of $z_{deck}$, $\Phi_{deck}$, $\Theta_{deck}$, and tilt magnitude $\psi_{tilt}$; predicted landing window shaded; S095 baseline crest windows overlaid for comparison.
- **Approach azimuth optimisation plot**: polar plot of the cost function $J(\psi)$ over 36 candidate azimuths; optimal $\psi^*$ marked; deck normal direction at $t_{land}$ indicated.
- **Attitude misalignment time series**: $\angle(\hat{\mathbf{z}}_{drone}, \hat{\mathbf{n}}_{deck})$ during the descent phase for both strategies; $\theta_{tilt,max} = 4°$ reference line; shows compensation law converging to deck tilt.
- **Monte Carlo success rate comparison**: four-condition success rate (heave + position + velocity + tilt) for 2D-equivalent vs 3D-optimised strategies; secondary bar showing heave-only success rate to isolate the tilt-condition contribution.
- **Landing approach animation (GIF)**: 3D animated view; deck surface rotates as a textured rectangle whose normal vector oscillates; drone descends along the optimal azimuth; live annotations show $z_{err}$, $|\Delta\dot{z}|$, and $\psi_{tilt}$; approach trajectory fades as a trail.

**Expected metric targets** (3D-optimised, deterministic seed 0):

| Metric | Target |
|--------|--------|
| All-condition landing success (single run) | YES |
| Crest timing error | $< 0.3$ s |
| Attitude misalignment at contact | $< 4°$ |
| Relative vertical velocity at contact | $\leq 0.1$ m/s |
| MC success rate (3D-optimised) | $\geq 80\%$ |
| MC success rate (2D-equivalent) | $\leq 55\%$ |

---

## Extensions

1. **JONSWAP spectrum with tilt coupling**: replace the two-frequency deterministic model with a JONSWAP realisation; derive correlated pitch/roll spectrum from linear wave theory (directional spreading); evaluate the 3D approach optimiser under irregular sea states from Sea State 3 to Sea State 5.
2. **Visual landing aid alignment**: add a simulated onboard camera that detects the deck's ArUco marker; fuse marker pose with FFT predictions in an EKF to track both deck position and normal vector; remove the requirement for direct tilt sensor access.
3. **MPC descent with tilt constraint**: replace the constant-speed descent with a short-horizon MPC (horizon $T_p = 1.5$ s) that jointly optimises the vertical thrust profile and attitude interpolation schedule under thrust saturation and tilt-rate constraints.
4. **Takeoff from tilted deck**: design the time-reversed problem — detect the current tilt phase, choose the takeoff instant that minimises attitude disturbance immediately after lift-off, and verify that the drone clears the deck edge.
5. **Multi-rotor asymmetric thrust compensation**: model each rotor independently; during the tilt-compensation phase, compute individual rotor speed commands that produce the required body-frame torque without exceeding any single motor's thrust limit.

---

## Related Scenarios

- Original 2D version: [S095](../S095_water_landing.md)
- Heaving platform with LQR: [S079 Offshore Wind Farm Installation](../../scenarios/04_industrial_agriculture/S079_offshore_wind.md)
- Precision approach to moving target: [S084 Aerial Refuelling Rendezvous](../S084_wind_endurance.md)
- Velocity-matching landing: [S023 Moving Landing Pad](../../scenarios/02_logistics_delivery/S023_moving_landing_pad.md)
- Disturbance rejection reference: [S058 Typhoon Eye Probing](../../scenarios/03_environmental_sar/S058_typhoon.md)
