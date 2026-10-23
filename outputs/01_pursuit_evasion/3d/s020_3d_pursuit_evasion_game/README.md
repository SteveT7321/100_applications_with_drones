# S020 3D — Pursuit-Evasion Game (RL Capstone)

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: the card's 3D game, with a 5 m/s pursuer against a 3.5 m/s evader. Both command a 3D unit-sphere velocity direction. Capture radius is 0.15 m, episodes last 30 s, and every observed position and velocity component carries noise σ = 0.1. **No PPO training is run in this 3D variant** (see the deviations table). The card's benchmark grid is run with hand-crafted policies on both sides:

| Pursuer | Rule (always exactly 5 m/s) |
|---------|------|
| PP | pure pursuit at the observed evader position |
| PN | 3D proportional navigation, N = 3, constant speed, launched along the initial line of sight (the S016 / S020 2D fix) |
| APF | attraction to the evader + repulsion from every arena face closer than 1 m (a genuine potential field) |
| Lead | pure pursuit at the predicted point e + v_e · 0.3 s |
| AltAdv | pure pursuit with a climb bias until the pursuer is 1 m above the evader |

| Evader (card) | Rule (always exactly 3.5 m/s) |
|---------|------|
| straight | fixed random 3D heading, elevation within ±30° |
| perpendicular | horizontal, perpendicular to the line of sight (random fixed side) |
| spiral | helix: horizontal heading rotating at 0.8 rad/s plus vertical component 0.5 sin(0.8 t + phase) |
| alt_escape | the card's altitude escape: away from the pursuer horizontally while climbing or diving to the altitude limit on the far side of the pursuer |

**Two arenas, identical spawns, evader parameters and noise (common random numbers)**:
- **card**: x, y ∈ [−5, 5] m, z ∈ [0.5, 5.5] m.
- **open**: no x-y walls, same z limits. This separates real pursuit from wall cornering.

Every cell (5 pursuers × 4 evaders × 2 arenas) runs 500 seeded episodes. For each episode the script records whether the evader touched an x-y wall or a z limit before capture.

**Key question**: Which pursuit and evasion rules work in 3D, and how much of the result in the card's small arena comes from the walls?

---

## Mathematical Model

**Motion**: $\mathbf{p}_{k+1} = \mathrm{clip}_{arena}(\mathbf{p}_k + \mathbf{v}\,\Delta t)$, with $\|\mathbf{v}_P\| = 5$ m/s and $\|\mathbf{v}_E\| = 3.5$ m/s. Agents act on the noisy observation $\tilde{\mathbf{x}} = \mathbf{x} + \mathcal{N}(0, 0.1^2)$ (positions and velocities).

**PN** (constant speed, launched along the initial LOS):

$$\dot{\hat{\boldsymbol\lambda}} = \frac{\mathbf{v}_{rel} - (\mathbf{v}_{rel}\cdot\hat{\boldsymbol\lambda})\hat{\boldsymbol\lambda}}{r}, \quad v_c = -\mathbf{v}_{rel}\cdot\hat{\boldsymbol\lambda}, \quad \mathbf{v}_P \leftarrow v_P\,\frac{\mathbf{v}_P + N v_c \dot{\hat{\boldsymbol\lambda}}\,\Delta t}{\|\cdot\|}$$

**APF**: $\mathbf{v}_P \parallel \widehat{\mathbf{p}_E - \mathbf{p}_P} + \sum_{faces,\, d < d_0} \eta\left(\frac{1}{d} - \frac{1}{d_0}\right)\frac{1}{d^2}\,\hat{\mathbf{n}}$ with $d_0 = 1$ m and $\eta = 1/48$. The repulsion equals the attraction 0.25 m from a face: (1/48)(1/0.25 − 1)/0.25² = 1.

**Lead**: $\mathbf{v}_P \parallel \tilde{\mathbf{p}}_E + 0.3\,\tilde{\mathbf{v}}_E - \tilde{\mathbf{p}}_P$. **AltAdv**: $\mathbf{v}_P \parallel v_P\,\widehat{\mathbf{p}_E - \mathbf{p}_P} + [0, 0, \min(2, 1.5(1 - \Delta z))]$ while $\Delta z = z_P - z_E < 1$ m.

**Capture**: closest approach of the linear relative motion inside a step < 0.15 m, or no capture by 30 s.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer / evader speed | 5.0 / 3.5 m/s |
| Card arena | x, y ∈ [−5, 5] m, z ∈ [0.5, 5.5] m |
| Open arena | no x-y walls, z ∈ [0.5, 5.5] m |
| Capture radius | 0.15 m |
| dt / episode length | 0.02 s / 30 s (1500 steps) |
| Observation noise σ | 0.1 on every observed position (m) and velocity (m/s) component, all agents |
| PN gain N | 3 |
| APF influence distance d₀ / gain η | 1.0 m / 1/48 |
| Lead time | 0.3 s |
| AltAdv target height above the evader | 1.0 m |
| Straight evader elevation range | ±30° |
| Spiral evader ω / vertical amplitude | 0.8 rad/s / 0.5 |
| Spawns | uniform in x, y ∈ [−4.5, 4.5], z ∈ [1, 5] m, separation ≥ 5 m |
| Episodes per cell / seed | 500 / 2020 |

---

## Implementation

```bash
conda activate drones
python src/01_pursuit_evasion/3d/s020_3d_pursuit_evasion_game.py
```

Runtime ≈ 55 s (40 vectorised cells × 500 episodes, a noise-free re-run of the 40 cells, a 125-frame GIF).

---

## Results

### Capture rate / mean capture time of captured episodes (500 episodes per cell)

| Pursuer | straight (card / open) | perpendicular | spiral | alt_escape |
|---------|----------|------|------|------|
| PP | 100% 1.96 s / 100% 4.40 s | 100% 1.38 s / 100% 1.41 s | 100% 2.32 s / 100% 4.79 s | 100% 2.06 s / 100% 2.93 s |
| PN | 83.2% 2.58 s / 79.4% 2.54 s | 66.0% 2.22 s / 35.0% 1.55 s | 75.0% 1.76 s / 58.4% 2.19 s | 73.2% 1.98 s / 69.6% 2.76 s |
| APF | 21.0% 2.06 s / 30.2% 3.74 s | 100% 1.39 s / 100% 1.41 s | 98.4% 8.48 s / 99.8% 7.10 s | 0.2% 29.68 s / 100% 4.93 s |
| Lead | 100% 2.86 s / 54.0% 3.27 s | 100% 1.72 s / 100% 1.90 s | 39.2% 2.38 s / 24.0% 2.70 s | 100% 2.06 s / 100% 2.85 s |
| AltAdv | 100% 2.04 s / 99.8% 5.76 s | 100% 1.43 s / 100% 1.47 s | 100% 2.74 s / 100% 5.78 s | 100% 2.36 s / 100% 3.27 s |

Averaged over the four evaders: card arena PP 100% / 1.93 s, AltAdv 100% / 2.14 s, Lead 84.8% / 2.23 s, PN 74.4% / 2.15 s, APF 54.9% / 4.66 s. Open arena: PP 100% / 3.38 s, AltAdv 100% / 4.07 s, APF 82.5% / 4.41 s, Lead 69.5% / 2.57 s, PN 60.6% / 2.38 s.

### Wall contact in the card arena (share of captures with the evader on an x-y wall before capture)

| Pursuer | straight | perpendicular | spiral | alt_escape |
|---------|----------|------|------|------|
| PP | 82.2% | 26.6% | 77.4% | 99.2% |
| PN | 70.4% | 73.3% | 69.9% | 98.9% |
| APF | 54.3% | 26.6% | 87.2% | 100% |
| Lead | 90.0% | 55.6% | 84.2% | 99.0% |
| AltAdv | 84.8% | 27.0% | 80.6% | 99.6% |

PP captures with no wall or z-limit contact at all: straight 53 of 500 (mean 1.24 s), perpendicular 367 (1.38 s), spiral 55 (1.53 s), alt_escape 0. The alt_escape evader reaches a z limit in 100% of PP captures (by design), and the straight / spiral evaders in 34–41%.

### Without observation noise (σ = 0, same spawns): capture rate, card / open arena

| Pursuer | straight | perpendicular | spiral | alt_escape |
|---------|----------|------|------|------|
| PP | 100 / 100% | 100 / 100% | 100 / 100% | 100 / 100% |
| PN | 100 / 100% | 100 / 100% | 100 / 86.4% | 100 / 100% |
| APF | 29.8 / 60.0% | 100 / 100% | 100 / 100% | 0 / 0% |
| Lead | 100 / 53.0% | 64.2 / 0% | 34.2 / 21.4% | 100 / 100% |
| AltAdv | 100 / 100% | 100 / 100% | 100 / 100% | 100 / 100% |

**Key Findings**:
- **In the card's 10 m arena the walls decide most captures.** Against the straight evader, 82% of PP captures happen after the evader has hit an x-y wall. The same spawns in the open arena take more than twice as long (4.40 vs 1.96 s); the spiral evader behaves the same way (4.79 vs 2.32 s). Only the perpendicular evader, which circles the pursuer instead of running, is caught equally fast in both arenas (1.38 vs 1.41 s). Times in the card arena therefore measure how fast an evader gets cornered, not pure 3D pursuit. The open arena is the fair comparison of the guidance laws.
- **Pure pursuit is the only rule that captures every evader in both arenas.** It needs neither range nor velocity estimates, so the σ = 0.1 noise does not hurt it, and the 1.5 m/s speed advantage always closes the gap. AltAdv also reaches 100% (99.8% against straight in the open arena), but its climb bias makes it slower than PP in the open arena (4.07 vs 3.38 s on average; 5.76 vs 4.40 s against straight). The old code comment calling the altitude-advantage pursuer "best" was wrong.
- **PN misses because of noise, then flies away.** Its escapes pass a median 0.19–0.26 m from the evader, just outside the 0.15 m capture radius. Without noise PN captures 100% everywhere except the open-arena spiral (86.4%). At short range the noisy line-of-sight rate (σ = 0.1 m on 0.15 m) dominates the PN command. After a miss, v_c < 0 and PN steers away, with no re-engagement (open-arena episode 0 ends 50 m away). When it does capture it is the fastest pursuer in the open arena (2.38 s mean over its captures).
- **APF cannot reach an evader in a corner or at a limit.** Its face repulsion balances the attraction 0.25 m from a face, more than the 0.15 m capture radius. The alt_escape evader sits on the floor or ceiling, usually against a wall too, and APF captures it in 0.2% of card-arena episodes (median closest approach 0.33 m). Against the straight evader APF captures 21.0% (card) and 30.2% (open). Every one of its straight-evader escapes ends with the evader on a face: all 395 card-arena escapes touched an x-y wall (375 also a z limit), and all 349 open-arena escapes touched a z limit; median closest approach 0.26 m. In the open arena with noise, APF does capture the alt_escape evader (100%), but without noise it captures 0%: the noise jitter is what brings it within 0.15 m of an evader on a single z limit.
- **A fixed 0.3 s lead overshoots turning evaders.** Lead aims 1.05 m ahead of the evader, which is more than the capture radius, so against the spiral it crosses ahead and misses (39.2% card, 24.0% open). Without noise it can even lock into a stable miss against the perpendicular evader (64.2% card, 0% open). The previous README's "predictive equals pure pursuit" came from one seed with walls; it is not true here.
- **Evader ranking (open arena, PP): spiral 4.79 s > straight 4.40 s > alt_escape 2.93 s > perpendicular 1.41 s.** Running away (straight, spiral) works best against the faster pursuer. Diving to a z limit (alt_escape) gives up one dimension of freedom and is caught sooner. Circling the pursuer (perpendicular) never opens the range.
- **The z motion is real.** The straight and spiral evaders use the whole 0.5–5.5 m band (altitude-usage plot), and PP's altitude follows them. Only alt_escape concentrates at the limits (76% of its steps in the top or bottom 0.5 m), as the card intends.

**Capture rate and mean capture time, card and open arenas:**

![Capture matrix](capture_matrix.png)

**Wall contact before capture (card arena) and capture time card vs open arena:**

![Wall contact](wall_contact.png)

**Altitude usage of evader and PP pursuer (card arena, 500 episodes per evader):**

![Altitude usage](altitude_usage.png)

**Capture-time CDF per evader (solid = card arena, dashed = open arena):**

![Capture CDF](capture_cdf.png)

**Episode 0, every pursuer vs the altitude-escape evader (top: card arena, bottom: open arena):**

![3D trajectories](trajectories_3d.png)

**Animation (card arena, episode 0, PP / PN / APF / AltAdv vs the spiral evader):**

![Animation](animation.gif)

### Deviations from the scenario card

| Card | Code | Reason |
|------|------|--------|
| PPO training, 3-phase curriculum (5000 / 5000 / 10000 episodes), learning curves, RL policy quiver and streamline plots | Not run; no learning curves or policy plots | The S020 2D fix trained PPO for the largest budget that fits a few minutes and the pursuer learned nothing (0 captures). The 3D variant only runs the card's benchmark grid with hand-crafted policies. The previous "rl_proxy" pursuer (a rule-based policy labelled as RL) and the synthetic "conceptual" learning curves were removed |
| Benchmark: RL pursuer vs PP, PN, APF; RL evader vs straight, perpendicular, spiral, altitude-escape | PP, PN, APF, Lead, AltAdv vs straight, perpendicular, spiral, alt_escape; 500 episodes per cell | Full grid without the RL agents; Lead and AltAdv kept from the previous version |
| Arena [−5, 5]³ m and z range 0.5–5.5 m (inconsistent) | x, y ∈ [−5, 5] m, z ∈ [0.5, 5.5] m, plus an open arena without x-y walls | The card's z range is used; the open arena shows what the walls contribute (the previous version used ±10 m and z 0.3–8 m without saying so) |
| Altitude bounds as soft reward penalties | Hard clip at every face | No reward is used without RL |
| Observation noise σ = 0.1 m | σ = 0.1 on every observed position and velocity component, for all agents; noise-free re-run reported | Same treatment as the S020 2D fix |
| PN, APF, evader behaviours not specified | As defined above (constant-speed PN launched along the LOS; genuine face-repulsion APF) | The previous PN started from rest; the S020 2D fix showed a renamed pure pursuit is not an APF |
| Altitude histogram over 1000 test episodes | 500 episodes per evader, PP pursuer, card arena | Same per-cell count as the benchmark |
| Capture-time CDF "3D RL vs 2D baselines at z = 2.5 m" | Card arena vs open arena per evader | No RL agent; the wall effect is the main confound |
| Spawn positions not specified | Uniform in the arena interior, separation ≥ 5 m, same for every cell | The previous single fixed spawn (one seed per cell) could not show variability |
| dt not specified | 0.02 s | — |

---

## Extensions

1. MAPPO for 3v3 swarm — both teams trained end-to-end in 3D with altitude layer coordination
2. Add 3D obstacle field (from S004 3D) to observation and action space
3. Transfer learning from 3D simulation to real hardware: fine-tune with domain randomisation over $v_{z,max}$
4. Curriculum altitude difficulty: start with z-variation σ = 0 (flat), gradually increase to full ±5 m

---

## Related Scenarios

- Original 2D version: [S020 Pursuit-Evasion Game (RL Capstone)](../../../../scenarios/01_pursuit_evasion/S020_pursuit_evasion_game.md)
- 3D card: [S020 3D Upgrade — Pursuit-Evasion Game (RL Capstone)](../../../../scenarios/01_pursuit_evasion/3d/S020_3d_pursuit_evasion_game.md)
- [S009 3D Upgrade — Differential Game (Lion & Man in 3D)](../../../../scenarios/01_pursuit_evasion/3d/S009_3d_differential_game.md), [S010 3D Upgrade — Asymmetric Speed Bounded Arena Trapping](../../../../scenarios/01_pursuit_evasion/3d/S010_3d_asymmetric_speed.md)
- [S016 3D Upgrade — Airspace Defense](../../../../scenarios/01_pursuit_evasion/3d/S016_3d_airspace_defense.md), [S019 3D Upgrade — Dynamic Target Reassignment](../../../../scenarios/01_pursuit_evasion/3d/S019_3d_dynamic_reassignment.md)
