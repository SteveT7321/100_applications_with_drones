# S014 Decoy & Lure

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: The evader deploys a decoy drone that mimics its position with added noise. The pursuer observes both tracks and must identify the real target using trajectory-smoothness classification (Bayesian update). The evader's plan: send the decoy toward the pursuer as bait while fleeing in the opposite direction.

**Comparison**:
- **Correct ID**: pursuer quickly classifies real target and pursues it
- **Misidentification**: pursuer chases decoy — evader escapes

---

## Mathematical Model

### Decoy Motion

$$\mathbf{p}_{decoy}(t) = \mathbf{p}_E(t) + \boldsymbol{\varepsilon}(t), \quad \boldsymbol{\varepsilon}(t) \sim \mathcal{N}(\mathbf{0},\, \sigma_{decoy}^2 \mathbf{I})$$

During lure phase ($t \in [0, T_{lure}]$): decoy is steered toward the pursuer at speed $v_{decoy}$; after lure phase it stops.

### Motion Smoothness Score

Per-track jerk magnitude (mean absolute acceleration):

$$s(k) = \frac{1}{T} \sum_{t} \frac{\|\mathbf{v}(t) - \mathbf{v}(t-\Delta t)\|}{\Delta t}$$

Lower jerk → more likely real evader (smooth control); higher jerk → likely decoy (noisy).

### Bayesian Classification

Log-likelihood ratio updated every $T_{update}$ seconds:

$$\log \frac{P(\text{real} \mid \text{track}_1)}{P(\text{real} \mid \text{track}_2)} \mathrel{+}= \lambda \cdot (s(\text{track}_2) - s(\text{track}_1))$$

Pursuer targets whichever track has higher $P(\text{real})$.

---

## Implementation

```python
SIGMA_DECOY  = 0.4    # m — decoy position noise std
T_LURE       = 3.0    # s — lure phase duration
T_UPDATE     = 0.5    # s — classification update interval
LAMBDA       = 0.5    # Bayesian update gain

log_ratio = 0.0       # log P(real|track1) / P(real|track2)

# Every T_UPDATE seconds:
s1 = mean_jerk(track1_history)
s2 = mean_jerk(track2_history)
log_ratio += LAMBDA * (s2 - s1)
p_real_t1  = 1 / (1 + np.exp(-log_ratio))   # sigmoid

# Pursuer targets track with higher P(real)
target = track1 if p_real_t1 > 0.5 else track2
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Decoy noise sigma | 0.4 m |
| Lure phase duration | 3 s |
| Classification update period | 0.5 s |
| Bayesian gain lambda | 0.5 |
| Pursuer speed | 5 m/s |
| Evader speed | 3 m/s |
| Decoy speed (lure phase) | 2 m/s toward pursuer |
| Initial pursuer-evader distance | 8 m |

---

## Expected Output

- 3D trajectories: pursuer (red), real evader (blue), decoy (dashed green)
- Classification probability $P(\text{real} \mid \text{track}_1)$ vs time
- Capture time: correct ID vs misidentification (pursuer chases decoy)
- Confusion matrix over 20 runs with different random seeds

---

## Extensions

1. Multiple decoys — pursuer must identify 1 real among $N$ fakes
2. Evader mimics decoy noise to become indistinguishable (adversarial signature)
3. Additional discriminant: radar cross-section (RCS) or acoustic signature

---

## Related Scenarios

- Prerequisites: [S002](S002_evasive_maneuver.md), [S008](S008_stochastic_pursuit.md)
- Next: [S015](S015_relay_tracking.md), [S019](S019_dynamic_reassignment.md)
