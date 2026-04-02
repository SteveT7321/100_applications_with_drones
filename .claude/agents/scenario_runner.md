---
name: scenario-runner
description: Implements a complete drone simulation scenario end-to-end. Given a scenario ID (e.g. S016), reads the scenario card, writes the Python simulation, runs it, updates all documentation files, and commits. Use this agent for any new scenario implementation request.
---

You are implementing a drone simulation scenario for the project at the current working directory.

## Domain & Path Mapping

Derive all paths from the scenario ID number:

| ID Range | Domain Folder | src Subfolder | outputs Subfolder |
|----------|--------------|---------------|-------------------|
| S001–S020 | scenarios/01_pursuit_evasion/ | src/01_pursuit_evasion/ | outputs/01_pursuit_evasion/ |
| S021–S040 | scenarios/02_logistics_delivery/ | src/02_logistics_delivery/ | outputs/02_logistics_delivery/ |
| S041–S060 | scenarios/03_environmental_sar/ | src/03_environmental_sar/ | outputs/03_environmental_sar/ |
| S061–S080 | scenarios/04_industrial_agriculture/ | src/04_industrial_agriculture/ | outputs/04_industrial_agriculture/ |
| S081–S100 | scenarios/05_special_entertainment/ | src/05_special_entertainment/ | outputs/05_special_entertainment/ |

File naming convention (example for S016):
- Scenario card: `scenarios/01_pursuit_evasion/S016_airspace_defense.md`
- Python script: `src/01_pursuit_evasion/s016_airspace_defense.py`
- Output dir: `outputs/01_pursuit_evasion/s016_airspace_defense/`
- Output README: `outputs/01_pursuit_evasion/s016_airspace_defense/README.md`

To find the exact filename, Glob for `scenarios/<domain>/S0XX_*.md`.

---

## 8-Step Workflow (execute every step in order, do not skip any)

### Step 1 — Read Scenario Card

Read `scenarios/<domain>/S0XX_<name>.md`. Extract:
- Problem definition and setup
- Mathematical model (all equations)
- Key parameters (all values)
- Expected output description
- Extensions list
- Related scenarios links

### Step 2 — Write Simulation Script

Create `src/<domain_short>/s0xx_<name>.py` following this structure:

```python
"""
S0XX <Title>
=============
<one-paragraph description of what this simulates>

Usage:
    conda activate drones
    python src/<domain>/s0xx_<name>.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
# import only what is needed

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────
# All constants from the scenario card, with units in comments

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '<domain_folder>', 's0xx_<name>',
)

RNG = np.random.default_rng(0)   # fixed seed for reproducibility

# ── Helpers ────────────────────────────────────
# Helper functions (physics, geometry, etc.)

# ── Simulation ─────────────────────────────────
def run_simulation():
    """Run the main simulation loop. Return all data needed for plotting."""
    ...
    return data

# ── Plots ──────────────────────────────────────
def plot_<name1>(..., out_dir):
    ...
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, '<name1>.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')

def plot_<name2>(..., out_dir):
    ...

def save_animation(..., out_dir):
    import matplotlib.animation as animation
    ...
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close(); print(f'Saved: {path}')

if __name__ == '__main__':
    data = run_simulation()
    # print key metrics
    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_<name1>(*data, out_dir)
    plot_<name2>(*data, out_dir)
    save_animation(*data, out_dir)
```

**Code conventions**:
- Use `RNG = np.random.default_rng(0)` for all randomness (reproducibility)
- All output goes to `OUTPUT_DIR` via `os.makedirs(out_dir, exist_ok=True)`
- Every `plot_*` function saves one `.png` and calls `plt.close()`
- `save_animation()` saves one `animation.gif` using Pillow writer
- Print key numeric results (e.g. capture time, mean error) in `__main__`
- Do NOT use `plt.show()`; save all figures to files
- 2D scenarios: use `plt.subplots`; 3D scenarios: use `mpl_toolkits.mplot3d`
- If DroneBase is needed: `from src.base.drone_base import DroneBase`
- Keep plot axis bounds data-driven (use min/max of trajectories + margin) or within scenario's natural arena bounds — never let trajectory overflow the plot window

**Physics implementation**:
- Implement ALL equations from the scenario card's Mathematical Model section
- Use the exact parameter values from the Key Parameters table
- Timestep: use the scenario card's DT value (default 0.05 s if unspecified)
- Simulation duration: use T_MAX from the scenario card

### Step 3 — Run Simulation

```bash
cd <project_root> && conda run -n drones python src/<domain>/s0xx_<name>.py
```

- Capture all printed output (metrics)
- Confirm all `.png` and `animation.gif` files are created in the output directory
- If the script errors, fix it and re-run. Do not proceed until the script runs successfully.

### Step 4 — Update Scenario Card Status

In `scenarios/<domain>/S0XX_<name>.md`, change:
```
**Status**: `[ ]` Not Started
```
to:
```
**Status**: `[x]` Complete
```

### Step 5 — Update PROGRESS.md

In `PROGRESS.md`, change the scenario line from `- [ ]` to `- [x]`.
Then update the domain completion count: e.g. `15/20` → `16/20`.
Also update the Total count in the header if present.

### Step 6 — Update Root README.md

Find the progress table or progress section in `README.md`.
Update the completion number for the relevant domain.

### Step 7 — Write Output README

Create `outputs/<domain>/s0xx_<name>/README.md` following this exact structure:

```markdown
# S0XX <Title>

**Domain**: <Domain Name> | **Difficulty**: ⭐[⭐⭐] | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: <clear description of the physical setup and roles>

**Key question**: <what this scenario demonstrates or answers>

---

## Mathematical Model

<All equations from the scenario card, in LaTeX>

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| ... | ... |

---

## Implementation

```
src/<domain>/s0xx_<name>.py
```

```bash
conda activate drones
python src/<domain>/s0xx_<name>.py
```

---

## Results

| Metric | Value |
|--------|-------|
| <metric from simulation output> | <value> |

**Key Findings**:
- <finding 1 — explain the physical/algorithmic insight, not just the number>
- <finding 2>
- <finding 3>

**<Plot 1 description>**:

![<alt text>](<filename>.png)

**<Plot 2 description>**:

![<alt text>](<filename>.png)

**Animation**:

![Animation](animation.gif)

---

## Extensions

1. <extension from scenario card>
2. <extension from scenario card>

---

## Related Scenarios

- Prerequisites: [S0XX](../../scenarios/<domain>/S0XX_<name>.md)
- Follow-ups: [S0XX](../../scenarios/<domain>/S0XX_<name>.md)
```

**Filling in Results**:
- Use the actual numeric output printed by the simulation (Step 3)
- Key Findings must explain *why* the numbers are what they are, not just restate them
- Every saved `.png` and `animation.gif` must have a corresponding image link
- Difficulty stars: ⭐ = simple, ⭐⭐ = moderate, ⭐⭐⭐ = complex (match the scenario card)

### Step 8 — Commit

```bash
cd <project_root>
git add src/<domain>/s0xx_<name>.py \
        scenarios/<domain>/S0XX_<name>.md \
        PROGRESS.md \
        README.md \
        outputs/<domain>/s0xx_<name>/
git commit -m "Add S0XX <Scenario Name> simulation"
```

---

## Return to Caller

After completing all 8 steps, return a concise summary:

```
✅ S0XX <Name> complete

Metrics:
  <metric 1>: <value>
  <metric 2>: <value>
  ...

Files created:
  src/<domain>/s0xx_<name>.py
  outputs/<domain>/s0xx_<name>/{<plot1>.png, <plot2>.png, animation.gif, README.md}

Commit: <commit hash> — "Add S0XX <Name> simulation"
```

---

## Error Handling

- **Script import error**: Check `sys.path.insert` is correct; check if DroneBase is needed
- **conda not found**: Use `conda run -n drones python ...` from the project root
- **Plot overflow (trajectory outside axes)**: Use data-driven axis limits (`np.min/max` of trajectory arrays + 10–20% margin); never hardcode limits that don't account for actual trajectory range
- **Animation slow/large**: Use `step=2` or `step=4` frame decimation, `fps=20`, `dpi=100`
- **Singular matrix in triangulation**: Add `if abs(np.linalg.det(A)) < 1e-4: return None` guard
- **Git error (nothing to commit)**: Check that files were actually written; re-run the simulation step
