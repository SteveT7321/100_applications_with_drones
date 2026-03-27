# 100 Drone Simulation Applications

> A systematic study of 100 drone simulation scenarios across 5 domains, built with Python, NumPy, and Matplotlib 3D.

---

## Overview

This project implements 100 carefully selected drone simulation scenarios — from basic 1v1 pursuit to large-scale swarm coordination — building a comprehensive knowledge base of drone control algorithms and multi-agent systems.

Each scenario includes a mathematical model, Python implementation, and 3D visualization.

---

## Tech Stack

| Component | Version | Role |
|-----------|---------|------|
| Python | 3.10 | Primary language |
| NumPy | 1.26 | Numerical computation |
| SciPy | 1.14.1 | Optimization, interpolation |
| Matplotlib | 3.10 | 3D trajectory visualization |
| gym-pybullet-drones | 2.0 | PyBullet physics interface (advanced scenarios) |
| conda env | `drones` | Environment name |

---

## Five Domains

| # | Domain | Core Problems | Scenarios |
|---|--------|--------------|-----------|
| 1 | [**Pursuit & Evasion**](domains/01_pursuit_evasion/) | Differential games, optimal control | S001–S020 |
| 2 | [**Logistics & Delivery**](domains/02_logistics_delivery/) | Path planning, task assignment | S021–S040 |
| 3 | [**Environmental & SAR**](domains/03_environmental_sar/) | Coverage, information gathering | S041–S060 |
| 4 | [**Industrial & Agriculture**](domains/04_industrial_agriculture/) | Precision control, systematic paths | S061–S080 |
| 5 | [**Special Ops & Entertainment**](domains/05_special_entertainment/) | Formation control, cooperative vision | S081–S100 |

---

## Quick Start

```bash
# 1. Clone the repository
git clone https://github.com/SteveT7321/100_applications_with_drones.git
cd drones

# 2. Create and activate conda environment
conda create -n drones python=3.10
conda activate drones

# 2. Install dependencies
pip install numpy==1.26.4 scipy==1.14.1 matplotlib==3.10

# 3. Run the first scenario
python src/pursuit/s001_basic_intercept.py
```

See [SETUP.md](SETUP.md) for full installation instructions.

---

## Repository Structure

```
drones/
├── src/
│   ├── base/
│   │   └── drone_base.py              # Shared point-mass drone base class
│   └── pursuit/
│       └── s001_basic_intercept.py
├── scenarios/                         # Scenario cards (math model + parameters)
│   └── pursuit/
│       ├── S001_basic_intercept.md
│       └── ...
├── outputs/                           # Simulation outputs (PNG, GIF)
│   └── s001_basic_intercept/
├── domains/                           # Domain overviews with theory background
│   ├── 01_pursuit_evasion/README.md
│   └── ...
├── docs/                              # Technical reference docs
├── MATH_FOUNDATIONS.md                # Shared mathematical foundations
├── PROGRESS.md                        # Progress tracker (100 scenarios)
└── SETUP.md                           # Environment setup guide
```

---

## Progress

| Domain | Done | Total |
|--------|------|-------|
| Pursuit & Evasion | 5 | 20 |
| Logistics & Delivery | 0 | 20 |
| Environmental & SAR | 0 | 20 |
| Industrial & Agriculture | 0 | 20 |
| Special Ops & Entertainment | 0 | 20 |
| **Total** | **5** | **100** |

→ Full tracker: [PROGRESS.md](PROGRESS.md)

---

## References

- Shneydor, N.A. (1998). *Missile Guidance and Pursuit*. Horwood.
- Isaacs, R. (1965). *Differential Games*. Wiley.
- Panerati, J., et al. (2021). [Learning to fly — a gym environment with PyBullet physics](https://arxiv.org/abs/2103.02142). IROS 2021.
- Mahony, R., et al. (2012). Multirotor aerial vehicles: Modeling, estimation, and control. *IEEE Robotics & Automation Magazine*.
