# Environment Setup Guide

## System Requirements

- macOS 10.15+ or Ubuntu 20.04+
- Anaconda / Miniconda
- Git
- (macOS) `brew install ffmpeg` (for video recording, optional)

## Installation Steps

### 1. Clone this project

```bash
git clone <this-repo> drones
cd drones
```

### 2. Create conda environment (Python 3.10)

```bash
conda create -n drones python=3.10 -y
conda activate drones
pip install --upgrade pip
```

### 3. Install scientific packages (via conda-forge to avoid binary compatibility issues)

```bash
conda install -n drones -c conda-forge numpy scipy matplotlib pybullet -y
pip install "gymnasium~=1.2" transforms3d control
pip install "scipy==1.14.1"   # pin version to avoid incompatibility with numpy 1.26
```

### 4. Install gym-pybullet-drones

```bash
cd lib/gym-pybullet-drones
pip install --no-deps -e .
cd ../..
```

### 5. Install additional tools

```bash
pip install stable-baselines3 tensorboard jupyter
```

### 6. Verify installation

```bash
python -c "
import numpy; print('numpy', numpy.__version__)
from scipy.spatial.transform import Rotation; print('scipy OK')
import matplotlib; print('matplotlib', matplotlib.__version__)
import pybullet; print('pybullet OK')
import gymnasium; print('gymnasium', gymnasium.__version__)
from gym_pybullet_drones.envs import CtrlAviary; print('CtrlAviary OK')
from mpl_toolkits.mplot3d import Axes3D; print('3D OK')
print('=== Verification complete ===')
"
```

Expected output:
```
numpy 1.26.4
scipy OK
matplotlib 3.9.x
pybullet OK
gymnasium 1.2.3
CtrlAviary OK
3D OK
=== Verification complete ===
```

## Verified Package Version Combinations

| Package | Version | Notes |
|---------|---------|-------|
| Python | 3.10.x | conda |
| numpy | 1.26.4 | conda-forge |
| scipy | 1.14.1 | pip (1.15.x has bug) |
| matplotlib | 3.9.4 | conda-forge |
| pybullet | 3.2.5 | conda-forge (required on macOS, cannot pip build) |
| gymnasium | 1.2.3 | pip |
| gym-pybullet-drones | 2.0.0 | pip --no-deps -e . |
| stable-baselines3 | 2.4.1 | pip (warning with gymnasium 1.2 but functional) |

## Troubleshooting

### pybullet cannot be installed via pip (macOS)
Cause: Requires Xcode Command Line Tools to compile C extensions.
Solution: Use `conda install -c conda-forge pybullet` instead.

### `scipy` import raises ValueError: All ufuncs must have type numpy.ufunc
Cause: scipy 1.15.x has a compatibility issue with numpy 1.26.x on macOS.
Solution: `pip install "scipy==1.14.1"`

### conda command raises OpenSSL AttributeError
Cause: pyOpenSSL version in the base environment is too old.
Solution: `/path/to/anaconda3/bin/pip install --upgrade pyOpenSSL cryptography`

## How to Run

```bash
conda activate drones
# Run any simulation script
python src/pursuit/s001_basic_intercept.py
```
