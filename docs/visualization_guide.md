# Matplotlib 3D Visualization Guide

> All simulations in this project output matplotlib 3D plots. This document defines standard templates and conventions.

## Basic Template

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def setup_3d_plot(title="", figsize=(10, 8)):
    """Standard 3D plot setup"""
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    return fig, ax

def plot_trajectory(ax, traj, color='blue', label='', alpha=0.8):
    """Draw trajectory line + start/end markers

    Args:
        traj: shape (T, 3) array of positions
    """
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
            color=color, alpha=alpha, linewidth=1.5, label=label)
    # Start: circle
    ax.scatter(*traj[0], color=color, s=80, marker='o', zorder=5)
    # End: star
    ax.scatter(*traj[-1], color=color, s=120, marker='*', zorder=5)

def save_plot(fig, filename, dpi=150):
    """Save figure"""
    plt.tight_layout()
    plt.savefig(f"outputs/{filename}.png", dpi=dpi, bbox_inches='tight')
    plt.show()
```

---

## Scene Type Chart Mapping

### Pursuit Scene (1v1)

```python
def plot_pursuit_1v1(pursuer_traj, evader_traj, capture_point=None):
    fig, ax = setup_3d_plot("Pursuit Trajectory (1v1)")
    plot_trajectory(ax, pursuer_traj, color='red',  label='Pursuer')
    plot_trajectory(ax, evader_traj,  color='blue', label='Evader')

    if capture_point is not None:
        ax.scatter(*capture_point, color='black', s=200,
                   marker='X', label='Capture point', zorder=10)
    ax.legend()
    return fig, ax
```

### Multi-Drone Trajectories (Nv1 / NvM)

```python
COLORS = ['red', 'orange', 'green', 'purple', 'brown', 'pink', 'gray', 'cyan']

def plot_multi_trajectory(trajs_dict):
    """
    trajs_dict: {'drone_0': array (T,3), 'drone_1': array(T,3), ...}
    """
    fig, ax = setup_3d_plot("Multi-Drone Trajectories")
    for i, (name, traj) in enumerate(trajs_dict.items()):
        plot_trajectory(ax, traj, color=COLORS[i % len(COLORS)], label=name)
    ax.legend()
    return fig, ax
```

### Coverage Heatmap (Search and Rescue Scene)

```python
def plot_coverage_heatmap(coverage_grid, xlim, ylim, title="Coverage"):
    """2D top-down coverage map (open separately alongside 3D trajectory)"""
    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(coverage_grid.T, origin='lower',
                   extent=[xlim[0], xlim[1], ylim[0], ylim[1]],
                   cmap='YlOrRd', vmin=0, vmax=1)
    plt.colorbar(im, ax=ax, label='Scan count')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title)
    return fig, ax
```

### Energy / Distance Time Series

```python
def plot_timeseries(time_arr, data_dict, ylabel="", title=""):
    """
    data_dict: {'Distance': array, 'Speed': array, ...}
    """
    fig, ax = plt.subplots(figsize=(10, 4))
    for label, data in data_dict.items():
        ax.plot(time_arr, data, label=label)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    return fig, ax
```

---

## Color Conventions

| Role | Color | Symbol |
|------|-------|--------|
| Pursuer / Defender | `red` | ▶ |
| Evader / Attacker | `blue` | ▶ |
| Neutral / Target | `green` | ★ |
| Obstacle | `gray` | ■ |
| Capture / Landing point | `black` | ✕ |
| Waypoint | `orange` | ● |

---

## Animation Output (Optional)

```python
from matplotlib.animation import FuncAnimation, FFMpegWriter

def animate_trajectories(trajs_dict, dt=0.02):
    fig, ax = setup_3d_plot("Animation")
    lines = {}
    for i, (name, _) in enumerate(trajs_dict.items()):
        lines[name], = ax.plot([], [], [], color=COLORS[i], label=name)

    T = max(len(t) for t in trajs_dict.values())

    def update(frame):
        for name, traj in trajs_dict.items():
            k = min(frame, len(traj)-1)
            lines[name].set_data(traj[:k+1, 0], traj[:k+1, 1])
            lines[name].set_3d_properties(traj[:k+1, 2])
        return list(lines.values())

    anim = FuncAnimation(fig, update, frames=T, interval=dt*1000, blit=True)
    writer = FFMpegWriter(fps=int(1/dt))
    anim.save('outputs/animation.mp4', writer=writer)
```

> Requires `brew install ffmpeg`

---

## Output Directory Convention

Each scene's output is stored in:

```
outputs/
├── s001_basic_intercept/
│   ├── trajectory_3d.png
│   ├── distance_time.png
│   └── animation.mp4     (optional)
├── s002_evasion/
│   └── ...
```

Create the output directory:

```python
import os
output_dir = f"outputs/{scenario_id}"
os.makedirs(output_dir, exist_ok=True)
```
