# nav/viz.py
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Tuple

def setup_axes(grid: np.ndarray, style: str = "grid"):
    fig, ax = plt.subplots(figsize=(9, 6))
    H, W = grid.shape
    if style == "grid":
        from matplotlib.colors import ListedColormap
        cmap = ListedColormap(["#4E3A63", "#F7EA75"])  # purple free, yellow obstacles
        ax.imshow(grid, origin="lower", extent=(0, W, 0, H), cmap=cmap, interpolation="nearest")
        for x in range(W + 1): ax.axvline(x, color=(1,1,1,0.08), lw=0.6)
        for y in range(H + 1): ax.axhline(y, color=(1,1,1,0.08), lw=0.6)
    else:
        ax.imshow(grid, origin="lower", extent=(0, W, 0, H),
                  cmap=plt.cm.gray_r, interpolation="nearest")
    ax.set_xlim(0, W); ax.set_ylim(0, H); ax.set_aspect('equal', adjustable='box')
    return fig, ax

def overlay_inflated(ax, inflated: np.ndarray):
    from matplotlib.colors import ListedColormap
    H, W = inflated.shape
    ax.imshow(inflated, origin='lower', extent=(0, W, 0, H),
              cmap=ListedColormap([(0,0,0,0), (1,0,0,0.12)]), interpolation='nearest')

def create_artists(ax, waypoints: np.ndarray, start_xy: Tuple[float, float], goal_xy: Tuple[float, float]):
    (path_line,) = ax.plot(waypoints[:,0], waypoints[:,1], 'b-', lw=2, label='A* path')
    (traj_line,) = ax.plot([], [], 'r-', lw=2, label='Robot trajectory')
    (robot_dot,) = ax.plot([], [], 'ro', ms=5)
    ax.scatter([start_xy[0]],[start_xy[1]], s=60, c='g', label='Start', zorder=3)
    ax.scatter([goal_xy[0]],[goal_xy[1]],   s=60, c='k', label='Goal',  zorder=3)
    ax.legend(loc='upper left'); ax.figure.tight_layout()
    return path_line, traj_line, robot_dot

def animate(step_fn, fig, ax, dt: float, frames: int = 4000):
    # blit=False is more compatible across Windows backends and ensures first frame draws correctly
    return FuncAnimation(fig, step_fn, frames=frames, interval=int(dt * 1000), blit=False)
