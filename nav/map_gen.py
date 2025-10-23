import numpy as np
from scipy.ndimage import distance_transform_edt

# ===============================
#  MAP GENERATION UTILITIES
# ===============================

def make_random_grid(cfg):
    """
    Generates a random binary occupancy grid using parameters from cfg.
    0 = free, 1 = obstacle.
    """
    w = cfg.grid_size
    n_rects = cfg.n_rects
    density = cfg.density
    min_rect = cfg.min_rect
    max_rect = cfg.max_rect

    rng = np.random.default_rng(cfg.seed)
    grid = np.zeros((w, w), dtype=np.uint8)

    placed = 0
    target_blocks = int(w * w * density)
    # Try placing rectangles until we roughly hit target density
    while placed < n_rects and grid.sum() < target_blocks:
        rh = int(rng.integers(min_rect, max_rect + 1))
        rw = int(rng.integers(min_rect, max_rect + 1))
        r0 = int(rng.integers(0, max(1, w - rh)))
        c0 = int(rng.integers(0, max(1, w - rw)))
        grid[r0:r0 + rh, c0:c0 + rw] = 1
        placed += 1

    # Punch a few random holes across rows so maps are usually solvable
    for _ in range(3):
        r = int(rng.integers(0, w))
        grid[r, :] = np.minimum(grid[r, :], rng.integers(0, 2, size=w))

    return grid

def inflate_obstacles_circular(grid, radius):
    """
    Inflate obstacles by a circular radius (in cells) using distance transform.
    """
    dist = distance_transform_edt(1 - grid)  # distance to nearest obstacle on free space
    return (dist <= radius).astype(np.uint8)

def random_free_cell(grid):
    """
    Returns a random free (row, col) cell from the given grid.
    """
    free = np.argwhere(grid == 0)
    if free.size == 0:
        raise RuntimeError("No free cells available; map too blocked.")
    idx = np.random.randint(len(free))
    r, c = free[idx]
    return int(r), int(c)
