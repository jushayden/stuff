"""
nav_random_demo.py  (adaptive final-approach so I actually reach the goal)
- I build a random grid map (0 free, 1 obstacle)
- I inflate obstacles with SciPy (circular clearance)
- I plan with A* on the inflated map
- I densify the path (controller gets more points)
- I follow it with Pure Pursuit + forward progress
- I animate, and I add a special "final approach" behavior near the goal
"""

#======IMPORTS======
import math
import heapq
import numpy as np
import matplotlib.pyplot as plt     
from matplotlib.animation import FuncAnimation
from scipy.ndimage import distance_transform_edt
from typing import Tuple, List, Optional

#======SETTINGS======
SEED                 = 7          # None = different map each run; set int for repeatability
H, W                 = 30, 40     # grid height (rows) and width (cols)
OBSTACLE_DENSITY     = 0.18       # roughly how full of obstacles I make the map
N_RECTS              = 14         # how many rectangular obstacles I attempt to place
MIN_RECT, MAX_RECT   = 3, 8       # rectangle size range (cells)
ROBOT_RADIUS_CELLS   = 2.0        # my clearance radius for inflation (in cells)
STYLE                = "grid"     # "grid" (with cell lines) or "map" (solid image)
ANIMATE              = True       # True = live animation; False = run then draw once

#======CONTROLLER BASE PARAMETERS======
DT            = 0.08              # sim timestep (s)
V_LINEAR      = 1.0               # cruise speed
LOOKAHEAD     = 4.0               # normal lookahead (cells ≈ meters)
ADVANCE_TOL   = 0.6               # when I'm this close to a waypoint I advance
DENSIFY_STEP  = 0.4               # path densification spacing (smaller = more points)

#======STOPS STOPPING EARLY======
NEAR_GOAL_RADIUS = 3.0            # when I'm within this, I switch to final-approach mode
LOOKAHEAD_NEAR   = 1.0            # smaller lookahead near goal
V_NEAR           = 0.6            # slower speed near goal
GOAL_TOL         = 0.25           # much tighter stop tolerance so I actually hit it

#====== SMALL UTILITIES ======
def rng_seed(seed: Optional[int]) -> np.random.Generator:
    return np.random.default_rng(seed)

def cell_to_xy(cell: Tuple[int, int]) -> Tuple[float, float]:
    r, c = cell
    return float(c) + 0.5, float(r) + 0.5  # I plot at the center of the cell

def angle_wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

#====== MAP GENERATION======
def make_random_grid(h: int, w: int, n_rects: int, density: float,
                     min_rect: int, max_rect: int, rng: np.random.Generator) -> np.ndarray:
    grid = np.zeros((h, w), dtype=np.uint8)                 # start all free
    placed = 0
    target_blocks = int(h * w * density)
    while placed < n_rects and grid.sum() < target_blocks:  # stamp rectangles
        rh = int(rng.integers(min_rect, max_rect + 1))
        rw = int(rng.integers(min_rect, max_rect + 1))
        r0 = int(rng.integers(0, max(1, h - rh)))
        c0 = int(rng.integers(0, max(1, w - rw)))
        grid[r0:r0 + rh, c0:c0 + rw] = 1
        placed += 1
    # poke a few holes so the map isn't over-blocked
    for _ in range(3):
        r = int(rng.integers(0, h))
        grid[r, :] = np.minimum(grid[r, :], rng.integers(0, 2, size=w))
    return grid

def random_free_cell(grid: np.ndarray, rng: np.random.Generator) -> Tuple[int, int]:
    Hh, Ww = grid.shape
    for _ in range(10000):
        r = int(rng.integers(0, Hh))
        c = int(rng.integers(0, Ww))
        if grid[r, c] == 0:
            return (r, c)
    raise RuntimeError("Map too blocked; lower OBSTACLE_DENSITY.")

# ====== INFLATION (SciPy distance transform) ======
def inflate_obstacles_circular(grid: np.ndarray, radius_cells: float) -> np.ndarray:
    # distance-from-obstacle on free space (1-grid because obstacles are 1s)
    dist_to_obstacle = distance_transform_edt(1 - grid)
    # anything with less clearance than my robot radius becomes blocked
    return (dist_to_obstacle <= radius_cells).astype(np.uint8)

# ====== A* PATH PLANNER ======
def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

def astar(grid: np.ndarray,
          start: Tuple[int, int],
          goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    Hh, Ww = grid.shape
    g = {start: 0.0}
    parent = {start: None}
    pq = [(heuristic(start, goal), 0, start)]
    counter = 0
    moves = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),
             (-1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),
             (1,-1,math.sqrt(2)), (1,1,math.sqrt(2))]

    def in_bounds(r, c): return 0 <= r < Hh and 0 <= c < Ww

    while pq:
        _, _, cur = heapq.heappop(pq)
        if cur == goal:
            path = []
            n = cur
            while n is not None:
                path.append(n)
                n = parent[n]
            return path[::-1]
        cr, cc = cur
        for dr, dc, cost in moves:
            nr, nc = cr + dr, cc + dc
            if not in_bounds(nr, nc) or grid[nr, nc] == 1:
                continue
            ng = g[cur] + cost
            if (nr, nc) not in g or ng < g[(nr, nc)]:
                g[(nr, nc)] = ng
                parent[(nr, nc)] = cur
                counter += 1
                f = ng + heuristic((nr, nc), goal)
                heapq.heappush(pq, (f, counter, (nr, nc)))
    return None

# ====== PATH UTILITIES ======
def densify(poly: np.ndarray, step: float = 0.4) -> np.ndarray:
    out = [poly[0]]
    for i in range(len(poly) - 1):
        p, q = poly[i], poly[i + 1]
        seg = q - p
        L = float(np.hypot(seg[0], seg[1]))
        n = max(1, int(L / step))
        for k in range(1, n + 1):
            out.append(p + seg * (k / n))
    return np.array(out)

# ====== PURE PURSUIT HELPERS ======
def nearest_index(px: float, py: float, pts: np.ndarray, start_idx: int, window: int = 12) -> int:
    i0 = start_idx
    i1 = min(len(pts), start_idx + window)
    d2 = np.sum((pts[i0:i1] - np.array([px, py]))**2, axis=1)
    return i0 + int(np.argmin(d2))

def find_lookahead(px: float, py: float, pts: np.ndarray, L: float, start_idx: int) -> Tuple[np.ndarray, int]:
    for i in range(start_idx, len(pts)):
        if np.hypot(pts[i][0] - px, pts[i][1] - py) >= L:
            return pts[i], i
    return pts[-1], len(pts) - 1

# ====== DRAWING HELPERS ======
def draw_grid(ax, grid: np.ndarray):
    from matplotlib.colors import ListedColormap
    Hh, Ww = grid.shape
    cmap = ListedColormap(["#4E3A63", "#F7EA75"])              # purple free, yellow obstacles
    ax.imshow(grid, origin="lower", extent=(0, Ww, 0, Hh), cmap=cmap, interpolation="nearest")
    for x in range(Ww + 1): ax.axvline(x, color=(1,1,1,0.08), lw=0.6)
    for y in range(Hh + 1): ax.axhline(y, color=(1,1,1,0.08), lw=0.6)

def draw_map(ax, grid: np.ndarray):
    ax.imshow(grid, origin="lower", extent=(0, grid.shape[1], 0, grid.shape[0]),
              cmap=plt.cm.gray_r, interpolation="nearest")

def overlay_inflated(ax, inflated: np.ndarray):
    from matplotlib.colors import ListedColormap
    ax.imshow(inflated, origin='lower', extent=(0, inflated.shape[1], 0, inflated.shape[0]),
              cmap=ListedColormap([(0,0,0,0), (1,0,0,0.12)]), interpolation='nearest')

# ====== MAIN ======
def main():
    rng = rng_seed(SEED)

    # -- make random map and inflated clearance map
    grid = make_random_grid(H, W, N_RECTS, OBSTACLE_DENSITY, MIN_RECT, MAX_RECT, rng)
    inflated = inflate_obstacles_circular(grid, ROBOT_RADIUS_CELLS)

    # -- pick start/goal that are free in both maps and not too close together
    start = random_free_cell(grid, rng)
    while inflated[start] == 1:
        start = random_free_cell(grid, rng)
    goal = random_free_cell(grid, rng)
    while inflated[goal] == 1 or heuristic(start, goal) < min(H, W) / 3:
        goal = random_free_cell(grid, rng)

    # -- plan with A* on the inflated map; retry a few times if unlucky
    path = astar(inflated, start, goal)
    tries = 0
    while path is None and tries < 10:
        grid = make_random_grid(H, W, N_RECTS, OBSTACLE_DENSITY, MIN_RECT, MAX_RECT, rng)
        inflated = inflate_obstacles_circular(grid, ROBOT_RADIUS_CELLS)
        start = random_free_cell(grid, rng)
        while inflated[start] == 1:
            start = random_free_cell(grid, rng)
        goal = random_free_cell(grid, rng)
        while inflated[goal] == 1 or heuristic(start, goal) < min(H, W) / 3:
            goal = random_free_cell(grid, rng)
        path = astar(inflated, start, goal)
        tries += 1
    if path is None:
        raise SystemExit("No path found. Lower OBSTACLE_DENSITY or radius.")

    # -- cells -> metric, then densify
    waypoints = np.array([cell_to_xy(p) for p in path])
    waypoints = densify(waypoints, step=DENSIFY_STEP)

    # -- initial pose facing the first segment (helps avoid circling)
    x, y = cell_to_xy(start)
    if len(waypoints) >= 2:
        dx0, dy0 = waypoints[1] - waypoints[0]
        theta = math.atan2(dy0, dx0)
    else:
        theta = 0.0
    progress_idx = 0
    traj_x, traj_y = [], []

    # -- plotting
    fig, ax = plt.subplots(figsize=(9, 6))
    if STYLE == "grid": draw_grid(ax, grid)
    else:               draw_map(ax, grid)
    overlay_inflated(ax, inflated)
    (path_line,) = ax.plot(waypoints[:,0], waypoints[:,1], 'b-', lw=2, label='A* path')
    (traj_line,) = ax.plot([], [], 'r-', lw=2, label='Robot trajectory')
    (robot_dot,) = ax.plot([], [], 'ro', ms=5)
    sx, sy = cell_to_xy(start); gx, gy = cell_to_xy(goal)
    ax.scatter([sx],[sy], s=60, c='g', label='Start', zorder=3)
    ax.scatter([gx],[gy], s=60, c='k', label='Goal',  zorder=3)
    ax.set_xlim(0, W); ax.set_ylim(0, H); ax.set_aspect('equal', adjustable='box')
    ax.set_title('Random 2D Navigation (A* + Pure Pursuit, adaptive final approach)')
    ax.legend(loc='upper left'); fig.tight_layout()

    # -- animation step
    def step(_frame):
        nonlocal x, y, theta, progress_idx

        # distance to goal (I use this a lot)
        dg = math.hypot(gx - x, gy - y)

        # if I'm basically at the goal, I stop updating
        if dg < GOAL_TOL:
            return traj_line, robot_dot

        # choose parameters (normal vs near-goal)
        if dg <= NEAR_GOAL_RADIUS:
            L = LOOKAHEAD_NEAR   # smaller lookahead so I don't overshoot
            v = V_NEAR           # slow down for precision
            target = np.array([gx, gy])  # explicitly aim at the exact goal
        else:
            L = LOOKAHEAD
            v = V_LINEAR
            # forward progress housekeeping
            if progress_idx < len(waypoints):
                if np.hypot(waypoints[progress_idx][0] - x, waypoints[progress_idx][1] - y) < ADVANCE_TOL:
                    progress_idx = min(progress_idx + 1, len(waypoints) - 1)
            progress_idx = nearest_index(x, y, waypoints, progress_idx, window=12)
            target, _ = find_lookahead(x, y, waypoints, L, progress_idx)

        # Pure Pursuit steering toward my chosen target
        tx, ty = target
        alpha = angle_wrap(math.atan2(ty - y, tx - x) - theta)
        kappa = 2.0 * math.sin(alpha) / max(L, 1e-6)
        omega = kappa * v

        # one unicycle integration step
        x_new = x + v * math.cos(theta) * DT
        y_new = y + v * math.sin(theta) * DT
        theta_new = angle_wrap(theta + omega * DT)

        # update trajectory plot
        traj_x.append(x_new); traj_y.append(y_new)
        traj_line.set_data(traj_x, traj_y)
        robot_dot.set_data([x_new], [y_new])

        # commit the new state
        x, y, theta = x_new, y_new, theta_new
        return traj_line, robot_dot

    if ANIMATE:
        anim = FuncAnimation(fig, step, frames=4000, interval=int(DT*1000), blit=True)
        plt.show()
    else:
        for _ in range(4000):
            step(_)
            if math.hypot(gx - x, gy - y) < GOAL_TOL:
                break
        plt.show()

# ====== GO ======
if __name__ == "__main__":
    main()
