# Real-time animated 2D navigation demo:
# - A* global path on a grid
# - Pure Pursuit local controller with forward progress tracking
# - Live animation with Matplotlib

import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ------------------------------
# 1) Map (occupancy grid)
# ------------------------------
H, W = 30, 40
grid = np.zeros((H, W), dtype=np.uint8)

def add_rect(r0, c0, r1, c1):
    grid[r0:r1, c0:c1] = 1

# Obstacles
add_rect(8, 5, 22, 8)
add_rect(10, 15, 14, 33)
add_rect(18, 24, 26, 27)

start = (2, 2)
goal  = (26, 35)

# ------------------------------
# 2) A* planner (8-connected)
# ------------------------------
def heuristic(a, b):  # Euclidean
    return math.hypot(a[0]-b[0], a[1]-b[1])

def astar(occ_grid, start, goal):
    H, W = occ_grid.shape
    g = {start: 0.0}
    parent = {start: None}
    pq = [(heuristic(start, goal), 0, start)]
    counter = 0
    moves = [(-1,0,1), (1,0,1), (0,-1,1), (0,1,1),
             (-1,-1,math.sqrt(2)), (-1,1,math.sqrt(2)),
             (1,-1,math.sqrt(2)), (1,1,math.sqrt(2))]

    def in_bounds(r, c): return 0 <= r < H and 0 <= c < W

    while pq:
        _, _, current = heapq.heappop(pq)
        if current == goal:
            path = []
            n = current
            while n is not None:
                path.append(n)
                n = parent[n]
            return path[::-1]

        cr, cc = current
        for dr, dc, cost in moves:
            nr, nc = cr + dr, cc + dc
            if not in_bounds(nr, nc) or occ_grid[nr, nc] == 1:
                continue
            ng = g[current] + cost
            if (nr, nc) not in g or ng < g[(nr, nc)]:
                g[(nr, nc)] = ng
                parent[(nr, nc)] = current
                counter += 1
                fscore = ng + heuristic((nr, nc), goal)
                heapq.heappush(pq, (fscore, counter, (nr, nc)))
    return None

path = astar(grid, start, goal)
if path is None:
    raise SystemExit("No path found. Try moving obstacles/start/goal.")

# grid->metric
def cell_to_xy(cell):
    r, c = cell
    return float(c) + 0.5, float(r) + 0.5

waypoints = np.array([cell_to_xy(p) for p in path])

# Optional tiny smoothing (comment out if you want raw A*)
# for _ in range(2):
#     waypoints = (waypoints[:-2] + waypoints[1:-1] + waypoints[2:]) / 3

# ------------------------------
# 3) Pure Pursuit (with forward progress)
# ------------------------------
dt = 0.08          # sim timestep (s)
v  = 1.0           # forward speed (m/s)
lookahead    = 4.0 # aim farther ahead to avoid circling
goal_tol     = 1.2 # distance to accept goal
advance_tol  = 0.6 # when close to a wp, advance progress

x, y = cell_to_xy(start)
if len(waypoints) >= 2:
    dx0, dy0 = waypoints[1] - waypoints[0]
    theta = math.atan2(dy0, dx0)  # face first segment
else:
    theta = 0.0

progress_idx = 0   # "where we are" along the path
traj_x, traj_y = [], []

def angle_wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def nearest_index(px, py, pts, start_idx, window=12):
    i0 = start_idx
    i1 = min(len(pts), start_idx + window)
    d2 = np.sum((pts[i0:i1] - np.array([px, py]))**2, axis=1)
    return i0 + int(np.argmin(d2))

def find_lookahead_point(px, py, pts, L, start_idx):
    for i in range(start_idx, len(pts)):
        if np.hypot(pts[i][0]-px, pts[i][1]-py) >= L:
            return pts[i], i
    return pts[-1], len(pts) - 1

# ------------------------------
# 4) Plot + Animation
# ------------------------------
fig, ax = plt.subplots(figsize=(8, 6))
ax.imshow(grid, origin='lower', extent=(0, W, 0, H), alpha=0.6)
(ax_path,) = ax.plot(waypoints[:,0], waypoints[:,1], 'b-', lw=2, label='A* path')
(traj_line,) = ax.plot([], [], 'r-', lw=2, label='Robot trajectory')
(robot_dot,) = ax.plot([], [], 'ro', ms=5)
sx, sy = cell_to_xy(start); gx, gy = cell_to_xy(goal)
ax.scatter([sx],[sy], s=60, c='g', label='Start')
ax.scatter([gx],[gy], s=60, c='k', label='Goal')
ax.set_xlim(0, W); ax.set_ylim(0, H)
ax.set_aspect('equal', adjustable='box')
ax.set_title('Animated 2D Navigation (A* + Pure Pursuit)')
ax.legend(loc='upper left')
fig.tight_layout()

def init():
    traj_line.set_data([], [])
    robot_dot.set_data([], [])
    return traj_line, robot_dot

def step(_frame):
    global x, y, theta, progress_idx

    # Stop if at goal
    if math.hypot(gx - x, gy - y) < goal_tol:
        return traj_line, robot_dot

    # Advance along path when close
    if progress_idx < len(waypoints):
        if np.hypot(waypoints[progress_idx][0]-x, waypoints[progress_idx][1]-y) < advance_tol:
            progress_idx = min(progress_idx + 1, len(waypoints) - 1)

    # Lock progress to nearest forward waypoint
    progress_idx = nearest_index(x, y, waypoints, progress_idx, window=12)

    # Look ahead from progress
    target, _ = find_lookahead_point(x, y, waypoints, lookahead, progress_idx)

    # Pure Pursuit steering
    tx, ty = target
    alpha = angle_wrap(math.atan2(ty - y, tx - x) - theta)
    kappa = 2.0 * math.sin(alpha) / max(lookahead, 1e-6)
    omega = kappa * v

    # Unicycle integration
    x_new = x + v * math.cos(theta) * dt
    y_new = y + v * math.sin(theta) * dt
    theta_new = angle_wrap(theta + omega * dt)

    # Update trajectory buffers
    traj_x.append(x_new); traj_y.append(y_new)
    traj_line.set_data(traj_x, traj_y)
    robot_dot.set_data([x_new], [y_new])

    # Commit new state
    x, y, theta = x_new, y_new, theta_new
    return traj_line, robot_dot

anim = FuncAnimation(fig, step, init_func=init, frames=2000, interval=int(dt*1000), blit=True)
plt.show()
