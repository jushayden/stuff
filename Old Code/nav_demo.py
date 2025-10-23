import math
import heapq
import numpy as np
import matplotlib.pyplot as plt

# ------------------------------
# 1) Build an example occupancy grid
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
# 2) A* path planning
# ------------------------------
def heuristic(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def astar(occ_grid, start, goal):
    H, W = occ_grid.shape
    g = {start: 0.0}
    parent = {start: None}
    pq = [(heuristic(start, goal), 0, start)]
    counter = 0

    moves = [(-1,0,1), (1,0,1), (0,-1,1), (0,1,1),
             (-1,-1,1.4), (-1,1,1.4), (1,-1,1.4), (1,1,1.4)]

    def in_bounds(r, c): return 0 <= r < H and 0 <= c < W

    while pq:
        _, _, current = heapq.heappop(pq)
        if current == goal:
            # Reconstruct path
            path = []
            n = current
            while n is not None:
                path.append(n)
                n = parent[n]
            return path[::-1]
        for dr, dc, cost in moves:
            nr, nc = current[0]+dr, current[1]+dc
            if not in_bounds(nr, nc): continue
            if occ_grid[nr, nc] == 1: continue
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
    raise SystemExit("No path found. Try changing obstacles.")

# ------------------------------
# 3) Convert grid cells -> XY waypoints
# ------------------------------
def cell_to_xy(cell):
    r, c = cell
    return float(c) + 0.5, float(r) + 0.5

waypoints = np.array([cell_to_xy(p) for p in path])

# ------------------------------
# 4) Pure Pursuit controller (with forward progress)
# ------------------------------
dt = 0.1            # slower update for stability
v = 1.0             # forward speed
lookahead = 4.0     # look farther ahead to avoid circling
goal_tol = 1.2      # accept goal when reasonably close
advance_tol = 0.6   # when this close to a waypoint, advance progress

x, y = cell_to_xy(start)

# Face the direction of the first segment (helps avoid initial orbit)
if len(waypoints) >= 2:
    dx0, dy0 = waypoints[1][0] - waypoints[0][0], waypoints[1][1] - waypoints[0][1]
    theta = math.atan2(dy0, dx0)
else:
    theta = 0.0

traj = []
progress_idx = 0  # index of the 'current' waypoint we've reached or passed

def angle_wrap(a): 
    return (a + math.pi) % (2 * math.pi) - math.pi

def nearest_index(px, py, pts, start_idx, window=8):
    """Find nearest waypoint index in a small forward window."""
    i0 = start_idx
    i1 = min(len(pts), start_idx + window)
    d2 = np.sum((pts[i0:i1] - np.array([px, py]))**2, axis=1)
    return i0 + int(np.argmin(d2))

def find_lookahead_point(px, py, pts, L, start_idx):
    """
    Find the first waypoint at least L ahead, searching only forward
    from start_idx. If none, return the last waypoint.
    """
    for i in range(start_idx, len(pts)):
        if np.hypot(pts[i][0] - px, pts[i][1] - py) >= L:
            return pts[i], i
    return pts[-1], len(pts) - 1

for _ in range(4000):
    traj.append((x, y))

    # Goal check
    gx, gy = cell_to_xy(goal)
    if math.hypot(gx - x, gy - y) < goal_tol:
        break

    # Advance progress if we're close to the current waypoint
    if progress_idx < len(waypoints):
        if np.hypot(waypoints[progress_idx][0] - x, waypoints[progress_idx][1] - y) < advance_tol:
            progress_idx = min(progress_idx + 1, len(waypoints) - 1)

    # Keep progress tied to the nearest forward waypoint (prevents falling back)
    progress_idx = nearest_index(x, y, waypoints, progress_idx, window=12)

    # Choose a target ahead of the current progress
    target, target_idx = find_lookahead_point(x, y, waypoints, lookahead, progress_idx)

    # Pure Pursuit steering
    tx, ty = target
    alpha = angle_wrap(math.atan2(ty - y, tx - why x) - theta)
    kappa = 2.0 * math.sin(alpha) / max(lookahead, 1e-6)
    omega = kappa * v

    # Unicycle integration
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta = angle_wrap(theta + omega * dt)

traj = np.array(traj)


# ------------------------------
# 5) Plot the result
# ------------------------------
fig, ax = plt.subplots(figsize=(8, 6))
ax.imshow(grid, origin='lower', extent=(0, W, 0, H), alpha=0.6)
ax.plot(waypoints[:,0], waypoints[:,1], 'b-', linewidth=2, label='A* path')
ax.plot(traj[:,0], traj[:,1], 'r-', linewidth=2, label='Robot trajectory')
sx, sy = cell_to_xy(start); gx, gy = cell_to_xy(goal)
ax.scatter([sx], [sy], s=60, c='g', label='Start')
ax.scatter([gx], [gy], s=60, c='k', label='Goal')
ax.set_aspect('equal')
ax.legend()
ax.set_title('Simple 2D Navigation Demo (A* + Pure Pursuit)')
plt.show()

