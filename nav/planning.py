import math
import heapq
import numpy as np
from typing import Tuple, List, Optional

def cell_to_xy(cell: Tuple[int, int]) -> Tuple[float, float]:
    r, c = cell
    return float(c) + 0.5, float(r) + 0.5

def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

def astar(grid: np.ndarray, start: Tuple[int, int],
          goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    H, W = grid.shape
    g = {start: 0.0}
    parent = {start: None}
    pq = [(heuristic(start, goal), 0, start)]
    counter = 0
    moves = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),
             (-1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),
             (1,-1,math.sqrt(2)), (1,1,math.sqrt(2))]

    def in_bounds(r, c): return 0 <= r < H and 0 <= c < W

    while pq:
        _, _, cur = heapq.heappop(pq)
        if cur == goal:
            path = []
            n = cur
            while n is not None:
                path.append(n); n = parent[n]
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
