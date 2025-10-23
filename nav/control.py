# nav/control.py
import math
import numpy as np
from dataclasses import dataclass
from typing import Tuple

# -----------------------------
# Robot state container
# -----------------------------
@dataclass
class State:
    x: float
    y: float
    theta: float
    progress_idx: int = 0  # which waypoint we’re roughly near / “ahead of”

# -----------------------------
# Small helpers
# -----------------------------
def angle_wrap(a: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi

def nearest_index(px: float, py: float, pts: np.ndarray, start_idx: int, window: int = 12) -> int:
    """
    Search forward in a small window for the closest waypoint.
    Keeps us from snapping back to earlier points.
    """
    i0 = start_idx
    i1 = min(len(pts), start_idx + window)
    d2 = np.sum((pts[i0:i1] - np.array([px, py]))**2, axis=1)
    return i0 + int(np.argmin(d2))

def find_lookahead(px: float, py: float, pts: np.ndarray, L: float, start_idx: int):
    """
    Find the first point that is at least L meters ahead of (px,py),
    scanning forward from start_idx.
    """
    for i in range(start_idx, len(pts)):
        if np.hypot(pts[i][0] - px, pts[i][1] - py) >= L:
            return pts[i], i
    return pts[-1], len(pts) - 1

def estimate_curvature(pts: np.ndarray, idx: int, window: int = 3) -> float:
    """
    Tiny curvature heuristic using two short segments ahead.
    Larger result = sharper turn coming.
    """
    if idx + window + 1 >= len(pts):
        return 0.0
    p0 = pts[idx]
    p1 = pts[idx + 1]
    p2 = pts[idx + window]
    v1 = p1 - p0
    v2 = p2 - p1
    n1 = np.linalg.norm(v1) + 1e-9
    n2 = np.linalg.norm(v2) + 1e-9
    cosang = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    ang = np.arccos(cosang)
    return float(ang / (n1 + 1e-6))  # “angle per length” ~ curvature

# -----------------------------
# Pure Pursuit (with optional dynamic lookahead)
# -----------------------------
def pure_pursuit_step(
    state: State,
    waypoints: np.ndarray,
    goal_xy: Tuple[float, float],
    dt: float,
    speed: float,
    lookahead: float,
    advance_tol: float,
    near_goal_radius: float,
    lookahead_near: float,
    v_near: float,
    goal_tol: float,
    # ---- adaptive lookahead knobs (optional) ----
    dynamic_lookahead: bool = True,
    L_min: float = 1.5,
    L_max: float = 6.0,
    curvature_gain: float = 4.0,
):
    """
    One control step for a unicycle model following waypoints with Pure Pursuit.
    Returns (new_state, (x_new, y_new)).
    """
    x, y, theta, progress_idx = state.x, state.y, state.theta, state.progress_idx
    gx, gy = goal_xy
    dg = float(np.hypot(gx - x, gy - y))

    # goal reached?
    if dg < goal_tol:
        return State(x, y, theta, progress_idx), (x, y)

    # Near-goal homing mode: shrink lookahead & speed to finish cleanly
    if dg <= near_goal_radius:
        L = lookahead_near
        v = v_near
        target = np.array([gx, gy])
    else:
        # Forward progress: advance to next waypoint when we get close
        if progress_idx < len(waypoints):
            if np.hypot(waypoints[progress_idx][0] - x,
                        waypoints[progress_idx][1] - y) < advance_tol:
                progress_idx = min(progress_idx + 1, len(waypoints) - 1)

        # Search forward for the nearest waypoint in a small window
        progress_idx = nearest_index(x, y, waypoints, progress_idx, window=12)

        # Adaptive lookahead based on upcoming curvature
        if dynamic_lookahead:
            curv = estimate_curvature(waypoints, progress_idx, window=3)
            L = float(np.clip(lookahead * (1.5 - curvature_gain * curv), L_min, L_max))
        else:
            L = lookahead

        v = speed
        target, _ = find_lookahead(x, y, waypoints, L, progress_idx)

    # Pure pursuit steering geometry
    tx, ty = target
    alpha = angle_wrap(math.atan2(ty - y, tx - x) - theta)
    kappa = 2.0 * math.sin(alpha) / max(L, 1e-6)  # curvature command
    omega = kappa * v

    # Integrate simple unicycle model
    x_new = x + v * math.cos(theta) * dt
    y_new = y + v * math.sin(theta) * dt
    theta_new = angle_wrap(theta + omega * dt)

    return State(x_new, y_new, theta_new, progress_idx), (x_new, y_new)
