# main/main_google_follow.py
"""
Follow a real Google Directions route with the same Pure Pursuit controller
you used in the grid demo.

Blue  = Google route (resampled in meters)
Red   = Robot trajectory (Pure Pursuit, dynamic lookahead)
Green = Start
Black = Goal
"""
from dotenv import load_dotenv
load_dotenv()

import os, sys
import numpy as np
import matplotlib.pyplot as plt

# Make 'nav' importable (project root on sys.path)
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nav.maps_google import get_directions, latlon_to_local_xy, resample_by_distance
from nav.control import State, pure_pursuit_step

# -----------------------------
# User knobs (edit these)
# -----------------------------
# Example near the Googleplex; replace with your own coordinates
ORIGIN_LATLON = (37.4219999, -122.0840575)  # start lat, lon
DEST_LATLON   = (37.4228000, -122.0770000)  # goal  lat, lon
MODE = "driving"          # "driving" | "walking" | "bicycling"
RESAMPLE_STEP_M = 1.5     # even spacing (m) along the route

# Controller / animation params
DT = 0.06                 # sim timestep (s)
SPEED = 5.0               # m/s robot nominal speed
LOOKAHEAD = 6.0           # base lookahead (m) before dynamic adjustment
ADVANCE_TOL = 0.8         # advance progress when within this (m) of a waypoint
NEAR_GOAL_RADIUS = 3.0    # when within this (m), switch to "goal homing" mode
LOOKAHEAD_NEAR = 1.0      # lookahead near goal (m)
V_NEAR = 0.8              # slow down near goal (m/s)
GOAL_TOL = 0.35           # done when closer than this (m)
MAX_FRAMES = 6000

def main():
    # --- fetch a route from the Directions API
    if not os.environ.get("GOOGLE_MAPS_API_KEY"):
        raise RuntimeError("GOOGLE_MAPS_API_KEY not set. In PowerShell: "
                           '$env:GOOGLE_MAPS_API_KEY = "YOUR_KEY_HERE"')

    print("📍 Requesting route from Google Directions API...")
    latlon_pts = get_directions(ORIGIN_LATLON, DEST_LATLON, mode=MODE)
    print(f"  Got {len(latlon_pts)} lat/lon points")

    # --- convert to local XY (meters) and resample to even spacing
    xy = latlon_to_local_xy(latlon_pts, origin_latlon=ORIGIN_LATLON)
    waypoints = resample_by_distance(xy, step_m=RESAMPLE_STEP_M)
    print(f"  Resampled to {len(waypoints)} evenly spaced points")

    # --- initial state: place robot at first point and face next point
    sx, sy = waypoints[0]
    if len(waypoints) >= 2:
        dx0, dy0 = waypoints[1] - waypoints[0]
        theta0 = float(np.arctan2(dy0, dx0))
    else:
        theta0 = 0.0
    state = State(x=float(sx), y=float(sy), theta=theta0, progress_idx=0)
    goal_xy = (float(waypoints[-1,0]), float(waypoints[-1,1]))

    # --- figure/axes
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Following a Google Directions route (Pure Pursuit)")

    # draw path + artists we update during animation
    (path_line,) = ax.plot(waypoints[:,0], waypoints[:,1], 'b-', lw=2, label="Google route")
    (traj_line,) = ax.plot([], [], 'r-', lw=2, label="Robot trajectory")
    (robot_dot,) = ax.plot([], [], 'ro', ms=5)
    ax.plot(sx, sy, 'go', ms=6, label="Start")
    ax.plot(goal_xy[0], goal_xy[1], 'ko', ms=6, label="Goal")
    ax.legend(loc="upper left")

    # nice bounds
    xmin, ymin = waypoints.min(axis=0)
    xmax, ymax = waypoints.max(axis=0)
    pad = max(10.0, 0.12 * max(xmax - xmin, ymax - ymin))
    ax.set_xlim(xmin - pad, xmax + pad)
    ax.set_ylim(ymin - pad, ymax + pad)

    # seed trajectory so the red line is visible immediately
    traj_x, traj_y = [sx], [sy]
    traj_line.set_data(traj_x, traj_y)
    robot_dot.set_data([sx], [sy])

    # --- one step of control/physics
    def step(_):
        nonlocal state
        state, (px, py) = pure_pursuit_step(
            state,
            waypoints=waypoints,
            goal_xy=goal_xy,
            dt=DT,
            speed=SPEED,
            lookahead=LOOKAHEAD,
            advance_tol=ADVANCE_TOL,
            near_goal_radius=NEAR_GOAL_RADIUS,
            lookahead_near=LOOKAHEAD_NEAR,
            v_near=V_NEAR,
            goal_tol=GOAL_TOL,
            # enable adaptive lookahead (controller supports these kwargs)
            dynamic_lookahead=True,
            L_min=1.5, L_max=8.0, curvature_gain=4.0
        )
        traj_x.append(px); traj_y.append(py)
        traj_line.set_data(traj_x, traj_y)
        robot_dot.set_data([px], [py])
        return traj_line, robot_dot

    # prime one frame so the red dot shows even before animation starts
    step(0)

    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, step, frames=MAX_FRAMES, interval=int(DT*1000), blit=False)
    plt._nav_anim = anim  # keep a strong reference so it won't be GC'd
    plt.show()

if __name__ == "__main__":
    main()
