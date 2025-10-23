# main/main_google_follow_addresses.py
"""
Follow a route defined by human-readable addresses.
Shows a Static Maps background and animates Pure Pursuit.
"""
from dotenv import load_dotenv
load_dotenv()

import os, sys, numpy as np, matplotlib.pyplot as plt
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nav.maps_google import (
    geocode_address, get_directions, latlon_to_local_xy, resample_by_distance,
    fetch_static_map
)
from nav.control import State, pure_pursuit_step

# ---------- EDIT THESE ADDRESSES ----------
ORIGIN_ADDRESS = "1866 Royal Majesty Ct, Orlando, FL"
DEST_ADDRESS   = "2525 Paseo Park Rd, Orlando, FL"
MODE = "driving"              # "driving" | "walking" | "bicycling"
RESAMPLE_STEP_M = 1.5

# controller knobs
DT = 0.06
SPEED = 50.0
LOOKAHEAD = 5.0
ADVANCE_TOL = 0.8
NEAR_GOAL_RADIUS = 3.0
LOOKAHEAD_NEAR = 1.2
V_NEAR = 0.8
GOAL_TOL = 0.35
MAX_FRAMES = 6000

# static map visual
SHOW_STATIC_MAP = True
STATIC_ZOOM = 18  # 17..19
MAPTYPE = "roadmap"  # "roadmap" | "satellite" | "hybrid" | "terrain"

def main():
    if not os.environ.get("GOOGLE_MAPS_API_KEY"):
        raise RuntimeError("GOOGLE_MAPS_API_KEY not set")

    print("📌 Geocoding addresses...")
    origin_ll = geocode_address(ORIGIN_ADDRESS)
    dest_ll   = geocode_address(DEST_ADDRESS)
    print("  origin:", origin_ll)
    print("  dest  :", dest_ll)

    print("📍 Fetching route...")
    latlon_pts = get_directions(origin_ll, dest_ll, mode=MODE)
    xy = latlon_to_local_xy(latlon_pts, origin_latlon=origin_ll)
    waypoints = resample_by_distance(xy, step_m=RESAMPLE_STEP_M)
    print(f"  route points: {len(waypoints)}")

    # initial state
    sx, sy = waypoints[0]
    if len(waypoints) >= 2:
        dx0, dy0 = waypoints[1] - waypoints[0]
        theta0 = float(np.arctan2(dy0, dx0))
    else:
        theta0 = 0.0
    state = State(x=float(sx), y=float(sy), theta=theta0, progress_idx=0)
    goal_xy = (float(waypoints[-1,0]), float(waypoints[-1,1]))

    # figure / axes
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Google route from addresses (Pure Pursuit)")

    # optional static map background
    if SHOW_STATIC_MAP:
        img, m_per_px, (w_px, h_px) = fetch_static_map(origin_ll, zoom=STATIC_ZOOM, maptype=MAPTYPE)
        half_w_m = (w_px * m_per_px) / 2.0
        half_h_m = (h_px * m_per_px) / 2.0
        ax.imshow(img, origin="upper", extent=(-half_w_m, half_w_m, -half_h_m, half_h_m))

    (path_line,) = ax.plot(waypoints[:,0], waypoints[:,1], 'b-', lw=2, label="Google route")
    (traj_line,) = ax.plot([], [], 'r-', lw=2, label="Robot trajectory")
    (robot_dot,) = ax.plot([], [], 'ro', ms=5)
    ax.plot(sx, sy, 'go', ms=6, label="Start")
    ax.plot(goal_xy[0], goal_xy[1], 'ko', ms=6, label="Goal")
    ax.legend(loc="upper left")

    xmin, ymin = waypoints.min(axis=0)
    xmax, ymax = waypoints.max(axis=0)
    pad = max(10.0, 0.12 * max(xmax - xmin, ymax - ymin))
    ax.set_xlim(xmin - pad, xmax + pad)
    ax.set_ylim(ymin - pad, ymax + pad)

    traj_x, traj_y = [sx], [sy]
    traj_line.set_data(traj_x, traj_y)
    robot_dot.set_data([sx], [sy])

    def step(_):
        nonlocal state
        state, (px, py) = pure_pursuit_step(
            state, waypoints, goal_xy,
            dt=DT, speed=SPEED, lookahead=LOOKAHEAD,
            advance_tol=ADVANCE_TOL, near_goal_radius=NEAR_GOAL_RADIUS,
            lookahead_near=LOOKAHEAD_NEAR, v_near=V_NEAR, goal_tol=GOAL_TOL,
            dynamic_lookahead=True, L_min=1.5, L_max=8.0, curvature_gain=4.0
        )
        traj_x.append(px); traj_y.append(py)
        traj_line.set_data(traj_x, traj_y)
        robot_dot.set_data([px], [py])
        return traj_line, robot_dot

    # show first frame immediately
    step(0)
    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, step, frames=MAX_FRAMES, interval=int(DT*1000), blit=False)
    plt._nav_anim = anim
    plt.show()

if __name__ == "__main__":
    main()
