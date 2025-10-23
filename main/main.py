import os
import sys
import time
import numpy as np

# Make 'nav' package importable (add project root to sys.path)
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nav.config import Config, ControlParams
from nav.map_gen import make_random_grid, inflate_obstacles_circular, random_free_cell
from nav.planning import astar, densify, cell_to_xy
from nav.control import State, pure_pursuit_step
from nav.viz import setup_axes, overlay_inflated, create_artists, animate

# ---------- Fancy boot messages ----------
def boot_sequence():
    print("🤖 Booting robot navigation system...")
    time.sleep(0.6)
    stages = [
        "Initializing CPU modules...",
        "Loading navigation parameters...",
        "Calibrating sensors...",
        "Scanning environment interfaces...",
        "Starting pathfinding core...",
        "Loading visual renderer..."
    ]
    for i, stage in enumerate(stages):
        print(f"[{i+1}/{len(stages)}] {stage}")
        time.sleep(0.45)
    print("✅ All systems operational.")
    print("────────────────────────────────")
    time.sleep(0.5)

def progress_bar(task_name, seconds=2.0):
    print(task_name)
    length = 22
    for i in range(length + 1):
        bar = "█" * i + "-" * (length - i)
        pct = int((i / length) * 100)
        print(f"\r[{bar}] {pct}%", end="")
        time.sleep(seconds / length)
    print("\n")

# ---------- Main ----------
def main():
    boot_sequence()
    progress_bar("🧠 Initializing navigation environment...", 1.8)

    cfg = Config()
    ctrl = ControlParams()

    # 1) Map
    print("🗺️  Generating random map grid...")
    grid = make_random_grid(cfg)

    # 2) Inflate obstacles (clearance)
    print("🧱  Inflating obstacles for safe navigation...")
    inflated = inflate_obstacles_circular(grid, cfg.inflation_radius)

    # 3) Pick start/goal (on inflated map)
    print("🎯  Selecting start and goal points...")
    start_cell = random_free_cell(inflated)
    goal_cell  = random_free_cell(inflated)
    print(f"   Start = {start_cell}, Goal = {goal_cell}")

    # 4) Plan A* on inflated map
    print("🧮  Computing optimal path (A*)...")
    path = astar(inflated, start_cell, goal_cell)
    if path is None:
        print("❌ No path found. Try running again (or lower density).")
        sys.exit(1)

    # 5) Convert to metric and densify
    print("🧩  Densifying path for smooth pursuit...")
    waypoints = np.array([cell_to_xy(p) for p in path])
    waypoints = densify(waypoints, step=0.4)

    # 6) Initial pose faces first segment
    sx, sy = cell_to_xy(start_cell)
    if len(waypoints) >= 2:
        dx0, dy0 = waypoints[1] - waypoints[0]
        theta0 = float(np.arctan2(dy0, dx0))
    else:
        theta0 = 0.0
    state = State(x=sx, y=sy, theta=theta0, progress_idx=0)
    goal_xy = cell_to_xy(goal_cell)

    # 7) Plotting
    print("🖼️  Launching simulation display...")
    fig, ax = setup_axes(grid, style=cfg.style)   # <<< pass the GRID here
    overlay_inflated(ax, inflated)
    path_line, traj_line, robot_dot = create_artists(ax, waypoints, (sx, sy), goal_xy)

    # --- make the trajectory visible immediately ---
    traj_x, traj_y = [sx], [sy]        # seed with start point
    traj_line.set_data(traj_x, traj_y) # show a red segment right away
    robot_dot.set_data([sx], [sy])     # show the robot at the start


    # 8) Animation step function
    def step(_frame):
        nonlocal state
        state, (px, py) = pure_pursuit_step(
            state, waypoints, goal_xy,
            dt=ctrl.dt,
            speed=ctrl.speed,
            lookahead=ctrl.lookahead,
            advance_tol=ctrl.advance_tol,
            near_goal_radius=ctrl.near_goal_radius,
            lookahead_near=ctrl.lookahead_near,
            v_near=ctrl.v_near,
            goal_tol=ctrl.goal_tol,
        )
        traj_x.append(px); traj_y.append(py)
        traj_line.set_data(traj_x, traj_y)
        robot_dot.set_data([px], [py])
        return traj_line, robot_dot

    print("🚗  Engaging control loop...")

    # (A) Prime one step so the red dot/line are visible immediately
    step(0)

    # (B) Create and KEEP a reference to the animation object
    import matplotlib.pyplot as plt
    anim = animate(step, fig, ax, dt=ctrl.dt, frames=ctrl.max_frames)

    # Extra safety: store on a global so GC can’t collect it early in some IDEs
    plt._nav_anim = anim

    # (C) Show the window (blocks until you close it)
    plt.show()


    print("✅ Simulation complete.")

if __name__ == "__main__":
    main()