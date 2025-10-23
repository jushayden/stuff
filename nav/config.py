# ===============================
#  CONFIGURATION SETTINGS
# ===============================

class Config:
    """
    Holds all environment and map generation parameters.
    """
    def __init__(self):
        # --- Map/grid size (square grid: grid_size x grid_size) ---
        self.grid_size = 40          # 40x40 cells

        # --- Obstacle generation settings ---
        self.n_rects = 14            # how many rectangles I try to place
        self.density = 0.18          # overall "fullness" target
        self.min_rect = 3            # min rectangle side (cells)
        self.max_rect = 8            # max rectangle side (cells)

        # --- Safety and inflation ---
        self.inflation_radius = 2.0  # circular clearance in cells

        # --- Randomness ---
        self.seed = 7                # None = different each run; int = repeatable

        # --- Visualization ---
        self.style = "grid"          # "grid" (cell lines) or "map" (solid)
        self.animate = True          # True = live animation

class ControlParams:
    """
    Motion and controller settings.
    """
    def __init__(self):
        self.dt = 0.08               # time step (s)
        self.speed = 1.2             # forward speed
        self.lookahead = 4.0         # pure pursuit lookahead (cells)
        self.advance_tol = 0.6       # advance to next wp when this close

        # Final approach (so I actually reach the goal)
        self.near_goal_radius = 3.0
        self.lookahead_near = 1.0
        self.v_near = 0.6
        self.goal_tol = 0.25

        # Animation limit (frames)
        self.max_frames = 4000
