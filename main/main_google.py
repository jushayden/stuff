# main/main_google.py
import os, sys
import numpy as np
import matplotlib.pyplot as plt

# add project root to path so 'nav' is importable
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nav.maps_google import get_directions, latlon_to_local_xy, resample_by_distance

# pick any two points (example: around Googleplex)
ORIGIN_LATLON = (37.4219999, -122.0840575)
DEST_LATLON   = (37.4228000, -122.0770000)
MODE = "driving"           # or "walking", "bicycling"
RESAMPLE_STEP_M = 1.5      # smaller = more points

def main():
    print("🔑 Checking key...")
    key = os.environ.get("GOOGLE_MAPS_API_KEY", "")
    print("Key set:", bool(key))

    print("📍 Requesting route from Directions API...")
    latlon_pts = get_directions(ORIGIN_LATLON, DEST_LATLON, mode=MODE)
    print(f"Got {len(latlon_pts)} lat/lon points.")

    print("📐 Converting to local XY (meters)...")
    xy = latlon_to_local_xy(latlon_pts, origin_latlon=ORIGIN_LATLON)
    xy = resample_by_distance(xy, step_m=RESAMPLE_STEP_M)
    print(f"Resampled to {len(xy)} points.")

    # quick plot of the route
    fig, ax = plt.subplots(figsize=(6,6))
    ax.plot(xy[:,0], xy[:,1], 'b-', lw=2, label="Google route")
    ax.plot(xy[0,0], xy[0,1], 'go', ms=6, label="Start")
    ax.plot(xy[-1,0], xy[-1,1], 'ko', ms=6, label="Goal")
    ax.set_aspect('equal', adjustable='box')
    ax.legend(loc='upper left')
    ax.set_title("Directions API route (meters, local frame)")
    # nice bounds
    xmin, ymin = xy.min(axis=0)
    xmax, ymax = xy.max(axis=0)
    pad = max(10.0, 0.1 * max(xmax-xmin, ymax-ymin))
    ax.set_xlim(xmin - pad, xmax + pad)
    ax.set_ylim(ymin - pad, ymax + pad)

    plt.show()

if __name__ == "__main__":
    main()
