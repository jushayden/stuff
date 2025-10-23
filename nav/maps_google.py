from dotenv import load_dotenv
load_dotenv()

# nav/maps_google.py
import os
import io
import math
import requests
import numpy as np
import polyline as pl
from PIL import Image

R_EARTH = 6371000.0  # meters

# -----------------------------
# API key
# -----------------------------
def get_api_key() -> str:
    key = os.environ.get("GOOGLE_MAPS_API_KEY", "").strip()
    if not key:
        raise RuntimeError("GOOGLE_MAPS_API_KEY is not set")
    return key

# -----------------------------
# Directions (lat/lon polyline)
# -----------------------------
def get_directions(origin_latlon, dest_latlon, mode="driving"):
    key = get_api_key()
    url = "https://maps.googleapis.com/maps/api/directions/json"
    params = {
        "origin": f"{origin_latlon[0]},{origin_latlon[1]}",
        "destination": f"{dest_latlon[0]},{dest_latlon[1]}",
        "mode": mode,
        "key": key,
    }
    r = requests.get(url, params=params, timeout=15)
    r.raise_for_status()
    data = r.json()
    if data.get("status") != "OK" or not data.get("routes"):
        raise RuntimeError(f"Directions API error: {data.get('status')} {data.get('error_message','')}")
    poly = data["routes"][0]["overview_polyline"]["points"]
    latlon_list = pl.decode(poly)  # list[(lat, lon), ...]
    return latlon_list

# -----------------------------
# Geocoding (address -> lat/lon)
# -----------------------------
def geocode_address(address: str):
    """
    Returns (lat, lon) for a human-readable address.
    Requires Geocoding API to be enabled for your key.
    """
    key = get_api_key()
    url = "https://maps.googleapis.com/maps/api/geocode/json"
    params = {"address": address, "key": key}
    r = requests.get(url, params=params, timeout=15)
    r.raise_for_status()
    data = r.json()
    if data.get("status") != "OK" or not data.get("results"):
        raise RuntimeError(f"Geocoding error: {data.get('status')} {data.get('error_message','')}")
    loc = data["results"][0]["geometry"]["location"]
    return (float(loc["lat"]), float(loc["lng"]))

# -----------------------------
# Coordinate transforms
# -----------------------------
def latlon_to_local_xy(latlon_list, origin_latlon):
    """
    Convert (lat,lon) to local (x,y) meters using equirectangular projection.
    Origin (0,0) is origin_latlon.
    """
    lat0 = math.radians(origin_latlon[0])
    lon0 = math.radians(origin_latlon[1])
    xs, ys = [], []
    for lat, lon in latlon_list:
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)
        dlon = lon_r - lon0
        dlat = lat_r - lat0
        x = R_EARTH * math.cos(lat0) * dlon
        y = R_EARTH * dlat
        xs.append(x); ys.append(y)
    return np.column_stack([np.array(xs, dtype=float), np.array(ys, dtype=float)])

def resample_by_distance(pts_xy: np.ndarray, step_m: float = 2.0) -> np.ndarray:
    """
    Evenly sample points along the polyline at given spacing.
    """
    out = [pts_xy[0]]
    for i in range(len(pts_xy) - 1):
        p, q = pts_xy[i], pts_xy[i + 1]
        seg = q - p
        L = float(np.hypot(seg[0], seg[1]))
        n = max(1, int(L / step_m))
        for k in range(1, n + 1):
            out.append(p + seg * (k / n))
    return np.array(out)

# -----------------------------
# Static Maps (background image)
# -----------------------------
def ground_resolution_m_per_px(lat_deg: float, zoom: int) -> float:
    return 156543.03392 * math.cos(math.radians(lat_deg)) / (2 ** zoom)

def fetch_static_map(center_latlon, zoom=18, size=(800, 800), scale=2, maptype="roadmap"):
    """
    Returns (image_numpy, meters_per_pixel, (width_px, height_px)).
    Requires Static Maps API.
    """
    key = get_api_key()
    url = "https://maps.googleapis.com/maps/api/staticmap"
    params = {
        "center": f"{center_latlon[0]},{center_latlon[1]}",
        "zoom": str(zoom),
        "size": f"{size[0]}x{size[1]}",
        "scale": str(scale),
        "maptype": maptype,
        "key": key,
    }
    r = requests.get(url, params=params, timeout=15)
    r.raise_for_status()
    img = Image.open(io.BytesIO(r.content)).convert("RGB")
    arr = np.asarray(img)
    m_per_px = ground_resolution_m_per_px(center_latlon[0], zoom) / scale
    return arr, m_per_px, (arr.shape[1], arr.shape[0])
