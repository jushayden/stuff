import os, requests

key = os.environ["GOOGLE_MAPS_API_KEY"]
url = "https://maps.googleapis.com/maps/api/directions/json"
params = {
    "origin": "Orlando,FL",
    "destination": "Tampa,FL",
    "mode": "driving",
    "key": key
}
r = requests.get(url, params=params, timeout=10)
data = r.json()
print("Status:", data.get("status"))
if data.get("status") == "OK":
    print("✅ Directions API is working!")
else:
    print("❌", data)
