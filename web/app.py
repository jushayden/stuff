# web/app.py
import os, time, math, json
from threading import Thread
from dotenv import load_dotenv
from flask import Flask, render_template
from flask_socketio import SocketIO

load_dotenv()

app = Flask(__name__, static_folder="static", template_folder="templates")
socketio = SocketIO(app, cors_allowed_origins="*")  # uses eventlet if installed

@app.route("/")
def index():
    browser_key = os.getenv("GOOGLE_MAPS_BROWSER_KEY", "")
    return render_template("index.html", api_key=browser_key)

# ----- demo telemetry loop (replace with real robot data) -----
def telemetry_loop():
    x = 0
    while True:
        payload = {
            "battery": 78,
            "mode": "AUTO",
            "speed_mps": 1.25,
            "lat": 37.4219999 + 0.0006 * math.sin(x / 60),  # fake pos
            "lng": -122.0840575 + 0.0006 * math.cos(x / 60),
            "ts": time.time(),
        }
        socketio.emit("telemetry", payload)
        x += 1
        time.sleep(0.2)

@socketio.on("connect")
def on_connect():
    print("Client connected")

@socketio.on("disconnect")
def on_disconnect():
    print("Client disconnected")

@socketio.on("mission")
def on_mission(data):
    print("Mission received:", json.dumps(data, indent=2))
    # TODO: invoke your planner/control here
    socketio.emit("mission_ack", {"ok": True, "received": data})

if __name__ == "__main__":
    # start background thread ONCE
    Thread(target=telemetry_loop, daemon=True).start()

    # bind once, on a clean port, with no reloader (prevents double bind)
    socketio.run(
        app,
        host="127.0.0.1",
        port=5060,        # <— using 5060 (change if you like)
        debug=False,
        use_reloader=False
    )
