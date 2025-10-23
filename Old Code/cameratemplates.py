import cv2 #library for camera access
from flask import Flask, Response, render_template_string #classes inside flask package
#flask = main class for web app instance
#response = class for video stream (sending back http responses)
#render = allows to run html inside this py file (import render_template class for the html to be in a different file)
# --- config ---
CAMERA_INDEX = 0
FRAME_WIDTH  = 1280
FRAME_HEIGHT = 720
FPS_TARGET   = 30

# --- init ---
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    raise RuntimeError(f"Could not open camera index {CAMERA_INDEX}")

cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS,         FPS_TARGET)

app = Flask(__name__)
#the html allowed to be in the py file because of the render_template_string class
INDEX_HTML = """
<!doctype html>
<html>
  <head>
    <meta charset="utf-8">
    <title>Live Camera</title>
    <style>
      body{margin:0;background:#111;color:#eee;font-family:system-ui}
      header{padding:14px 16px;font-weight:600}
      img{display:block;max-width:100vw;height:auto;margin:auto}
      .wrap{padding-bottom:20px}
      .note{opacity:.7;text-align:center;margin-top:8px}
    </style>
  </head>
  <body>
    <header>Live Camera Stream</header>
    <div class="wrap">
      <img src="/video" alt="Camera stream"/>
      <div class="note">If the image stalls, refresh. Press Ctrl+C in the server to stop.</div>
    </div>
  </body>
</html>
"""

def mjpeg_generator():
    """Yield JPEG frames as multipart MJPEG."""
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        ok, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            continue
        jpg_bytes = buffer.tobytes()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + jpg_bytes + b"\r\n")

@app.route("/")
def index():
    return render_template_string(INDEX_HTML)

@app.route("/video")
def video():
    return Response(mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)


#checklist (features)
#camera control
#-recording
#-livestreaming
#-object detection
#face detection/recognition training
#motion-tracking
#AR application (overlay objects on feed)