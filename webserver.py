from flask import Flask, Response, render_template
from image_processing import generate_frames
from config import BROWSER_WS_PORT

app = Flask(__name__, template_folder="templates")

@app.route("/")
def index():
    return render_template("index.html", ws_port=BROWSER_WS_PORT)

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

def run_webserver(port):
    print(f"[FLASK] running on 0.0.0.0:{port}")
    app.run(host="0.0.0.0", port=port, threaded=True)
