from flask import Flask, Response, render_template, redirect, url_for
from image_processing import generate_frames
from config import BROWSER_WS_PORT

app = Flask(__name__, template_folder="templates")

@app.route("/")
def index():
    is_setup = True   # if prototype is setup properly
    return render_template("index.html", is_setup=is_setup)

@app.route("/admin")
def admin_page():
    return render_template("admin.html", ws_port=BROWSER_WS_PORT)

@app.route("/client")
def client_page():
    return render_template("client.html", ws_port=BROWSER_WS_PORT)

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

def run_webserver(port):
    print(f"[FLASK] running on 0.0.0.0:{port}")
    app.run(
        host="0.0.0.0",
        port=port,
        debug=True,   # ✅ enables development mode
        threaded=True
    )

if __name__ == "__main__":
    run_webserver(5000)
