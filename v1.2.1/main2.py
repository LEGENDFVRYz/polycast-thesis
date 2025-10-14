"""
Run:
    pip install flask pillow websocket-client websocket-server
    python main.py
"""

import threading, time, io, os, json, struct
from datetime import datetime
from flask import Flask, Response, render_template
from PIL import Image, ImageDraw
import websocket
from websocket_server import WebsocketServer

# CONFIG ----------------------------------------------------------------
ESP32_WS_URL = "ws://192.168.1.19/ws"           # ESP32 endpoint

# Fixed coordinate space (logical units)
CANVAS_WIDTH, CANVAS_HEIGHT = 35560, 22219

# Output image resolution (pixels)
MJPEG_WIDTH, MJPEG_HEIGHT = 600, 400

ARCHIVE_DIR = "archive"
MJPEG_FPS = 20
BROWSER_WS_PORT = 5001
FLASK_PORT = 5050
IS_AUTO_ARCHIVING = True
# -----------------------------------------------------------------------

# --- Derived scale between logical coordinates and image pixels ---
scale_x = MJPEG_WIDTH / CANVAS_WIDTH
scale_y = MJPEG_HEIGHT / CANVAS_HEIGHT
AVG_SCALE = (scale_x + scale_y) / 2.0  # uniform scaling for stroke width
STROKE_FACTOR = 0.5  # tune this if strokes are too thin or too thick

# -----------------------------------------------------------------------
canvas = Image.new("L", (MJPEG_WIDTH, MJPEG_HEIGHT), "white")
draw = ImageDraw.Draw(canvas)
image_lock = threading.Lock()
ws_server = None
last_point = None

# ---------------- Coordinate Mapping -----------------
def logical_to_pixel(x, y):
    """Map logical coordinates to fixed image pixel coordinates."""
    px = int((x / CANVAS_WIDTH) * MJPEG_WIDTH)
    py = int((y / CANVAS_HEIGHT) * MJPEG_HEIGHT)
    px = max(0, min(MJPEG_WIDTH - 1, px))
    py = max(0, min(MJPEG_HEIGHT - 1, py))
    return px, py

# ---------------- Drawing -----------------
def draw_segment(x0, y0, x1, y1, p):
    """Draw a segment between logical coordinates (x0,y0)-(x1,y1) with scaled width."""
    px0, py0 = logical_to_pixel(x0, y0)
    px1, py1 = logical_to_pixel(x1, y1)

    # Scale the line width based on coordinate-to-pixel ratio
    width_px = max(1, int(p * AVG_SCALE * STROKE_FACTOR))
    draw.line([(px0, py0), (px1, py1)], fill=0, width=width_px)

# ---------------- ESP32 WebSocket Client -----------------
def parse_esp_binary(msg: bytes):
    i = 0
    global last_point
    points = []
    while i < len(msg):
        header = msg[i]
        if header == 0 and i + 6 <= len(msg):
            x = struct.unpack_from("<h", msg, i + 1)[0]
            y = struct.unpack_from("<h", msg, i + 3)[0]
            p = msg[i + 5]
            last_point = (x, y)
            points.append((x, y, p))
            i += 6
        elif header == 1 and i + 4 <= len(msg):
            dx = struct.unpack_from("<b", msg, i + 1)[0]
            dy = struct.unpack_from("<b", msg, i + 2)[0]
            p = msg[i + 3]
            if last_point:
                x = last_point[0] + dx
                y = last_point[1] + dy
                last_point = (x, y)
                points.append((x, y, p))
            i += 4
        else:
            break
    return points

def on_esp_message(wsapp, message):
    global last_point, ws_server
    try:
        points = parse_esp_binary(message)
        with image_lock:
            for (x, y, p) in points:
                if p < 3:
                    last_point = None
                else:
                    if last_point:
                        draw_segment(last_point[0], last_point[1], x, y, p)
                    last_point = (x, y)
        if ws_server and points:
            ws_server.send_message_to_all(json.dumps(points[-1]))
    except Exception as e:
        print("[ESP WS] parse/draw error:", e)

def on_esp_open(wsapp): print("[ESP WS] connected")
def on_esp_error(wsapp, err): print("[ESP WS] error:", err)
def on_esp_close(wsapp, code, msg): print("[ESP WS] closed:", code, msg)

def ws_client_thread():
    while True:
        try:
            print("[ESP WS] connecting to", ESP32_WS_URL)
            wsapp = websocket.WebSocketApp(
                ESP32_WS_URL,
                on_open=on_esp_open,
                on_message=on_esp_message,
                on_error=on_esp_error,
                on_close=on_esp_close,
            )
            wsapp.run_forever()
        except Exception as e:
            print("[ESP WS] client exception:", e)
        print("[ESP WS] reconnect in 2s...")
        time.sleep(2)

# ---------------- Browser WebSocket Server -----------------
def new_browser_client(c, s): print(f"[BROWSER WS] + Client {c['id']}")
def browser_client_left(c, s): print(f"[BROWSER WS] - Client {c['id']}")
def browser_message_received(c, s, m): pass

def start_browser_ws_server():
    global ws_server
    ws_server = WebsocketServer(host="0.0.0.0", port=BROWSER_WS_PORT, loglevel=20)
    ws_server.set_fn_new_client(new_browser_client)
    ws_server.set_fn_client_left(browser_client_left)
    ws_server.set_fn_message_received(browser_message_received)
    print(f"[BROWSER WS] serving on {BROWSER_WS_PORT}")
    ws_server.run_forever()

# ---------------- Archiver -----------------
def archiver_thread():
    os.makedirs(ARCHIVE_DIR, exist_ok=True)
    while True:
        time.sleep(1)
        with image_lock:
            snap = canvas.copy()
        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        fname = os.path.join(ARCHIVE_DIR, f"note_{ts}.jpg")
        try:
            snap.convert("RGB").save(fname, quality=85)
            print(f"[ARCHIVER] saved {fname}")
        except Exception as e:
            print("[ARCHIVER] error:", e)

# ---------------- Flask MJPEG -----------------
app = Flask(__name__, template_folder="templates")

@app.route("/")
def index():
    return render_template("index.html", ws_port=BROWSER_WS_PORT)

def generate_frames():
    interval = 1.0 / MJPEG_FPS
    while True:
        start_time = time.time()
        with image_lock:
            frame = canvas.copy()
        try:
            buf = io.BytesIO()
            frame.convert("RGB").save(buf, format="JPEG")
            jpg = buf.getvalue()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
        except Exception as e:
            print(f"[MJPEG] Error generating frame: {e}")
        elapsed = time.time() - start_time
        time.sleep(max(0, interval - elapsed))

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ---------------- Main -----------------
if __name__ == "__main__":
    threading.Thread(target=start_browser_ws_server, daemon=True).start()
    threading.Thread(target=ws_client_thread, daemon=True).start()

    if IS_AUTO_ARCHIVING:
        threading.Thread(target=archiver_thread, daemon=True).start()
        print("[ARCHIVER] auto-archiving enabled")
    else:
        print("[ARCHIVER] auto-archiving disabled")

    print(f"[FLASK] running on 0.0.0.0:{FLASK_PORT}")
    app.run(host="0.0.0.0", port=FLASK_PORT, threaded=True)
